"""빈자리 bbox 추적 (optical-flow similarity, detection handover).

empty_slot_detector가 고른 빈자리 image bbox를 받아 추적하고, 그 결과를 기존 IBVS/NN 스택이
그대로 소비할 수 있는 합성 TrackedObjectArray 로 재발행한다. 즉 "빈 공간 bbox 자체"를 servo
대상 객체로 만들어 IBVS가 image-based 수렴하게 한다.

왜 CSRT가 아니라 optical-flow 인가:
  - 빈자리는 흰색·무특징이라 CSRT 로 직접 추적하면 응답이 평평해 drift 하고, 무엇보다
    CSRT 의 box 크기(scale)가 카메라 거리 변화에 거의 반응하지 않는다(실측: 실제 +40% 확대에도
    CSRT box 폭은 고정~역방향). 그러면 IBVS 의 area-jacobian(면적으로 접근 깊이 추정)이
    'area 변화 0'을 받아 실패한다.
  - 그래서 슬롯 주변 feature(선반 레일/가격표/양옆 상품)를 포함하는 context 영역에서
    feature point 를 잡아 LK optical-flow 로 추적하고, estimateAffinePartial2D 로
    프레임간 similarity(translation+scale)를 구해 슬롯 bbox 에 누적 적용한다. feature 들이
    벌어지는 비율 = 실제 거리 신호라, 슬롯 면적이 거리에 정확히 반응한다(실측: 실제 zoom 추종
    오차 <1%). center 도 다수 point 의 robust 추정이라 흰자리에서도 안정적이다.

동작:
  - init: 빈자리 bbox 중심을 context_expand 배로 확장한 영역에서 goodFeaturesToTrack.
  - 프레임마다: LK 추적 -> RANSAC similarity -> 슬롯 bbox 에 step(translation+scale) 누적.
    point 가 줄면 현재 슬롯 주변 context 에서 재seed(누적 scale 은 슬롯 상태에 보존됨).
  - eye-in-hand 하강으로 feature 가 가려져 추적이 끊기면 발행을 멈춘다. 이후 final 구간은
    NN controller가 anchor frozen/zero 로 detection 없이 처리한다.

구독:
  /place/target_bbox  (Float64MultiArray) : [cx, cy, w, h, angle_deg] (center 기준, init/re-init)
  infer/image_raw     (Image)             : 라이브 프레임
발행:
  /place/tracked_objects (TrackedObjectArray) : 단일 empty_slot 객체(추적 슬롯 bbox)
  /place/context_bbox    (Float64MultiArray)  : 추적 중 feature point 들의 bbox(디버그/overlay)
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

from just_pick_it_interfaces.msg import TrackedObject, TrackedObjectArray

from just_pick_it_perception.cv_image_utils import imgmsg_to_bgr


_LK_PARAMS = dict(
    winSize=(21, 21),
    maxLevel=3,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03),
)


def _latched_qos(depth: int = 1) -> QoSProfile:
    # transient_local: 늦게 뜬 구독자도 마지막 bbox를 받는다(agent on-demand 기동 지연 대비).
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        depth=depth,
    )


class CsrtPlaceTrackerNode(Node):

    def __init__(self):
        super().__init__('csrt_place_tracker_node')
        self.declare_parameter('class_label', 'empty_slot')
        self.declare_parameter('track_id', 1)
        self.declare_parameter('min_size_px', 6.0)    # 슬롯이 이 이하로 줄면 추적 실패 처리
        # context: 슬롯 주변 feature 를 담을 영역(슬롯 대비 확장 + 최소 크기). 여기서 feature seed.
        self.declare_parameter('context_expand', 3.0)
        self.declare_parameter('context_min_px', 120.0)
        # optical-flow
        self.declare_parameter('flow_max_points', 150)   # seed 최대 corner 수
        self.declare_parameter('flow_min_points', 12)    # 이 미만이면 재seed
        self.declare_parameter('flow_quality', 0.01)     # goodFeaturesToTrack qualityLevel
        self.declare_parameter('flow_min_distance', 4.0)
        self.declare_parameter('flow_step_scale_max', 1.2)  # 프레임당 scale 변화 clamp(이상치 방지)

        self.class_label = str(self.get_parameter('class_label').value)
        self.track_id = int(self.get_parameter('track_id').value)
        self.min_size_px = float(self.get_parameter('min_size_px').value)
        self.context_expand = float(self.get_parameter('context_expand').value)
        self.context_min_px = float(self.get_parameter('context_min_px').value)
        self.flow_max_points = int(self.get_parameter('flow_max_points').value)
        self.flow_min_points = int(self.get_parameter('flow_min_points').value)
        self.flow_quality = float(self.get_parameter('flow_quality').value)
        self.flow_min_distance = float(self.get_parameter('flow_min_distance').value)
        self.flow_step_scale_max = float(self.get_parameter('flow_step_scale_max').value)

        self._pending_init = None   # (cx, cy, w, h) center 기준 슬롯 init 요청
        self._angle_deg = 0.0
        self._active = False
        self._slot = None           # {cx, cy, w, h} 현재 슬롯 추정
        self._prev_gray = None
        self._p_cur = None          # 현재 추적 중 feature points (Nx1x2 float32)

        self.create_subscription(
            Float64MultiArray, '/place/target_bbox', self._bbox_cb, _latched_qos())
        self.create_subscription(Image, 'infer/image_raw', self._image_cb, 5)
        self._pub = self.create_publisher(
            TrackedObjectArray, '/place/tracked_objects', 10)
        self._ctx_pub = self.create_publisher(
            Float64MultiArray, '/place/context_bbox', 10)

        self.get_logger().info(
            'CsrtPlaceTrackerNode(optical-flow) 준비 완료. /place/target_bbox 대기.')

    # ── init 요청 ─────────────────────────────────────────────────────────

    def _bbox_cb(self, msg: Float64MultiArray):
        d = list(msg.data)
        if len(d) < 4:
            self.get_logger().warn('target_bbox 형식 오류 (len<4) - 무시.')
            return
        cx, cy, bw, bh = d[0], d[1], d[2], d[3]
        self._angle_deg = float(d[4]) if len(d) >= 5 else 0.0
        self._pending_init = (float(cx), float(cy), float(bw), float(bh))
        self.get_logger().info(
            f'새 빈자리 bbox 수신: center=({cx:.0f},{cy:.0f}), size=({bw:.0f}x{bh:.0f}). '
            f'다음 프레임에서 optical-flow init.')

    # ── 프레임 처리 ───────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        frame = imgmsg_to_bgr(msg)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self._pending_init is not None:
            self._init_flow(gray, msg.header)
            self._pending_init = None
            return

        if not self._active or self._slot is None or self._prev_gray is None:
            return

        self._track_flow(gray, msg.header)

    def _init_flow(self, gray, header):
        cx, cy, bw, bh = self._pending_init
        # feature seed 영역 = 슬롯 중심 기준 context 박스.
        cw = max(bw * self.context_expand, self.context_min_px)
        ch = max(bh * self.context_expand, self.context_min_px)
        pts = self._seed_points(gray, cx, cy, cw, ch)
        if pts is None or len(pts) < 6:
            n = 0 if pts is None else len(pts)
            self.get_logger().error(
                f'optical-flow init 실패: context 영역 feature 부족({n}개). '
                'context_expand 확대 또는 조명/텍스처 확인.')
            self._active = False
            self._slot = None
            return
        self._slot = {'cx': float(cx), 'cy': float(cy), 'w': float(bw), 'h': float(bh)}
        self._p_cur = pts
        self._prev_gray = gray
        self._active = True
        self.get_logger().info(
            f'optical-flow init 완료 (feature {len(pts)}개, context {cw:.0f}x{ch:.0f}) - 추적 시작.')
        self._publish_slot(header)
        self._publish_context_from_points()

    def _track_flow(self, gray, header):
        if self._p_cur is None or len(self._p_cur) < 3:
            self._p_cur = self._seed_around_slot(gray)
            if self._p_cur is None or len(self._p_cur) < 3:
                self.get_logger().warn('추적 feature 부족 - 발행 중단.')
                self._active = False
                return
            self._prev_gray = gray
            return

        p_new, status, _ = cv2.calcOpticalFlowPyrLK(
            self._prev_gray, gray, self._p_cur, None, **_LK_PARAMS)
        if p_new is None or status is None:
            self.get_logger().warn('optical-flow 추적 실패 - 발행 중단.')
            self._active = False
            return
        status = status.reshape(-1)
        good_prev = self._p_cur[status == 1]
        good_new = p_new[status == 1]

        if len(good_new) >= 3:
            M, _ = cv2.estimateAffinePartial2D(good_prev, good_new, method=cv2.RANSAC)
            if M is not None:
                step = float(np.hypot(M[0, 0], M[1, 0]))
                lo, hi = 1.0 / self.flow_step_scale_max, self.flow_step_scale_max
                step = float(np.clip(step, lo, hi))
                cx, cy = self._slot['cx'], self._slot['cy']
                self._slot['cx'] = float(M[0, 0] * cx + M[0, 1] * cy + M[0, 2])
                self._slot['cy'] = float(M[1, 0] * cx + M[1, 1] * cy + M[1, 2])
                self._slot['w'] *= step
                self._slot['h'] *= step

        self._p_cur = good_new.reshape(-1, 1, 2) if len(good_new) else None
        self._prev_gray = gray

        # feature 가 줄면 현재 슬롯 주변 context 에서 재seed(누적 scale 은 슬롯에 이미 반영됨).
        if self._p_cur is None or len(self._p_cur) < self.flow_min_points:
            reseed = self._seed_around_slot(gray)
            if reseed is not None and len(reseed) >= 3:
                self._p_cur = reseed

        if self._slot['w'] < self.min_size_px or self._slot['h'] < self.min_size_px:
            self.get_logger().warn('추적 슬롯 bbox 과소 - 발행 중단.')
            self._active = False
            return

        self._publish_slot(header)
        self._publish_context_from_points()

    # ── feature seed ─────────────────────────────────────────────────────

    def _seed_around_slot(self, gray):
        s = self._slot
        cw = max(s['w'] * self.context_expand, self.context_min_px)
        ch = max(s['h'] * self.context_expand, self.context_min_px)
        return self._seed_points(gray, s['cx'], s['cy'], cw, ch)

    def _seed_points(self, gray, cx, cy, w, h):
        H, W = gray.shape[:2]
        x0 = int(np.clip(cx - w / 2.0, 0, W - 1))
        y0 = int(np.clip(cy - h / 2.0, 0, H - 1))
        x1 = int(np.clip(cx + w / 2.0, x0 + 1, W))
        y1 = int(np.clip(cy + h / 2.0, y0 + 1, H))
        mask = np.zeros((H, W), np.uint8)
        mask[y0:y1, x0:x1] = 255
        return cv2.goodFeaturesToTrack(
            gray, self.flow_max_points, self.flow_quality, self.flow_min_distance, mask=mask)

    # ── 발행 ─────────────────────────────────────────────────────────────

    def _publish_slot(self, header):
        s = self._slot
        x = s['cx'] - s['w'] / 2.0
        y = s['cy'] - s['h'] / 2.0
        self._publish(x, y, s['w'], s['h'], header)

    def _publish(self, x, y, bw, bh, header):
        cx = x + bw / 2.0
        cy = y + bh / 2.0

        obj = TrackedObject()
        obj.track_id = self.track_id
        obj.class_label = self.class_label
        obj.class_id = 0
        obj.confidence = 1.0
        obj.bbox_x = float(cx)      # center 기준 (yolo_seg_infer 와 동일 convention)
        obj.bbox_y = float(cy)
        obj.bbox_w = float(bw)
        obj.bbox_h = float(bh)
        obj.mask_cx = float(cx)
        obj.mask_cy = float(cy)
        obj.orientation_angle = float(self._angle_deg)
        obj.mask_polygon = [
            float(x), float(y),
            float(x + bw), float(y),
            float(x + bw), float(y + bh),
            float(x), float(y + bh),
        ]
        obj.pose_valid = False
        obj.frame_count = 0

        arr = TrackedObjectArray()
        arr.header = header
        arr.objects = [obj]
        self._pub.publish(arr)

    def _publish_context_from_points(self):
        if self._p_cur is None or len(self._p_cur) < 2:
            return
        pts = self._p_cur.reshape(-1, 2)
        x0, y0 = pts.min(axis=0)
        x1, y1 = pts.max(axis=0)
        msg = Float64MultiArray()
        msg.data = [float((x0 + x1) / 2.0), float((y0 + y1) / 2.0),
                    float(x1 - x0), float(y1 - y0)]
        self._ctx_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CsrtPlaceTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
