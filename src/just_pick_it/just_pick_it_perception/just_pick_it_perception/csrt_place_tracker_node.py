"""빈자리 bbox CSRT tracker (detection handover).

empty_slot_detector가 고른 빈자리 image bbox를 받아 CSRT로 추적하고, 그 결과를 기존
IBVS/NN 픽 스택이 그대로 소비할 수 있는 합성 TrackedObjectArray 로 재발행한다.
즉 "빈 공간 bbox 자체"를 servo 대상 객체로 만들어 IBVS가 image-based 수렴하게 한다.

핵심:
  - YOLO는 빈 공간을 검출하지 못하므로, 한 번 잡은 bbox를 CSRT appearance tracker로
    프레임마다 따라가 IBVS 접근 동안 타깃을 유지한다.
  - eye-in-hand 하강으로 빈자리가 가려져 추적이 끊기면 발행을 멈춘다. 이후 final 구간은
    NN controller가 anchor frozen/zero 로 detection 없이 처리한다.

구독:
  /place/target_bbox  (Float64MultiArray) : [cx, cy, w, h, angle_deg] (center 기준, init/re-init)
  infer/image_raw     (Image)             : 라이브 프레임
발행:
  /place/tracked_objects (TrackedObjectArray) : 단일 empty_slot 객체
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


def _latched_qos(depth: int = 1) -> QoSProfile:
    # transient_local: 늦게 뜬 구독자도 마지막 bbox를 받는다(agent on-demand 기동 지연 대비).
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        depth=depth,
    )


def _make_csrt():
    """OpenCV 버전별 CSRT 생성자 호환."""
    if hasattr(cv2, 'legacy') and hasattr(cv2.legacy, 'TrackerCSRT_create'):
        return cv2.legacy.TrackerCSRT_create()
    if hasattr(cv2, 'TrackerCSRT_create'):
        return cv2.TrackerCSRT_create()
    raise RuntimeError('cv2 TrackerCSRT 사용 불가 (opencv-contrib 필요).')


class CsrtPlaceTrackerNode(Node):

    def __init__(self):
        super().__init__('csrt_place_tracker_node')
        self.declare_parameter('class_label', 'empty_slot')
        self.declare_parameter('track_id', 1)
        self.declare_parameter('min_size_px', 6.0)   # 이 이하로 줄면 추적 실패 처리

        self.class_label = str(self.get_parameter('class_label').value)
        self.track_id = int(self.get_parameter('track_id').value)
        self.min_size_px = float(self.get_parameter('min_size_px').value)

        self._tracker = None
        self._pending_init = None   # (x, y, w, h) topleft
        self._angle_deg = 0.0
        self._active = False

        self.create_subscription(
            Float64MultiArray, '/place/target_bbox', self._bbox_cb, _latched_qos())
        self.create_subscription(Image, 'infer/image_raw', self._image_cb, 5)
        self._pub = self.create_publisher(
            TrackedObjectArray, '/place/tracked_objects', 10)

        self.get_logger().info('CsrtPlaceTrackerNode 준비 완료. /place/target_bbox 대기.')

    def _bbox_cb(self, msg: Float64MultiArray):
        d = list(msg.data)
        if len(d) < 4:
            self.get_logger().warn('target_bbox 형식 오류 (len<4) - 무시.')
            return
        cx, cy, bw, bh = d[0], d[1], d[2], d[3]
        self._angle_deg = float(d[4]) if len(d) >= 5 else 0.0
        # center -> topleft (CSRT init은 topleft 기준).
        x = cx - bw / 2.0
        y = cy - bh / 2.0
        self._pending_init = (float(x), float(y), float(bw), float(bh))
        self.get_logger().info(
            f'새 빈자리 bbox 수신: center=({cx:.0f},{cy:.0f}), size=({bw:.0f}x{bh:.0f}). '
            f'다음 프레임에서 CSRT init.')

    def _image_cb(self, msg: Image):
        frame = imgmsg_to_bgr(msg)
        h, w = frame.shape[:2]

        # 대기 중인 init 처리(실제 프레임에서 init 해야 함).
        if self._pending_init is not None:
            x, y, bw, bh = self._pending_init
            x = float(np.clip(x, 0, w - 2))
            y = float(np.clip(y, 0, h - 2))
            bw = float(np.clip(bw, 2, w - x))
            bh = float(np.clip(bh, 2, h - y))
            try:
                self._tracker = _make_csrt()
                self._tracker.init(frame, (int(x), int(y), int(bw), int(bh)))
                self._active = True
                self.get_logger().info('CSRT init 완료 - 추적 시작.')
            except Exception as e:
                self.get_logger().error(f'CSRT init 실패: {e}')
                self._tracker = None
                self._active = False
            self._pending_init = None
            return

        if not self._active or self._tracker is None:
            return

        ok, box = self._tracker.update(frame)
        if not ok:
            self.get_logger().warn('CSRT 추적 실패 - 발행 중단.')
            self._active = False
            return
        x, y, bw, bh = box
        if bw < self.min_size_px or bh < self.min_size_px:
            self.get_logger().warn('추적 bbox 과소 - 발행 중단.')
            self._active = False
            return

        self._publish(x, y, bw, bh, w, h, msg.header)

    def _publish(self, x, y, bw, bh, img_w, img_h, header):
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
        # mask_polygon = bbox 사각형 4점(픽셀). IBVS J6 정렬 등에서 polygon 필요 시 사용.
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
