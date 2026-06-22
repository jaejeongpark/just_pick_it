"""선반 빈자리 detector (image space, calibration-free, 스캔 누적형).

DISPLAY_SCAN 단계에서 진열대 위 빈 공간을 이미지 평면에서 찾는다. 배치는 IBVS+NN이
그 bbox로 image-based 수렴하므로 카메라 intrinsic/hand-eye/메트릭 투영이 모두 불필요하다.

동작(사용자 확정): 스캔 자세를 스윕하며 각 자세 프레임에서 빈자리 후보를 1개씩 누적하고,
스윕이 끝나면 누적된 후보 전체에서 최적 1곳을 optimization으로 선정한다. 스윕 내내 후보가
하나도 없으면 found=0으로 알려 상위에서 error 처리 후 재스캔하게 한다.

각 capture는 스캔 자세 하나에 대응한다. plan은 우승 후보의 capture_index를 함께 반환하므로
상위(run_scanning)가 그 자세로 복귀해 동일 bbox를 CSRT에 넘길 수 있다.

알고리즘 (한 프레임, 모두 이미지 픽셀 기준):
  1) Canny + HoughLinesP로 선반 near-horizontal 모서리를 찾아 선반 표면 ROI를 한정
     (실패 시 파라미터 사각형 ROI fallback).
  2) YOLO seg polygon(이미 진열된 물건)을 ROI 안에서 점유 마스크로 채움.
  3) place footprint + clearance(px)만큼 점유를 팽창(보수적 overlap 방지).
  4) free 영역에 distance transform → 각 셀 여유(px).
  5) 여유가 footprint 이상인 후보 중 "팔이 놓기 좋은 조건"(여유 크고, 가로 중앙·앞쪽 선호)을
     가중치로 최대화해 그 프레임의 best 1개 선정.

트리거:
  /place/reset         (Empty) : 누적 초기화(스윕 시작 전)
  /place/capture_view  (Empty) : 자세 1곳에서 여러 프레임을 샘플링해 안정 후보 1개 누적
  /place/plan          (Empty) : 누적 후보 전체에서 최적 1곳 선정 후 발행
구독:
  infer/image_raw        (Image)              : 라이브 프레임
  infer/tracked_objects  (TrackedObjectArray) : YOLO 물체(점유) polygon
발행:
  /place/scan_result (Float64MultiArray) : [found, cx, cy, w, h, angle_deg, capture_index, score]
                                            found=1.0 성공 / 0.0 후보 없음. bbox는 center 기준 px.
                                            (PLACE 시 CSRT init 토픽 /place/target_bbox 와 분리)
  /place/debug_image (Image)
"""

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Float64MultiArray

from just_pick_it_interfaces.msg import TrackedObjectArray

from just_pick_it_perception.cv_image_utils import bgr_to_imgmsg, imgmsg_to_bgr


class EmptySlotDetectorNode(Node):

    def __init__(self):
        super().__init__('empty_slot_detector_node')
        self._declare_parameters()

        self._latest_image = None
        self._latest_objects = []
        # 누적 후보. 각 원소 = capture(자세) 하나의 안정 후보 dict 또는 None.
        self._captures = []
        # capture 1회의 다중 프레임 샘플링 상태(단일스레드 executor 라 비동기로 누적).
        self._sampling = False
        self._sample_buf = []
        self._sample_start = None

        self.create_subscription(Image, 'infer/image_raw', self._image_cb, 5)
        self.create_subscription(
            TrackedObjectArray, 'infer/tracked_objects', self._objects_cb, 10)
        self.create_subscription(Empty, '/place/reset', self._reset_cb, 10)
        self.create_subscription(Empty, '/place/capture_view', self._capture_cb, 10)
        self.create_subscription(Empty, '/place/plan', self._plan_cb, 10)

        self._pub_bbox = self.create_publisher(Float64MultiArray, '/place/scan_result', 10)
        self._pub_debug = self.create_publisher(Image, '/place/debug_image', 5)
        self._pub_edges = self.create_publisher(Image, '/place/debug_edges', 5)

        live_hz = float(self.get_parameter('debug_live_hz').value)
        if live_hz > 0.0:
            self.create_timer(1.0 / live_hz, self._publish_live_debug)
            self.get_logger().info(f'라이브 디버그 ON ({live_hz}Hz) — /place/debug_image, /place/debug_edges')

        self.get_logger().info(
            'EmptySlotDetectorNode 준비. reset -> capture_view(스윕) -> plan 순서로 트리거.')

    # ------------------------------------------------------------------ params

    def _declare_parameters(self):
        self.declare_parameter('obstacle_classes', '')  # 비우면 모든 검출을 obstacle
        self.declare_parameter('min_confidence', 0.3)

        self.declare_parameter('roi_top_frac', 0.30)
        self.declare_parameter('roi_bottom_frac', 0.98)
        self.declare_parameter('roi_left_frac', 0.05)
        self.declare_parameter('roi_right_frac', 0.95)

        self.declare_parameter('use_edge_roi', True)
        self.declare_parameter('canny_low', 50)
        self.declare_parameter('canny_high', 150)
        self.declare_parameter('hough_threshold', 50)
        self.declare_parameter('hough_min_line_len', 40)
        self.declare_parameter('hough_max_line_gap', 10)
        self.declare_parameter('edge_max_image_slope', 0.30)
        self.declare_parameter('use_white_mask', True)

        self.declare_parameter('place_radius_px', 30.0)
        self.declare_parameter('clearance_px', 8.0)
        self.declare_parameter('edge_margin_px', 6.0)

        self.declare_parameter('w_clear', 1.0)
        self.declare_parameter('w_center', 0.6)
        self.declare_parameter('w_front', 0.5)
        # 자세(capture) 우승용 면적 가중치. candidate(초록=놓을 수 있는 공간) 면적이
        # 넓은 자세를 선호한다. 좁은 틈보다 텅 빈 자세를 이기게 하는 주 노브.
        self.declare_parameter('w_area', 4.0)

        # clearance 정규화 기준 px(자세 간 비교용 절대 척도). <=0 이면 자동으로
        # 2*(place_radius_px + clearance_px) 사용. 이 값 이상 여유는 만점으로 포화.
        self.declare_parameter('clear_ref_px', 0.0)

        self.declare_parameter('bbox_scale', 1.0)

        # 자세별 후보 안정화: 한 capture(자세)에서 여러 프레임을 샘플링해, 같은 위치에 반복
        # 검출되는(안정적인) 후보만 채택한다. jitter/1프레임 허위검출 제거.
        #   capture_sample_count : 한 capture 에서 detection 할 최대 프레임 수
        #   capture_sample_sec   : 샘플링 최대 시간(이 시간 지나면 모인 프레임으로 확정)
        #   capture_stable_tol_px: 같은 위치로 묶을 center 허용 오차(px)
        #   capture_min_stable   : 채택에 필요한 최소 일관 검출 프레임 수(미만이면 후보 없음)
        self.declare_parameter('capture_sample_count', 5)
        self.declare_parameter('capture_sample_sec', 1.0)
        self.declare_parameter('capture_stable_tol_px', 20.0)
        self.declare_parameter('capture_min_stable', 3)

        # 라이브 디버그: >0 이면 트리거 없이 주기적으로 최신 프레임에 detection 을 돌려
        # /place/debug_image(점유/후보/best) 와 /place/debug_edges(Canny) 를 계속 발행한다.
        # 검증/튜닝용. 스캔 누적(_captures)에는 영향 없음. 0 이면 비활성(운영 기본).
        self.declare_parameter('debug_live_hz', 0.0)

    # --------------------------------------------------------------- callbacks

    def _image_cb(self, msg: Image):
        self._latest_image = imgmsg_to_bgr(msg)
        # 샘플링 중이면 프레임마다 detection 을 누적하고, 목표 프레임 수 또는 시간이 차면 확정.
        if self._sampling:
            self._sample_buf.append(self._detect_best_in_frame())
            n = int(self.get_parameter('capture_sample_count').value)
            win = float(self.get_parameter('capture_sample_sec').value)
            elapsed = (self.get_clock().now() - self._sample_start).nanoseconds * 1e-9
            if len(self._sample_buf) >= n or elapsed >= win:
                self._finalize_capture()

    def _objects_cb(self, msg: TrackedObjectArray):
        self._latest_objects = list(msg.objects)

    def _reset_cb(self, _msg):
        self._captures.clear()
        self._sampling = False
        self._sample_buf = []
        self.get_logger().info('누적 초기화(reset).')

    def _capture_cb(self, _msg):
        # 한 자세의 다중 프레임 샘플링 시작. image_cb 가 누적 후 안정 후보 1개를 확정한다.
        if self._sampling:
            self.get_logger().warn(
                'capture_view 수신했으나 이전 샘플링 진행 중 - 무시(스캔 간격을 늘릴 것).')
            return
        self._sampling = True
        self._sample_buf = []
        self._sample_start = self.get_clock().now()

    def _plan_cb(self, _msg):
        # 샘플링이 끝나기 전에 plan 이 와도 모인 프레임으로 마지막 capture 를 확정한다.
        if self._sampling:
            self._finalize_capture()
        self._plan_and_publish()

    def _finalize_capture(self):
        """샘플 프레임들에서 안정적으로 반복 검출된 후보 1개를 골라 _captures 에 추가."""
        self._sampling = False
        buf = self._sample_buf
        self._sample_buf = []
        idx = len(self._captures)
        stable = self._select_stable(buf)
        self._captures.append(stable)
        detected = sum(1 for c in buf if c is not None)
        if stable is None:
            self.get_logger().info(
                f'capture #{idx}: 안정 후보 없음 (검출 {detected}/{len(buf)} 프레임).')
        else:
            self.get_logger().info(
                f"capture #{idx}: center=({stable['cx']:.0f},{stable['cy']:.0f}), "
                f"clear={stable['clearance']:.0f}px, area={stable['area_frac']:.2f}, "
                f"score={stable['score']:.3f} "
                f"(안정 {stable['stable_count']}/{len(buf)} 프레임).")

    def _select_stable(self, buf):
        """샘플 후보들을 center 근접으로 묶어 가장 일관된 군집의 median bbox 를 반환(없으면 None)."""
        cands = [c for c in buf if c is not None]
        if not cands:
            return None
        tol = float(self.get_parameter('capture_stable_tol_px').value)
        min_stable = int(self.get_parameter('capture_min_stable').value)
        # greedy clustering: 각 후보를 seed 로 tol 안 후보를 묶고 최대 군집 선택.
        best = []
        for seed in cands:
            grp = [c for c in cands
                   if abs(c['cx'] - seed['cx']) <= tol and abs(c['cy'] - seed['cy']) <= tol]
            if len(grp) > len(best):
                best = grp
        if len(best) < min_stable:
            return None  # 같은 위치에 충분히 반복 검출되지 않음 -> 불안정으로 폐기
        cx = float(np.median([c['cx'] for c in best]))
        cy = float(np.median([c['cy'] for c in best]))
        bw = float(np.median([c['w'] for c in best]))
        bh = float(np.median([c['h'] for c in best]))
        score = float(np.mean([c['score'] for c in best]))
        clearance = float(np.median([c['clearance'] for c in best]))
        area_frac = float(np.median([c.get('area_frac', 0.0) for c in best]))
        # 오버레이용 대표 프레임: median center 에 가장 가까운 군집 멤버.
        rep = min(best, key=lambda c: (c['cx'] - cx) ** 2 + (c['cy'] - cy) ** 2)
        return {
            'cx': cx, 'cy': cy, 'w': bw, 'h': bh, 'angle': rep['angle'],
            'score': score, 'clearance': clearance, 'area_frac': area_frac,
            'debug': rep['debug'], 'stable_count': len(best),
        }

    # ------------------------------------------------------------ ROI (선반 영역)

    def _obstacle_class_set(self):
        raw = str(self.get_parameter('obstacle_classes').value or '').strip()
        if not raw:
            return None
        return {c.strip() for c in raw.split(',') if c.strip()}

    def _roi_mask(self, img):
        h, w = img.shape[:2]
        left = int(self.get_parameter('roi_left_frac').value * w)
        right = int(self.get_parameter('roi_right_frac').value * w)
        top = int(self.get_parameter('roi_top_frac').value * h)
        bottom = int(self.get_parameter('roi_bottom_frac').value * h)

        if self.get_parameter('use_edge_roi').value:
            edge_top, edge_bottom = self._detect_shelf_band(img)
            if edge_top is not None:
                top = max(top, edge_top)
            if edge_bottom is not None:
                bottom = min(bottom, edge_bottom)

        top = int(np.clip(top, 0, h - 1))
        bottom = int(np.clip(bottom, top + 1, h))
        left = int(np.clip(left, 0, w - 1))
        right = int(np.clip(right, left + 1, w))

        mask = np.zeros((h, w), dtype=np.uint8)
        mask[top:bottom, left:right] = 255
        return mask, (top, bottom, left, right)

    def _canny_edges(self, img):
        """선반 모서리 검출용 Canny 엣지(옵션 white mask 적용). 시각화·ROI 공용."""
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if self.get_parameter('use_white_mask').value:
            hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            white = cv2.inRange(hsv, (0, 0, 150), (180, 60, 255))
            white = cv2.dilate(white, np.ones((9, 9), np.uint8))
            gray = cv2.bitwise_and(gray, gray, mask=white)
        return cv2.Canny(gray,
                         int(self.get_parameter('canny_low').value),
                         int(self.get_parameter('canny_high').value))

    def _detect_shelf_band(self, img):
        h, w = img.shape[:2]
        edges = self._canny_edges(img)
        lines = cv2.HoughLinesP(
            edges, 1, np.pi / 180,
            int(self.get_parameter('hough_threshold').value),
            minLineLength=int(self.get_parameter('hough_min_line_len').value),
            maxLineGap=int(self.get_parameter('hough_max_line_gap').value))
        if lines is None:
            return None, None

        max_slope = self.get_parameter('edge_max_image_slope').value
        ys = []
        for ln in lines[:, 0, :]:
            x1, y1, x2, y2 = map(float, ln)
            dx, dy = x2 - x1, y2 - y1
            if abs(dx) < 1e-3:
                continue
            if abs(dy / dx) > max_slope:
                continue
            ys.append(0.5 * (y1 + y2))
        if not ys:
            return None, None

        ys = np.array(ys)
        front_y = int(np.max(ys))
        upper = ys[ys < front_y - 0.10 * h]
        back_y = int(np.min(upper)) if upper.size > 0 else None
        return back_y, front_y

    # ------------------------------------------------------------ 점유 마스크

    def _occupancy_mask(self, img):
        h, w = img.shape[:2]
        occ = np.zeros((h, w), dtype=np.uint8)
        cls_set = self._obstacle_class_set()
        min_conf = float(self.get_parameter('min_confidence').value)
        for o in self._latest_objects:
            if float(o.confidence) < min_conf:
                continue
            if cls_set is not None and str(o.class_label) not in cls_set:
                continue
            if o.mask_polygon and len(o.mask_polygon) >= 6:
                poly = np.array(o.mask_polygon, dtype=np.float64).reshape(-1, 2)
                cv2.fillPoly(occ, [poly.astype(np.int32)], 255)
            else:
                cx, cy = float(o.bbox_x), float(o.bbox_y)
                bw, bh = float(o.bbox_w), float(o.bbox_h)
                cv2.rectangle(occ, (int(cx - bw / 2), int(cy - bh / 2)),
                              (int(cx + bw / 2), int(cy + bh / 2)), 255, -1)
        return occ

    # ------------------------------------------------------------ 단일 프레임 detection

    def _free_and_candidate(self, img):
        """공통 파이프라인: ROI 한정 -> 점유 dilation -> distance transform -> 후보.

        반환: (roi_rect, occ_dil, dt, candidate(bool HxW), place_r).
        """
        roi, roi_rect = self._roi_mask(img)
        occ = cv2.bitwise_and(self._occupancy_mask(img), roi)

        place_r = float(self.get_parameter('place_radius_px').value)
        clear = float(self.get_parameter('clearance_px').value)
        margin = float(self.get_parameter('edge_margin_px').value)

        dil = max(1, int(round(place_r + clear)))
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * dil + 1, 2 * dil + 1))
        occ_dil = cv2.dilate(occ, kernel)

        valid = roi.copy()
        if margin > 0:
            er = max(1, int(round(margin)))
            ek = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * er + 1, 2 * er + 1))
            valid = cv2.erode(valid, ek)

        free = ((occ_dil == 0) & (valid > 0)).astype(np.uint8) * 255
        dt = cv2.distanceTransform(free, cv2.DIST_L2, 5)
        candidate = dt >= (place_r + clear)
        return roi_rect, occ_dil, dt, candidate, place_r

    def _detect_best_in_frame(self):
        """현재 프레임에서 best 빈자리 후보 1개를 찾아 dict 반환(없으면 None).

        dict: {cx, cy, w, h, angle, score, clearance, debug:(img, occ_dil, candidate, roi_rect)}
        """
        img = self._latest_image
        if img is None:
            self.get_logger().warn('이미지 없음 - detection 불가.')
            return None
        h, w = img.shape[:2]

        roi_rect, occ_dil, dt, candidate, place_r = self._free_and_candidate(img)
        if not np.any(candidate):
            return None

        bcx, bcy, score = self._score_candidates(candidate, dt, w, h, place_r)

        # 자세 우승은 "놓을 수 있는 공간의 양"이 많은 쪽을 선호한다. best 점 하나의
        # 여유(clearance)만으로는 텅 빈 자세와 좁은 틈만 있는 자세를 구분 못하므로,
        # ROI 대비 candidate(초록) 면적 비율을 score 에 더한다(자세 간 비교용 절대값).
        top, bottom, left, right = roi_rect
        roi_area = max((bottom - top) * (right - left), 1)
        area_frac = float(np.count_nonzero(candidate)) / roi_area
        score += float(self.get_parameter('w_area').value) * area_frac

        scale = float(self.get_parameter('bbox_scale').value)
        side = float(np.clip(2.0 * place_r * scale, 8.0, min(w, h)))
        return {
            'cx': float(bcx), 'cy': float(bcy), 'w': side, 'h': side,
            'angle': 0.0, 'score': float(score), 'clearance': float(dt[bcy, bcx]),
            'area_frac': area_frac,
            'debug': (img, occ_dil, candidate, roi_rect),
        }

    def _score_candidates(self, candidate, dt, w, h, place_r):
        rows, cols = np.where(candidate)
        clear = dt[rows, cols]
        # 절대 여유(px)를 고정 기준으로 정규화한다. 프레임별 max 로 나누면(상대 정규화)
        # 모든 자세의 best 가 1.0 으로 평탄화돼 capture 간 "어디가 더 넓은가"를 구분하지
        # 못한다. thr(=후보 최소 여유) 에서 0, ref 에서 1.0 으로 선형, 그 이상은 포화.
        clear_px = float(self.get_parameter('clearance_px').value)
        thr = place_r + clear_px
        ref = float(self.get_parameter('clear_ref_px').value)
        if ref <= 0.0:
            ref = 2.0 * thr
        span = max(ref - thr, 1.0)
        clear_norm = np.clip((clear - thr) / span, 0.0, 1.0)
        center_norm = 1.0 - np.abs(cols - w * 0.5) / (w * 0.5)
        front_norm = rows / float(max(h - 1, 1))

        w_clear = float(self.get_parameter('w_clear').value)
        w_center = float(self.get_parameter('w_center').value)
        w_front = float(self.get_parameter('w_front').value)
        score = w_clear * clear_norm + w_center * center_norm + w_front * front_norm

        idx = int(np.argmax(score))
        return int(cols[idx]), int(rows[idx]), float(score[idx])

    # ------------------------------------------------------------ plan(누적 최적화)

    def _plan_and_publish(self):
        scored = [(i, c) for i, c in enumerate(self._captures) if c is not None]
        if not scored:
            self.get_logger().warn(
                '스윕 전체에서 빈자리 후보 없음 - found=0 발행(상위 재스캔 필요).')
            self._publish_result(found=False)
            return

        win_idx, win = max(scored, key=lambda t: t[1]['score'])
        self._publish_result(
            found=True, cx=win['cx'], cy=win['cy'], w=win['w'], h=win['h'],
            angle=win['angle'], capture_index=win_idx, score=win['score'])
        self._publish_debug(win['debug'], (win['cx'], win['cy']))
        self.get_logger().info(
            f"빈자리 최종 선정: capture #{win_idx}, center=({win['cx']:.0f},{win['cy']:.0f}), "
            f"score={win['score']:.3f} (후보 {len(scored)}/{len(self._captures)} 자세).")

    # ------------------------------------------------------------ publish

    def _publish_result(self, found, cx=0.0, cy=0.0, w=0.0, h=0.0,
                        angle=0.0, capture_index=-1, score=0.0):
        msg = Float64MultiArray()
        msg.data = [
            1.0 if found else 0.0,
            float(cx), float(cy), float(w), float(h), float(angle),
            float(capture_index), float(score),
        ]
        self._pub_bbox.publish(msg)

    def _draw_overlay(self, img, occ_dil, candidate, roi_rect, best, edges=None):
        """디버그 오버레이 렌더: 점유(빨강)/후보(초록)/ROI(노랑)/best(노란원)/Canny(시안)."""
        top, bottom, left, right = roi_rect
        vis = img.copy()
        red = np.zeros_like(vis)
        red[occ_dil > 0] = (0, 0, 200)
        vis = cv2.addWeighted(vis, 1.0, red, 0.4, 0)
        green = np.zeros_like(vis)
        green[candidate] = (0, 200, 0)
        vis = cv2.addWeighted(vis, 1.0, green, 0.25, 0)
        if edges is not None:
            vis[edges > 0] = (255, 255, 0)  # Canny 엣지 시안
        cv2.rectangle(vis, (left, top), (right, bottom), (255, 255, 0), 1)
        if best is not None:
            r = int(self.get_parameter('place_radius_px').value)
            cv2.circle(vis, (int(best[0]), int(best[1])), 6, (0, 255, 255), -1)
            cv2.circle(vis, (int(best[0]), int(best[1])), r, (0, 255, 255), 2)
        return vis

    def _publish_debug(self, debug, best):
        if self._pub_debug.get_subscription_count() == 0:
            return
        img, occ_dil, candidate, roi_rect = debug
        vis = self._draw_overlay(img, occ_dil, candidate, roi_rect, best)
        self._pub_debug.publish(bgr_to_imgmsg(vis))

    def _publish_live_debug(self):
        """트리거 없이 최신 프레임에 detection 을 돌려 디버그 영상을 계속 발행(검증/튜닝용)."""
        img = self._latest_image
        if img is None:
            return
        h, w = img.shape[:2]
        edges = self._canny_edges(img)
        if self._pub_edges.get_subscription_count() > 0:
            self._pub_edges.publish(bgr_to_imgmsg(cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)))
        if self._pub_debug.get_subscription_count() == 0:
            return
        roi_rect, occ_dil, dt, candidate, place_r = self._free_and_candidate(img)
        best = None
        if np.any(candidate):
            bcx, bcy, _ = self._score_candidates(candidate, dt, w, h, place_r)
            best = (bcx, bcy)
        vis = self._draw_overlay(img, occ_dil, candidate, roi_rect, best, edges=edges)
        n_obj = len(self._latest_objects)
        cv2.putText(vis, f'objects={n_obj}  candidate={"Y" if best else "N"}',
                    (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        self._pub_debug.publish(bgr_to_imgmsg(vis))


def main(args=None):
    rclpy.init(args=args)
    node = EmptySlotDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
