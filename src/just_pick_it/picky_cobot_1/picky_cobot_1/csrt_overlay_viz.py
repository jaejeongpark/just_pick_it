#!/usr/bin/env python3
"""CSRT 빈자리 추적 시각화 overlay 노드 (디버그 전용, perception 무수정).

csrt_place_tracker 가 내보내는 /place/tracked_objects(추적 중 bbox)와 cobot_controller 가
발행한 /place/target_bbox(최초 init bbox)를 라이브 프레임(/infer/image_raw)에 그려
/place/csrt_overlay(Image)로 재발행한다. rqt_image_view 로 실시간 확인용.

  rqt: ros2 run rqt_image_view rqt_image_view /place/csrt_overlay

perception 패키지(csrt_place_tracker 등)는 건드리지 않고, 토픽만 구독해 합성한다.
이미지 변환은 numpy2 환경 cv_bridge segfault 회피를 위해 cv_image_utils 를 쓴다.

구독:
  /infer/image_raw       (Image)              : 라이브 프레임
  /place/tracked_objects (TrackedObjectArray) : 유도된 빈자리 bbox(center 기준, 초록)
  /place/target_bbox     (Float64MultiArray)  : 최초 init bbox [cx,cy,w,h,angle] (노랑)
  /place/context_bbox    (Float64MultiArray)  : CSRT 가 실제 추적하는 context 박스 [cx,cy,w,h] (시안)
발행:
  /place/csrt_overlay    (Image)              : 위를 입힌 시각화 프레임
"""
import math

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray

from just_pick_it_interfaces.msg import TrackedObjectArray

from just_pick_it_perception.cv_image_utils import bgr_to_imgmsg, imgmsg_to_bgr


COLOR_TRACK = (0, 255, 0)      # 초록: 유도된 빈자리 bbox
COLOR_TRACK_STALE = (0, 0, 255)  # 빨강: 추적 끊김(오래된 데이터)
COLOR_TARGET = (0, 255, 255)   # 노랑: 최초 init bbox
COLOR_CONTEXT = (255, 255, 0)  # 시안: CSRT 가 실제 추적하는 context 박스
COLOR_TEXT = (255, 255, 255)


def _latched_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        depth=depth,
    )


class CsrtOverlayViz(Node):

    def __init__(self):
        super().__init__('csrt_overlay_viz')
        # 추적 데이터가 이 시간보다 오래되면 'LOST'(끊김)로 표시.
        self.declare_parameter('track_stale_sec', 0.5)
        self._stale_sec = float(self.get_parameter('track_stale_sec').value)

        self._latest_track = None       # (cx, cy, w, h, angle, conf, label)
        self._latest_track_time = None
        self._target_bbox = None        # (cx, cy, w, h, angle)
        self._context_bbox = None       # (cx, cy, w, h) CSRT 가 추적하는 context 박스

        self.create_subscription(Image, 'infer/image_raw', self._image_cb, 5)
        self.create_subscription(
            TrackedObjectArray, '/place/tracked_objects', self._track_cb, 10)
        self.create_subscription(
            Float64MultiArray, '/place/target_bbox', self._target_cb, _latched_qos())
        self.create_subscription(
            Float64MultiArray, '/place/context_bbox', self._context_cb, 10)

        self._pub = self.create_publisher(Image, '/place/csrt_overlay', 5)

        self.get_logger().info(
            'CsrtOverlayViz 준비. rqt_image_view 로 /place/csrt_overlay 확인.')

    # ── 콜백 ─────────────────────────────────────────────────────────────

    def _track_cb(self, msg: TrackedObjectArray):
        if not msg.objects:
            return
        o = msg.objects[0]
        self._latest_track = (
            float(o.bbox_x), float(o.bbox_y), float(o.bbox_w), float(o.bbox_h),
            float(o.orientation_angle), float(o.confidence), str(o.class_label),
        )
        self._latest_track_time = self.get_clock().now()

    def _target_cb(self, msg: Float64MultiArray):
        d = list(msg.data)
        if len(d) < 4:
            return
        angle = float(d[4]) if len(d) >= 5 else 0.0
        self._target_bbox = (float(d[0]), float(d[1]), float(d[2]), float(d[3]), angle)

    def _context_cb(self, msg: Float64MultiArray):
        d = list(msg.data)
        if len(d) < 4:
            return
        self._context_bbox = (float(d[0]), float(d[1]), float(d[2]), float(d[3]))

    def _image_cb(self, msg: Image):
        # 구독자가 없으면 합성 비용을 들이지 않는다(rqt 등이 붙을 때만 발행).
        if self._pub.get_subscription_count() == 0:
            return
        frame = imgmsg_to_bgr(msg)
        img_h, img_w = frame.shape[:2]

        # 0) CSRT 가 실제 추적하는 context 박스(시안) — drift 진단용. 가장 큰 박스라 먼저 그림.
        if self._context_bbox is not None:
            cx, cy, bw, bh = self._context_bbox
            self._draw_box(frame, cx, cy, bw, bh, 0.0, COLOR_CONTEXT, 'context(CSRT)')

        # 1) 최초 init bbox(노랑) — 어디를 잡았는지 기준선.
        if self._target_bbox is not None:
            cx, cy, bw, bh, angle = self._target_bbox
            self._draw_box(frame, cx, cy, bw, bh, angle, COLOR_TARGET, 'init')

        # 2) 현재 CSRT 추적 bbox(초록=실시간 / 빨강=끊김).
        if self._latest_track is not None:
            cx, cy, bw, bh, angle, conf, label = self._latest_track
            stale = self._is_stale()
            color = COLOR_TRACK_STALE if stale else COLOR_TRACK
            area_norm = (bw * bh) / float(img_w * img_h)
            tag = (f'{label} {"LOST" if stale else "TRACK"} '
                   f'area={area_norm:.3f} conf={conf:.2f}')
            self._draw_box(frame, cx, cy, bw, bh, angle, color, tag)

        # 하단 HUD: 추적 상태 한 줄.
        self._draw_hud(frame, img_w, img_h)
        self._pub.publish(bgr_to_imgmsg(frame, header=msg.header))

    # ── 그리기 유틸 ──────────────────────────────────────────────────────

    def _draw_box(self, frame, cx, cy, bw, bh, angle_deg, color, label):
        x1 = int(round(cx - bw / 2.0))
        y1 = int(round(cy - bh / 2.0))
        x2 = int(round(cx + bw / 2.0))
        y2 = int(round(cy + bh / 2.0))
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.circle(frame, (int(round(cx)), int(round(cy))), 3, color, -1)

        # orientation_angle 시각화: center 에서 장축 방향 헤딩 선.
        if abs(angle_deg) > 1e-3:
            rad = math.radians(angle_deg)
            length = max(bw, bh) / 2.0
            hx = int(round(cx + length * math.cos(rad)))
            hy = int(round(cy + length * math.sin(rad)))
            cv2.line(frame, (int(round(cx)), int(round(cy))), (hx, hy), color, 2)

        ty = y1 - 6 if y1 - 6 > 10 else y2 + 16
        cv2.putText(frame, label, (x1, ty),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    def _draw_hud(self, frame, img_w, img_h):
        if self._latest_track is None:
            text = 'CSRT: 추적 데이터 없음 (target_bbox 대기 / init 전)'
        elif self._is_stale():
            text = 'CSRT: LOST (추적 끊김 — 발행 중단됨)'
        else:
            text = 'CSRT: TRACKING'
        cv2.putText(frame, text, (8, img_h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, COLOR_TEXT, 1, cv2.LINE_AA)

    def _is_stale(self) -> bool:
        if self._latest_track_time is None:
            return True
        age = (self.get_clock().now() - self._latest_track_time).nanoseconds * 1e-9
        return age > self._stale_sec


def main(args=None):
    rclpy.init(args=args)
    node = CsrtOverlayViz()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
