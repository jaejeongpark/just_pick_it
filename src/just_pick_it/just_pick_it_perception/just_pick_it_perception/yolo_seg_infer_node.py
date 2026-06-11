"""YOLO-seg 커스텀 모델 실시간 추론 노드 (ByteTrack + OBB 시각화)."""

import socket
import struct

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from just_pick_it_interfaces.msg import TrackedObject, TrackedObjectArray
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header

_HEADER_FMT = '>IHH'
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)
_RECV_BUF = 65536


class YoloSegInferNode(Node):
    """커스텀 학습 YOLO-seg 모델 + ByteTrack + OBB 시각화 노드.

    result/jetcobot_1/best.pt 를 기본 모델로 로드한다.
    UDP로 수신한 프레임에 추론 후 annotated 이미지와 TrackedObjectArray를 발행한다.
    """

    def __init__(self):
        super().__init__('yolo_seg_infer_node')

        self._declare_parameters()
        self._bridge = CvBridge()
        self._model = self._load_model()

        self._track_first_seen: dict[int, int] = {}
        self._frame_counter = 0
        self._udp_frames: dict[int, dict[int, bytes]] = {}

        self._pub_objects = self.create_publisher(
            TrackedObjectArray, 'infer/tracked_objects', 10)
        self._pub_annotated = self.create_publisher(
            Image, 'infer/annotated_image', 10)

        udp_port = self.get_parameter('udp_port').value
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(('', udp_port))
        self._sock.setblocking(False)

        self.create_timer(1.0 / 30.0, self._timer_cb)
        self.get_logger().info(
            f'YoloSegInferNode 준비 완료 — '
            f'model: {self.get_parameter("model_path").value}, '
            f'UDP port: {udp_port}'
        )

    # ------------------------------------------------------------------ setup

    def _declare_parameters(self):
        pkg_share = get_package_share_directory('just_pick_it_perception')
        default_model = f'{pkg_share}/result/jetcobot_1/best.pt'

        self.declare_parameter('model_path', default_model)
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('target_classes', [''])
        self.declare_parameter('consistency_frames', 3)
        self.declare_parameter('udp_port', 9871)
        self.declare_parameter('frame_id', 'camera_link')

    def _load_model(self):
        try:
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().fatal('ultralytics 패키지가 없습니다. pip install ultralytics')
            raise

        model_path = self.get_parameter('model_path').value
        model = YOLO(model_path)
        self.get_logger().info(f'모델 로드 완료: {model_path}')
        self.get_logger().info(f'클래스 목록: {model.names}')

        raw_param = self.get_parameter('target_classes').value
        if isinstance(raw_param, str):
            raw_param = [raw_param]
        target_raw = [
            name.strip()
            for entry in raw_param
            for name in entry.split(',')
            if name.strip()
        ]
        if target_raw:
            name_to_id = {v: k for k, v in model.names.items()}
            self._classes_filter = [name_to_id[c] for c in target_raw if c in name_to_id]
            missing = [c for c in target_raw if c not in name_to_id]
            if missing:
                self.get_logger().warn(f'모델에 없는 클래스 무시: {missing}')
        else:
            self._classes_filter = None

        return model

    # --------------------------------------------------------------- callback

    def _timer_cb(self):
        frame = self._recv_frame()
        if frame is None:
            return

        self._frame_counter += 1
        stamp = self.get_clock().now().to_msg()
        frame_id = self.get_parameter('frame_id').value
        header = Header(stamp=stamp, frame_id=frame_id)

        results = self._model.track(
            frame,
            persist=True,
            conf=self.get_parameter('confidence').value,
            iou=self.get_parameter('iou_threshold').value,
            classes=self._classes_filter,
            tracker='bytetrack.yaml',
            verbose=False,
        )

        consistency = self.get_parameter('consistency_frames').value
        tracked_array = TrackedObjectArray()
        tracked_array.header = header
        annotated = frame.copy()
        boxes = None

        if results and results[0].boxes is not None:
            r = results[0]
            boxes = r.boxes
            masks = r.masks

            for i, box in enumerate(boxes):
                if box.id is None:
                    continue

                track_id = int(box.id.item())
                if track_id not in self._track_first_seen:
                    self._track_first_seen[track_id] = self._frame_counter

                frame_count = self._frame_counter - self._track_first_seen[track_id] + 1
                if frame_count < consistency:
                    continue

                pts = masks.xy[i] if (masks is not None and i < len(masks.xy)) else None
                obj = self._build_tracked_object(box, pts, track_id, frame_count, r.names, header)
                tracked_array.objects.append(obj)
                annotated = self._draw_obb(annotated, box, pts, obj)

        self._pub_objects.publish(tracked_array)

        annotated_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        annotated_msg.header = header
        self._pub_annotated.publish(annotated_msg)

        self._cleanup_stale_tracks(boxes)

    def _recv_frame(self):
        """소켓 패킷을 드레인하며 완성된 첫 프레임을 반환한다."""
        while True:
            try:
                packet, _ = self._sock.recvfrom(_RECV_BUF)
            except BlockingIOError:
                return None

            if len(packet) < _HEADER_SIZE:
                continue

            fid, pkt_idx, total = struct.unpack(_HEADER_FMT, packet[:_HEADER_SIZE])
            self._udp_frames.setdefault(fid, {})[pkt_idx] = packet[_HEADER_SIZE:]

            if len(self._udp_frames[fid]) == total:
                data = b''.join(self._udp_frames[fid][i] for i in range(total))
                del self._udp_frames[fid]

                stale = [f for f in list(self._udp_frames) if f < fid - 30]
                for f in stale:
                    del self._udp_frames[f]

                img = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if img is not None:
                    return img

    # --------------------------------------------------------------- builders

    def _build_tracked_object(self, box, pts, track_id, frame_count, names, header):
        obj = TrackedObject()
        obj.header = header
        obj.track_id = track_id
        obj.frame_count = frame_count

        cls_id = int(box.cls.item())
        obj.class_id = cls_id
        obj.class_label = names[cls_id] if cls_id in names else str(cls_id)
        obj.confidence = float(box.conf.item())

        xyxy = box.xyxy[0].cpu().numpy()
        cx = float((xyxy[0] + xyxy[2]) / 2)
        cy = float((xyxy[1] + xyxy[3]) / 2)
        obj.bbox_x = cx
        obj.bbox_y = cy
        obj.bbox_w = float(xyxy[2] - xyxy[0])
        obj.bbox_h = float(xyxy[3] - xyxy[1])

        if pts is not None and len(pts) > 0:
            pts_arr = np.array(pts)
            obj.mask_cx = float(pts_arr[:, 0].mean())
            obj.mask_cy = float(pts_arr[:, 1].mean())
            obj.orientation_angle = self._compute_obb_angle(pts_arr)
        else:
            obj.mask_cx = cx
            obj.mask_cy = cy
            obj.orientation_angle = 0.0

        obj.pose_valid = False
        return obj

    @staticmethod
    def _compute_obb_angle(pts: np.ndarray) -> float:
        """세그멘테이션 폴리곤에서 OBB 장축 각도를 반환한다 (단위: deg, 범위: [-90, 90))."""
        if len(pts) < 3:
            return 0.0
        rect = cv2.minAreaRect(np.array(pts, dtype=np.float32))
        w, h = rect[1]
        angle = rect[2]
        if w < h:
            angle += 90.0
        return float(angle)

    # ------------------------------------------------------------ visualization

    def _draw_obb(self, frame: np.ndarray, box, pts, obj: TrackedObject) -> np.ndarray:
        """세그멘테이션 마스크 + OBB + 레이블을 프레임에 그린다."""
        color = self._id_to_color(obj.track_id)

        if pts is not None and len(pts) >= 3:
            pts_arr = np.array(pts, dtype=np.int32)

            # 반투명 마스크 채우기
            overlay = frame.copy()
            cv2.fillPoly(overlay, [pts_arr], color)
            cv2.addWeighted(overlay, 0.3, frame, 0.7, 0, frame)

            # 세그멘테이션 외곽선
            cv2.polylines(frame, [pts_arr], True, color, 1)

            # OBB (minAreaRect 회전 박스)
            rect = cv2.minAreaRect(np.array(pts, dtype=np.float32))
            obb_pts = cv2.boxPoints(rect).astype(np.int32)
            cv2.drawContours(frame, [obb_pts], 0, color, 2)

            # 장축 방향 화살표
            cx, cy = int(rect[0][0]), int(rect[0][1])
            angle_rad = np.deg2rad(obj.orientation_angle)
            arrow_len = max(rect[1]) / 2
            ex = int(cx + arrow_len * np.cos(angle_rad))
            ey = int(cy - arrow_len * np.sin(angle_rad))
            cv2.arrowedLine(frame, (cx, cy), (ex, ey), color, 2, tipLength=0.2)

        else:
            xyxy = box.xyxy[0].cpu().numpy().astype(int)
            cv2.rectangle(frame, (xyxy[0], xyxy[1]), (xyxy[2], xyxy[3]), color, 2)

        label = (
            f'#{obj.track_id} {obj.class_label} '
            f'{obj.confidence:.2f} {obj.orientation_angle:.1f}deg'
        )
        lx = int(obj.mask_cx)
        ly = max(int(obj.mask_cy) - 12, 12)
        cv2.putText(frame, label, (lx, ly),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)

        return frame

    @staticmethod
    def _id_to_color(track_id: int) -> tuple[int, int, int]:
        """track_id를 HSV 색공간 기반 고정 색상으로 변환한다."""
        hue = (track_id * 47) % 180
        hsv = np.uint8([[[hue, 220, 220]]])
        bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0]
        return int(bgr[0]), int(bgr[1]), int(bgr[2])

    # ----------------------------------------------------------- housekeeping

    def _cleanup_stale_tracks(self, boxes):
        if boxes is None or len(boxes) == 0:
            self._track_first_seen.clear()
            return
        active_ids = {int(b.id.item()) for b in boxes if b.id is not None}
        stale = [tid for tid in self._track_first_seen if tid not in active_ids]
        for tid in stale:
            del self._track_first_seen[tid]


def main(args=None):
    rclpy.init(args=args)
    node = YoloSegInferNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
