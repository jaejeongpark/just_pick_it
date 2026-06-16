import socket
import struct

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from just_pick_it_interfaces.msg import TrackedObject, TrackedObjectArray
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header

_HEADER_FMT = '>IHH'
_HEADER_SIZE = struct.calcsize(_HEADER_FMT)
_RECV_BUF = 65536


class DetectionTrackerNode(Node):
    """YOLOv8-seg + ByteTrack 통합 노드.

    N프레임 이상 연속으로 추적된 track만 발행한다.
    """

    def __init__(self):
        super().__init__('detection_tracker_node')

        self._declare_parameters()

        self._bridge = CvBridge()
        self._model = self._load_model()

        self._track_first_seen: dict[int, int] = {}
        self._frame_counter = 0
        self._udp_frames: dict[int, dict[int, bytes]] = {}

        self._pub_objects = self.create_publisher(
            TrackedObjectArray, 'detection/tracked_objects', 10)
        self._pub_annotated = self.create_publisher(
            Image, 'detection/annotated_image', 10)

        udp_port = self.get_parameter('udp_port').value
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(('', udp_port))
        self._sock.setblocking(False)

        self.create_timer(1.0 / 30.0, self._image_cb)

        self.get_logger().info(
            f'DetectionTrackerNode ready — model: '
            f'{self.get_parameter("model_path").value}, '
            f'UDP port: {udp_port}'
        )

    # ------------------------------------------------------------------ setup

    def _declare_parameters(self):
        self.declare_parameter('model_path', 'yolov8n-seg.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('target_classes', [''])
        self.declare_parameter('consistency_frames', 5)
        self.declare_parameter('udp_port', 9870)
        self.declare_parameter('frame_id', 'camera_link')

    def _load_model(self):
        try:
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().fatal('ultralytics 패키지가 없습니다. pip install ultralytics')
            raise

        model_path = self.get_parameter('model_path').value
        model = YOLO(model_path)
        self.get_logger().info(f'YOLO 모델 로드: {model_path}, 클래스: {model.names}')

        target_classes_param = [
            c for c in self.get_parameter('target_classes').value if c
        ]
        if target_classes_param:
            name_to_id = {v: k for k, v in model.names.items()}
            self._classes_filter = [
                name_to_id[c] for c in target_classes_param if c in name_to_id
            ]
            missing = [c for c in target_classes_param if c not in name_to_id]
            if missing:
                self.get_logger().warn(f'모델에 없는 클래스: {missing}')
        else:
            self._classes_filter = None

        return model

    # ---------------------------------------------------------------- callback

    def _image_cb(self):
        frame = self._recv_frame()
        if frame is None:
            return

        self._frame_counter += 1
        stamp = self.get_clock().now().to_msg()
        frame_id = self.get_parameter('frame_id').value
        header = Header(stamp=stamp, frame_id=frame_id)

        conf = self.get_parameter('confidence').value
        iou = self.get_parameter('iou_threshold').value
        classes = self._classes_filter

        results = self._model.track(
            frame,
            persist=True,
            conf=conf,
            iou=iou,
            classes=classes,
            tracker='bytetrack.yaml',
            verbose=False,
        )

        consistency = self.get_parameter('consistency_frames').value
        tracked_array = TrackedObjectArray()
        tracked_array.header = header

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

                obj = self._build_tracked_object(
                    box, masks, i, track_id, frame_count, r.names, header,
                )
                tracked_array.objects.append(obj)

        self._pub_objects.publish(tracked_array)

        annotated = results[0].plot() if results else frame
        annotated_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        annotated_msg.header = header
        self._pub_annotated.publish(annotated_msg)

        self._cleanup_stale_tracks(boxes)

    def _recv_frame(self):
        """소켓에서 패킷을 드레인하며 완성된 첫 프레임을 반환한다."""
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

    # --------------------------------------------------------------- helpers

    def _build_tracked_object(self, box, masks, mask_idx, track_id, frame_count, names, header):
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

        if masks is not None and mask_idx < len(masks.xy):
            pts = masks.xy[mask_idx]
            if len(pts) > 0:
                pts_arr = np.array(pts)
                obj.mask_cx = float(pts_arr[:, 0].mean())
                obj.mask_cy = float(pts_arr[:, 1].mean())
                obj.orientation_angle = DetectionTrackerNode._compute_obb_angle(pts_arr)
            else:
                obj.mask_cx = cx
                obj.mask_cy = cy
                obj.orientation_angle = 0.0
        else:
            obj.mask_cx = cx
            obj.mask_cy = cy
            obj.orientation_angle = 0.0

        obj.pose_valid = False
        return obj

    @staticmethod
    def _compute_obb_angle(pts: np.ndarray) -> float:
        """세그멘테이션 마스크 폴리곤에서 OBB 장축 각도를 계산한다.

        Returns:
            장축 각도 (deg). 0=수평, +90=수직, 범위 [-90, 90).
        """
        if len(pts) < 3:
            return 0.0
        rect = cv2.minAreaRect(np.array(pts, dtype=np.float32))
        w, h = rect[1]
        angle = rect[2]       # OpenCV 반환값: (-90, 0]
        if w < h:
            angle += 90.0     # 장축이 수직에 가까운 경우 보정
        return float(angle)

    def _cleanup_stale_tracks(self, boxes):
        if boxes is None or len(boxes) == 0:
            self._track_first_seen.clear()
            return

        active_ids = set()
        for box in boxes:
            if box.id is not None:
                active_ids.add(int(box.id.item()))

        stale = [tid for tid in self._track_first_seen if tid not in active_ids]
        for tid in stale:
            del self._track_first_seen[tid]


def main(args=None):
    rclpy.init(args=args)
    node = DetectionTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
