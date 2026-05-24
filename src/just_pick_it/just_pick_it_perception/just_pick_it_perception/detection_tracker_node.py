import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

from just_pick_it_interfaces.msg import TrackedObject, TrackedObjectArray

import numpy as np


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

        self._pub_objects = self.create_publisher(
            TrackedObjectArray, 'detection/tracked_objects', 10)
        self._pub_annotated = self.create_publisher(
            Image, 'detection/annotated_image', 10)

        image_topic = self.get_parameter('image_topic').value
        self.create_subscription(Image, image_topic, self._image_cb, 10)

        self.get_logger().info(
            f'DetectionTrackerNode ready — model: '
            f'{self.get_parameter("model_path").value}, '
            f'topic: {image_topic}'
        )

    # ------------------------------------------------------------------ setup

    def _declare_parameters(self):
        self.declare_parameter('model_path', 'yolov8n-seg.pt')
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('target_classes', rclpy.Parameter.Type.STRING_ARRAY)
        self.declare_parameter('consistency_frames', 5)
        self.declare_parameter('image_topic', '/camera_head/color/image_raw')

    def _load_model(self):
        try:
            from ultralytics import YOLO
        except ImportError:
            self.get_logger().fatal('ultralytics 패키지가 없습니다. pip install ultralytics')
            raise

        model_path = self.get_parameter('model_path').value
        model = YOLO(model_path)
        self.get_logger().info(f'YOLO 모델 로드: {model_path}')
        return model

    # ---------------------------------------------------------------- callback

    def _image_cb(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._frame_counter += 1

        conf = self.get_parameter('confidence').value
        iou = self.get_parameter('iou_threshold').value
        target_classes_param = self.get_parameter('target_classes').value
        classes = target_classes_param if target_classes_param else None

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
        tracked_array.header = Header(
            stamp=msg.header.stamp,
            frame_id=msg.header.frame_id,
        )

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
                    box, masks, i, track_id, frame_count, r.names, msg.header,
                )
                tracked_array.objects.append(obj)

        self._pub_objects.publish(tracked_array)

        annotated = results[0].plot() if results else frame
        annotated_msg = self._bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        annotated_msg.header = msg.header
        self._pub_annotated.publish(annotated_msg)

        self._cleanup_stale_tracks(boxes if (results and results[0].boxes is not None) else None)

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
            else:
                obj.mask_cx = cx
                obj.mask_cy = cy
        else:
            obj.mask_cx = cx
            obj.mask_cy = cy

        obj.pose_valid = False
        return obj

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
