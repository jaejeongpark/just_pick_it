import math
import yaml

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from cv_bridge import CvBridge
import tf2_ros

from just_pick_it_interfaces.msg import TrackedObject, TrackedObjectArray
from just_pick_it_interfaces.srv import SelectTarget


class TargetManagerNode(Node):
    """AprilTag 카테고리 검출 + 타겟 선택 서비스 + 선반 높이 기반 3D 위치 추정 노드."""

    def __init__(self):
        super().__init__('target_manager_node')

        self._declare_parameters()

        self._bridge = CvBridge()
        self._K = None
        self._tag_category_map: dict[int, str] = {}
        self._active_track_id: int = -1
        self._current_objects: list[TrackedObject] = []
        self._current_image: np.ndarray | None = None

        self._load_camera_calibration()
        self._load_apriltag_poses()

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._pub_target = self.create_publisher(
            TrackedObject, 'detection/target', 10)
        self._pub_category = self.create_publisher(
            String, 'detection/category', 10)

        self.create_subscription(
            TrackedObjectArray, 'detection/tracked_objects', self._tracked_cb, 10)

        image_topic = self.get_parameter('image_topic').value
        self.create_subscription(Image, image_topic, self._image_cb, 10)

        self.create_service(SelectTarget, 'detection/select_target', self._select_target_cb)

        self.create_timer(0.05, self._publish_target)

        self.get_logger().info('TargetManagerNode 준비 완료')

    # ------------------------------------------------------------------ setup

    def _declare_parameters(self):
        self.declare_parameter('camera_calibration_file', '')
        self.declare_parameter('shelf_height_z', 0.15)
        self.declare_parameter('apriltag_poses_file', '')
        self.declare_parameter('camera_frame', 'camera_link')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('image_topic', '/camera_head/color/image_raw')
        self.declare_parameter('apriltag_tag_size', 0.15)

    def _load_camera_calibration(self):
        path = self.get_parameter('camera_calibration_file').value
        if not path:
            self.get_logger().warn('camera_calibration_file 파라미터가 비어 있습니다. 3D 추정 비활성화.')
            return
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            k = data['camera_matrix']['data']
            self._K = np.array(k, dtype=np.float64).reshape(3, 3)
            self.get_logger().info(f'카메라 캘리브레이션 로드: {path}')
        except Exception as e:
            self.get_logger().error(f'캘리브레이션 파일 로드 실패: {e}')

    def _load_apriltag_poses(self):
        path = self.get_parameter('apriltag_poses_file').value
        if not path:
            self.get_logger().warn('apriltag_poses_file 파라미터가 비어 있습니다.')
            return
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            for tag_id, info in data.get('tags', {}).items():
                self._tag_category_map[int(tag_id)] = info.get('group', f'TAG_{tag_id}')
            self.get_logger().info(f'AprilTag 카테고리 맵 로드: {self._tag_category_map}')
        except Exception as e:
            self.get_logger().error(f'apriltag_poses_file 로드 실패: {e}')

    # --------------------------------------------------------------- callbacks

    def _image_cb(self, msg: Image):
        self._current_image = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._detect_apriltag_category()

    def _tracked_cb(self, msg: TrackedObjectArray):
        self._current_objects = list(msg.objects)

    def _select_target_cb(self, request: SelectTarget.Request, response: SelectTarget.Response):
        if not self._current_objects:
            response.success = False
            response.selected_track_id = -1
            response.message = '현재 추적 중인 객체가 없습니다.'
            return response

        if request.track_id >= 0:
            ids = [o.track_id for o in self._current_objects]
            if request.track_id in ids:
                self._active_track_id = request.track_id
                response.success = True
                response.selected_track_id = request.track_id
                response.message = f'track_id {request.track_id} 선택됨'
            else:
                response.success = False
                response.selected_track_id = -1
                response.message = f'track_id {request.track_id}를 찾을 수 없습니다.'
        else:
            best = self._select_nearest_to_center()
            if best is not None:
                self._active_track_id = best.track_id
                response.success = True
                response.selected_track_id = best.track_id
                response.message = f'화면 중심 기준 자동 선택: track_id {best.track_id}'
            else:
                response.success = False
                response.selected_track_id = -1
                response.message = '자동 선택 실패'

        self.get_logger().info(response.message)
        return response

    # --------------------------------------------------------------- detection

    def _detect_apriltag_category(self):
        if self._current_image is None:
            return

        gray = cv2.cvtColor(self._current_image, cv2.COLOR_BGR2GRAY)
        aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        params = cv2.aruco.DetectorParameters()
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is None:
            return

        detected_categories = []
        for tag_id in ids.flatten():
            cat = self._tag_category_map.get(int(tag_id))
            if cat:
                detected_categories.append(cat)

        if detected_categories:
            msg = String()
            msg.data = ','.join(set(detected_categories))
            self._pub_category.publish(msg)

    # --------------------------------------------------------------- publish

    def _publish_target(self):
        if self._active_track_id < 0 or not self._current_objects:
            return

        target = next(
            (o for o in self._current_objects if o.track_id == self._active_track_id), None
        )
        if target is None:
            return

        target_with_pose = TrackedObject()
        target_with_pose.header = target.header
        target_with_pose.track_id = target.track_id
        target_with_pose.class_label = target.class_label
        target_with_pose.class_id = target.class_id
        target_with_pose.confidence = target.confidence
        target_with_pose.bbox_x = target.bbox_x
        target_with_pose.bbox_y = target.bbox_y
        target_with_pose.bbox_w = target.bbox_w
        target_with_pose.bbox_h = target.bbox_h
        target_with_pose.mask_cx = target.mask_cx
        target_with_pose.mask_cy = target.mask_cy
        target_with_pose.frame_count = target.frame_count

        pose, valid = self._estimate_3d_pose(target.mask_cx, target.mask_cy, target.header)
        target_with_pose.estimated_pose = pose
        target_with_pose.pose_valid = valid

        self._pub_target.publish(target_with_pose)

    # --------------------------------------------------------------- 3D pose

    def _estimate_3d_pose(self, u: float, v: float, header) -> tuple[PoseStamped, bool]:
        pose = PoseStamped()
        pose.header.frame_id = self.get_parameter('world_frame').value
        pose.pose.orientation = Quaternion(w=1.0)

        if self._K is None:
            return pose, False

        camera_frame = self.get_parameter('camera_frame').value
        world_frame = self.get_parameter('world_frame').value
        shelf_z = self.get_parameter('shelf_height_z').value

        try:
            tf = self._tf_buffer.lookup_transform(
                world_frame, camera_frame, rclpy.time.Time())
        except Exception:
            return pose, False

        t = tf.transform.translation
        q = tf.transform.rotation
        R = self._quat_to_rotation_matrix(q.x, q.y, q.z, q.w)
        p_cam = np.array([t.x, t.y, t.z])

        fx = self._K[0, 0]
        fy = self._K[1, 1]
        cx = self._K[0, 2]
        cy = self._K[1, 2]
        d_cam = np.array([(u - cx) / fx, (v - cy) / fy, 1.0])
        d_cam /= np.linalg.norm(d_cam)
        d_world = R @ d_cam

        if abs(d_world[2]) < 0.01:
            return pose, False

        t_param = (shelf_z - p_cam[2]) / d_world[2]
        if t_param < 0:
            return pose, False

        p_world = p_cam + t_param * d_world

        pose.header.stamp = header.stamp
        pose.pose.position = Point(x=float(p_world[0]), y=float(p_world[1]), z=float(p_world[2]))
        return pose, True

    @staticmethod
    def _quat_to_rotation_matrix(qx, qy, qz, qw) -> np.ndarray:
        R = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
        ])
        return R

    def _select_nearest_to_center(self) -> TrackedObject | None:
        if not self._current_image is None:
            h, w = self._current_image.shape[:2]
            img_cx, img_cy = w / 2.0, h / 2.0
        else:
            img_cx, img_cy = 320.0, 240.0

        best = None
        best_dist = float('inf')
        for obj in self._current_objects:
            dist = math.hypot(obj.mask_cx - img_cx, obj.mask_cy - img_cy)
            if dist < best_dist:
                best_dist = dist
                best = obj
        return best


def main(args=None):
    rclpy.init(args=args)
    node = TargetManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
