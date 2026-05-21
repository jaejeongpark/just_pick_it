import os

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener
from transforms3d.quaternions import mat2quat, quat2mat

_DEFAULT_CALIB = os.path.join(
    get_package_share_directory('just_pick_it_perception'),
    'result',
    'camera_calibration.yaml',
)


class AprilTagDetector(Node):

    def __init__(self):
        super().__init__('apriltag_detector')

        self.declare_parameter('calibration_file', _DEFAULT_CALIB)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('annotated_topic', '/apriltag/image_annotated')
        self.declare_parameter('tag_size_m', 0.12)
        self.declare_parameter('camera_frame', 'front_camera_link')
        self.declare_parameter('base_frame', 'base_link')

        calib_path = self.get_parameter('calibration_file').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        annotated_topic = self.get_parameter('annotated_topic').get_parameter_value().string_value
        tag_half = self.get_parameter('tag_size_m').get_parameter_value().double_value / 2.0
        self._camera_frame = self.get_parameter('camera_frame').get_parameter_value().string_value
        self._base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        # solvePnP용 태그 모서리 3D 좌표 (태그 로컬 프레임, z=0 평면)
        # 순서: TL, TR, BR, BL  (cv2.aruco.detectMarkers 코너 순서와 동일)
        self._obj_pts = np.array([
            [-tag_half,  tag_half, 0.0],
            [ tag_half,  tag_half, 0.0],
            [ tag_half, -tag_half, 0.0],
            [-tag_half, -tag_half, 0.0],
        ], dtype=np.float64)

        self.K, self.D = self._load_calibration(calib_path)
        self.get_logger().info(f'calibration loaded: K=\n{self.K}\nD={self.D}')

        self._map1 = None
        self._map2 = None

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        # static transform 캐시 — 성공 시 한 번만 lookup
        self._T_camera_link_base_link: np.ndarray | None = None
        self._T_map_marker: dict[int, np.ndarray] = {}

        self._build_detector()
        self.bridge = CvBridge()
        self.annotated_pub = self.create_publisher(Image, annotated_topic, 1)
        self._pose_pub = self.create_publisher(PoseStamped, '/apriltag/robot_pose', 1)
        self.create_subscription(Image, self.image_topic, self._image_cb, 5)

        self.get_logger().info(
            f'subscribing to {self.image_topic}, '
            f'publishing annotated to {annotated_topic}'
        )

        self.pose_est_check = 0
    # ------------------------------------------------------------------
    # 초기화 헬퍼
    # ------------------------------------------------------------------

    def _load_calibration(self, path: str):
        if not os.path.isfile(path):
            raise FileNotFoundError(f'calibration_file not found: {path}')
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
        K = np.array(data['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
        D = np.array(data['distortion_coefficients']['data'], dtype=np.float64).reshape(-1)
        return K, D

    def _build_detector(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            params = cv2.aruco.DetectorParameters()
            detector = cv2.aruco.ArucoDetector(dictionary, params)
            self._detect = detector.detectMarkers
        else:
            self._dictionary = dictionary
            self._params = cv2.aruco.DetectorParameters_create()
            self._detect = lambda gray: cv2.aruco.detectMarkers(
                gray, self._dictionary, parameters=self._params)

    def _build_undistort_maps(self, h: int, w: int):
        new_K, _ = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w, h), alpha=1)
        self._map1, self._map2 = cv2.initUndistortRectifyMap(
            self.K, self.D, None, new_K, (w, h), cv2.CV_16SC2)
        self._new_K = new_K
        self.get_logger().info(f'undistort maps built for {w}x{h}\nnew_K=\n{new_K}')

    @staticmethod
    def _get_T_camera_link_camera_optical() -> np.ndarray:
        # camera_link  : x forward, y left,  z up
        # camera_optical: x right,  y down,  z forward (OpenCV convention)
        # T_camera_link_camera_optical transforms points from camera_optical to camera_link
        R = np.array([
            [ 0,  0,  1],
            [-1,  0,  0],
            [ 0, -1,  0],
        ], dtype=np.float64)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        return T

    # ------------------------------------------------------------------
    # TF 헬퍼
    # ------------------------------------------------------------------

    def _lookup_tf_matrix(self, parent: str, child: str) -> np.ndarray | None:
        try:
            tf_stamped = self._tf_buffer.lookup_transform(
                parent, child, rclpy.time.Time()
            )
            return self._tf_stamped_to_matrix(tf_stamped)
        except Exception as e:
            self.get_logger().warn(f'TF lookup {parent} -> {child} 실패: {e}')
            return None

    @staticmethod
    def _tf_stamped_to_matrix(tf_stamped) -> np.ndarray:
        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = quat2mat([q.w, q.x, q.y, q.z])
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    # ------------------------------------------------------------------
    # 콜백
    # ------------------------------------------------------------------

    def _image_cb(self, msg: Image):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge 변환 실패: {e}')
            return

        h, w = cv_img.shape[:2]
        if self._map1 is None:
            self._build_undistort_maps(h, w)

        undistorted = cv2.remap(cv_img, self._map1, self._map2, cv2.INTER_LINEAR)
        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self._detect(gray)

        annotated = undistorted.copy()
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
            # self.get_logger().info(
            #     f'detected tag ids: {ids.flatten().tolist()}, count={len(ids)}'
            # )
            self._estimate_poses(corners, ids, msg.header)

        out = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        out.header = msg.header
        self.annotated_pub.publish(out)

    def _estimate_poses(self, corners, ids, header):
        K = self._new_K
        D = np.zeros(5, dtype=np.float64)

        T_camera_link_camera_optical = self._get_T_camera_link_camera_optical()

        

        # static transform: 성공할 때까지 매 프레임 재시도, 이후 캐시 사용
        if self._T_camera_link_base_link is None:
            result = self._lookup_tf_matrix(self._camera_frame, self._base_frame)
            if result is not None:
                self._T_camera_link_base_link = result
                # self.get_logger().info(
                #     f'T_{self._camera_frame}_{self._base_frame} cached:\n{np.round(result, 4)}'
                # )

        for i, tag_id in enumerate(ids.flatten()):
            if tag_id not in self._T_map_marker:
                result = self._lookup_tf_matrix('map', f'apriltag_{tag_id}')
                if result is not None:
                    self._T_map_marker[tag_id] = result
                    # self.get_logger().info(
                    #     f'[tag {tag_id}] T_map_marker cached:\n{np.round(result, 4)}'
                    # )

            T_map_marker = self._T_map_marker.get(tag_id)
            T_camera_link_base_link = self._T_camera_link_base_link

            if T_map_marker is None or T_camera_link_base_link is None:
                self.get_logger().warn(
                    f'[tag {tag_id}] TF 미준비 — '
                    f'T_map_marker={T_map_marker is not None}, '
                    f'T_camera_link_base_link={T_camera_link_base_link is not None}'
                )
                continue

            else:
                if self.pose_est_check == 0:
                    self.get_logger().info(
                            f'[tag {tag_id}] TF 준비 완료! — '
                            f'T_map_marker={T_map_marker is not None}, '
                            f'T_camera_link_base_link={T_camera_link_base_link is not None}'
                        )
                    self.pose_est_check = 1

                img_pts = corners[i].reshape(4, 2)
                ok, rvec, tvec = cv2.solvePnP(
                    self._obj_pts, img_pts, K, D,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                )
                if not ok:
                    self.get_logger().warn(f'[tag {tag_id}] solvePnP 실패')
                    continue

                R, _ = cv2.Rodrigues(rvec)
                T_camera_optical_marker = np.eye(4, dtype=np.float64)
                T_camera_optical_marker[:3, :3] = R
                T_camera_optical_marker[:3, 3] = tvec.flatten()

                T_camera_link_marker = T_camera_link_camera_optical @ T_camera_optical_marker

                # T_map_base_link = T_map_marker @ T_marker_camera_link @ T_camera_link_base_link
                T_map_base_link = (
                    T_map_marker
                    @ np.linalg.inv(T_camera_link_marker)
                    @ T_camera_link_base_link
                )

            # self.get_logger().info(
            #     f'[tag {tag_id}] T_camera_optical_marker:\n{np.round(T_camera_optical_marker, 4)}\n'
            #     f'[tag {tag_id}] T_map_base_link:\n{np.round(T_map_base_link, 4)}'
            # )

            self._publish_robot_pose(T_map_base_link, header)

    def _publish_robot_pose(self, T_map_base_link: np.ndarray, header):
        w, qx, qy, qz = mat2quat(T_map_base_link[:3, :3])
        msg = PoseStamped()
        msg.header.stamp = header.stamp
        msg.header.frame_id = 'map'
        msg.pose.position.x = T_map_base_link[0, 3]
        msg.pose.position.y = T_map_base_link[1, 3]
        msg.pose.position.z = T_map_base_link[2, 3]
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = w
        self._pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
