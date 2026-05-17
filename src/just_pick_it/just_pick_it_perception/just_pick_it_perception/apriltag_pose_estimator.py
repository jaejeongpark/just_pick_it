"""
AprilTag 36h11 검출 결과와 DB에 등록된 태그의 절대 월드 자세를 이용해
이동 로봇(pinky_pro) base_footprint의 6DOF를 map 프레임에서 추정한다.

파이프라인 (프레임당):
  1. cv2.aruco로 태그 코너 검출
  2. cv2.solvePnP(SOLVEPNP_IPPE_SQUARE)로 tag-optical -> cam-optical 변환 추정
  3. DB의 tag 월드 자세, 태그 광학/모델 프레임 보정, 카메라 body/optical 보정,
     tf2의 front_camera_link -> base_footprint 변환을 합성해 T_world_base 계산
  4. PoseWithCovarianceStamped로 /robot_pose 발행

사용법:
    ros2 run just_pick_it_perception apriltag_pose_estimator
    ros2 run just_pick_it_perception apriltag_pose_estimator --ros-args \\
        -p calibration_file:=$HOME/camera_calibration.yaml \\
        -p tag_db_file:=/path/to/apriltag_world_poses.yaml \\
        -p use_camera_info:=true
"""

import os
from threading import Lock

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from visualization_msgs.msg import Marker
from tf2_ros import (
    Buffer,
    StaticTransformBroadcaster,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)
from transforms3d.euler import euler2mat, mat2euler
from transforms3d.quaternions import mat2quat


# 태그 광학 좌표(x-right, y-up, z-out-of-page)를 태그 모델 좌표
# (x-normal, y-image-right, z-image-up)로 옮기는 회전.
# 모델 박스가 X축으로 얇으므로 이미지면은 모델 YZ 평면, 법선은 모델 X.
R_TAGMODEL_TAGOPT = np.array(
    [
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ],
    dtype=np.float64,
)

# 카메라 body 좌표(x-forward, y-left, z-up)를 카메라 광학 좌표
# (x-right, y-down, z-forward)로 옮기는 회전. (REP-103)
R_CAMOPT_CAMBODY = np.array(
    [
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
        [1.0, 0.0, 0.0],
    ],
    dtype=np.float64,
)


def make_transform(R, t):
    """3x3 회전과 3x1 평행이동으로 4x4 동차변환행렬을 만든다."""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t).reshape(3)
    return T


def invert_transform(T):
    """동차변환의 빠른 역행렬 (회전+이동 가정)."""
    R = T[:3, :3]
    t = T[:3, 3]
    Tinv = np.eye(4, dtype=np.float64)
    Tinv[:3, :3] = R.T
    Tinv[:3, 3] = -R.T @ t
    return Tinv


def rpy_to_matrix(R, P, Y):
    return euler2mat(R, P, Y, axes='sxyz')


class AprilTagPoseEstimator(Node):

    def __init__(self):
        super().__init__('apriltag_pose_estimator')

        self.declare_parameter('calibration_file', os.path.expanduser('~/camera_calibration.yaml'))
        self.declare_parameter('tag_db_file', '')
        self.declare_parameter('use_camera_info', True)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('world_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('pose_topic', '/robot_pose')
        self.declare_parameter('multi_tag_strategy', 'closest')
        self.declare_parameter('publish_debug_image', False)
        self.declare_parameter('debug_image_topic', '/apriltag/image_annotated')
        self.declare_parameter('april_odom_topic', '/april_odom')
        self.declare_parameter('april_odom_frame', 'april_odom')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('bridge_map_to_odom', True)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('line_marker_topic', '/april_odom_line')
        self.declare_parameter('line_width', 0.01)

        self.calibration_file = os.path.expanduser(
            self.get_parameter('calibration_file').get_parameter_value().string_value)
        self.tag_db_file = self.get_parameter('tag_db_file').get_parameter_value().string_value
        self.use_camera_info = self.get_parameter('use_camera_info').get_parameter_value().bool_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        self.multi_tag_strategy = self.get_parameter('multi_tag_strategy').get_parameter_value().string_value
        self.publish_debug_image = self.get_parameter('publish_debug_image').get_parameter_value().bool_value
        self.debug_image_topic = self.get_parameter('debug_image_topic').get_parameter_value().string_value
        self.april_odom_topic = self.get_parameter('april_odom_topic').get_parameter_value().string_value
        self.april_odom_frame = self.get_parameter('april_odom_frame').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.bridge_map_to_odom = self.get_parameter('bridge_map_to_odom').get_parameter_value().bool_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.line_marker_topic = self.get_parameter('line_marker_topic').get_parameter_value().string_value
        self.line_width = self.get_parameter('line_width').get_parameter_value().double_value

        if not self.tag_db_file:
            from ament_index_python.packages import get_package_share_directory
            self.tag_db_file = os.path.join(
                get_package_share_directory('just_pick_it_perception'),
                'config', 'apriltag_world_poses.yaml')

        self._load_tag_db(self.tag_db_file)
        self._load_calibration(self.calibration_file)

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self._latest_caminfo = None
        self._caminfo_lock = Lock()

        self._build_detector()

        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, 10)
        self.april_odom_pub = self.create_publisher(Odometry, self.april_odom_topic, 10)
        self.line_pub = self.create_publisher(Marker, self.line_marker_topic, 10)
        self.debug_pub = (
            self.create_publisher(Image, self.debug_image_topic, 1)
            if self.publish_debug_image else None
        )

        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.static_tf_broadcaster = (
            StaticTransformBroadcaster(self) if (self.publish_tf and self.bridge_map_to_odom) else None
        )
        if self.static_tf_broadcaster is not None:
            self._publish_static_map_to_odom()

        self.create_subscription(Image, self.image_topic, self._image_cb, 5)
        if self.use_camera_info:
            self.create_subscription(CameraInfo, self.camera_info_topic, self._caminfo_cb, 5)

        self.get_logger().info(
            f'AprilTag pose estimator ready. world_frame={self.world_frame}, '
            f'base_frame={self.base_frame}, tags={sorted(self.tag_db.keys())}, '
            f'use_camera_info={self.use_camera_info}, '
            f'publish_tf={self.publish_tf}, '
            f'april_odom_frame={self.april_odom_frame}'
        )

    def _build_detector(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            params = cv2.aruco.DetectorParameters()
            self._detector = cv2.aruco.ArucoDetector(dictionary, params)
            self._detect = self._detect_new_api
        else:
            self._dictionary = dictionary
            self._params = cv2.aruco.DetectorParameters_create()
            self._detect = self._detect_old_api

    def _detect_new_api(self, gray):
        return self._detector.detectMarkers(gray)

    def _detect_old_api(self, gray):
        return cv2.aruco.detectMarkers(gray, self._dictionary, parameters=self._params)

    def _load_tag_db(self, path):
        if not os.path.isfile(path):
            raise FileNotFoundError(f'tag_db_file not found: {path}')
        with open(path, 'r') as f:
            data = yaml.safe_load(f)

        default_size = float(data.get('default_size_m', 0.15))
        tags = data.get('tags', {}) or {}
        self.tag_db = {}
        for tid, entry in tags.items():
            tid_int = int(tid)
            size_m = float(entry.get('size_m', default_size))
            pose = entry['pose']
            R = rpy_to_matrix(float(pose['R']), float(pose['P']), float(pose['Y']))
            t = [float(pose['x']), float(pose['y']), float(pose['z'])]
            T_world_tagmodel = make_transform(R, t)
            self.tag_db[tid_int] = {
                'size_m': size_m,
                'T_world_tagmodel': T_world_tagmodel,
            }

    def _load_calibration(self, path):
        if not os.path.isfile(path):
            self.get_logger().warn(
                f'calibration_file not found: {path}. '
                f'use_camera_info=true 이면 무시되지만 false면 노드가 동작하지 않는다.'
            )
            self._calib_K = None
            self._calib_D = None
            return
        with open(path, 'r') as f:
            calib = yaml.safe_load(f)
        K = np.array(calib['camera_matrix']['data'], dtype=np.float64).reshape(3, 3)
        D = np.array(calib['distortion_coefficients']['data'], dtype=np.float64).reshape(-1)
        self._calib_K = K
        self._calib_D = D

    def _caminfo_cb(self, msg: CameraInfo):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64).reshape(-1) if len(msg.d) else np.zeros(5)
        with self._caminfo_lock:
            self._latest_caminfo = (K, D, msg.header.frame_id)

    def _current_intrinsics(self, image_frame_id):
        """이미지에 적용할 (K, D, cam_frame_id) 반환. CameraInfo 우선, 없으면 calibration yaml."""
        if self.use_camera_info:
            with self._caminfo_lock:
                if self._latest_caminfo is not None:
                    K, D, frame = self._latest_caminfo
                    return K, D, (frame or image_frame_id)
        if self._calib_K is not None:
            return self._calib_K, (self._calib_D if self._calib_D is not None else np.zeros(5)), image_frame_id
        return None

    def _image_cb(self, msg: Image):
        intr = self._current_intrinsics(msg.header.frame_id)
        if intr is None:
            self.get_logger().warn(
                'intrinsic 미가용. CameraInfo 또는 calibration_file 필요.',
                throttle_duration_sec=5.0,
            )
            return
        K, D, cam_frame = intr
        if not cam_frame:
            self.get_logger().warn(
                '카메라 frame_id가 비어 있음. tf lookup 불가.',
                throttle_duration_sec=5.0,
            )
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge 변환 실패: {e}')
            return

        gray = cv2.cvtColor(cv_img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect(gray)
        if ids is None or len(ids) == 0:
            return

        try:
            tf_base_in_cam = self.tf_buffer.lookup_transform(
                cam_frame, self.base_frame, msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.05))
        except TransformException as e:
            self.get_logger().warn(
                f'TF lookup 실패 (target={cam_frame}, source={self.base_frame}): {e}',
                throttle_duration_sec=2.0,
            )
            return

        T_camBody_base = self._tf_to_matrix(tf_base_in_cam)

        candidates = []
        for i, marker_id in enumerate(ids.flatten().tolist()):
            # print(f"detected id :",{marker_id})
            if marker_id not in self.tag_db:
                continue
            tag = self.tag_db[marker_id]
            img_pts = corners[i].reshape(4, 2).astype(np.float64)
            obj_pts = self._tag_object_points(tag['size_m'])

            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue

            R_co, _ = cv2.Rodrigues(rvec)
            T_camOpt_tagOpt = make_transform(R_co, tvec.reshape(3))
            T_tagOpt_camOpt = invert_transform(T_camOpt_tagOpt)
            T_tagModel_tagOpt = make_transform(R_TAGMODEL_TAGOPT, [0, 0, 0])
            T_camOpt_camBody = make_transform(R_CAMOPT_CAMBODY, [0, 0, 0])

            T_world_base = (
                tag['T_world_tagmodel']
                @ T_tagModel_tagOpt
                @ T_tagOpt_camOpt
                @ T_camOpt_camBody
                @ T_camBody_base
            )

            distance = float(np.linalg.norm(tvec))
            candidates.append((marker_id, distance, T_world_base, (rvec, tvec)))

        if not candidates:
            return

        chosen_id, chosen_dist, T_world_base, chosen_rt = self._select_candidate(candidates)
        self._publish_pose(T_world_base, msg.header.stamp)
        self._publish_april_odom(T_world_base, msg.header.stamp)
        self._publish_april_odom_line(T_world_base, msg.header.stamp)

        if self.debug_pub is not None:
            self._publish_debug_image(cv_img, corners, ids, K, D, chosen_rt, msg.header)

        self.get_logger().debug(
            f'tag id={chosen_id}, dist={chosen_dist:.3f} m, '
            f'pos={T_world_base[:3, 3].tolist()}, '
            f'rpy={mat2euler(T_world_base[:3, :3], axes="sxyz")}'
        )

    def _select_candidate(self, candidates):
        if self.multi_tag_strategy == 'closest' or len(candidates) == 1:
            return min(candidates, key=lambda c: c[1])
        # 다른 전략은 후속 작업; 일단 closest로 폴백
        return min(candidates, key=lambda c: c[1])

    @staticmethod
    def _tag_object_points(size_m):
        s = size_m / 2.0
        return np.array(
            [
                [-s,  s, 0.0],
                [ s,  s, 0.0],
                [ s, -s, 0.0],
                [-s, -s, 0.0],
            ],
            dtype=np.float64,
        )

    @staticmethod
    def _tf_to_matrix(tf_stamped):
        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        # transforms3d quaternion 순서: (w, x, y, z)
        from transforms3d.quaternions import quat2mat
        R = quat2mat([q.w, q.x, q.y, q.z])
        return make_transform(R, [t.x, t.y, t.z])

    def _publish_pose(self, T_world_base, stamp):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.world_frame

        pos = T_world_base[:3, 3]
        # transforms3d quaternion 순서 (w,x,y,z) → ROS (x,y,z,w)
        q_wxyz = mat2quat(T_world_base[:3, :3])
        msg.pose.pose.position.x = float(pos[0])
        msg.pose.pose.position.y = float(pos[1])
        msg.pose.pose.position.z = float(pos[2])
        msg.pose.pose.orientation.x = float(q_wxyz[1])
        msg.pose.pose.orientation.y = float(q_wxyz[2])
        msg.pose.pose.orientation.z = float(q_wxyz[3])
        msg.pose.pose.orientation.w = float(q_wxyz[0])

        # 고정 대각 covariance (xyz: 1cm sigma, rpy: 1deg sigma)
        # TODO: reprojection error 기반 동적 산출로 교체
        sigma_xyz = 0.01
        sigma_rpy = np.deg2rad(1.0)
        cov = [0.0] * 36
        cov[0] = sigma_xyz ** 2
        cov[7] = sigma_xyz ** 2
        cov[14] = sigma_xyz ** 2
        cov[21] = sigma_rpy ** 2
        cov[28] = sigma_rpy ** 2
        cov[35] = sigma_rpy ** 2
        msg.pose.covariance = cov

        self.pose_pub.publish(msg)

    def _publish_april_odom(self, T_world_base, stamp):
        pos = T_world_base[:3, 3]
        q_wxyz = mat2quat(T_world_base[:3, :3])
        qx, qy, qz, qw = float(q_wxyz[1]), float(q_wxyz[2]), float(q_wxyz[3]), float(q_wxyz[0])

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = self.april_odom_frame
        odom.pose.pose.position.x = float(pos[0])
        odom.pose.pose.position.y = float(pos[1])
        odom.pose.pose.position.z = float(pos[2])
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        sigma_xyz = 0.01
        sigma_rpy = np.deg2rad(1.0)
        pose_cov = [0.0] * 36
        pose_cov[0] = sigma_xyz ** 2
        pose_cov[7] = sigma_xyz ** 2
        pose_cov[14] = sigma_xyz ** 2
        pose_cov[21] = sigma_rpy ** 2
        pose_cov[28] = sigma_rpy ** 2
        pose_cov[35] = sigma_rpy ** 2
        odom.pose.covariance = pose_cov

        # 속도는 추정하지 않으므로 0, covariance는 매우 큰 값으로 신뢰도 낮춤
        twist_cov = [0.0] * 36
        big = 1e6
        twist_cov[0] = big
        twist_cov[7] = big
        twist_cov[14] = big
        twist_cov[21] = big
        twist_cov[28] = big
        twist_cov[35] = big
        odom.twist.covariance = twist_cov

        self.april_odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = stamp
            tf_msg.header.frame_id = self.world_frame
            tf_msg.child_frame_id = self.april_odom_frame
            tf_msg.transform.translation.x = float(pos[0])
            tf_msg.transform.translation.y = float(pos[1])
            tf_msg.transform.translation.z = float(pos[2])
            tf_msg.transform.rotation.x = qx
            tf_msg.transform.rotation.y = qy
            tf_msg.transform.rotation.z = qz
            tf_msg.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_msg)

    def _publish_april_odom_line(self, T_world_base, stamp):
        # map 원점에서 추정된 april_odom 위치까지의 초록색 선
        pos = T_world_base[:3, 3]
        marker = Marker()
        marker.header.stamp = stamp
        marker.header.frame_id = self.world_frame
        marker.ns = 'april_odom_line'
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = float(self.line_width)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        p0 = Point()
        p0.x = 0.0
        p0.y = 0.0
        p0.z = 0.0
        p1 = Point()
        p1.x = float(pos[0])
        p1.y = float(pos[1])
        p1.z = float(pos[2])
        marker.points = [p0, p1]
        marker.pose.orientation.w = 1.0
        self.line_pub.publish(marker)

    def _publish_static_map_to_odom(self):
        # 시뮬에서 odom 프레임이 월드 원점에서 시작하므로 항등 변환.
        # 실로봇에서 odom drift가 있다면 동적으로 보정해야 한다.
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = self.world_frame
        tf_msg.child_frame_id = self.odom_frame
        tf_msg.transform.translation.x = 0.0
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(tf_msg)
        self.get_logger().info(
            f'정적 TF 발행: {self.world_frame} -> {self.odom_frame} (identity)'
        )

    def _publish_debug_image(self, cv_img, corners, ids, K, D, chosen_rt, header):
        annotated = cv_img.copy()
        cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
        try:
            rvec, tvec = chosen_rt
            cv2.drawFrameAxes(annotated, K, D, rvec, tvec, 0.1)
        except Exception:
            pass
        out = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        out.header = header
        self.debug_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
