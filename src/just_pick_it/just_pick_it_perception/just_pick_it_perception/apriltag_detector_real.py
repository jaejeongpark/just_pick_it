import collections
import math
import os
import socket
import struct
import threading

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

HEADER_FMT = '>IHH'
HEADER_SIZE = struct.calcsize(HEADER_FMT)
RECV_BUF = 65536


class AprilTagDetectorReal(Node):

    def __init__(self):
        super().__init__('apriltag_detector_real')

        self.declare_parameter('robot_name', '')
        self.declare_parameter('calibration_file', '')
        self.declare_parameter('udp_port', 9870)
        self.declare_parameter('annotated_topic', '/apriltag/image_annotated')
        self.declare_parameter('tag_size_m', 0.05)
        self.declare_parameter('base_frame', 'base_link')
        # Static transform: base_link -> front_camera_mount
        self.declare_parameter('camera_mount_x', 0.02)
        self.declare_parameter('camera_mount_y', 0.0)
        self.declare_parameter('camera_mount_z', 0.0495)
        # Physical camera tilt around y-axis of mount frame.
        # Positive = camera tilts upward (optical axis pitches toward +z_mount).
        self.declare_parameter('camera_pitch_deg', 0.0)

        robot_name = self.get_parameter('robot_name').get_parameter_value().string_value
        calib_path = self.get_parameter('calibration_file').get_parameter_value().string_value
        calib_path = self._resolve_calibration_path(robot_name, calib_path)
        udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value
        annotated_topic = self.get_parameter('annotated_topic').get_parameter_value().string_value
        tag_half = self.get_parameter('tag_size_m').get_parameter_value().double_value / 2.0
        self._base_frame = self.get_parameter('base_frame').get_parameter_value().string_value

        mount_x = self.get_parameter('camera_mount_x').get_parameter_value().double_value
        mount_y = self.get_parameter('camera_mount_y').get_parameter_value().double_value
        mount_z = self.get_parameter('camera_mount_z').get_parameter_value().double_value
        pitch_deg = self.get_parameter('camera_pitch_deg').get_parameter_value().double_value
        self._camera_pitch_rad = math.radians(pitch_deg)

        # solvePnP 3D object points — DICT_APRILTAG_36h11 실측 corner 순서: TR, TL, BL, BR
        # (x right, y up, z toward camera; right-hand)
        self._obj_pts = np.array([
            [-tag_half,  tag_half, 0.0],  # corner 0: TL
            [ tag_half,  tag_half, 0.0],  # corner 1: TR
            [ tag_half, -tag_half, 0.0],  # corner 2: BR
            [-tag_half, -tag_half, 0.0],  # corner 3: BL
        ], dtype=np.float64)

        # self._obj_pts = np.array([
        #     [ tag_half,  tag_half, 0.0],  # corner 0: TL
        #     [ -tag_half,  tag_half, 0.0],  # corner 1: TR
        #     [ -tag_half, -tag_half, 0.0],  # corner 2: BR
        #     [ tag_half, -tag_half, 0.0],  # corner 3: BL
        # ], dtype=np.float64)

        self.K, self.D = self._load_calibration(calib_path)
        self.get_logger().info(f'calibration loaded: K=\n{self.K}\nD={self.D}')

        self._map1 = None
        self._map2 = None

        # T_camera_mount_base_link = inv(T_base_link_camera_mount)
        # T_base_link_camera_mount: rotation=identity, translation=(mount_x, mount_y, mount_z)
        T_base_link_camera_mount = np.eye(4, dtype=np.float64)
        T_base_link_camera_mount[:3, 3] = [mount_x, mount_y, mount_z]
        self._T_camera_mount_base_link = np.linalg.inv(T_base_link_camera_mount)
        self.get_logger().info(
            f'T_camera_mount_base_link:\n{np.round(self._T_camera_mount_base_link, 4)}'
        )

        # TF buffer for map -> apriltag_N (published by apriltag_map_tf_publisher)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._T_map_marker: dict[int, np.ndarray] = {}

        self._build_detector()
        self.bridge = CvBridge()
        self.annotated_pub = self.create_publisher(Image, annotated_topic, 1)
        self._pose_pub = self.create_publisher(PoseStamped, '/apriltag/robot_pose', 1)

        # UDP frame queue — deque(maxlen=1) keeps only the latest assembled frame
        self._latest_frame: collections.deque = collections.deque(maxlen=1)
        self._udp_frames: dict[int, dict[int, bytes]] = {}

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(('', udp_port))
        self._sock.settimeout(1.0)

        self._recv_thread = threading.Thread(target=self._udp_recv_loop, daemon=True)
        self._recv_thread.start()

        # Timer drives the detection loop
        self.create_timer(1.0 / 30.0, self._process_frame)

        self.get_logger().info(f'listening on UDP port {udp_port}')
        self.get_logger().info(f'publishing annotated to {annotated_topic}')

        self._pose_est_logged = False

    # ------------------------------------------------------------------
    # Init helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _resolve_calibration_path(robot_name: str, explicit_path: str) -> str:
        pkg_result = os.path.join(
            get_package_share_directory('just_pick_it_perception'), 'result'
        )
        if explicit_path:
            return explicit_path
        if robot_name:
            return os.path.join(pkg_result, robot_name, 'camera_calibration.yaml')
        return os.path.join(pkg_result, 'camera_calibration.yaml')

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
        new_K, _ = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w, h), alpha=0)
        self._map1, self._map2 = cv2.initUndistortRectifyMap(
            self.K, self.D, None, new_K, (w, h), cv2.CV_16SC2)
        self._new_K = new_K
        self.get_logger().info(
            f'undistort maps built for {w}x{h}'
            f'\noriginal K[fx]={self.K[0,0]:.1f}  new_K[fx]={new_K[0,0]:.1f}'
            f'\nnew_K=\n{new_K}'
        )

    def _get_T_camera_mount_camera_optical(self) -> np.ndarray:
        # camera_mount : x forward, y left,  z up
        # camera_optical: x right,  y down,  z forward (OpenCV convention)
        #
        # Pipeline: raw camera -> cv2.flip(-1) at receiver -> calibration K is
        # for the flipped image. The solvePnP output is therefore in a "flipped
        # optical frame" rotated 180 deg around z_optical from the standard
        # optical convention. The rotation below maps that flipped optical frame
        # directly to camera_mount (equivalent to R_standard @ R_z(180 deg)):
        #   flipped_optical x -> +y_mount   (image right = mount left)
        #   flipped_optical y -> +z_mount   (image down  = mount up)
        #   flipped_optical z -> +x_mount   (forward)
        # camera_pitch_deg > 0: camera tilts upward (optical z gains +z_mount component).
        pitch = self._camera_pitch_rad
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        # R_y(-pitch) so that positive pitch_deg means looking upward
        R_pitch = np.array([
            [ cp, 0, -sp],
            [  0, 1,   0],
            [ sp, 0,  cp],
        ], dtype=np.float64)
        R_flipped = np.array([
            [ 0,  0,  1],
            [ 1,  0,  0],
            [ 0,  1,  0],
        ], dtype=np.float64)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R_pitch @ R_flipped
        return T

    # ------------------------------------------------------------------
    # UDP receiver thread
    # ------------------------------------------------------------------

    def _udp_recv_loop(self):
        while True:
            try:
                packet, _ = self._sock.recvfrom(RECV_BUF)
            except socket.timeout:
                continue
            except OSError:
                break

            if len(packet) < HEADER_SIZE:
                continue

            frame_id, pkt_idx, total = struct.unpack(HEADER_FMT, packet[:HEADER_SIZE])
            chunk = packet[HEADER_SIZE:]
            self._udp_frames.setdefault(frame_id, {})[pkt_idx] = chunk

            if len(self._udp_frames[frame_id]) == total:
                data = b''.join(self._udp_frames[frame_id][i] for i in range(total))
                del self._udp_frames[frame_id]

                stale = [fid for fid in list(self._udp_frames) if fid < frame_id - 30]
                for fid in stale:
                    del self._udp_frames[fid]

                img = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    continue

                # Camera is physically mounted upside-down;
                # calibration was performed on the flipped image.
                img = cv2.flip(img, -1)
                self._latest_frame.append(img)

    # ------------------------------------------------------------------
    # TF helpers
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
    # Detection loop (driven by timer)
    # ------------------------------------------------------------------

    def _process_frame(self):
        try:
            cv_img = self._latest_frame.popleft()
        except IndexError:
            return

        h, w = cv_img.shape[:2]
        if self._map1 is None:
            self._build_undistort_maps(h, w)

        undistorted = cv2.remap(cv_img, self._map1, self._map2, cv2.INTER_LINEAR)
        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect(gray)

        annotated = undistorted.copy()
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
            self._estimate_poses(corners, ids)

        out = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'camera_optical'
        self.annotated_pub.publish(out)

    def _estimate_poses(self, corners, ids):
        K = self._new_K
        D = np.zeros(5, dtype=np.float64)

        T_camera_mount_camera_optical = self._get_T_camera_mount_camera_optical()
        T_camera_mount_base_link = self._T_camera_mount_base_link

        for i, tag_id in enumerate(ids.flatten()):
            if tag_id not in self._T_map_marker:
                result = self._lookup_tf_matrix('map', f'apriltag_{tag_id}')
                if result is not None:
                    self._T_map_marker[tag_id] = result

            T_map_marker = self._T_map_marker.get(tag_id)
            if T_map_marker is None:
                self.get_logger().warn(
                    f'[tag {tag_id}] TF map->apriltag_{tag_id} 미준비'
                )
                continue

            if not self._pose_est_logged:
                self.get_logger().info(f'[tag {tag_id}] TF 준비 완료, pose 추정 시작')
                self._pose_est_logged = True

            img_pts = corners[i].reshape(4, 2)
            print(f'[tag {tag_id}] img_pts:\n{img_pts}')
            print(f'[tag {tag_id}] obj_pts:\n{self._obj_pts}')

            # IPPE_SQUARE는 두 개의 해를 반환한다. solvePnPGeneric으로 모두 얻어서
            # tvec[2] > 0 (마커가 카메라 앞쪽)인 해를 명시적으로 선택한다.
            retval, rvecs, tvecs, reproj_errors = cv2.solvePnPGeneric(
                self._obj_pts, img_pts, K, D,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )

            best_idx = None
            for j in range(len(rvecs)):
                if tvecs[j][2, 0] > 0:
                    if best_idx is None or reproj_errors[j] < reproj_errors[best_idx]:
                        best_idx = j

            if best_idx is None:
                self.get_logger().warn(f'[tag {tag_id}] 유효한 solvePnP 해 없음 (tvec.z<=0)')
                continue

            rvec = rvecs[best_idx]
            tvec = tvecs[best_idx]
            print(
                f'[tag {tag_id}] solvePnP (sol={best_idx}): '
                f'tvec={tvec.flatten()}, reproj={reproj_errors[best_idx, 0]:.2f}px'
            )

            R, _ = cv2.Rodrigues(rvec)
            T_camera_optical_marker = np.eye(4, dtype=np.float64)
            T_camera_optical_marker[:3, :3] = R
            T_camera_optical_marker[:3, 3] = tvec.flatten()

            T_camera_mount_marker = T_camera_mount_camera_optical @ T_camera_optical_marker

            # T_map_base_link = T_map_marker @ T_marker_camera_mount @ T_camera_mount_base_link
            T_map_base_link = (
                T_map_marker
                @ np.linalg.inv(T_camera_mount_marker)
                @ T_camera_mount_base_link
            )

            self._publish_robot_pose(T_map_base_link)

    def _publish_robot_pose(self, T_map_base_link: np.ndarray):
        w, qx, qy, qz = mat2quat(T_map_base_link[:3, :3])
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = T_map_base_link[0, 3]
        msg.pose.position.y = T_map_base_link[1, 3]
        msg.pose.position.z = T_map_base_link[2, 3]
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = w
        self._pose_pub.publish(msg)

    def destroy_node(self):
        try:
            self._sock.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagDetectorReal()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
