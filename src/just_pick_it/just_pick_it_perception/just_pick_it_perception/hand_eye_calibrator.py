import os

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from transforms3d.quaternions import quat2mat

_DEFAULT_CALIB = os.path.join(
    get_package_share_directory('just_pick_it_perception'),
    'result',
    'camera_calibration.yaml',
)

_DEFAULT_RESULT = os.path.expanduser('~/hand_eye_calibration.yaml')


class HandEyeCalibrator(Node):

    def __init__(self):
        super().__init__('hand_eye_calibrator')

        self.declare_parameter('calibration_file', _DEFAULT_CALIB)
        self.declare_parameter('result_file', _DEFAULT_RESULT)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('gripper_frame', 'tool0') # 이부분 맞게 확인이 필요함.
        self.declare_parameter('tag_size_m', 0.045)
        self.declare_parameter('tag_spacing_m', 0.012)
        self.declare_parameter('board_cols', 4)
        self.declare_parameter('board_rows', 3)

        calib_path = self.get_parameter('calibration_file').get_parameter_value().string_value
        self._result_path = self.get_parameter('result_file').get_parameter_value().string_value
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self._base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self._gripper_frame = self.get_parameter('gripper_frame').get_parameter_value().string_value
        tag_size = self.get_parameter('tag_size_m').get_parameter_value().double_value
        tag_spacing = self.get_parameter('tag_spacing_m').get_parameter_value().double_value
        cols = self.get_parameter('board_cols').get_parameter_value().integer_value
        rows = self.get_parameter('board_rows').get_parameter_value().integer_value

        self.K, self.D = self._load_calibration(calib_path)
        self._board_obj_pts = self._build_board_obj_pts(tag_size, tag_spacing, cols, rows)
        self._build_detector()

        self._map1 = None
        self._map2 = None
        self._new_K = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.bridge = CvBridge()
        self._latest_image: np.ndarray | None = None

        self._R_gripper2base_list: list[np.ndarray] = []
        self._t_gripper2base_list: list[np.ndarray] = []
        self._R_target2cam_list: list[np.ndarray] = []
        self._t_target2cam_list: list[np.ndarray] = []

        self.create_subscription(Image, image_topic, self._image_cb, 5)
        self.create_service(Trigger, '~/capture_sample', self._capture_cb)
        self.create_service(Trigger, '~/run_calibration', self._calibrate_cb)

        self.get_logger().info(
            f'HandEyeCalibrator 준비 완료\n'
            f'  base: {self._base_frame}, gripper: {self._gripper_frame}\n'
            f'  ~/capture_sample  : 현재 pose 쌍 수집\n'
            f'  ~/run_calibration : 캘리브레이션 실행 및 저장'
        )

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

    @staticmethod
    def _build_board_obj_pts(tag_size: float, spacing: float, cols: int, rows: int) -> dict:
        """board 좌표계 기준 각 태그 코너의 3D 위치 (z=0 평면)"""
        obj_pts: dict[int, np.ndarray] = {}
        for row in range(rows):
            for col in range(cols):
                tag_id = row * cols + col
                cx = col * (tag_size + spacing)
                cy = row * (tag_size + spacing)
                h = tag_size / 2.0
                # 코너 순서: TL, TR, BR, BL (cv2.aruco.detectMarkers 기준)
                obj_pts[tag_id] = np.array([
                    [cx - h, cy - h, 0.0],
                    [cx + h, cy - h, 0.0],
                    [cx + h, cy + h, 0.0],
                    [cx - h, cy + h, 0.0],
                ], dtype=np.float64)
        return obj_pts

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

    # ------------------------------------------------------------------
    # 이미지 콜백
    # ------------------------------------------------------------------

    def _image_cb(self, msg: Image):
        try:
            self._latest_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge 변환 실패: {e}')

    # ------------------------------------------------------------------
    # pose 추정 헬퍼
    # ------------------------------------------------------------------

    def _detect_board_pose(self, image: np.ndarray):
        """AprilTag board의 pose를 카메라 기준으로 추정."""
        h, w = image.shape[:2]
        if self._map1 is None:
            self._build_undistort_maps(h, w)

        undistorted = cv2.remap(image, self._map1, self._map2, cv2.INTER_LINEAR)
        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect(gray)

        if ids is None or len(ids) == 0:
            return None, None

        all_obj = []
        all_img = []
        for i, tag_id in enumerate(ids.flatten()):
            if tag_id not in self._board_obj_pts:
                continue
            all_obj.append(self._board_obj_pts[tag_id])
            all_img.append(corners[i].reshape(4, 2))

        if not all_obj:
            return None, None

        all_obj_np = np.vstack(all_obj)
        all_img_np = np.vstack(all_img).astype(np.float64)

        ret, rvec, tvec = cv2.solvePnP(
            all_obj_np, all_img_np, self._new_K, np.zeros(5),
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ret:
            return None, None

        R_target2cam, _ = cv2.Rodrigues(rvec)
        return R_target2cam, tvec

    def _get_gripper_pose(self):
        """TF에서 base_link 기준 gripper pose를 읽는다."""
        try:
            tf_stamped = self._tf_buffer.lookup_transform(
                self._base_frame, self._gripper_frame, rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(
                f'TF lookup {self._base_frame} -> {self._gripper_frame} 실패: {e}'
            )
            return None, None

        t = tf_stamped.transform.translation
        q = tf_stamped.transform.rotation
        R = quat2mat([q.w, q.x, q.y, q.z])
        tvec = np.array([[t.x], [t.y], [t.z]], dtype=np.float64)
        return R, tvec

    # ------------------------------------------------------------------
    # 서비스 콜백
    # ------------------------------------------------------------------

    def _capture_cb(self, request, response):
        if self._latest_image is None:
            response.success = False
            response.message = '이미지 미수신 — 카메라 토픽 확인 필요'
            return response

        R_g2b, t_g2b = self._get_gripper_pose()
        if R_g2b is None:
            response.success = False
            response.message = 'gripper TF 획득 실패'
            return response

        R_t2c, t_t2c = self._detect_board_pose(self._latest_image)
        if R_t2c is None:
            response.success = False
            response.message = 'AprilTag board 미검출 — board가 카메라에 보이는지 확인'
            return response

        self._R_gripper2base_list.append(R_g2b)
        self._t_gripper2base_list.append(t_g2b)
        self._R_target2cam_list.append(R_t2c)
        self._t_target2cam_list.append(t_t2c)

        n = len(self._R_gripper2base_list)
        response.success = True
        response.message = f'샘플 {n}개 수집됨'
        self.get_logger().info(f'샘플 수집 완료: {n}개 (최소 3개, 권장 15개 이상)')
        return response

    def _calibrate_cb(self, request, response):
        n = len(self._R_gripper2base_list)
        if n < 3:
            response.success = False
            response.message = f'샘플 부족 ({n}개, 최소 3개 필요)'
            return response

        try:
            R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
                self._R_gripper2base_list,
                self._t_gripper2base_list,
                self._R_target2cam_list,
                self._t_target2cam_list,
            )
        except Exception as e:
            response.success = False
            response.message = f'calibrateHandEye 실패: {e}'
            return response

        self._save_result(R_cam2gripper, t_cam2gripper)

        response.success = True
        response.message = f'완료 (샘플 {n}개). 저장: {self._result_path}'
        self.get_logger().info(
            f'Hand-Eye 캘리브레이션 완료 (샘플 {n}개)\n'
            f'R_cam2gripper:\n{np.round(R_cam2gripper, 6)}\n'
            f't_cam2gripper: {np.round(t_cam2gripper.flatten(), 6)}\n'
            f'저장 경로: {self._result_path}'
        )
        return response

    # ------------------------------------------------------------------
    # 결과 저장
    # ------------------------------------------------------------------

    def _save_result(self, R: np.ndarray, t: np.ndarray):
        result_dir = os.path.dirname(self._result_path)
        if result_dir:
            os.makedirs(result_dir, exist_ok=True)
        data = {
            'R_cam2gripper': {
                'rows': 3,
                'cols': 3,
                'data': R.flatten().tolist(),
            },
            't_cam2gripper': {
                'rows': 3,
                'cols': 1,
                'data': t.flatten().tolist(),
            },
        }
        with open(self._result_path, 'w') as f:
            yaml.dump(data, f, default_flow_style=False)


def main(args=None):
    rclpy.init(args=args)
    node = HandEyeCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
