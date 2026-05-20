import os

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

_DEFAULT_CALIB = os.path.realpath(os.path.join(
    os.path.dirname(__file__), '..', 'result', 'camera_calibration.yaml',
))


class AprilTagDetector(Node):

    def __init__(self):
        super().__init__('apriltag_detector')

        self.declare_parameter('calibration_file', _DEFAULT_CALIB)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('annotated_topic', '/apriltag/image_annotated')

        calib_path = self.get_parameter('calibration_file').get_parameter_value().string_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        annotated_topic = self.get_parameter('annotated_topic').get_parameter_value().string_value

        self.K, self.D = self._load_calibration(calib_path)
        self.get_logger().info(
            f'calibration loaded: K=\n{self.K}\nD={self.D}'
        )

        # undistort 최적화 맵을 미리 계산 (이미지 크기는 첫 프레임에서 확정)
        self._map1 = None
        self._map2 = None

        self._build_detector()
        self.bridge = CvBridge()
        self.annotated_pub = self.create_publisher(Image, annotated_topic, 1)
        self.create_subscription(Image, self.image_topic, self._image_cb, 5)

        self.get_logger().info(
            f'subscribing to {self.image_topic}, '
            f'publishing annotated to {annotated_topic}'
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
        # getOptimalNewCameraMatrix: alpha=1 이면 원본 픽셀 전부 보존 (검은 테두리 허용)
        new_K, _ = cv2.getOptimalNewCameraMatrix(self.K, self.D, (w, h), alpha=1)
        self._map1, self._map2 = cv2.initUndistortRectifyMap(
            self.K, self.D, None, new_K, (w, h), cv2.CV_16SC2)
        self._new_K = new_K
        self.get_logger().info(
            f'undistort maps built for {w}x{h}\nnew_K=\n{new_K}'
        )

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

        # 왜곡 보정
        undistorted = cv2.remap(cv_img, self._map1, self._map2, cv2.INTER_LINEAR)

        gray = cv2.cvtColor(undistorted, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self._detect(gray)

        annotated = undistorted.copy()
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
            self.get_logger().info(
                f'detected tag ids: {ids.flatten().tolist()}, count={len(ids)}'
            )

        out = self.bridge.cv2_to_imgmsg(annotated, 'bgr8')
        out.header = msg.header
        self.annotated_pub.publish(out)


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
