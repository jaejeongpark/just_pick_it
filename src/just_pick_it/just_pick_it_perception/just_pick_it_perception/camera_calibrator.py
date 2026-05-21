"""
카메라 캘리브레이션 인터랙티브 노드.

live preview를 보며 spacebar로 이미지를 캡처하고,
충분히 모이면 Enter를 눌러 캘리브레이션을 실행한다.

  Space  : 현재 프레임 저장 (image_dir에 영구 보관)
  Enter  : 수집된 이미지로 캘리브레이션 실행 → YAML 저장
  q / ESC: 종료 (캘리브레이션 없이)

사용법:
    ros2 run just_pick_it_perception camera_calibrator
    ros2 run just_pick_it_perception camera_calibrator --ros-args \
        -p image_topic:=/camera/image_raw \
        -p image_dir:=~/img_captures \
        -p board_width:=8 \
        -p board_height:=6 \
        -p square_size:=0.025 \
        -p output_file:=~/camera_calibration.yaml
"""

import os
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

_DEFAULT_RESULT_DIR = os.path.join(
    get_package_share_directory('just_pick_it_perception'), 'result')


class CameraCalibrator(Node):

    def __init__(self):
        super().__init__('camera_calibrator')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('image_dir', os.path.expanduser('~/img_captures'))
        self.declare_parameter('board_width', 8)
        self.declare_parameter('board_height', 6)
        self.declare_parameter('square_size', 0.025)
        self.declare_parameter(
            'output_file',
            os.path.join(_DEFAULT_RESULT_DIR, 'camera_calibration.yaml'))

        self._topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self._image_dir = os.path.expanduser(
            self.get_parameter('image_dir').get_parameter_value().string_value)
        self._board_w = self.get_parameter('board_width').get_parameter_value().integer_value
        self._board_h = self.get_parameter('board_height').get_parameter_value().integer_value
        self._square_size = self.get_parameter('square_size').get_parameter_value().double_value
        self._output_file = os.path.expanduser(
            self.get_parameter('output_file').get_parameter_value().string_value)

        os.makedirs(self._image_dir, exist_ok=True)

        self._bridge = CvBridge()
        self._latest_frame = None
        self._capture_count = 0

        self._sub = self.create_subscription(Image, self._topic, self._image_cb, 10)

        self.get_logger().info(f'토픽 구독: {self._topic}')
        self.get_logger().info(f'이미지 저장 위치: {self._image_dir}')
        self.get_logger().info('Space: 캡처 | Enter: 캘리브레이션 실행 | q/ESC: 종료')

    def _image_cb(self, msg):
        self._latest_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def run_loop(self):
        pattern_size = (self._board_w, self._board_h)
        cv2.namedWindow('Camera Calibrator', cv2.WINDOW_NORMAL)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self._latest_frame is None:
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    break
                continue

            display = self._latest_frame.copy()
            gray = cv2.cvtColor(display, cv2.COLOR_BGR2GRAY)
            found, corners = cv2.findChessboardCorners(gray, pattern_size, None)
            if found:
                cv2.drawChessboardCorners(display, pattern_size, corners, found)

            status = 'DETECTED' if found else 'NOT FOUND'
            color = (0, 255, 0) if found else (0, 100, 255)
            cv2.putText(display, f'Checkerboard: {status}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
            cv2.putText(display, f'Captured: {self._capture_count}', (10, 65),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(display, 'Space: capture | Enter: calibrate | q: quit',
                        (10, display.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

            cv2.imshow('Camera Calibrator', display)
            key = cv2.waitKey(1) & 0xFF

            if key == ord(' '):
                filename = os.path.join(
                    self._image_dir,
                    f'calib_{time.strftime("%Y%m%d_%H%M%S")}_{self._capture_count:03d}.png')
                cv2.imwrite(filename, self._latest_frame)
                self._capture_count += 1
                self.get_logger().info(f'저장 [{self._capture_count}]: {os.path.basename(filename)}')

            elif key in (13, ord('\r')):  # Enter
                self.get_logger().info('캘리브레이션 시작...')
                self._calibrate()
                break

            elif key in (ord('q'), 27):  # q or ESC
                self.get_logger().info('종료 (캘리브레이션 없이)')
                break

        cv2.destroyAllWindows()

    def _calibrate(self):
        import glob
        pattern_size = (self._board_w, self._board_h)

        objp = np.zeros((self._board_h * self._board_w, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self._board_w, 0:self._board_h].T.reshape(-1, 2)
        objp *= self._square_size

        obj_points, img_points = [], []
        paths = sorted(
            glob.glob(os.path.join(self._image_dir, '*.png')) +
            glob.glob(os.path.join(self._image_dir, '*.jpg'))
        )

        if not paths:
            self.get_logger().error(f'이미지 없음: {self._image_dir}')
            return

        self.get_logger().info(f'총 {len(paths)}장 처리 중...')
        img_size = None
        refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        for path in paths:
            img = cv2.imread(path)
            if img is None:
                self.get_logger().warn(f'  읽기 실패: {os.path.basename(path)}')
                continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            if img_size is None:
                img_size = (gray.shape[1], gray.shape[0])

            found, corners = cv2.findChessboardCorners(gray, pattern_size, None)
            if found:
                corners_refined = cv2.cornerSubPix(
                    gray, corners, (11, 11), (-1, -1), refine_criteria)
                obj_points.append(objp)
                img_points.append(corners_refined)
                self.get_logger().info(f'  OK : {os.path.basename(path)}')
            else:
                self.get_logger().warn(f'  SKIP: {os.path.basename(path)} - 코너 미검출')

        valid = len(obj_points)
        self.get_logger().info(f'유효 이미지: {valid}/{len(paths)}')

        if valid < 10:
            self.get_logger().error(f'유효 이미지 부족 ({valid}장). 최소 10장 필요.')
            return

        ret, K, dist, _, _ = cv2.calibrateCamera(
            obj_points, img_points, img_size, None, None)

        self.get_logger().info(f'재투영 오차 (RMS): {ret:.4f} px')
        self.get_logger().info(f'Intrinsic matrix K:\n{K}')
        self.get_logger().info(f'Distortion coefficients: {dist.ravel().tolist()}')

        self._save_yaml(img_size, K, dist)

    def _save_yaml(self, img_size, K, dist):
        output_dir = os.path.dirname(self._output_file)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        calib = {
            'image_width': img_size[0],
            'image_height': img_size[1],
            'camera_name': 'pinky_camera',
            'distortion_model': 'plumb_bob',
            'camera_matrix': {
                'rows': 3, 'cols': 3,
                'data': K.ravel().tolist(),
            },
            'distortion_coefficients': {
                'rows': 1, 'cols': int(dist.size),
                'data': dist.ravel().tolist(),
            },
            'rectification_matrix': {
                'rows': 3, 'cols': 3,
                'data': np.eye(3).ravel().tolist(),
            },
            'projection_matrix': {
                'rows': 3, 'cols': 4,
                'data': np.hstack([K, np.zeros((3, 1))]).ravel().tolist(),
            },
        }

        with open(self._output_file, 'w') as f:
            yaml.dump(calib, f, default_flow_style=False, sort_keys=False)

        self.get_logger().info(f'캘리브레이션 결과 저장: {self._output_file}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrator()
    try:
        node.run_loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
