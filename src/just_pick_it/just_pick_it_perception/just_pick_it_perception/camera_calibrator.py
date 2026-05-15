"""
checkerboard 이미지 디렉터리를 읽어 카메라 내부 파라미터(intrinsic matrix K,
왜곡 계수 dist)를 계산하고 ROS camera_info 호환 YAML로 저장한다.

board_width / board_height 는 체커보드의 내부 코너(inner corner) 수를 의미한다.
  예) 9x7 사각형 보드 -> 내부 코너 8x6  -> board_width=8, board_height=6

사용법:
    ros2 run just_pick_it_perception camera_calibrator
    ros2 run just_pick_it_perception camera_calibrator --ros-args \
        -p image_dir:=/tmp/calib_imgs \
        -p board_width:=8 \
        -p board_height:=6 \
        -p square_size:=0.025 \
        -p output_file:=$HOME/camera_calibration.yaml
"""

import glob
import os

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node


class CameraCalibrator(Node):

    def __init__(self):
        super().__init__('camera_calibrator')

        self.declare_parameter('image_dir', os.path.expanduser('~/aruco_captures'))
        self.declare_parameter('board_width', 8)
        self.declare_parameter('board_height', 6)
        self.declare_parameter('square_size', 0.025)
        self.declare_parameter('output_file', os.path.expanduser('~/camera_calibration.yaml'))

        image_dir = os.path.expanduser(
            self.get_parameter('image_dir').get_parameter_value().string_value)
        board_w = self.get_parameter('board_width').get_parameter_value().integer_value
        board_h = self.get_parameter('board_height').get_parameter_value().integer_value
        square_size = self.get_parameter('square_size').get_parameter_value().double_value
        output_file = os.path.expanduser(
            self.get_parameter('output_file').get_parameter_value().string_value)

        self._run(image_dir, board_w, board_h, square_size, output_file)

    def _run(self, image_dir, board_w, board_h, square_size, output_file):
        pattern_size = (board_w, board_h)

        # 체커보드 한 장에 대응하는 3D 실세계 좌표 (z=0 평면)
        objp = np.zeros((board_h * board_w, 3), np.float32)
        objp[:, :2] = np.mgrid[0:board_w, 0:board_h].T.reshape(-1, 2)
        objp *= square_size

        obj_points = []
        img_points = []

        paths = sorted(
            glob.glob(os.path.join(image_dir, '*.png')) +
            glob.glob(os.path.join(image_dir, '*.jpg'))
        )

        if not paths:
            self.get_logger().error(f'이미지를 찾을 수 없음: {image_dir}')
            return

        self.get_logger().info(f'총 {len(paths)}장 발견, 코너 검출 시작...')

        img_size = None
        refine_criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        for path in paths:
            img = cv2.imread(path)
            if img is None:
                self.get_logger().warn(f'  읽기 실패: {os.path.basename(path)}')
                continue

            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            if img_size is None:
                img_size = (gray.shape[1], gray.shape[0])  # (width, height)

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
            self.get_logger().error(
                f'유효 이미지 부족 ({valid}장). 최소 10장 필요. 종료.')
            return

        ret, K, dist, _, _ = cv2.calibrateCamera(
            obj_points, img_points, img_size, None, None)

        self.get_logger().info(f'재투영 오차 (RMS): {ret:.4f} px')
        self.get_logger().info(f'Intrinsic matrix K:\n{K}')
        self.get_logger().info(f'Distortion coefficients: {dist.ravel().tolist()}')

        self._save_yaml(output_file, img_size, K, dist)

    def _save_yaml(self, output_file, img_size, K, dist):
        output_dir = os.path.dirname(output_file)
        if output_dir:
            os.makedirs(output_dir, exist_ok=True)

        # ROS sensor_msgs/CameraInfo 와 호환되는 포맷
        calib = {
            'image_width': img_size[0],
            'image_height': img_size[1],
            'camera_name': 'pinky_camera',
            'distortion_model': 'plumb_bob',
            'camera_matrix': {
                'rows': 3,
                'cols': 3,
                'data': K.ravel().tolist(),
            },
            'distortion_coefficients': {
                'rows': 1,
                'cols': int(dist.size),
                'data': dist.ravel().tolist(),
            },
            'rectification_matrix': {
                'rows': 3,
                'cols': 3,
                'data': np.eye(3).ravel().tolist(),
            },
            'projection_matrix': {
                'rows': 3,
                'cols': 4,
                'data': np.hstack([K, np.zeros((3, 1))]).ravel().tolist(),
            },
        }

        with open(output_file, 'w') as f:
            yaml.dump(calib, f, default_flow_style=False, sort_keys=False)

        self.get_logger().info(f'캘리브레이션 결과 저장: {output_file}')


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrator()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
