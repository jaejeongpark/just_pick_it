"""
/camera/image_raw 토픽 라이브 프리뷰를 보며 Space로 이미지를 한 장씩 캡처한다.

  Space  : 현재 프레임 저장
  q / ESC: 종료

사용법:
    ros2 run just_pick_it_perception capture_image
    ros2 run just_pick_it_perception capture_image --ros-args \
        -p topic:=/camera/image_raw \
        -p output_dir:=~/img_captures
"""

import glob
import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageCapture(Node):

    def __init__(self):
        super().__init__('image_capture')

        self.declare_parameter('topic', '/camera/image_raw')
        self.declare_parameter('output_dir', os.path.expanduser('~/img_captures'))

        topic = self.get_parameter('topic').get_parameter_value().string_value
        self._output_dir = self.get_parameter('output_dir').get_parameter_value().string_value

        os.makedirs(self._output_dir, exist_ok=True)

        self._bridge = CvBridge()
        self._latest_frame = None
        self._capture_count = 0

        self._sub = self.create_subscription(Image, topic, self._cb, 10)
        self.get_logger().info(f'토픽 구독: {topic}')
        self.get_logger().info(f'저장 위치: {self._output_dir}')
        self.get_logger().info('Space: 캡처 | q/ESC: 종료')

    def _next_index(self) -> int:
        existing = glob.glob(os.path.join(self._output_dir, 'image_*.png'))
        nums = []
        for f in existing:
            stem = os.path.splitext(os.path.basename(f))[0]
            try:
                nums.append(int(stem.split('_', 1)[1]))
            except (IndexError, ValueError):
                pass
        return max(nums, default=0) + 1

    def _cb(self, msg: Image):
        self._latest_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def run_loop(self):
        cv2.namedWindow('Image Capture', cv2.WINDOW_NORMAL)

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self._latest_frame is None:
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    break
                continue

            display = self._latest_frame.copy()
            cv2.putText(display, f'Captured: {self._capture_count}', (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(display, 'Space: capture | q: quit',
                        (10, display.shape[0] - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

            cv2.imshow('Image Capture', display)
            key = cv2.waitKey(1) & 0xFF

            if key == ord(' '):
                filename = os.path.join(self._output_dir, f'image_{self._next_index()}.png')
                cv2.imwrite(filename, self._latest_frame)
                self._capture_count += 1
                self.get_logger().info(f'저장 [{self._capture_count}]: {os.path.basename(filename)}')

            elif key in (ord('q'), 27):
                break

        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ImageCapture()
    try:
        node.run_loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
