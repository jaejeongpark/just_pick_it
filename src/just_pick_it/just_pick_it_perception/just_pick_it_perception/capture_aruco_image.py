"""
/camera/image_raw 토픽에서 이미지를 주기적으로 수신하여 PNG 파일로 저장한다.
Ctrl+C로 종료할 때까지 interval 초 간격으로 image_N.png 형식으로 저장한다.

사용법:
    ros2 run just_pick_it_perception capture_aruco_image
    ros2 run just_pick_it_perception capture_aruco_image --ros-args \
        -p output_dir:=/tmp/calib_imgs \
        -p interval:=2.0 \
        -p topic:=/image_raw
"""

import glob
import os
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class ArucoImageCapture(Node):

    def __init__(self):
        super().__init__('aruco_image_capture')

        self.declare_parameter('topic', '/camera/image_raw')
        self.declare_parameter('output_dir', os.path.expanduser('~/aruco_captures'))
        self.declare_parameter('interval', 1.0)

        topic = self.get_parameter('topic').get_parameter_value().string_value
        self._output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self._interval = self.get_parameter('interval').get_parameter_value().double_value

        os.makedirs(self._output_dir, exist_ok=True)

        self._bridge = CvBridge()
        self._last_saved = 0.0

        self._sub = self.create_subscription(Image, topic, self._cb, 10)
        self.get_logger().info(
            f'[{topic}] 구독 시작. {self._interval}초 간격으로 저장. 종료: Ctrl+C')

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
        now = time.monotonic()
        if now - self._last_saved < self._interval:
            return

        self._last_saved = now
        img = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        filename = os.path.join(self._output_dir, f'image_{self._next_index()}.png')
        cv2.imwrite(filename, img)
        self.get_logger().info(f'Saved: {filename}  ({msg.width}x{msg.height})')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoImageCapture()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
