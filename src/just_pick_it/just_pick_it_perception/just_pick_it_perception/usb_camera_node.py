"""
USB 카메라를 열어 /camera/image_raw 토픽으로 퍼블리시하는 ROS2 노드.

사용법:
    ros2 run just_pick_it_perception usb_camera
    ros2 run just_pick_it_perception usb_camera --ros-args \
        -p device_id:=2 \
        -p topic:=/camera/image_raw \
        -p width:=640 \
        -p height:=480 \
        -p fps:=30
"""

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class UsbCameraNode(Node):

    def __init__(self):
        super().__init__('usb_camera')

        self.declare_parameter('device_id', 2)
        self.declare_parameter('topic', '/camera/image_raw')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)

        device_id = self.get_parameter('device_id').get_parameter_value().integer_value
        topic = self.get_parameter('topic').get_parameter_value().string_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value
        fps = self.get_parameter('fps').get_parameter_value().integer_value

        self._bridge = CvBridge()
        self._pub = self.create_publisher(Image, topic, 10)

        self._cap = cv2.VideoCapture(device_id)
        if not self._cap.isOpened():
            self.get_logger().error(f'/dev/video{device_id} 열기 실패')
            raise RuntimeError(f'Cannot open /dev/video{device_id}')

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self._cap.set(cv2.CAP_PROP_FPS, fps)

        actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self._cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'/dev/video{device_id} 열림: {actual_w}x{actual_h} @ {actual_fps:.1f}fps'
        )
        self.get_logger().info(f'퍼블리시 토픽: {topic}')

        timer_period = 1.0 / fps
        self._timer = self.create_timer(timer_period, self._timer_cb)

    def _timer_cb(self):
        ret, frame = self._cap.read()
        if not ret:
            self.get_logger().warn('프레임 읽기 실패, 스킵')
            return

        msg = self._bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'usb_camera'
        self._pub.publish(msg)

    def destroy_node(self):
        if self._cap.isOpened():
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = UsbCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
