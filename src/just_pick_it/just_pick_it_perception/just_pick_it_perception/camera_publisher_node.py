import struct
import socket

import cv2
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import Header
from cv_bridge import CvBridge

from picamera2 import Picamera2


HEADER_FMT = '>IHH'   # frame_id(uint32), packet_idx(uint16), total_packets(uint16)
HEADER_SIZE = struct.calcsize(HEADER_FMT)
MAX_CHUNK = 60000


class PiCameraUdpPublisher(Node):
    def __init__(self):
        super().__init__('pi_camera_udp_publisher')

        # ROS image topic
        self.declare_parameter('topic', '/camera/image_raw')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('frame_id', 'camera_link')

        # UDP streaming
        self.declare_parameter('enable_udp', True)
        self.declare_parameter('dest_ip', '192.168.1.73')
        self.declare_parameter('dest_port', 9870)
        self.declare_parameter('jpeg_quality', 80)

        topic = self.get_parameter('topic').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.frame_id = self.get_parameter('frame_id').value

        self.enable_udp = bool(self.get_parameter('enable_udp').value)
        self.dest_ip = self.get_parameter('dest_ip').value
        self.dest_port = int(self.get_parameter('dest_port').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, topic, 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dest = (self.dest_ip, self.dest_port)
        self.udp_frame_id = 0

        self.picam2 = Picamera2()

        config = self.picam2.create_video_configuration(
            main={
                "size": (self.width, self.height),
                "format": "RGB888",
            }
        )

        self.picam2.configure(config)
        self.picam2.start()

        self.timer = self.create_timer(1.0 / self.fps, self.timer_cb)

        self.get_logger().info(f'ROS image publish: {topic}')
        self.get_logger().info(f'Camera: {self.width}x{self.height} @ {self.fps} fps')
        self.get_logger().info(f'frame_id: {self.frame_id}')

        if self.enable_udp:
            self.get_logger().info(
                f'UDP streaming enabled: {self.dest_ip}:{self.dest_port}, '
                f'JPEG quality={self.jpeg_quality}'
            )
        else:
            self.get_logger().info('UDP streaming disabled')

    def timer_cb(self):
        # Picamera2 output: RGB
        frame_rgb = self.picam2.capture_array()

        # OpenCV / ROS bgr8 용도
        frame_bgr = cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR)

        self.publish_ros_image(frame_bgr)

        if self.enable_udp:
            self.send_udp_image(frame_bgr)

    def publish_ros_image(self, frame_bgr):
        msg = self.bridge.cv2_to_imgmsg(frame_bgr, encoding='bgr8')
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id

        self.image_pub.publish(msg)

    def send_udp_image(self, frame_bgr):
        ok, buf = cv2.imencode(
            '.jpg',
            frame_bgr,
            [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        )

        if not ok:
            self.get_logger().warn('JPEG encoding failed')
            return

        data = buf.tobytes()

        chunks = [
            data[i:i + MAX_CHUNK]
            for i in range(0, len(data), MAX_CHUNK)
        ]

        total_packets = len(chunks)

        if total_packets > 0xFFFF:
            self.get_logger().warn(
                f'Too many UDP packets for one frame: {total_packets}'
            )
            return

        for packet_idx, chunk in enumerate(chunks):
            header = struct.pack(
                HEADER_FMT,
                self.udp_frame_id,
                packet_idx,
                total_packets
            )
            self.sock.sendto(header + chunk, self.dest)

        self.udp_frame_id = (self.udp_frame_id + 1) % 0xFFFFFFFF

    def destroy_node(self):
        try:
            self.picam2.stop()
        except Exception:
            pass

        try:
            self.sock.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PiCameraUdpPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()