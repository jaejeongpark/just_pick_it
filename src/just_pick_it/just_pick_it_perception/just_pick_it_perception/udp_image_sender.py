import struct
import socket

import cv2
import rclpy
from rclpy.node import Node

from picamera2 import Picamera2

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


HEADER_FMT = '>IHH'   # frame_id(uint32), packet_idx(uint16), total_packets(uint16)
HEADER_SIZE = struct.calcsize(HEADER_FMT)
MAX_CHUNK = 60000


class UdpCameraSenderNode(Node):
    def __init__(self):
        super().__init__('udp_camera_sender')

        # Camera parameters
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30)

        # UDP parameters
        self.declare_parameter('dest_ip', '192.168.1.21')
        self.declare_parameter('dest_port', 9870)
        self.declare_parameter('jpeg_quality', 80)

        # 보드 로컬 노드(reverse_docking 마커 검출)가 쓰도록 ROS Image 토픽도 발행한다.
        # UDP 는 관제 PC 원격 뷰어용, ROS Image 는 보드 내 처리용. 끄려면 false.
        self.declare_parameter('publish_ros_image', True)
        self.declare_parameter('image_frame_id', 'camera_link')

        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)

        self.dest_ip = self.get_parameter('dest_ip').value
        self.dest_port = int(self.get_parameter('dest_port').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.publish_ros = bool(self.get_parameter('publish_ros_image').value)
        self.image_frame_id = self.get_parameter('image_frame_id').value

        self._bridge = CvBridge()
        self.image_pub = (
            self.create_publisher(Image, 'camera/image_raw', 10)
            if self.publish_ros else None
        )

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.dest = (self.dest_ip, self.dest_port)
        self.frame_id = 0

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

        self.get_logger().info(
            f'UDP camera streaming started: {self.width}x{self.height} @ {self.fps} fps'
        )
        self.get_logger().info(
            f'Destination: {self.dest_ip}:{self.dest_port}, JPEG quality={self.jpeg_quality}'
        )

    def timer_cb(self):
        frame = self.picam2.capture_array()
        self.send_udp_image(frame)
        self.publish_ros_image(frame)

    def publish_ros_image(self, frame_bgr):
        if self.image_pub is None:
            return
        try:
            # Picamera2 RGB888 배열은 BGR 바이트 순서(UDP 경로도 BGR 로 인코딩) → bgr8.
            msg = self._bridge.cv2_to_imgmsg(frame_bgr, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.image_frame_id
            self.image_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f'ROS image publish error: {e}')

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
                self.frame_id,
                packet_idx,
                total_packets
            )
            self.sock.sendto(header + chunk, self.dest)

        self.frame_id = (self.frame_id + 1) % 0xFFFFFFFF

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

    node = UdpCameraSenderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()