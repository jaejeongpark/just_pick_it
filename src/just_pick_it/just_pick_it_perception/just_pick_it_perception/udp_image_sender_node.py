import struct
import socket

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

HEADER_FMT = '>IHH'   # frame_id (uint32), packet_idx (uint16), total_packets (uint16)
HEADER_SIZE = struct.calcsize(HEADER_FMT)
MAX_CHUNK = 60000


class UdpImageSenderNode(Node):
    def __init__(self):
        super().__init__('udp_image_sender')

        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('dest_ip', '192.168.1.73')
        self.declare_parameter('dest_port', 9870)
        self.declare_parameter('jpeg_quality', 80)

        topic = self.get_parameter('image_topic').value
        dest_ip = self.get_parameter('dest_ip').value
        dest_port = self.get_parameter('dest_port').value
        self._quality = self.get_parameter('jpeg_quality').value

        self._bridge = CvBridge()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._dest = (dest_ip, dest_port)
        self._frame_id = 0

        self.create_subscription(Image, topic, self._cb, 10)
        self.get_logger().info(f'Streaming {topic} -> {dest_ip}:{dest_port}')

    def _cb(self, msg: Image):
        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        ok, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self._quality])
        if not ok:
            return

        data = buf.tobytes()
        chunks = [data[i:i + MAX_CHUNK] for i in range(0, len(data), MAX_CHUNK)]
        total = len(chunks)

        for idx, chunk in enumerate(chunks):
            header = struct.pack(HEADER_FMT, self._frame_id, idx, total)
            self._sock.sendto(header + chunk, self._dest)

        self._frame_id = (self._frame_id + 1) % 0xFFFFFFFF


def main(args=None):
    rclpy.init(args=args)
    node = UdpImageSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
