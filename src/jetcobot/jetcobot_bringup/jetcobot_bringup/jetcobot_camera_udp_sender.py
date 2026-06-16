#!/usr/bin/env python3

import socket
import struct

import cv2
import rclpy
from rclpy.node import Node


HEADER_FMT = ">IHH"
MAX_CHUNK = 60000


class JetcobotCameraUdpSender(Node):
    def __init__(self):
        super().__init__("jetcobot_camera_udp_sender")

        self.declare_parameter("camera_device", "/dev/jetcocam0")
        self.declare_parameter("dest_ip", "192.168.1.21")
        self.declare_parameter("dest_port", 5003)
        self.declare_parameter("dest_port2", 0)
        self.declare_parameter("jpeg_quality", 80)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("width", 0)
        self.declare_parameter("height", 0)

        self.camera_device = self.get_parameter("camera_device").value
        self.dest_ip = self.get_parameter("dest_ip").value
        self.dest_port = int(self.get_parameter("dest_port").value)
        self.dest_port2 = int(self.get_parameter("dest_port2").value)
        self.jpeg_quality = int(self.get_parameter("jpeg_quality").value)
        self.fps = float(self.get_parameter("fps").value)
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)

        self.jpeg_quality = max(1, min(100, self.jpeg_quality))
        self.fps = max(1.0, min(60.0, self.fps))

        self.camera = cv2.VideoCapture(self.camera_device)

        if self.width > 0:
            self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        if self.height > 0:
            self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        if not self.camera.isOpened():
            raise RuntimeError(f"Cannot open camera: {self.camera_device}")

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.frame_id = 0

        self.encode_param = [
            int(cv2.IMWRITE_JPEG_QUALITY),
            self.jpeg_quality,
        ]

        self.get_logger().info(
            f"Streaming camera UDP only: {self.camera_device} -> "
            f"{self.dest_ip}:{self.dest_port}"
        )
        self.get_logger().info(
            f"Resolution: {self.camera.get(cv2.CAP_PROP_FRAME_WIDTH)} x "
            f"{self.camera.get(cv2.CAP_PROP_FRAME_HEIGHT)}"
        )
        self.get_logger().info(f"JPEG quality: {self.jpeg_quality}, fps: {self.fps}")

        self.timer = self.create_timer(1.0 / self.fps, self.timer_callback)

    def timer_callback(self):
        success, img = self.camera.read()

        if not success:
            self.get_logger().warn("Failed to read camera frame")
            return

        success, buffer = cv2.imencode(".jpg", img, self.encode_param)

        if not success:
            self.get_logger().warn("JPEG encode failed")
            return

        data = buffer.tobytes()
        total_packets = (len(data) + MAX_CHUNK - 1) // MAX_CHUNK

        for packet_idx in range(total_packets):
            start = packet_idx * MAX_CHUNK
            end = start + MAX_CHUNK
            chunk = data[start:end]

            header = struct.pack(
                HEADER_FMT,
                self.frame_id,
                packet_idx,
                total_packets,
            )

            try:
                self.sock.sendto(header + chunk, (self.dest_ip, self.dest_port))
                if self.dest_port2 > 0:
                    self.sock.sendto(header + chunk, (self.dest_ip, self.dest_port2))
            except Exception as e:
                self.get_logger().warn(f"UDP send failed: {e}")
                return

        self.frame_id = (self.frame_id + 1) % 4294967295

    def destroy_node(self):
        try:
            self.camera.release()
        except Exception:
            pass

        try:
            self.sock.close()
        except Exception:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = JetcobotCameraUdpSender()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()