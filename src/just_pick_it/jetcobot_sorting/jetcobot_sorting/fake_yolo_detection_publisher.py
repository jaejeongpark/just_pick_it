#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class FakeYoloDetectionPublisher(Node):
    def __init__(self):
        super().__init__("fake_yolo_detection_publisher")

        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("publish_rate", 10.0)

        self.declare_parameter("bbox_width", 100.0)
        self.declare_parameter("bbox_height", 120.0)
        self.declare_parameter("score", 0.95)

        # fake target motion
        self.declare_parameter("amplitude_x", 160.0)
        self.declare_parameter("amplitude_y", 100.0)
        self.declare_parameter("motion_period", 6.0)

        self.image_width = float(self.get_parameter("image_width").value)
        self.image_height = float(self.get_parameter("image_height").value)
        self.publish_rate = float(self.get_parameter("publish_rate").value)

        self.bbox_width = float(self.get_parameter("bbox_width").value)
        self.bbox_height = float(self.get_parameter("bbox_height").value)
        self.score = float(self.get_parameter("score").value)

        self.amplitude_x = float(self.get_parameter("amplitude_x").value)
        self.amplitude_y = float(self.get_parameter("amplitude_y").value)
        self.motion_period = float(self.get_parameter("motion_period").value)

        self.center_x = self.image_width / 2.0
        self.center_y = self.image_height / 2.0

        self.start_time = time.time()

        self.pub = self.create_publisher(
            Float32MultiArray,
            "/target_bbox",
            10,
        )

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info("Fake YOLO detection publisher started")
        self.get_logger().info(
            f"Publishing Float32MultiArray [cx, cy, w, h, score] to /target_bbox"
        )

    def timer_callback(self):
        t = time.time() - self.start_time

        phase = 2.0 * math.pi * t / self.motion_period

        # 화면 중심을 기준으로 target bbox 중심을 천천히 움직임
        cx = self.center_x + self.amplitude_x * math.sin(phase)
        cy = self.center_y + self.amplitude_y * math.cos(phase)

        msg = Float32MultiArray()
        msg.data = [
            float(cx),
            float(cy),
            float(self.bbox_width),
            float(self.bbox_height),
            float(self.score),
        ]

        self.pub.publish(msg)

        self.get_logger().info(
            f"fake bbox: cx={cx:.1f}, cy={cy:.1f}, "
            f"w={self.bbox_width:.1f}, h={self.bbox_height:.1f}, score={self.score:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = FakeYoloDetectionPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()