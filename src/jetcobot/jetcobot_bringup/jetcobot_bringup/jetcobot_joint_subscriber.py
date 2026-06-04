#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray

from pymycobot.mycobot import MyCobot
# 만약 MyCobot280을 써야 하면 아래로 교체
# from pymycobot.mycobot280 import MyCobot280 as MyCobot


JOINT_LIMITS = [
    (-168.0, 168.0),   # J1
    (-135.0, 135.0),   # J2
    (-150.0, 150.0),   # J3
    (-145.0, 145.0),   # J4
    (-155.0, 160.0),   # J5
    (-180.0, 180.0),   # J6
]


class JetcobotJointSubscriber(Node):
    def __init__(self):
        super().__init__("jetcobot_joint_subscriber")

        self.declare_parameter("port", "/dev/ttyJETCOBOT")
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter("default_speed", 20)

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.default_speed = int(self.get_parameter("default_speed").value)

        self.mc = MyCobot(self.port, self.baudrate)
        self.mc.thread_lock = True

        self.sub = self.create_subscription(
            Float64MultiArray,
            "/jetcobot/target_angles",
            self.angle_callback,
            10,
        )

        self.get_logger().info("Jetcobot joint subscriber started")
        self.get_logger().info(f"port={self.port}, baudrate={self.baudrate}")

    def clamp_angles(self, angles):
        clamped = []

        for i, angle in enumerate(angles):
            low, high = JOINT_LIMITS[i]

            if angle < low:
                self.get_logger().warn(
                    f"J{i+1} angle {angle:.2f} below limit {low:.2f}. Clamped."
                )
                angle = low

            if angle > high:
                self.get_logger().warn(
                    f"J{i+1} angle {angle:.2f} above limit {high:.2f}. Clamped."
                )
                angle = high

            clamped.append(float(angle))

        return clamped

    def angle_callback(self, msg):
        data = list(msg.data)

        if len(data) == 6:
            angles = data
            speed = self.default_speed
        elif len(data) == 7:
            angles = data[:6]
            speed = int(data[6])
        else:
            self.get_logger().warn(
                f"Invalid message length: {len(data)}. Expected 6 or 7."
            )
            return

        angles = self.clamp_angles(angles)

        self.get_logger().info(f"send_angles: {angles}, speed={speed}")

        try:
            try:
                self.mc.send_angles(angles, speed, _async=True)
            except TypeError:
                self.mc.send_angles(angles, speed)
        except Exception as e:
            self.get_logger().error(f"send_angles failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = JetcobotJointSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()