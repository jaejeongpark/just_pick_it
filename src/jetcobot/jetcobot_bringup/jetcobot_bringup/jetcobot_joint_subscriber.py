#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, Empty

from pymycobot.mycobot280 import MyCobot280 as MyCobot


CMD_JOINT = 0
CMD_COORD = 1

JOINT_LIMITS = [
    (-168.0, 168.0),
    (-135.0, 135.0),
    (-150.0, 150.0),
    (-145.0, 145.0),
    (-155.0, 160.0),
    (-180.0, 180.0),
]

COORD_LIMITS = [
    (-280.0, 280.0),
    (-280.0, 280.0),
    (-70.0, 523.0),
    (-180.0, 180.0),
    (-180.0, 180.0),
    (-180.0, 180.0),
]


class JetcobotCommandSubscriber(Node):
    def __init__(self):
        super().__init__("jetcobot_command_subscriber")

        self.declare_parameter("port", "/dev/ttyJETCOBOT")
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter("default_speed", 20)

        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.default_speed = int(self.get_parameter("default_speed").value)

        self.mc = MyCobot(self.port, self.baudrate)
        self.mc.thread_lock = True

        self.command_sub = self.create_subscription(
            Float64MultiArray,
            "/jetcobot/target_pose",
            self.command_callback,
            10,
        )

        self.status_request_sub = self.create_subscription(
            Empty,
            "/jetcobot/request_status",
            self.status_request_callback,
            10,
        )

        self.status_pub = self.create_publisher(
            Float64MultiArray,
            "/jetcobot/status",
            10,
        )

        self.get_logger().info("Jetcobot command subscriber started")
        self.get_logger().info("Sub: /jetcobot/target_pose")
        self.get_logger().info("Sub: /jetcobot/request_status")
        self.get_logger().info("Pub: /jetcobot/status")

        self.publish_status()

    def safe_read_6(self, read_fn, name):
        try:
            value = read_fn()
            if isinstance(value, list) and len(value) == 6:
                return [float(v) for v in value]
            self.get_logger().warn(f"{name} invalid: {value}")
        except Exception as e:
            self.get_logger().warn(f"{name} read failed: {e}")

        return [0.0] * 6

    def safe_read_scalar(self, read_fn, name):
        try:
            value = read_fn()
            return float(value)
        except Exception as e:
            self.get_logger().warn(f"{name} read failed: {e}")
            return -1.0

    def publish_status(self):
        tool_reference = self.safe_read_6(
            self.mc.get_tool_reference,
            "tool_reference",
        )
        world_reference = self.safe_read_6(
            self.mc.get_world_reference,
            "world_reference",
        )
        reference_frame = self.safe_read_scalar(
            self.mc.get_reference_frame,
            "reference_frame",
        )
        end_type = self.safe_read_scalar(
            self.mc.get_end_type,
            "end_type",
        )
        angles = self.safe_read_6(
            self.mc.get_angles,
            "angles",
        )
        coords = self.safe_read_6(
            self.mc.get_coords,
            "coords",
        )

        msg = Float64MultiArray()
        msg.data = (
            tool_reference
            + world_reference
            + [reference_frame, end_type]
            + angles
            + coords
        )

        self.status_pub.publish(msg)

        self.get_logger().info(
            f"status published | ref_frame={reference_frame}, end_type={end_type}, "
            f"angles={angles}, coords={coords}"
        )

    def status_request_callback(self, msg):
        self.get_logger().info("status request received")
        self.publish_status()

    def clamp_values(self, values, limits, label):
        clamped = []

        for i, value in enumerate(values):
            low, high = limits[i]
            original = float(value)
            value = max(low, min(high, original))

            if value != original:
                self.get_logger().warn(
                    f"{label}{i+1} {original:.2f} out of range "
                    f"[{low}, {high}], clamped to {value:.2f}"
                )

            clamped.append(float(value))

        return clamped

    def command_callback(self, msg):
        data = list(msg.data)

        if len(data) < 7:
            self.get_logger().warn(
                f"Invalid message length: {len(data)}. Expected at least 7."
            )
            return

        command_type = int(data[0])
        values = data[1:7]

        if len(data) >= 8:
            speed = int(data[7])
        else:
            speed = self.default_speed

        speed = max(1, min(100, speed))

        if len(data) >= 9:
            coord_move_mode = int(data[8])
        else:
            coord_move_mode = 0

        if command_type == CMD_JOINT:
            self.handle_joint_command(values, speed)

        elif command_type == CMD_COORD:
            self.handle_coord_command(values, speed, coord_move_mode)

        else:
            self.get_logger().warn(f"Unknown command_type: {command_type}")

    def handle_joint_command(self, values, speed):
        angles = self.clamp_values(values, JOINT_LIMITS, "J")

        self.get_logger().info(f"send_angles: {angles}, speed={speed}")

        try:
            try:
                self.mc.send_angles(angles, speed, _async=True)
            except TypeError:
                self.mc.send_angles(angles, speed)

        except Exception as e:
            self.get_logger().error(f"send_angles failed: {e}")

    def handle_coord_command(self, values, speed, coord_move_mode):
        coords = self.clamp_values(values, COORD_LIMITS, "C")

        if coord_move_mode not in [0, 1]:
            self.get_logger().warn(
                f"Invalid coord_move_mode={coord_move_mode}. Use 0 instead."
            )
            coord_move_mode = 0

        self.get_logger().info(
            f"send_coords: {coords}, speed={speed}, mode={coord_move_mode}"
        )

        try:
            try:
                self.mc.send_coords(coords, speed, coord_move_mode, _async=True)
            except TypeError:
                self.mc.send_coords(coords, speed, coord_move_mode)

        except Exception as e:
            self.get_logger().error(f"send_coords failed: {e}")


def main(args=None):
    rclpy.init(args=args)

    node = JetcobotCommandSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()