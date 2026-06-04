#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, Empty

from pymycobot.mycobot280 import MyCobot280


CMD_JOINT = 0
CMD_COORD = 1

GRIPPER_MIN = 0.0
GRIPPER_MAX = 100.0


JOINT_LIMITS = [
    (-168.0, 168.0),   # J1
    (-135.0, 135.0),   # J2
    (-150.0, 150.0),   # J3
    (-145.0, 145.0),   # J4
    (-155.0, 160.0),   # J5
    (-180.0, 180.0),   # J6
]

COORD_LIMITS = [
    (-280.0, 280.0),   # x
    (-280.0, 280.0),   # y
    (-70.0, 523.0),    # z
    (-180.0, 180.0),   # rx
    (-180.0, 180.0),   # ry
    (-180.0, 180.0),   # rz
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

        self.mc = MyCobot280(self.port, self.baudrate)
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

        self.tool_reference_sub = self.create_subscription(
            Float64MultiArray,
            "/jetcobot/set_tool_reference",
            self.tool_reference_callback,
            10,
        )

        self.gripper_sub = self.create_subscription(
            Float64MultiArray,
            "/jetcobot/set_gripper",
            self.gripper_callback,
            10,
        )

        self.status_pub = self.create_publisher(
            Float64MultiArray,
            "/jetcobot/status",
            10,
        )

        self.get_logger().info("Jetcobot command subscriber started")
        self.get_logger().info(f"port={self.port}, baudrate={self.baudrate}")
        self.get_logger().info("Sub: /jetcobot/target_pose")
        self.get_logger().info("Sub: /jetcobot/request_status")
        self.get_logger().info("Sub: /jetcobot/set_tool_reference")
        self.get_logger().info("Sub: /jetcobot/set_gripper")
        self.get_logger().info("Pub: /jetcobot/status")
        self.get_logger().info(
            "/jetcobot/target_pose data = "
            "[command_type, v1, v2, v3, v4, v5, v6, speed, coord_move_mode]"
        )
        self.get_logger().info(
            "/jetcobot/set_gripper data = [gripper_value, speed]"
        )

        self.publish_status()

    def safe_read_6(self, func_name, default, label):
        if not hasattr(self.mc, func_name):
            self.get_logger().warn(f"{label}: API not available: {func_name}")
            return default

        try:
            fn = getattr(self.mc, func_name)
            value = fn()

            if isinstance(value, list) and len(value) == 6:
                return [float(v) for v in value]

            self.get_logger().warn(f"{label} invalid: {value}")
            return default

        except Exception as e:
            self.get_logger().warn(f"{label} read failed: {e}")
            return default

    def safe_read_scalar(self, func_name, default, label):
        if not hasattr(self.mc, func_name):
            self.get_logger().warn(f"{label}: API not available: {func_name}")
            return default

        try:
            fn = getattr(self.mc, func_name)
            value = fn()
            return float(value)

        except Exception as e:
            self.get_logger().warn(f"{label} read failed: {e}")
            return default

    def publish_status(self):
        tool_reference = self.safe_read_6(
            "get_tool_reference",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "tool_reference",
        )

        world_reference = self.safe_read_6(
            "get_world_reference",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "world_reference",
        )

        reference_frame = self.safe_read_scalar(
            "get_reference_frame",
            -1.0,
            "reference_frame",
        )

        end_type = self.safe_read_scalar(
            "get_end_type",
            -1.0,
            "end_type",
        )

        angles = self.safe_read_6(
            "get_angles",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "angles",
        )

        coords = self.safe_read_6(
            "get_coords",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "coords",
        )

        gripper_value = self.safe_read_scalar(
            "get_gripper_value",
            -1.0,
            "gripper_value",
        )

        # status data layout:
        # 0~5   : tool_reference
        # 6~11  : world_reference
        # 12    : reference_frame
        # 13    : end_type
        # 14~19 : current_angles
        # 20~25 : current_coords
        # 26    : gripper_value
        msg = Float64MultiArray()
        msg.data = (
            tool_reference
            + world_reference
            + [reference_frame, end_type]
            + angles
            + coords
            + [gripper_value]
        )

        self.status_pub.publish(msg)

        self.get_logger().info(
            f"status published | "
            f"tool_ref={tool_reference}, "
            f"ref_frame={reference_frame}, "
            f"end_type={end_type}, "
            f"angles={angles}, "
            f"coords={coords}, "
            f"gripper={gripper_value}"
        )

    def status_request_callback(self, msg):
        self.get_logger().info("status request received")
        self.publish_status()

    def tool_reference_callback(self, msg):
        data = list(msg.data)

        if len(data) != 6:
            self.get_logger().warn(
                f"Invalid tool_reference length: {len(data)}. Expected 6."
            )
            return

        tool_reference = [float(v) for v in data]

        self.get_logger().info(f"set_tool_reference: {tool_reference}")

        try:
            self.mc.set_tool_reference(tool_reference)
            self.publish_status()

        except Exception as e:
            self.get_logger().error(f"set_tool_reference failed: {e}")

    def gripper_callback(self, msg):
        data = list(msg.data)

        if len(data) < 1:
            self.get_logger().warn(
                f"Invalid gripper message length: {len(data)}. "
                "Expected [value, speed]."
            )
            return

        value = float(data[0])

        if len(data) >= 2:
            speed = int(data[1])
        else:
            speed = self.default_speed

        value = max(GRIPPER_MIN, min(GRIPPER_MAX, value))
        speed = max(1, min(100, speed))

        self.get_logger().info(f"set_gripper_value: value={value}, speed={speed}")

        try:
            try:
                self.mc.set_gripper_value(int(value), speed, _async=True)
            except TypeError:
                self.mc.set_gripper_value(int(value), speed)

            self.publish_status()

        except Exception as e:
            self.get_logger().error(f"set_gripper_value failed: {e}")

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
                f"Invalid target_pose length: {len(data)}. Expected at least 7."
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