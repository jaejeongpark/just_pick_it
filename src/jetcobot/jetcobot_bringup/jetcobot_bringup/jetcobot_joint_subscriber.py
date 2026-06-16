#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Empty
from pymycobot.mycobot280 import MyCobot280


CMD_JOINT = 0
CMD_COORD = 1

ARM_RELEASE = 0
ARM_POWER_ON = 1

GRIPPER_MIN = 0.0
GRIPPER_MAX = 100.0

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

        self.declare_parameter("robot_name", "jetcobot1")
        self.declare_parameter("port", "/dev/ttyJETCOBOT")
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter("default_speed", 20)
        # status publish 시 거의 안 변하는 config(tool/world/reference/end_type)는 매번
        # 시리얼로 읽지 않고 캐시한다. publish_status를 7 read -> 3 read로 줄여 status
        # rate를 높이고 지연을 낮춘다(human 기록/NN 추론 피드백 충실도 향상).
        self.declare_parameter("status_cache_config", True)

        self.robot_name = self.get_parameter("robot_name").value
        self.port = self.get_parameter("port").value
        self.baudrate = int(self.get_parameter("baudrate").value)
        self.default_speed = int(self.get_parameter("default_speed").value)
        self.status_cache_config = bool(self.get_parameter("status_cache_config").value)

        self.ns = f"/{self.robot_name}"

        self.mc = MyCobot280(self.port, self.baudrate)
        self.mc.thread_lock = True

        self.command_sub = self.create_subscription(
            Float64MultiArray,
            f"{self.ns}/target_pose",
            self.command_callback,
            10,
        )

        self.status_request_sub = self.create_subscription(
            Empty,
            f"{self.ns}/request_status",
            self.status_request_callback,
            10,
        )

        self.tool_reference_sub = self.create_subscription(
            Float64MultiArray,
            f"{self.ns}/set_tool_reference",
            self.tool_reference_callback,
            10,
        )

        self.gripper_sub = self.create_subscription(
            Float64MultiArray,
            f"{self.ns}/set_gripper",
            self.gripper_callback,
            10,
        )

        self.arm_sub = self.create_subscription(
            Float64MultiArray,
            f"{self.ns}/set_arm",
            self.arm_callback,
            10,
        )

        self.status_pub = self.create_publisher(
            Float64MultiArray,
            f"{self.ns}/status",
            10,
        )

        self.get_logger().info("Jetcobot command subscriber started")
        self.get_logger().info(f"robot_name={self.robot_name}")
        self.get_logger().info(f"namespace={self.ns}")
        self.get_logger().info(f"port={self.port}, baudrate={self.baudrate}")
        self.get_logger().info(f"Sub: {self.ns}/target_pose")
        self.get_logger().info(f"Sub: {self.ns}/request_status")
        self.get_logger().info(f"Sub: {self.ns}/set_tool_reference")
        self.get_logger().info(f"Sub: {self.ns}/set_gripper")
        self.get_logger().info(f"Sub: {self.ns}/set_arm")
        self.get_logger().info(f"Pub: {self.ns}/status")

        # 거의 안 변하는 config는 시작 시 1회 읽어 캐시한다(set 시 갱신).
        self._cfg_tool_reference = self.safe_read_6(
            "get_tool_reference", [0.0] * 6, "tool_reference")
        self._cfg_world_reference = self.safe_read_6(
            "get_world_reference", [0.0] * 6, "world_reference")
        self._cfg_reference_frame = self.safe_read_scalar(
            "get_reference_frame", -1.0, "reference_frame")
        self._cfg_end_type = self.safe_read_scalar(
            "get_end_type", -1.0, "end_type")
        self.get_logger().info(
            f"status_cache_config={self.status_cache_config} "
            f"(cached config: tool/world/reference/end_type)"
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
        if self.status_cache_config:
            # 캐시된 config 사용(시리얼 read 생략). angles+coords+gripper만 live로 읽는다.
            tool_reference = list(self._cfg_tool_reference)
            world_reference = list(self._cfg_world_reference)
            reference_frame = self._cfg_reference_frame
            end_type = self._cfg_end_type
        else:
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

    def status_request_callback(self, msg):
        self.get_logger().info(f"status request received: {self.ns}/request_status")
        self.publish_status()

    def tool_reference_callback(self, msg):
        data = list(msg.data)

        if len(data) != 6:
            self.get_logger().warn(
                f"Invalid tool_reference length: {len(data)}. Expected 6."
            )
            return

        tool_reference = [float(v) for v in data]

        try:
            self.get_logger().info(f"set_tool_reference: {tool_reference}")
            self.mc.set_tool_reference(tool_reference)
            # 캐시 갱신(set 값 반영).
            self._cfg_tool_reference = [float(v) for v in tool_reference]
            self.publish_status()
        except Exception as e:
            self.get_logger().error(f"set_tool_reference failed: {e}")

    def gripper_callback(self, msg):
        data = list(msg.data)

        if len(data) < 1:
            self.get_logger().warn(
                f"Invalid gripper message length: {len(data)}. Expected [value, speed]."
            )
            return

        value = float(data[0])
        speed = int(data[1]) if len(data) >= 2 else self.default_speed

        value = max(GRIPPER_MIN, min(GRIPPER_MAX, value))
        speed = max(1, min(100, speed))

        try:
            self.get_logger().info(f"set_gripper_value: value={value}, speed={speed}")

            try:
                self.mc.set_gripper_value(int(value), speed, _async=True)
            except TypeError:
                self.mc.set_gripper_value(int(value), speed)

            self.publish_status()

        except Exception as e:
            self.get_logger().error(f"set_gripper_value failed: {e}")

    def arm_callback(self, msg):
        data = list(msg.data)

        if len(data) < 1:
            self.get_logger().warn("Invalid arm command. Expected [0] or [1].")
            return

        cmd = int(data[0])

        try:
            if cmd == ARM_RELEASE:
                self.get_logger().warn("DISARM requested: release_all_servos()")
                self.mc.release_all_servos()

            elif cmd == ARM_POWER_ON:
                self.get_logger().info("ARM requested: power_on()")
                self.mc.power_on()

            else:
                self.get_logger().warn(f"Unknown arm command: {cmd}")
                return

            self.publish_status()

        except Exception as e:
            self.get_logger().error(f"arm command failed: {e}")

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

        speed = int(data[7]) if len(data) >= 8 else self.default_speed
        speed = max(1, min(100, speed))

        coord_move_mode = int(data[8]) if len(data) >= 9 else 0

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