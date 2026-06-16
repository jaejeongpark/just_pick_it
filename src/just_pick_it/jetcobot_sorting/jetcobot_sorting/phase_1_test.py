#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from pymycobot.mycobot280 import MyCobot280


class JetcobotScanningMover(Node):
    def __init__(self):
        super().__init__("jetcobot_scanning_mover")

        # Parameters
        self.declare_parameter("port", "/dev/ttyJETCOBOT")
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter("speed", 20)
        self.declare_parameter("repeat", 10)

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.speed = self.get_parameter("speed").value
        self.repeat = self.get_parameter("repeat").value

        # -----------------------------
        # Joint angle targets
        # -----------------------------

        self.home_angles = [
            -79.27, 2.19, -0.79, 2.72, 5.88, -131.57
        ]

        self.phase1_center_angles = [
            -82.88, 56.42, -19.86, -93.51, 16.78, -124.71
        ]

        self.phase1_left_angles = [
            -82.88, 56.51, -19.33, -93.60, 24.96, -121.46
        ]

        self.phase1_right_angles = [
            -82.88, 56.51, -19.77, -94.65, 2.10, -129.55
        ]

        # 실행 순서:
        # 1. home 1회
        # 2. center -> left -> right 반복
        self.scan_waypoints = [
            ("phase1_center", self.phase1_center_angles),
            ("phase1_left", self.phase1_left_angles),
            ("phase1_right", self.phase1_right_angles),
        ]

        self.mc = MyCobot280(self.port, self.baudrate)
        self.mc.thread_lock = True

        try:
            self.mc.set_fresh_mode(1)
        except Exception as e:
            self.get_logger().warn(f"set_fresh_mode failed: {e}")

        # State
        self.state = "MOVE_HOME"

        self.current_scan_idx = 0
        self.current_repeat = 0

        self.command_sent = False
        self.last_command_time = None

        self.motion_timeout_sec = 15.0

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Jetcobot scanning mover node started")
        self.get_logger().info(f"port={self.port}, baudrate={self.baudrate}")
        self.get_logger().info(f"speed={self.speed}, repeat={self.repeat}")

    def timer_callback(self):
        try:
            moving = self.mc.is_moving()
        except Exception as e:
            self.get_logger().error(f"is_moving() failed: {e}")
            self.safe_stop()
            return

        if moving == -1:
            self.get_logger().error("Robot moving state error")
            self.safe_stop()
            return

        # -----------------------------
        # State 1: move to home once
        # -----------------------------
        if self.state == "MOVE_HOME":
            if not self.command_sent:
                self.send_angles("home", self.home_angles)
                return

            if moving == 1:
                self.check_timeout()
                return

            if moving == 0:
                self.get_logger().info("Home position reached")

                self.state = "PHASE1_SCAN"
                self.command_sent = False
                self.current_scan_idx = 0
                self.current_repeat = 0
                return

        # -----------------------------
        # State 2: phase 1 scanning loop
        # -----------------------------
        if self.state == "PHASE1_SCAN":
            if self.current_repeat >= self.repeat:
                self.finish_motion()
                return

            if not self.command_sent:
                name, angles = self.scan_waypoints[self.current_scan_idx]
                self.send_angles(name, angles)
                return

            if moving == 1:
                self.check_timeout()
                return

            if moving == 0:
                name, _ = self.scan_waypoints[self.current_scan_idx]

                self.get_logger().info(
                    f"{name} reached "
                    f"at repeat {self.current_repeat + 1}/{self.repeat}"
                )

                self.current_scan_idx += 1
                self.command_sent = False

                if self.current_scan_idx >= len(self.scan_waypoints):
                    self.current_scan_idx = 0
                    self.current_repeat += 1

                    self.get_logger().info(
                        f"Phase 1 scan cycle completed: "
                        f"{self.current_repeat}/{self.repeat}"
                    )

    def send_angles(self, name, angles):
        self.get_logger().info(
            f"Sending {name} angles: {angles}"
        )

        try:
            self.mc.send_angles(
                angles,
                self.speed,
                _async=True,
            )
        except Exception as e:
            self.get_logger().error(f"send_angles() failed: {e}")
            self.safe_stop()
            return

        self.command_sent = True
        self.last_command_time = time.time()

    def check_timeout(self):
        if self.last_command_time is None:
            return

        elapsed = time.time() - self.last_command_time

        if elapsed > self.motion_timeout_sec:
            self.get_logger().error("Motion timeout")
            self.safe_stop()

    def finish_motion(self):
        self.get_logger().info("Phase 1 scanning completed")

        try:
            coords = self.mc.get_coords()
            angles = self.mc.get_angles()

            self.get_logger().info(f"Current coords: {coords}")
            self.get_logger().info(f"Current angles: {angles}")
        except Exception as e:
            self.get_logger().warn(f"Failed to read final robot state: {e}")

        self.timer.cancel()

    def safe_stop(self):
        self.get_logger().warn("Stopping robot")

        try:
            self.mc.stop()
        except Exception as e:
            self.get_logger().error(f"mc.stop() failed: {e}")

        self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)

    node = JetcobotScanningMover()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received")
        node.safe_stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()