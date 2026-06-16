#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node
from pymycobot.mycobot280 import MyCobot280


class JetcobotAsyncMover(Node):
    def __init__(self):
        super().__init__("jetcobot_async_mover")

        # Parameters
        self.declare_parameter("port", "/dev/ttyJETCOBOT")
        self.declare_parameter("baudrate", 1000000)
        self.declare_parameter("speed", 20)
        self.declare_parameter("mode", 1)
        self.declare_parameter("repeat", 10)

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.speed = self.get_parameter("speed").value
        self.mode = self.get_parameter("mode").value
        self.repeat = self.get_parameter("repeat").value

        self.pick_coords1 = [-100.0, -100.0, 210.0, -180.0, 0.0, 130.0]
        self.pick_coords2 = [100.0, -100.0, 210.0, -180.0, 0.0, 130.0]

        self.waypoints = [
            self.pick_coords1,
            self.pick_coords2,
        ]

        self.mc = MyCobot280(self.port, self.baudrate)
        self.mc.thread_lock = True

        # vision/servoing 계열이면 최신 명령 우선 모드가 유리할 수 있음
        try:
            self.mc.set_fresh_mode(1)
        except Exception as e:
            self.get_logger().warn(f"set_fresh_mode failed: {e}")

        self.current_waypoint_idx = 0
        self.current_repeat = 0
        self.command_sent = False
        self.last_command_time = None

        self.motion_timeout_sec = 15.0

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Jetcobot async mover node started")
        self.get_logger().info(f"port={self.port}, baudrate={self.baudrate}")

    def timer_callback(self):
        if self.current_repeat >= self.repeat:
            self.finish_motion()
            return

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

        # 아직 명령을 안 보냈으면 첫 명령 전송
        if not self.command_sent:
            self.send_current_waypoint()
            return

        # 이동 중이면 timeout만 확인
        if moving == 1:
            if self.last_command_time is not None:
                elapsed = time.time() - self.last_command_time
                if elapsed > self.motion_timeout_sec:
                    self.get_logger().error("Motion timeout")
                    self.safe_stop()
            return

        # moving == 0이면 현재 waypoint 도착 완료
        if moving == 0:
            self.get_logger().info(
                f"Waypoint {self.current_waypoint_idx} reached "
                f"at repeat {self.current_repeat}"
            )

            self.current_waypoint_idx += 1
            self.command_sent = False

            if self.current_waypoint_idx >= len(self.waypoints):
                self.current_waypoint_idx = 0
                self.current_repeat += 1
                self.get_logger().info(f"Cycle completed: {self.current_repeat}/{self.repeat}")

    def send_current_waypoint(self):
        coords = self.waypoints[self.current_waypoint_idx]

        self.get_logger().info(
            f"Sending waypoint {self.current_waypoint_idx}, "
            f"repeat {self.current_repeat}/{self.repeat}: {coords}"
        )

        try:
            self.mc.send_coords(
                coords,
                self.speed,
                mode=self.mode,
                _async=True,
            )
        except Exception as e:
            self.get_logger().error(f"send_coords() failed: {e}")
            self.safe_stop()
            return

        self.command_sent = True
        self.last_command_time = time.time()

    def finish_motion(self):
        self.get_logger().info("Motion completed")

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

    node = JetcobotAsyncMover()

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