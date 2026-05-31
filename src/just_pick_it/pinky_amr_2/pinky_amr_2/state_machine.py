"""PICKY2 Fleet communication skeleton.

This node opens the ROS contracts Fleet Manager needs before real driving is
implemented: MoveCommand, DockCommand, EmergencyControl, picky_state, battery,
and pose telemetry. Move/Dock actions are dry-run only in this first version.
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32, String

from just_pick_it_interfaces.action import DockCommand, MoveCommand
from just_pick_it_interfaces.srv import EmergencyControl
from pinky_amr_2.emergency_guard import Amr2EmergencyGuard


TASK_TO_MOVING_STATE = {
    "MOVE_TO_PRODUCT": "MOVING_TO_PRODUCT",
    "MOVE_TO_PICKUP": "MOVING_TO_PICKUP",
    "MOVE_TO_STOCK": "MOVING_TO_STOCK",
    "MOVE_TO_DISPLAY": "MOVING_TO_DISPLAY",
    "RETURN_HOME": "RETURNING",
}

ARRIVAL_STATE = {
    "MOVE_TO_PRODUCT": "WAITING_FOR_COBOT",
    "MOVE_TO_PICKUP": "WAITING_FOR_COBOT",
    "MOVE_TO_STOCK": "WAITING_FOR_COBOT",
    "MOVE_TO_DISPLAY": "WAITING_FOR_COBOT",
    "RETURN_HOME": "STANDBY",
}


class Amr2StateMachine(Node):
    """Dry-run State Machine for Fleet Manager communication smoke tests."""

    def __init__(self) -> None:
        super().__init__("amr2_state_machine")

        self.declare_parameter("initial_state", "STANDBY")
        self.declare_parameter("dry_run_step_sec", 0.2)
        self.declare_parameter("dry_run_battery_percent", 100.0)

        self._state = self.get_parameter("initial_state").value
        self._dry_run_step_sec = float(self.get_parameter("dry_run_step_sec").value)
        self._battery_percent = float(
            self.get_parameter("dry_run_battery_percent").value
        )
        self._emergency = Amr2EmergencyGuard()

        callback_group = ReentrantCallbackGroup()

        self._state_pub = self.create_publisher(String, "picky_state", 10)
        self._battery_pub = self.create_publisher(Float32, "battery/percent", 10)
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "amcl_pose",
            10,
        )
        self._cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._cmd_vel_raw_pub = self.create_publisher(Twist, "cmd_vel_raw", 10)

        self._move_server = ActionServer(
            self,
            MoveCommand,
            "move_command",
            execute_callback=self._execute_move,
            goal_callback=self._on_move_goal,
            cancel_callback=self._on_move_cancel,
            callback_group=callback_group,
        )
        self._dock_server = ActionServer(
            self,
            DockCommand,
            "dock_command",
            execute_callback=self._execute_dock,
            goal_callback=self._on_dock_goal,
            cancel_callback=self._on_dock_cancel,
            callback_group=callback_group,
        )
        self._emergency_service = self.create_service(
            EmergencyControl,
            "emergency_control",
            self._handle_emergency_control,
            callback_group=callback_group,
        )

        self.create_timer(1.0, self._publish_telemetry, callback_group=callback_group)
        self._publish_telemetry()

        self.get_logger().info(
            "PICKY2 communication skeleton started. "
            f"state={self._state}, namespace={self.get_namespace()}"
        )

    # ==================================================================
    # State / telemetry
    # ==================================================================

    def _set_state(self, new_state: str) -> None:
        previous = self._state
        self._state = new_state
        if previous != new_state:
            self.get_logger().info(f"picky_state {previous} -> {new_state}")
        self._publish_state()

    def _publish_state(self) -> None:
        msg = String()
        msg.data = self._state
        self._state_pub.publish(msg)

    def _publish_telemetry(self) -> None:
        self._publish_state()

        battery = Float32()
        battery.data = self._battery_percent
        self._battery_pub.publish(battery)

        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.pose.orientation.w = 1.0
        self._pose_pub.publish(pose)

    def _publish_stop(self) -> None:
        stop = Twist()
        self._cmd_vel_raw_pub.publish(stop)
        self._cmd_vel_pub.publish(stop)

    # ==================================================================
    # MoveCommand dry-run
    # ==================================================================

    def _on_move_goal(self, goal_request: MoveCommand.Goal) -> GoalResponse:
        if self._emergency.should_reject_goal():
            self.get_logger().warn(
                f"MoveCommand rejected: emergency_stop reason={self._emergency.reason}"
            )
            return GoalResponse.REJECT

        if goal_request.task_type not in TASK_TO_MOVING_STATE:
            self.get_logger().warn(
                f"MoveCommand rejected: unknown task_type={goal_request.task_type}"
            )
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _on_move_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info("MoveCommand cancel requested")
        self._publish_stop()
        return CancelResponse.ACCEPT

    def _execute_move(self, goal_handle) -> MoveCommand.Result:
        task_type = goal_handle.request.task_type
        waypoints = goal_handle.request.waypoints
        total = len(waypoints)

        self.get_logger().info(
            f"MoveCommand dry-run start: task_type={task_type}, waypoints={total}"
        )
        self._set_state(TASK_TO_MOVING_STATE[task_type])

        feedback = MoveCommand.Feedback()
        feedback.total_waypoints = total

        if total == 0:
            feedback.current_waypoint_index = 0
            goal_handle.publish_feedback(feedback)

        for index in range(total):
            if goal_handle.is_cancel_requested:
                self._publish_stop()
                goal_handle.canceled()
                return MoveCommand.Result(success=False, message="CANCELLED")

            if self._emergency.is_stopped():
                self._publish_stop()
                goal_handle.abort()
                return MoveCommand.Result(success=False, message="EMERGENCY_STOPPED")

            feedback.current_waypoint_index = index
            goal_handle.publish_feedback(feedback)
            time.sleep(self._dry_run_step_sec)

        if self._emergency.is_stopped():
            self._publish_stop()
            goal_handle.abort()
            return MoveCommand.Result(success=False, message="EMERGENCY_STOPPED")

        self._set_state(ARRIVAL_STATE[task_type])
        goal_handle.succeed()
        return MoveCommand.Result(success=True, message="dry-run move complete")

    # ==================================================================
    # DockCommand dry-run
    # ==================================================================

    def _on_dock_goal(self, goal_request: DockCommand.Goal) -> GoalResponse:
        if self._emergency.should_reject_goal():
            self.get_logger().warn(
                f"DockCommand rejected: emergency_stop reason={self._emergency.reason}"
            )
            return GoalResponse.REJECT

        if not goal_request.dock_name:
            self.get_logger().warn("DockCommand rejected: dock_name is empty")
            return GoalResponse.REJECT

        return GoalResponse.ACCEPT

    def _on_dock_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info("DockCommand cancel requested")
        self._publish_stop()
        return CancelResponse.ACCEPT

    def _execute_dock(self, goal_handle) -> DockCommand.Result:
        request = goal_handle.request
        self.get_logger().info(
            "DockCommand dry-run start: "
            f"task_id={request.task_id}, dock={request.dock_name}, "
            f"start_zone={request.start_zone_name}"
        )

        self._set_state("DOCKING")

        feedback = DockCommand.Feedback()
        feedback.phase = "DRY_RUN"
        feedback.progress = 0.5
        feedback.message = "dry-run docking"
        goal_handle.publish_feedback(feedback)
        time.sleep(self._dry_run_step_sec)

        if goal_handle.is_cancel_requested:
            self._publish_stop()
            goal_handle.canceled()
            return DockCommand.Result(success=False, message="CANCELLED")

        if self._emergency.is_stopped():
            self._publish_stop()
            goal_handle.abort()
            return DockCommand.Result(success=False, message="EMERGENCY_STOPPED")

        feedback.progress = 1.0
        feedback.message = "dry-run docking complete"
        goal_handle.publish_feedback(feedback)

        self._set_state("CHARGING")
        goal_handle.succeed()
        return DockCommand.Result(success=True, message="dry-run dock complete")

    # ==================================================================
    # EmergencyControl
    # ==================================================================

    def _handle_emergency_control(
        self,
        request: EmergencyControl.Request,
        response: EmergencyControl.Response,
    ) -> EmergencyControl.Response:
        if request.emergency_stop:
            self._emergency.stop(request.reason)
            self._publish_stop()
            response.accepted = True
            response.status = "EMERGENCY_STOP"
            response.message = (
                f"emergency stop accepted: reason={self._emergency.reason}, "
                f"task_id={request.task_id}, request_id={request.request_id}"
            )
            self.get_logger().warn(response.message)
            return response

        self._emergency.resume()
        response.accepted = True
        response.status = "RESUMED"
        response.message = (
            f"resume accepted: reason={request.reason}, "
            f"task_id={request.task_id}, request_id={request.request_id}"
        )
        self.get_logger().info(response.message)
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Amr2StateMachine()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
