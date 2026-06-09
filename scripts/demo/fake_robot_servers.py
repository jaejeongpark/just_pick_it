#!/usr/bin/env python3
from __future__ import annotations

from dataclasses import dataclass, field
import math
import os
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32, String

from just_pick_it_interfaces.action import DockCommand, ExecuteTask, MoveCommand
from just_pick_it_interfaces.srv import EmergencyControl


PICKY_NAMES = ("PICKY1", "PICKY2")
COBOT_NAMES = ("COBOT1", "COBOT2")

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

DOCK_POSES = {
    "CHARGING_DOCK_1": (0.11, 0.08, 0.0),
    "CHARGING_DOCK_2": (0.28, 0.08, 0.0),
}

INITIAL_PICKY_POSES = {
    "PICKY1": (0.11, 0.08, 0.0),
    "PICKY2": (0.28, 0.08, 0.0),
}

COBOT_TASK_PHASES = {
    "SORTING_AND_LOAD": ("SORTING", "LOADING"),
    "INSPECTION": ("INSPECTING",),
    "UNLOAD": ("UNLOADING",),
    "DISPLAY_SCAN": ("SCANNING",),
    "DISPLAY_PLACE": ("PLACING",),
}

FLOW_COMPLETION_PHASE_BY_TASK = {
    "UNLOAD": "UNLOADING",
    "DISPLAY_PLACE": "PLACING",
}


@dataclass
class PickyRuntime:
    name: str
    state: str = "STANDBY"
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    battery_percent: float = 100.0
    emergency_stop: bool = False
    lock: threading.RLock = field(default_factory=threading.RLock)


@dataclass
class CobotRuntime:
    name: str
    state: str = "STANDBY"
    emergency_stop: bool = False
    lock: threading.RLock = field(default_factory=threading.RLock)


def env_float(name: str, default: float) -> float:
    raw_value = os.environ.get(name)
    if raw_value is None or raw_value.strip() == "":
        return default
    try:
        return float(raw_value)
    except ValueError:
        return default


def env_bool(name: str, default: bool) -> bool:
    raw_value = os.environ.get(name)
    if raw_value is None or raw_value.strip() == "":
        return default
    return raw_value.strip().lower() in {"1", "true", "yes", "y", "on"}


def yaw_to_quaternion_z_w(yaw: float) -> tuple[float, float]:
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class FakeRobotServers(Node):
    """ROS2 fake action/service servers for local Fleet Manager demos."""

    def __init__(self) -> None:
        super().__init__("demo_fake_robot_servers")

        self._picky_speed_mps = env_float("DEMO_PICKY_LINEAR_SPEED_MPS", 0.20)
        self._picky_pose_hz = env_float("DEMO_PICKY_POSE_HZ", 1.0)
        self._dock_speed_mps = env_float("DEMO_DOCK_LINEAR_SPEED_MPS", 0.05)
        self._battery_standby_threshold = env_float("DEMO_PICKY_BATTERY_STANDBY_THRESHOLD", 30.0)
        self._battery_drain_per_flow = env_float("DEMO_PICKY_BATTERY_DRAIN_PER_FLOW", 30.0)
        self._charge_complete_seconds = env_float("DEMO_PICKY_CHARGE_COMPLETE_SECONDS", 5.0)
        self._cobot_auto_complete = env_bool("DEMO_COBOT_AUTO_COMPLETE", True)
        self._state_publish_interval_sec = env_float(
            "DEMO_STATE_PUBLISH_INTERVAL_SECONDS",
            1.0,
        )

        callback_group = ReentrantCallbackGroup()
        self._shutdown_requested = threading.Event()
        self._picky: dict[str, PickyRuntime] = {}
        self._cobots: dict[str, CobotRuntime] = {}
        self._state_pubs: dict[str, object] = {}
        self._battery_pubs: dict[str, object] = {}
        self._pose_pubs: dict[str, object] = {}
        self._servers: list[object] = []
        self._services: list[object] = []

        for name in PICKY_NAMES:
            ns = name.lower()
            x, y, theta = INITIAL_PICKY_POSES[name]
            runtime = PickyRuntime(name=name, x=x, y=y, theta=theta)
            self._picky[name] = runtime
            self._state_pubs[name] = self.create_publisher(String, f"/{ns}/picky_state", 10)
            self._battery_pubs[name] = self.create_publisher(Float32, f"/{ns}/battery/percent", 10)
            self._pose_pubs[name] = self.create_publisher(
                PoseWithCovarianceStamped,
                f"/{ns}/amcl_pose",
                10,
            )
            self._servers.append(
                ActionServer(
                    self,
                    MoveCommand,
                    f"/{ns}/move_command",
                    execute_callback=lambda goal_handle, robot=name: self._execute_move(
                        robot,
                        goal_handle,
                    ),
                    goal_callback=lambda goal, robot=name: self._on_move_goal(robot, goal),
                    cancel_callback=lambda goal_handle, robot=name: self._on_move_cancel(
                        robot,
                        goal_handle,
                    ),
                    callback_group=callback_group,
                )
            )
            self._servers.append(
                ActionServer(
                    self,
                    DockCommand,
                    f"/{ns}/dock_command",
                    execute_callback=lambda goal_handle, robot=name: self._execute_dock(
                        robot,
                        goal_handle,
                    ),
                    goal_callback=lambda goal, robot=name: self._on_dock_goal(robot, goal),
                    cancel_callback=lambda goal_handle, robot=name: self._on_dock_cancel(
                        robot,
                        goal_handle,
                    ),
                    callback_group=callback_group,
                )
            )
            self._services.append(
                self.create_service(
                    EmergencyControl,
                    f"/{ns}/emergency_control",
                    lambda request, response, robot=name: self._handle_picky_emergency(
                        robot,
                        request,
                        response,
                    ),
                    callback_group=callback_group,
                )
            )

        for name in COBOT_NAMES:
            ns = name.lower()
            runtime = CobotRuntime(name=name)
            self._cobots[name] = runtime
            self._state_pubs[name] = self.create_publisher(String, f"/{ns}/cobot_state", 10)
            self._servers.append(
                ActionServer(
                    self,
                    ExecuteTask,
                    f"/{ns}/execute_task",
                    execute_callback=lambda goal_handle, robot=name: self._execute_cobot_task(
                        robot,
                        goal_handle,
                    ),
                    goal_callback=lambda goal, robot=name: self._on_cobot_goal(robot, goal),
                    cancel_callback=lambda goal_handle, robot=name: self._on_cobot_cancel(
                        robot,
                        goal_handle,
                    ),
                    callback_group=callback_group,
                )
            )
            self._services.append(
                self.create_service(
                    EmergencyControl,
                    f"/{ns}/emergency_control",
                    lambda request, response, robot=name: self._handle_cobot_emergency(
                        robot,
                        request,
                        response,
                    ),
                    callback_group=callback_group,
                )
            )

        self.create_timer(
            self._state_publish_interval_sec,
            self._publish_all_telemetry,
            callback_group=callback_group,
        )
        self._publish_all_telemetry()

        self.get_logger().info(
            "fake robot servers ready: "
            "/picky{1,2}/move_command, /picky{1,2}/dock_command, "
            f"/cobot{{1,2}}/execute_task, cobot_auto_complete={self._cobot_auto_complete}"
        )

    def request_shutdown(self) -> None:
        self._shutdown_requested.set()

    def _is_shutdown_requested(self) -> bool:
        return self._shutdown_requested.is_set()

    # ------------------------------------------------------------------
    # PICKY telemetry
    # ------------------------------------------------------------------

    def _publish_all_telemetry(self) -> None:
        self._publish_all_state_pose()
        self._publish_all_battery()

    def _publish_all_state_pose(self) -> None:
        if self._is_shutdown_requested():
            return
        for runtime in self._picky.values():
            self._publish_picky_state_pose(runtime)
        for runtime in self._cobots.values():
            self._publish_cobot_state(runtime)

    def _publish_all_battery(self) -> None:
        if self._is_shutdown_requested():
            return
        for runtime in self._picky.values():
            self._publish_picky_battery(runtime)

    def _publish_picky_state_pose(self, runtime: PickyRuntime) -> None:
        if self._is_shutdown_requested():
            return
        with runtime.lock:
            state = runtime.state
            x = runtime.x
            y = runtime.y
            theta = runtime.theta

        state_msg = String()
        state_msg.data = state
        self._state_pubs[runtime.name].publish(state_msg)

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.pose.position.x = float(x)
        pose_msg.pose.pose.position.y = float(y)
        pose_msg.pose.pose.position.z = 0.0
        z, w = yaw_to_quaternion_z_w(theta)
        pose_msg.pose.pose.orientation.z = z
        pose_msg.pose.pose.orientation.w = w
        self._pose_pubs[runtime.name].publish(pose_msg)

    def _publish_picky_battery(self, runtime: PickyRuntime) -> None:
        if self._is_shutdown_requested():
            return
        with runtime.lock:
            battery_percent = runtime.battery_percent

        battery_msg = Float32()
        battery_msg.data = float(battery_percent)
        self._battery_pubs[runtime.name].publish(battery_msg)
        self._transition_charged_picky_to_standby(runtime)

    def _set_picky_state(self, runtime: PickyRuntime, state: str) -> None:
        with runtime.lock:
            previous = runtime.state
            runtime.state = state
        if previous != state:
            self.get_logger().info(f"{runtime.name} state {previous} -> {state}")
        self._publish_picky_state_pose(runtime)

    def _set_picky_battery(self, runtime: PickyRuntime, battery_percent: float) -> None:
        with runtime.lock:
            previous = runtime.battery_percent
            runtime.battery_percent = max(0.0, min(100.0, battery_percent))
            current = runtime.battery_percent
        if abs(previous - current) >= 0.1:
            self.get_logger().info(
                f"{runtime.name} battery {previous:.0f}% -> {current:.0f}%"
            )
        self._publish_picky_battery(runtime)

    def _transition_charged_picky_to_standby(self, runtime: PickyRuntime) -> bool:
        with runtime.lock:
            should_standby = (
                runtime.state == "CHARGING"
                and runtime.battery_percent > self._battery_standby_threshold
            )
        if not should_standby:
            return False

        self.get_logger().info(
            f"{runtime.name} battery above {self._battery_standby_threshold:.0f}%: "
            "CHARGING -> STANDBY"
        )
        self._set_picky_state(runtime, "STANDBY")
        return True

    def _schedule_fake_charge_if_needed(self, runtime: PickyRuntime) -> None:
        thread = threading.Thread(
            target=self._complete_fake_charge_after_delay,
            args=(runtime,),
            daemon=True,
        )
        thread.start()

    def _complete_fake_charge_after_delay(self, runtime: PickyRuntime) -> None:
        deadline = time.monotonic() + max(self._charge_complete_seconds, 0.0)
        while time.monotonic() < deadline:
            if self._is_shutdown_requested():
                return
            time.sleep(min(0.1, deadline - time.monotonic()))

        self._set_picky_battery(runtime, 100.0)

    def _drain_picky_battery_for_completed_flow(self, cobot_name: str, task_type: str) -> None:
        picky_name = cobot_name.replace("COBOT", "PICKY", 1)
        runtime = self._picky.get(picky_name)
        if runtime is None:
            return

        with runtime.lock:
            next_level = runtime.battery_percent - self._battery_drain_per_flow
        self.get_logger().info(
            f"{picky_name} fake flow complete via {task_type}: "
            f"battery -{self._battery_drain_per_flow:.0f}%"
        )
        self._set_picky_battery(runtime, next_level)

    def _update_picky_pose(
        self,
        runtime: PickyRuntime,
        x: float,
        y: float,
        theta: float,
    ) -> None:
        with runtime.lock:
            runtime.x = x
            runtime.y = y
            runtime.theta = theta
        self._publish_picky_state_pose(runtime)

    # ------------------------------------------------------------------
    # PICKY MoveCommand
    # ------------------------------------------------------------------

    def _on_move_goal(self, robot_name: str, goal: MoveCommand.Goal) -> GoalResponse:
        if self._is_shutdown_requested():
            return GoalResponse.REJECT
        runtime = self._picky[robot_name]
        with runtime.lock:
            emergency_stop = runtime.emergency_stop
        if emergency_stop:
            self.get_logger().warn(f"{robot_name} move rejected: emergency stop")
            return GoalResponse.REJECT
        if goal.task_type not in TASK_TO_MOVING_STATE:
            self.get_logger().warn(
                f"{robot_name} move rejected: unknown task_type={goal.task_type}"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_move_cancel(self, robot_name: str, goal_handle) -> CancelResponse:
        self.get_logger().info(f"{robot_name} move cancel requested")
        return CancelResponse.ACCEPT

    def _execute_move(self, robot_name: str, goal_handle) -> MoveCommand.Result:
        runtime = self._picky[robot_name]
        task_type = goal_handle.request.task_type
        waypoints = list(goal_handle.request.waypoints)
        total = len(waypoints)
        feedback = MoveCommand.Feedback()
        feedback.total_waypoints = total

        self.get_logger().info(
            f"{robot_name} fake move start: task_type={task_type}, waypoints={total}"
        )
        self._set_picky_state(runtime, TASK_TO_MOVING_STATE[task_type])

        if total == 0:
            goal_handle.succeed()
            self._set_picky_state(runtime, ARRIVAL_STATE[task_type])
            return MoveCommand.Result(success=True, message="fake move complete")

        for index, waypoint in enumerate(waypoints):
            target_x = float(waypoint.pose.position.x)
            target_y = float(waypoint.pose.position.y)
            if not self._drive_to_pose(runtime, target_x, target_y, goal_handle):
                return self._finish_interrupted_move(runtime, goal_handle)

            feedback.current_waypoint_index = index
            goal_handle.publish_feedback(feedback)

        self._set_picky_state(runtime, ARRIVAL_STATE[task_type])
        goal_handle.succeed()
        return MoveCommand.Result(success=True, message="fake move complete")

    def _drive_to_pose(
        self,
        runtime: PickyRuntime,
        target_x: float,
        target_y: float,
        goal_handle,
    ) -> bool:
        with runtime.lock:
            start_x = runtime.x
            start_y = runtime.y

        dx = target_x - start_x
        dy = target_y - start_y
        distance = math.hypot(dx, dy)
        if distance <= 1e-6:
            return True

        theta = math.atan2(dy, dx)
        duration = distance / max(self._picky_speed_mps, 1e-6)
        period = 1.0 / max(self._picky_pose_hz, 1.0)
        steps = max(1, math.ceil(duration / period))
        sleep_sec = duration / steps

        for step in range(1, steps + 1):
            if self._is_shutdown_requested() or goal_handle.is_cancel_requested:
                return False
            with runtime.lock:
                if runtime.emergency_stop:
                    return False
            ratio = step / steps
            x = start_x + dx * ratio
            y = start_y + dy * ratio
            self._update_picky_pose(runtime, x, y, theta)
            time.sleep(sleep_sec)
        return True

    def _finish_interrupted_move(self, runtime: PickyRuntime, goal_handle) -> MoveCommand.Result:
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self._set_picky_state(runtime, "STANDBY")
            return MoveCommand.Result(success=False, message="CANCELLED")
        goal_handle.abort()
        self._set_picky_state(runtime, "ERROR_RECOVERY")
        return MoveCommand.Result(success=False, message="EMERGENCY_STOPPED")

    # ------------------------------------------------------------------
    # PICKY DockCommand
    # ------------------------------------------------------------------

    def _on_dock_goal(self, robot_name: str, goal: DockCommand.Goal) -> GoalResponse:
        if self._is_shutdown_requested():
            return GoalResponse.REJECT
        runtime = self._picky[robot_name]
        with runtime.lock:
            emergency_stop = runtime.emergency_stop
        if emergency_stop:
            self.get_logger().warn(f"{robot_name} dock rejected: emergency stop")
            return GoalResponse.REJECT
        if not goal.dock_name:
            self.get_logger().warn(f"{robot_name} dock rejected: empty dock name")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_dock_cancel(self, robot_name: str, goal_handle) -> CancelResponse:
        self.get_logger().info(f"{robot_name} dock cancel requested")
        return CancelResponse.ACCEPT

    def _execute_dock(self, robot_name: str, goal_handle) -> DockCommand.Result:
        runtime = self._picky[robot_name]
        dock_name = goal_handle.request.dock_name
        self.get_logger().info(f"{robot_name} fake dock start: dock={dock_name}")
        self._set_picky_state(runtime, "DOCKING")

        target = DOCK_POSES.get(dock_name)
        if target is not None:
            target_x, target_y, target_theta = target
        else:
            with runtime.lock:
                target_x, target_y, target_theta = runtime.x, runtime.y, runtime.theta

        with runtime.lock:
            start_x = runtime.x
            start_y = runtime.y
        distance = math.hypot(target_x - start_x, target_y - start_y)
        duration = distance / max(self._dock_speed_mps, 1e-6)
        period = 1.0 / max(self._picky_pose_hz, 1.0)
        steps = max(1, math.ceil(duration / period))
        feedback = DockCommand.Feedback()
        feedback.phase = "DOCKING"

        for step in range(1, steps + 1):
            if self._is_shutdown_requested() or goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._set_picky_state(runtime, "STANDBY")
                return DockCommand.Result(success=False, message="CANCELLED")
            with runtime.lock:
                if runtime.emergency_stop:
                    goal_handle.abort()
                    self._set_picky_state(runtime, "ERROR_RECOVERY")
                    return DockCommand.Result(success=False, message="EMERGENCY_STOPPED")

            ratio = step / steps
            x = start_x + (target_x - start_x) * ratio
            y = start_y + (target_y - start_y) * ratio
            self._update_picky_pose(runtime, x, y, target_theta)
            feedback.progress = float(ratio)
            feedback.message = f"docking {dock_name}"
            goal_handle.publish_feedback(feedback)
            time.sleep(duration / steps)

        self._set_picky_state(runtime, "CHARGING")
        self._schedule_fake_charge_if_needed(runtime)
        goal_handle.succeed()
        return DockCommand.Result(success=True, message=f"fake dock complete: {dock_name}")

    # ------------------------------------------------------------------
    # COBOT ExecuteTask
    # ------------------------------------------------------------------

    def _publish_cobot_state(self, runtime: CobotRuntime) -> None:
        if self._is_shutdown_requested():
            return
        with runtime.lock:
            state = runtime.state
        msg = String()
        msg.data = state
        self._state_pubs[runtime.name].publish(msg)

    def _set_cobot_state(self, runtime: CobotRuntime, state: str) -> None:
        with runtime.lock:
            previous = runtime.state
            runtime.state = state
        if previous != state:
            self.get_logger().info(f"{runtime.name} state {previous} -> {state}")
        self._publish_cobot_state(runtime)

    def _on_cobot_goal(self, robot_name: str, goal: ExecuteTask.Goal) -> GoalResponse:
        if self._is_shutdown_requested():
            return GoalResponse.REJECT
        runtime = self._cobots[robot_name]
        with runtime.lock:
            emergency_stop = runtime.emergency_stop
        if emergency_stop:
            self.get_logger().warn(f"{robot_name} task rejected: emergency stop")
            return GoalResponse.REJECT
        if goal.task_type not in COBOT_TASK_PHASES:
            self.get_logger().warn(
                f"{robot_name} task rejected: unknown task_type={goal.task_type}"
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cobot_cancel(self, robot_name: str, goal_handle) -> CancelResponse:
        self.get_logger().info(f"{robot_name} cobot task cancel requested")
        return CancelResponse.ACCEPT

    def _execute_cobot_task(self, robot_name: str, goal_handle) -> ExecuteTask.Result:
        runtime = self._cobots[robot_name]
        request = goal_handle.request
        task_type = request.task_type
        phases = list(COBOT_TASK_PHASES[task_type]) + ["STOWING_ARM"]
        quantity = max(int(request.quantity), 0)
        total_duration = sum(self._cobot_phase_duration(phase) for phase in phases)
        elapsed = 0.0
        processed_quantity = 0
        battery_drained = False

        self.get_logger().info(
            f"{robot_name} fake cobot task start: task_id={request.task_id}, "
            f"task_type={task_type}, quantity={quantity}"
        )
        self._publish_cobot_feedback(
            goal_handle,
            state="ACCEPTED",
            message=f"task {request.task_id} accepted",
            progress=0.0,
            processed_quantity=0,
        )

        if not self._cobot_auto_complete:
            return self._hold_cobot_task_for_manual_success(
                runtime,
                goal_handle,
                phase=phases[0],
            )

        for phase in phases:
            self._set_cobot_state(runtime, phase)
            duration = self._cobot_phase_duration(phase)
            if not self._run_cobot_phase(
                runtime,
                goal_handle,
                phase=phase,
                duration=duration,
                elapsed_before=elapsed,
                total_duration=total_duration,
                processed_quantity=processed_quantity,
            ):
                return self._finish_interrupted_cobot(runtime, goal_handle)

            elapsed += duration
            if phase == "PLACING":
                processed_quantity = quantity or 1
            if (
                not battery_drained
                and FLOW_COMPLETION_PHASE_BY_TASK.get(task_type) == phase
            ):
                self._drain_picky_battery_for_completed_flow(robot_name, task_type)
                battery_drained = True

        self._set_cobot_state(runtime, "STANDBY")
        goal_handle.succeed()
        stock_delta = processed_quantity if task_type == "DISPLAY_PLACE" else 0
        return ExecuteTask.Result(
            success=True,
            status="SUCCESS",
            message=f"fake {task_type} complete",
            processed_quantity=processed_quantity,
            stock_delta=stock_delta,
        )

    def _hold_cobot_task_for_manual_success(
        self,
        runtime: CobotRuntime,
        goal_handle,
        *,
        phase: str,
    ) -> ExecuteTask.Result:
        """COBOT task를 RUNNING 상태로 유지해 Fleet debug curl로 완료하게 한다."""
        self._set_cobot_state(runtime, phase)
        self._publish_cobot_feedback(
            goal_handle,
            state=phase,
            message="manual debug success required",
            progress=0.0,
            processed_quantity=0,
        )
        self.get_logger().info(
            f"{runtime.name} fake cobot task held for manual success: "
            f"task_id={goal_handle.request.task_id}, task_type={goal_handle.request.task_type}"
        )

        while True:
            if self._is_shutdown_requested() or goal_handle.is_cancel_requested:
                return self._finish_interrupted_cobot(runtime, goal_handle)
            with runtime.lock:
                if runtime.emergency_stop:
                    return self._finish_interrupted_cobot(runtime, goal_handle)
            time.sleep(0.2)

    def _run_cobot_phase(
        self,
        runtime: CobotRuntime,
        goal_handle,
        *,
        phase: str,
        duration: float,
        elapsed_before: float,
        total_duration: float,
        processed_quantity: int,
    ) -> bool:
        progress = min(1.0, elapsed_before / max(total_duration, 1e-6))
        self._publish_cobot_feedback(
            goal_handle,
            state=phase,
            message=f"{phase} started",
            progress=progress,
            processed_quantity=processed_quantity,
        )

        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if self._is_shutdown_requested() or goal_handle.is_cancel_requested:
                return False
            with runtime.lock:
                if runtime.emergency_stop:
                    return False

            remaining = deadline - time.monotonic()
            time.sleep(min(0.1, max(remaining, 0.0)))

        return True

    def _finish_interrupted_cobot(self, runtime: CobotRuntime, goal_handle) -> ExecuteTask.Result:
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self._set_cobot_state(runtime, "STANDBY")
            return ExecuteTask.Result(
                success=False,
                status="CANCELLED",
                message="fake cobot task cancelled",
                processed_quantity=0,
                stock_delta=0,
            )
        goal_handle.abort()
        self._set_cobot_state(runtime, "SAFETY_STOPPED")
        return ExecuteTask.Result(
            success=False,
            status="FAILED",
            message="fake cobot task stopped by emergency",
            processed_quantity=0,
            stock_delta=0,
        )

    def _publish_cobot_feedback(
        self,
        goal_handle,
        *,
        state: str,
        message: str,
        progress: float,
        processed_quantity: int,
    ) -> None:
        feedback = ExecuteTask.Feedback()
        feedback.state = state
        feedback.message = message
        feedback.progress = float(progress)
        feedback.processed_quantity = int(processed_quantity)
        if self._is_shutdown_requested():
            return
        goal_handle.publish_feedback(feedback)

    def _cobot_phase_duration(self, phase: str) -> float:
        defaults = {
            "SORTING": 2.0,
            "LOADING": 2.0,
            "INSPECTING": 3.0,
            "UNLOADING": 3.0,
            "SCANNING": 3.0,
            "PLACING": 4.0,
            "STOWING_ARM": 1.0,
        }
        return env_float(f"DEMO_COBOT_{phase}_SECONDS", defaults.get(phase, 2.0))

    # ------------------------------------------------------------------
    # EmergencyControl
    # ------------------------------------------------------------------

    def _handle_picky_emergency(
        self,
        robot_name: str,
        request: EmergencyControl.Request,
        response: EmergencyControl.Response,
    ) -> EmergencyControl.Response:
        runtime = self._picky[robot_name]
        with runtime.lock:
            runtime.emergency_stop = bool(request.emergency_stop)
        if request.emergency_stop:
            self._set_picky_state(runtime, "ERROR_RECOVERY")
            status = "EMERGENCY_STOP"
        else:
            self._set_picky_state(runtime, "STANDBY")
            status = "RESUMED"
        response.accepted = True
        response.status = status
        response.message = f"{robot_name} fake emergency state: {status}"
        return response

    def _handle_cobot_emergency(
        self,
        robot_name: str,
        request: EmergencyControl.Request,
        response: EmergencyControl.Response,
    ) -> EmergencyControl.Response:
        runtime = self._cobots[robot_name]
        with runtime.lock:
            runtime.emergency_stop = bool(request.emergency_stop)
        if request.emergency_stop:
            self._set_cobot_state(runtime, "SAFETY_STOPPED")
            status = "EMERGENCY_STOP"
        else:
            self._set_cobot_state(runtime, "STANDBY")
            status = "RESUMED"
        response.accepted = True
        response.status = status
        response.message = f"{robot_name} fake emergency state: {status}"
        return response


def main() -> None:
    rclpy.init()
    node = FakeRobotServers()
    executor = MultiThreadedExecutor(num_threads=8)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.request_shutdown()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
