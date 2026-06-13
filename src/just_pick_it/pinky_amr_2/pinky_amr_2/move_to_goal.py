#!/usr/bin/env python3
"""
Move to Goal
- 1단계: MoveCommand waypoint는 Nav2 NavigateThroughPoses로 목표 근처까지 이동
- 2단계: precision_approach_distance 이내 도달 시 Nav2 취소 후
         cmd_vel 직접 제어로 저속 정밀 접근
- 3단계: TF 기반 최종 yaw 보정 후 완료 보고

state_machine이 move_through_goals()를 blocking 방식으로 호출한다.
move_to_goal()은 단일 목표 수동 확인용 legacy 경로로 남겨둔다.
"""

import math
import time
import threading
from typing import Callable

import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from tf2_ros import Buffer, TransformListener


def quat_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def normalize_angle(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


# 최종 목적지 정지 자세(사방향 90° 스냅) 회전 파라미터
STOP_ROTATE_SPEED = 0.8        # [rad/s]
STOP_ROTATE_TOL = 0.05         # [rad] 약 3도
STOP_ROTATE_TIMEOUT = 8.0      # [s]

# 최종 목적지 정지 자세 모드. state_machine 이 task_type 에 따라 지정한다.
STOP_NEAREST_90 = "NEAREST_90"  # 도착 heading 기준 가장 가까운 90° (기본)
STOP_NEAREST_Y = "NEAREST_Y"    # +y/-y 중 회전이 적은 쪽 (MOVE_TO_PRODUCT/DISPLAY)
STOP_PLUS_Y = "PLUS_Y"          # 월드 +y 고정 (RETURN_HOME, standby)
STOP_PLUS_X = "PLUS_X"          # 월드 +x 고정 (MOVE_TO_STOCK)
STOP_MINUS_X = "MINUS_X"        # 월드 -x 고정 (MOVE_TO_PICKUP)


GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "UNKNOWN",
    GoalStatus.STATUS_ACCEPTED: "ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "EXECUTING",
    GoalStatus.STATUS_CANCELING: "CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
    GoalStatus.STATUS_CANCELED: "CANCELED",
    GoalStatus.STATUS_ABORTED: "ABORTED",
}

NAV_DIRECT_REACH = "direct_reach"
NAV_ACTION_SUCCEEDED = "action_succeeded"


def goal_status_name(status: int) -> str:
    return GOAL_STATUS_NAMES.get(status, f"UNKNOWN({status})")


class MoveToGoal(Node):
    def __init__(self):
        super().__init__("move_to_goal")

        self.declare_parameter("precision_approach_distance", 0.03)
        self.declare_parameter("waypoint_reach_distance", 0.15)
        self.declare_parameter("xy_goal_tolerance", 0.01)
        self.declare_parameter("yaw_goal_tolerance", 0.05)
        self.declare_parameter("nav_timeout_sec", 120.0)
        self.declare_parameter("arrival_success_tolerance", 0.10)
        self.declare_parameter("precision_max_linear_vel", 0.10)
        self.declare_parameter("precision_max_angular_vel", 0.6)
        self.declare_parameter("precision_heading_deadband_distance", 0.03)
        self.declare_parameter("precision_heading_gate_angle", 0.35)

        self._prec_dist = self.get_parameter("precision_approach_distance").value
        self._waypoint_reach = self.get_parameter("waypoint_reach_distance").value
        self._xy_tol = self.get_parameter("xy_goal_tolerance").value
        self._yaw_tol = self.get_parameter("yaw_goal_tolerance").value
        self._nav_timeout = self.get_parameter("nav_timeout_sec").value
        self._arrival_success_tol = self.get_parameter("arrival_success_tolerance").value
        self._precision_max_linear = self.get_parameter("precision_max_linear_vel").value
        self._precision_max_angular = self.get_parameter("precision_max_angular_vel").value
        self._precision_heading_deadband = self.get_parameter(
            "precision_heading_deadband_distance"
        ).value
        self._precision_heading_gate = self.get_parameter(
            "precision_heading_gate_angle"
        ).value

        self._lock = threading.Lock()
        self._cur_x = 0.0
        self._cur_y = 0.0
        self._cur_yaw = 0.0
        self._cancel_requested = False
        self._active_goal_handle = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self.create_timer(0.05, self._update_pose)

        self._nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._nav_through_client = ActionClient(
            self, NavigateThroughPoses, "navigate_through_poses"
        )
        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.get_logger().info("MoveToGoal ready.")

    # ------------------------------------------------------------------ #
    # 외부 인터페이스 (blocking, executor 스레드에서 호출)
    # ------------------------------------------------------------------ #

    def move_to_goal(
        self, x: float, y: float, final: bool = True, final_mode: str = STOP_NEAREST_90
    ) -> bool:
        """목표 (x, y) '위치'까지 이동(도착)만 한다. 정지 자세(회전)는 State Machine 담당.

        zone 의 theta 는 사용하지 않는다.
        - 중간 경유지(final=False): nav2 로 근처까지 통과만 한다(정밀접근 생략).
        - 목적지(final=True): 정밀접근으로 위치 오차 이내까지 도달한다(회전 없음).
        task_manager의 daemon 스레드에서 호출.
        """
        self.get_logger().info(f"move_to_goal: target=({x:.3f},{y:.3f}) final={final}")

        # 중간 경유지와 최종 목적지의 도달 판정을 분리한다. 작은 맵에서는 30cm 전환이
        # 코너를 크게 잘라 장애물에 붙기 쉬우므로 cm 단위로 가깝게 붙은 뒤 넘어간다.
        reach_dist = self._prec_dist if final else self._waypoint_reach

        # Nav2 목표 헤딩을 "현재→목표 진행 방향" bearing 으로 준다. yaw=0(동쪽) 하드코딩 시
        # use_rotate_to_heading 컨트롤러가 매 목표마다 로봇을 동쪽으로 돌려(불필요한 90°)
        # 축이 틀어졌다. 최종 정지 자세는 도착 후 task_type 별 final_mode 로 잡는다.
        with self._lock:
            cur_x, cur_y = self._cur_x, self._cur_y
        bearing = math.atan2(y - cur_y, x - cur_x)

        nav_result = self._nav2_navigate(x, y, bearing, reach_dist)
        if not nav_result:
            return False

        if not final:
            self.get_logger().info("move_to_goal: 경유지 통과")
            return True

        if nav_result == NAV_ACTION_SUCCEEDED and self._is_close_enough_to_arrive(x, y):
            self.get_logger().info("move_to_goal: Nav2 근접 도착 인정, 정밀접근 생략")
        elif not self._precision_approach(x, y):
            return False

        # 최종 목적지(goal zone)에서만 task_type 별 정지 자세로 회전한다.
        # 중간 경유지(final=False)는 회전하지 않는다. zone theta 는 쓰지 않는다.
        self._rotate_to_stop_pose(final_mode)

        self.get_logger().info("move_to_goal: 위치 도착")
        return True

    def move_through_goals(
        self,
        points: list[tuple[float, float]],
        final_mode: str = STOP_NEAREST_90,
        progress_callback: Callable[[int], None] | None = None,
    ) -> bool:
        """MoveCommand waypoint를 Nav2 NavigateThroughPoses 한 번으로 통과한다."""
        if not points:
            self.get_logger().error("move_through_goals: empty waypoint list")
            return False

        self.get_logger().info(
            "move_through_goals: waypoints="
            + str([(round(x, 3), round(y, 3)) for x, y in points])
            + f" final_mode={final_mode}"
        )

        tx, ty = points[-1]
        nav_result = self._nav2_navigate_through(
            points, self._prec_dist, progress_callback
        )
        if not nav_result:
            return False

        if nav_result == NAV_ACTION_SUCCEEDED and self._is_close_enough_to_arrive(tx, ty):
            self.get_logger().info(
                "move_through_goals: Nav2 근접 도착 인정, 정밀접근 생략"
            )
        elif not self._precision_approach(tx, ty):
            return False

        self._rotate_to_stop_pose(final_mode)
        self._publish_waypoint_progress(progress_callback, len(points) - 1)

        self.get_logger().info("move_through_goals: 위치 도착")
        return True

    def cancel_navigation(self):
        """로봇을 즉시 정지하고 진행 중인 Nav2 goal 취소를 요청한다."""
        with self._lock:
            self._cancel_requested = True
            goal_handle = self._active_goal_handle
        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warn(f"Nav2 goal cancel request failed: {exc}")
        self._stop_robot()

    def clear_cancel(self):
        """이전 취소 플래그를 지워 다음 명령을 받을 수 있게 한다."""
        with self._lock:
            self._cancel_requested = False

    def _is_cancel_requested(self) -> bool:
        with self._lock:
            return self._cancel_requested

    # ------------------------------------------------------------------ #
    # 내부 단계
    # ------------------------------------------------------------------ #

    def _nav2_navigate(self, x: float, y: float, yaw: float, reach_dist: float):
        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_to_pose action server unavailable")
            return False
        if self._is_cancel_requested():
            return False

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)

        send_future = self._nav_client.send_goal_async(goal)

        # MultiThreadedExecutor가 콜백을 처리하므로 future 완료를 polling으로 대기
        # (spin_until_future_complete은 이미 스피닝 중인 노드와 충돌)
        send_deadline = time.time() + 5.0
        while not send_future.done():
            if self._is_cancel_requested():
                self._stop_robot()
                return False
            if time.time() > send_deadline:
                self.get_logger().error("Nav2 goal send timeout")
                return False
            time.sleep(0.05)

        try:
            goal_handle = send_future.result()
        except Exception as exc:
            self.get_logger().error(f"Nav2 goal send failed: {exc}")
            return False

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Nav2 goal rejected")
            return False
        with self._lock:
            self._active_goal_handle = goal_handle

        result_future = goal_handle.get_result_async()
        self.get_logger().info(
            f"Nav2 goal accepted — target=({x:.3f},{y:.3f}), "
            f"reach_dist={reach_dist:.3f}m"
        )

        deadline = time.time() + self._nav_timeout
        while time.time() < deadline:
            if self._is_cancel_requested():
                goal_handle.cancel_goal_async()
                self._stop_robot()
                self._clear_active_goal(goal_handle)
                return False

            with self._lock:
                dx = x - self._cur_x
                dy = y - self._cur_y
                dist = math.hypot(dx, dy)

            if dist <= reach_dist:
                # 정밀 접근 전환: Nav2 취소
                return self._cancel_for_direct_reach(
                    goal_handle, result_future, dist, "Nav2 phase"
                )

            if result_future.done():
                return self._handle_nav2_result(result_future, goal_handle, dist)

            time.sleep(0.1)

        # 타임아웃 — Nav2 취소
        goal_handle.cancel_goal_async()
        self._stop_robot()
        self._clear_active_goal(goal_handle)
        self.get_logger().warn("Nav2 navigation timeout")
        return False

    def _nav2_navigate_through(
        self,
        points: list[tuple[float, float]],
        reach_dist: float,
        progress_callback: Callable[[int], None] | None = None,
    ):
        if not self._nav_through_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("navigate_through_poses action server unavailable")
            return False
        if self._is_cancel_requested():
            return False

        goal = NavigateThroughPoses.Goal()
        goal.poses = self._make_path_poses(points)

        send_future = self._nav_through_client.send_goal_async(goal)
        send_deadline = time.time() + 5.0
        while not send_future.done():
            if self._is_cancel_requested():
                self._stop_robot()
                return False
            if time.time() > send_deadline:
                self.get_logger().error("Nav2 through-poses goal send timeout")
                return False
            time.sleep(0.05)

        try:
            goal_handle = send_future.result()
        except Exception as exc:
            self.get_logger().error(f"Nav2 through-poses goal send failed: {exc}")
            return False

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Nav2 through-poses goal rejected")
            return False
        with self._lock:
            self._active_goal_handle = goal_handle

        tx, ty = points[-1]
        result_future = goal_handle.get_result_async()
        self.get_logger().info(
            f"Nav2 through-poses goal accepted — waypoints={len(points)}, "
            f"final=({tx:.3f},{ty:.3f}), reach_dist={reach_dist:.3f}m"
        )

        next_waypoint_index = 0
        deadline = time.time() + self._nav_timeout
        while time.time() < deadline:
            if self._is_cancel_requested():
                goal_handle.cancel_goal_async()
                self._stop_robot()
                self._clear_active_goal(goal_handle)
                return False

            next_waypoint_index = self._advance_waypoint_progress(
                points, next_waypoint_index, progress_callback
            )
            dist = self._distance_to_goal(tx, ty)
            if dist <= reach_dist:
                if next_waypoint_index < len(points) - 1:
                    self.get_logger().error(
                        "Traffic waypoint path violation: final target reached "
                        f"before waypoint index {next_waypoint_index} "
                        f"({points[next_waypoint_index][0]:.3f}, "
                        f"{points[next_waypoint_index][1]:.3f}) was passed"
                    )
                    self._cancel_nav2_goal(
                        goal_handle,
                        result_future,
                        "Nav2 through-poses path violation",
                    )
                    return False
                return self._cancel_for_direct_reach(
                    goal_handle, result_future, dist, "Nav2 through-poses phase"
                )

            if result_future.done():
                nav_result = self._handle_nav2_result(result_future, goal_handle, dist)
                if (
                    nav_result == NAV_ACTION_SUCCEEDED
                    and next_waypoint_index < len(points) - 1
                ):
                    self.get_logger().error(
                        "Traffic waypoint path violation: Nav2 succeeded before "
                        f"waypoint index {next_waypoint_index} "
                        f"({points[next_waypoint_index][0]:.3f}, "
                        f"{points[next_waypoint_index][1]:.3f}) was passed"
                    )
                    return False
                return nav_result

            time.sleep(0.1)

        goal_handle.cancel_goal_async()
        self._stop_robot()
        self._clear_active_goal(goal_handle)
        self.get_logger().warn("Nav2 through-poses navigation timeout")
        return False

    def _handle_nav2_result(self, result_future, goal_handle, dist: float):
        """Nav2 action result를 state_machine이 판단할 수 있는 bool로 변환한다."""
        try:
            response = result_future.result()
        except Exception as exc:
            self._stop_robot()
            self._clear_active_goal(goal_handle)
            self.get_logger().error(f"Nav2 result read failed: {exc}")
            return False

        status = response.status
        status_name = goal_status_name(status)
        nav_result = getattr(response, "result", None)
        error_code = getattr(nav_result, "error_code", None)
        error_msg = getattr(nav_result, "error_msg", "")

        self._stop_robot()
        self._clear_active_goal(goal_handle)

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Nav2 result SUCCEEDED — dist={dist:.3f}m")
            return NAV_ACTION_SUCCEEDED

        detail = f"status={status_name}, dist={dist:.3f}m"
        if error_code is not None:
            detail += f", error_code={error_code}"
        if error_msg:
            detail += f", error_msg={error_msg}"
        self.get_logger().error(f"Nav2 result failed — {detail}")
        return False

    def _cancel_nav2_goal(self, goal_handle, result_future, label: str) -> bool:
        cancel_future = goal_handle.cancel_goal_async()
        cancel_deadline = time.time() + 3.0
        while not cancel_future.done():
            if self._is_cancel_requested():
                self._stop_robot()
                self._clear_active_goal(goal_handle)
                return False
            if time.time() > cancel_deadline:
                self.get_logger().warn(f"{label}: cancel response timeout")
                break
            time.sleep(0.05)

        result_deadline = time.time() + 3.0
        while not result_future.done():
            if self._is_cancel_requested():
                self._stop_robot()
                self._clear_active_goal(goal_handle)
                return False
            if time.time() > result_deadline:
                self.get_logger().warn(f"{label}: cancel result timeout")
                break
            time.sleep(0.05)

        if result_future.done():
            try:
                response = result_future.result()
                status_name = goal_status_name(response.status)
                self.get_logger().info(f"{label}: cancel settled with status={status_name}")
            except Exception as exc:
                self.get_logger().warn(f"{label}: cancel result read failed: {exc}")

        self._stop_robot()
        self._clear_active_goal(goal_handle)
        return True

    def _cancel_for_direct_reach(self, goal_handle, result_future, dist: float, label: str):
        """정밀 접근 전환을 위해 Nav2 goal을 취소하고 action 결과 정리까지 기다린다."""
        if not self._cancel_nav2_goal(goal_handle, result_future, label):
            return False
        self.get_logger().info(
            f"{label} done — dist={dist:.3f}m, switching to precision"
        )
        return NAV_DIRECT_REACH

    def _clear_active_goal(self, goal_handle) -> None:
        with self._lock:
            if self._active_goal_handle is goal_handle:
                self._active_goal_handle = None

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _make_path_poses(self, points: list[tuple[float, float]]) -> list[PoseStamped]:
        with self._lock:
            prev_x, prev_y = self._cur_x, self._cur_y

        poses = []
        for x, y in points:
            yaw = math.atan2(y - prev_y, x - prev_x)
            poses.append(self._make_pose(x, y, yaw))
            prev_x, prev_y = x, y
        return poses

    def _distance_to_goal(self, tx: float, ty: float) -> float:
        with self._lock:
            return math.hypot(tx - self._cur_x, ty - self._cur_y)

    def _advance_waypoint_progress(
        self,
        points: list[tuple[float, float]],
        next_index: int,
        progress_callback: Callable[[int], None] | None,
    ) -> int:
        """TrafficManager가 준 waypoint를 순서대로 실제 통과했는지 확인한다."""
        last_intermediate_index = len(points) - 2
        if next_index > last_intermediate_index:
            return next_index

        with self._lock:
            cur_x, cur_y = self._cur_x, self._cur_y

        while next_index <= last_intermediate_index:
            wx, wy = points[next_index]
            dist = math.hypot(wx - cur_x, wy - cur_y)
            if dist > self._waypoint_reach:
                break

            self.get_logger().info(
                f"Traffic waypoint passed — index={next_index}, "
                f"target=({wx:.3f},{wy:.3f}), dist={dist:.3f}m"
            )
            self._publish_waypoint_progress(progress_callback, next_index)
            next_index += 1

        return next_index

    def _publish_waypoint_progress(
        self,
        progress_callback: Callable[[int], None] | None,
        waypoint_index: int,
    ) -> None:
        if progress_callback is None:
            return
        try:
            progress_callback(waypoint_index)
        except Exception as exc:
            self.get_logger().warn(
                f"MoveCommand waypoint feedback publish failed: {exc}"
            )

    def _is_close_enough_to_arrive(self, tx: float, ty: float) -> bool:
        dist = self._distance_to_goal(tx, ty)
        if dist <= self._arrival_success_tol:
            self.get_logger().warn(
                f"Nav2 대체 goal 도착: 원 목표까지 {dist:.3f}m "
                f"<= 성공허용 {self._arrival_success_tol:.3f}m"
            )
            return True
        self.get_logger().warn(
            f"Nav2는 성공했지만 원 목표까지 {dist:.3f}m "
            f"> 성공허용 {self._arrival_success_tol:.3f}m"
        )
        return False

    def _precision_approach(self, tx: float, ty: float) -> bool:
        """저속 직진으로 목표 xy 오차 이하까지 접근."""
        deadline = time.time() + 15.0
        KP = 0.5

        while time.time() < deadline:
            if self._is_cancel_requested():
                self._stop_robot()
                return False

            with self._lock:
                dx = tx - self._cur_x
                dy = ty - self._cur_y
                cur_yaw = self._cur_yaw

            dist = math.hypot(dx, dy)
            if dist <= self._xy_tol:
                self._stop_robot()
                return True

            twist = Twist()
            if dist > self._precision_heading_deadband:
                target_heading = math.atan2(dy, dx)
                angle_err = normalize_angle(target_heading - cur_yaw)
                twist.angular.z = max(
                    -self._precision_max_angular,
                    min(self._precision_max_angular, angle_err),
                )
                heading_scale = math.cos(angle_err)
                if heading_scale > 0.0:
                    if abs(angle_err) > self._precision_heading_gate:
                        heading_scale = max(0.2, heading_scale)
                    twist.linear.x = min(
                        KP * dist * heading_scale,
                        self._precision_max_linear,
                    )
            else:
                twist.angular.z = 0.0
                twist.linear.x = min(KP * dist, self._precision_max_linear)
            self._cmd_pub.publish(twist)
            time.sleep(0.05)

        self._stop_robot()
        dist = self._distance_to_goal(tx, ty)
        if dist <= self._arrival_success_tol:
            self.get_logger().warn(
                f"Precision approach: {self._xy_tol}m 미달이나 {dist:.3f}m "
                f"<= 성공허용 {self._arrival_success_tol:.3f}m → 도착 인정"
            )
            return True
        self.get_logger().warn(
            f"Precision approach timeout (dist={dist:.3f}m > 성공허용 "
            f"{self._arrival_success_tol:.3f}m)"
        )
        return False

    def _yaw_correction(self, target_yaw: float) -> bool:
        """제자리 회전으로 최종 yaw 보정."""
        deadline = time.time() + 8.0
        KP = 1.2

        while time.time() < deadline:
            if self._is_cancel_requested():
                self._stop_robot()
                return False

            with self._lock:
                err = normalize_angle(target_yaw - self._cur_yaw)

            if abs(err) <= self._yaw_tol:
                self._stop_robot()
                return True

            twist = Twist()
            twist.angular.z = max(min(KP * err, 0.3), -0.3)
            self._cmd_pub.publish(twist)
            time.sleep(0.05)

        self._stop_robot()
        self.get_logger().warn("Yaw correction timeout")
        return False

    def _stop_robot(self):
        self._cmd_pub.publish(Twist())

    def _rotate_to_stop_pose(self, mode: str = STOP_NEAREST_90) -> None:
        """최종 목적지 정지 자세로 제자리 회전한다(중간 경유지 제외).

        mode 에 따라 목표 yaw(월드 프레임)를 정한다. cmd_vel 로 직접 회전한다.
        - STOP_PLUS_Y / STOP_PLUS_X / STOP_MINUS_X: 해당 축 방향으로 고정.
        - STOP_NEAREST_Y: +y/-y 중 도착 heading 에서 회전이 적은 쪽.
        - STOP_NEAREST_90(기본): 가장 가까운 90°(축 정렬) 스냅.
        """
        half_pi = math.pi / 2.0
        with self._lock:
            cur = self._cur_yaw

        if mode == STOP_PLUS_Y:
            target = half_pi
        elif mode == STOP_PLUS_X:
            target = 0.0
        elif mode == STOP_MINUS_X:
            target = math.pi
        elif mode == STOP_NEAREST_Y:
            target = half_pi if abs(normalize_angle(half_pi - cur)) <= abs(
                normalize_angle(-half_pi - cur)
            ) else -half_pi
        else:
            target = round(cur / half_pi) * half_pi

        target = normalize_angle(target)
        self.get_logger().info(f"move_to_goal: 정지 자세 회전 {cur:.2f} -> {target:.2f} rad")
        deadline = time.time() + STOP_ROTATE_TIMEOUT
        while time.time() < deadline:
            if self._is_cancel_requested():
                break

            with self._lock:
                err = normalize_angle(target - self._cur_yaw)
            if abs(err) < STOP_ROTATE_TOL:
                break
            twist = Twist()
            twist.angular.z = STOP_ROTATE_SPEED if err > 0 else -STOP_ROTATE_SPEED
            self._cmd_pub.publish(twist)
            time.sleep(0.05)
        self._cmd_pub.publish(Twist())

    # ------------------------------------------------------------------ #
    # TF 위치 업데이트
    # ------------------------------------------------------------------ #

    def _update_pose(self):
        try:
            trans = self._tf_buffer.lookup_transform("map", "base_link", Time())
            t = trans.transform
            with self._lock:
                self._cur_x = t.translation.x
                self._cur_y = t.translation.y
                self._cur_yaw = quat_to_yaw(t.rotation)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = MoveToGoal()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
