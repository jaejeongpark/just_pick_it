#!/usr/bin/env python3
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import String

from just_pick_it_interfaces.action import DockCommand, MoveCommand

from pinky_amr_1.move_to_goal import MoveToGoal
from pinky_amr_1.reverse_docking import ReverseDocking


# task_type → 이동 시작 시 전환할 picky_state
TASK_TO_MOVING_STATE = {
    'MOVE_TO_PRODUCT': 'MOVING_TO_PRODUCT',
    'MOVE_TO_PICKUP':  'MOVING_TO_PICKUP',
    'MOVE_TO_STOCK':   'MOVING_TO_STOCK',
    'MOVE_TO_STORAGE': 'MOVING_TO_STORAGE',
    'RETURN_HOME':     'RETURNING',
}

# 목적지 도착 후 전환할 picky_state.
# RETURN_HOME 은 STANDBY_ZONE 도착까지만 담당하고 도킹은 DOCK_IN task 가 별도 수행한다.
ARRIVAL_STATE = {
    'MOVE_TO_PRODUCT': 'WAITING_FOR_COBOT',
    'MOVE_TO_PICKUP':  'WAITING_FOR_COBOT',
    'MOVE_TO_STOCK':   'WAITING_FOR_COBOT',
    'MOVE_TO_STORAGE': 'WAITING_FOR_COBOT',
    'RETURN_HOME':     'STANDBY',
}


def quat_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class StateManager(Node):
    """
    AMR picky_state 상태 기계 노드.

    Task Manager로부터 MoveCommand / DockCommand Action으로 명령을 수신하여
    waypoint 이동 또는 후진 도킹을 수행하고 picky_state를 전환한다.

    외부 인터페이스 (모두 launch namespace 기준 상대경로):
      Action Server : move_command  (just_pick_it_interfaces/MoveCommand)
      Action Server : dock_command  (just_pick_it_interfaces/DockCommand)
      Publisher     : picky_state   (std_msgs/String)

    자세한 모듈 설계는 docs/state_manager.md 참고.
    """

    def __init__(self, move_node: MoveToGoal, reverse_docking_node: ReverseDocking) -> None:
        super().__init__('state_manager')

        self.declare_parameter('robot_id', 'PICKY1')
        self.declare_parameter('state_publish_interval_sec', 1.0)
        self.declare_parameter('dock_departure_distance', 0.08)

        # 충전 도크별 ArUco 마커 ID와 도크의 절대 좌표(map frame).
        # DockCommand goal 의 dock_name 으로 lookup 한다.
        self.declare_parameter('charging_dock_1.marker_id', 0)
        self.declare_parameter('charging_dock_1.map_x', 0.10)
        self.declare_parameter('charging_dock_1.map_y', 0.10)
        self.declare_parameter('charging_dock_1.map_yaw', 0.0)
        self.declare_parameter('charging_dock_2.marker_id', 1)
        self.declare_parameter('charging_dock_2.map_x', 0.27)
        self.declare_parameter('charging_dock_2.map_y', 0.10)
        self.declare_parameter('charging_dock_2.map_yaw', 0.0)

        self._robot_id = self.get_parameter('robot_id').value
        self._depart_dist = self.get_parameter('dock_departure_distance').value

        # dock_name → (marker_id, map_x, map_y, map_yaw)
        self._dock_pose_by_name: dict[str, tuple[int, float, float, float]] = {
            'CHARGING_DOCK_1': (
                int(self.get_parameter('charging_dock_1.marker_id').value),
                float(self.get_parameter('charging_dock_1.map_x').value),
                float(self.get_parameter('charging_dock_1.map_y').value),
                float(self.get_parameter('charging_dock_1.map_yaw').value),
            ),
            'CHARGING_DOCK_2': (
                int(self.get_parameter('charging_dock_2.marker_id').value),
                float(self.get_parameter('charging_dock_2.map_x').value),
                float(self.get_parameter('charging_dock_2.map_y').value),
                float(self.get_parameter('charging_dock_2.map_yaw').value),
            ),
        }

        self._move = move_node
        self._reverse_docking = reverse_docking_node

        self._lock = threading.Lock()
        self._picky_state = 'CHARGING'

        # Action과 타이머를 동시에 처리하기 위해 ReentrantCallbackGroup 사용
        cb_group = ReentrantCallbackGroup()

        # picky_state 퍼블리셔 (Traffic Manager가 구독).
        # 노드 namespace 가 'picky1' 이면 자동으로 /picky1/picky_state 가 된다.
        self._state_pub = self.create_publisher(String, 'picky_state', 10)
        # 도크 이탈 시 직접 구동용
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Task Manager 이동 명령 수신 Action Server
        self._move_action_server = ActionServer(
            self,
            MoveCommand,
            'move_command',
            execute_callback=self._execute_move,
            goal_callback=self._on_move_goal,
            cancel_callback=self._on_move_cancel,
            callback_group=cb_group,
        )

        # Task Manager 도킹 명령 수신 Action Server (DOCK_IN task 전용)
        self._dock_action_server = ActionServer(
            self,
            DockCommand,
            'dock_command',
            execute_callback=self._execute_dock,
            goal_callback=self._on_dock_goal,
            cancel_callback=self._on_dock_cancel,
            callback_group=cb_group,
        )

        # 주기 상태 publish 타이머 (late subscriber 를 위한 picky_state heartbeat)
        interval = self.get_parameter('state_publish_interval_sec').value
        self.create_timer(interval, self._periodic_publish, callback_group=cb_group)

        self.get_logger().info(
            f'[StateManager] 시작 — robot_id={self._robot_id}, '
            f'namespace=/{self.get_namespace().strip("/")}'
        )

    # ── picky_state 상태 전환 ──────────────────────────────────────────

    def _set_state(self, new_state: str) -> None:
        with self._lock:
            prev = self._picky_state
            self._picky_state = new_state

        if prev != new_state:
            self.get_logger().info(f'[StateManager] {prev} -> {new_state}')

        self._publish_state(new_state)

    def _publish_state(self, state: str) -> None:
        msg = String()
        msg.data = state
        self._state_pub.publish(msg)

    # ── MoveCommand Action 콜백 ────────────────────────────────────────

    def _on_move_goal(self, goal_request) -> GoalResponse:
        task_type = goal_request.task_type
        if task_type not in TASK_TO_MOVING_STATE:
            self.get_logger().warn(f'[StateManager] 알 수 없는 task_type: {task_type}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_move_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info('[StateManager] MOVE 취소 요청 수신')
        self._move.cancel_navigation()
        return CancelResponse.ACCEPT

    def _execute_move(self, goal_handle) -> MoveCommand.Result:
        task_type = goal_handle.request.task_type
        waypoints = goal_handle.request.waypoints

        self.get_logger().info(
            f'[StateManager] MOVE 실행: {task_type}, waypoints={len(waypoints)}개'
        )

        # 충전 도크에 있을 경우 이탈 동작 우선 수행
        # 이탈 직후 picky_state 변경 → Traffic Manager가 도크 점유 자동 해제
        with self._lock:
            current_state = self._picky_state
        if current_state == 'CHARGING':
            self._depart_from_dock()

        self._set_state(TASK_TO_MOVING_STATE[task_type])

        feedback = MoveCommand.Feedback()
        feedback.total_waypoints = len(waypoints)

        # waypoint 순차 이동
        for i, wp in enumerate(waypoints):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return MoveCommand.Result(success=False, message='canceled')

            feedback.current_waypoint_index = i
            goal_handle.publish_feedback(feedback)

            x = wp.pose.position.x
            y = wp.pose.position.y
            theta = quat_to_yaw(wp.pose.orientation)

            if not self._move.move_to_goal(x, y, theta):
                self._set_state('ERROR_RECOVERY')
                goal_handle.abort()
                return MoveCommand.Result(
                    success=False, message=f'navigation failed at waypoint {i}'
                )

        # 전체 이동 완료 후 상태 전환.
        # RETURN_HOME 도 여기서 STANDBY 로 종료한다. 도킹은 별도 DOCK_IN task 가 수행.
        self._set_state(ARRIVAL_STATE[task_type])

        goal_handle.succeed()
        return MoveCommand.Result(success=True, message='ok')

    # ── 도크 이탈 ──────────────────────────────────────────────────────

    def _depart_from_dock(self) -> None:
        """충전 도크에서 dock_departure_distance 만큼 전진하여 이탈한다."""
        self.get_logger().info('[StateManager] 충전 도크 이탈 시작')
        speed = 0.05  # m/s
        duration = self._depart_dist / speed
        deadline = time.time() + duration
        while time.time() < deadline:
            twist = Twist()
            twist.linear.x = speed
            self._cmd_vel_pub.publish(twist)
            time.sleep(0.05)
        self._cmd_vel_pub.publish(Twist())
        self.get_logger().info('[StateManager] 충전 도크 이탈 완료')

    # ── DockCommand Action 콜백 ────────────────────────────────────────

    def _on_dock_goal(self, goal_request) -> GoalResponse:
        dock_name = goal_request.dock_name
        if dock_name not in self._dock_pose_by_name:
            self.get_logger().warn(
                f'[StateManager] 알 수 없는 dock_name: {dock_name}'
            )
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_dock_cancel(self, goal_handle) -> CancelResponse:
        # reverse_docking 은 phase 도중 cancel 을 지원하지 않으므로
        # cancel 요청은 받아두되 phase 종료 후에만 반영된다.
        self.get_logger().info('[StateManager] DOCK 취소 요청 수신')
        return CancelResponse.ACCEPT

    def _execute_dock(self, goal_handle) -> DockCommand.Result:
        """DOCK_IN task 수신 시 reverse_docking 으로 ArUco 기반 후진 도킹을 수행한다."""
        request = goal_handle.request
        dock_name = request.dock_name
        start_zone_name = request.start_zone_name
        task_id = int(request.task_id)

        marker_id, map_x, map_y, map_yaw = self._dock_pose_by_name[dock_name]

        self.get_logger().info(
            f'[StateManager] DOCK_IN 실행: task_id={task_id}, '
            f'dock={dock_name}, start_zone={start_zone_name}, marker={marker_id}'
        )

        self._set_state('DOCKING')

        feedback = DockCommand.Feedback()
        feedback.phase = 'REVERSE_DOCKING'
        feedback.progress = 0.0
        feedback.message = f'starting {dock_name}'
        goal_handle.publish_feedback(feedback)

        success = self._reverse_docking.reverse_dock(marker_id, map_x, map_y, map_yaw)

        if not success:
            self._set_state('ERROR_RECOVERY')
            goal_handle.abort()
            return DockCommand.Result(
                success=False, message=f'reverse docking failed at {dock_name}'
            )

        self._set_state('CHARGING')
        goal_handle.succeed()
        return DockCommand.Result(success=True, message=f'docked at {dock_name}')

    # ── 주기 상태 publish ──────────────────────────────────────────────

    def _periodic_publish(self) -> None:
        with self._lock:
            state = self._picky_state
        self._publish_state(state)


def main(args=None) -> None:
    rclpy.init(args=args)

    move_node = MoveToGoal()
    reverse_docking_node = ReverseDocking()
    state_mgr = StateManager(move_node, reverse_docking_node)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(move_node)
    executor.add_node(reverse_docking_node)
    executor.add_node(state_mgr)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
