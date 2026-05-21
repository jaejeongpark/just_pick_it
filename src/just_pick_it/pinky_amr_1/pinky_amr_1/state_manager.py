#!/usr/bin/env python3
import math
import threading
import time

import requests
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32, String
from tf2_ros import Buffer, TransformListener

from just_pick_it_interfaces.action import MoveCommand

from pinky_amr_1.move_to_goal import MoveToGoal
from pinky_amr_1.aruco_parking import ArucoParking


# task_type → 이동 시작 시 전환할 picky_state
TASK_TO_MOVING_STATE = {
    'MOVE_TO_PRODUCT': 'MOVING_TO_PRODUCT',
    'MOVE_TO_PICKUP':  'MOVING_TO_PICKUP',
    'MOVE_TO_STOCK':   'MOVING_TO_STOCK',
    'MOVE_TO_STORAGE': 'MOVING_TO_STORAGE',
    'RETURN_HOME':     'RETURNING',
}

# 목적지 도착 후 전환할 picky_state (RETURN_HOME 제외)
ARRIVAL_STATE = {
    'MOVE_TO_PRODUCT': 'WAITING_FOR_COBOT',
    'MOVE_TO_PICKUP':  'WAITING_FOR_COBOT',
    'MOVE_TO_STOCK':   'WAITING_FOR_COBOT',
    'MOVE_TO_STORAGE': 'WAITING_FOR_COBOT',
}


def quat_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class StateManager(Node):
    """
    AMR picky_state 상태 기계 노드.

    Task Manager로부터 MoveCommand Action으로 명령을 수신하여
    waypoint 이동을 수행하고 picky_state를 전환한다.

    외부 인터페이스:
      Action Server : /{ns}/move_command  (MoveCommand)
      Publisher     : /{ns}/picky_state   (std_msgs/String)
    """

    def __init__(self, move_node: MoveToGoal, aruco_node: ArucoParking) -> None:
        super().__init__('state_manager')

        self.declare_parameter('server_base_url', 'http://192.168.4.1:8000')
        self.declare_parameter('robot_id', 'AMR_001')
        self.declare_parameter('robot_namespace', 'amr_001')
        self.declare_parameter('report_interval_sec', 1.0)
        self.declare_parameter('aruco_marker_id', 0)
        self.declare_parameter('standby_x', 0.0)
        self.declare_parameter('standby_y', 0.0)
        self.declare_parameter('standby_theta', 0.0)
        self.declare_parameter('dock_departure_distance', 0.08)
        self.declare_parameter('battery_full_voltage', 8.4)
        self.declare_parameter('battery_empty_voltage', 6.8)

        self._url = self.get_parameter('server_base_url').value
        self._robot_id = self.get_parameter('robot_id').value
        self._ns = self.get_parameter('robot_namespace').value
        self._aruco_id = self.get_parameter('aruco_marker_id').value
        self._standby_x = self.get_parameter('standby_x').value
        self._standby_y = self.get_parameter('standby_y').value
        self._standby_theta = self.get_parameter('standby_theta').value
        self._depart_dist = self.get_parameter('dock_departure_distance').value
        self._bat_full = self.get_parameter('battery_full_voltage').value
        self._bat_empty = self.get_parameter('battery_empty_voltage').value

        self._move = move_node
        self._aruco = aruco_node

        self._lock = threading.Lock()
        self._picky_state = 'CHARGING'
        self._battery_pct = 100
        self._pos_x = 0.0
        self._pos_y = 0.0
        self._pos_theta = 0.0

        # Action과 타이머를 동시에 처리하기 위해 ReentrantCallbackGroup 사용
        cb_group = ReentrantCallbackGroup()

        # picky_state 퍼블리셔 (Traffic Manager가 구독)
        self._state_pub = self.create_publisher(
            String, f'/{self._ns}/picky_state', 10
        )
        # 도크 이탈 시 직접 구동용
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Task Manager 명령 수신 Action Server
        self._action_server = ActionServer(
            self,
            MoveCommand,
            f'/{self._ns}/move_command',
            execute_callback=self._execute_move,
            goal_callback=self._on_goal,
            cancel_callback=self._on_cancel,
            callback_group=cb_group,
        )

        # 배터리 구독
        self.create_subscription(Float32, '/battery/voltage', self._battery_voltage_cb, 10)
        self.create_subscription(BatteryState, '/battery_state', self._battery_state_cb, 10)

        # TF
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self.create_timer(0.1, self._update_pose, callback_group=cb_group)

        # 주기 보고 타이머 (상태 publish + Control Server 보고)
        interval = self.get_parameter('report_interval_sec').value
        self.create_timer(interval, self._periodic_report, callback_group=cb_group)

        self.get_logger().info(
            f'[StateManager] 시작 — robot_id={self._robot_id}, ns=/{self._ns}'
        )

    # ── picky_state 상태 전환 ──────────────────────────────────────────

    def _set_state(self, new_state: str) -> None:
        with self._lock:
            prev = self._picky_state
            self._picky_state = new_state

        if prev != new_state:
            self.get_logger().info(f'[StateManager] {prev} -> {new_state}')

        self._publish_state(new_state)
        self._report_to_server(new_state)

    def _publish_state(self, state: str) -> None:
        msg = String()
        msg.data = state
        self._state_pub.publish(msg)

    # ── Action 콜백 ────────────────────────────────────────────────────

    def _on_goal(self, goal_request) -> GoalResponse:
        task_type = goal_request.task_type
        if task_type not in TASK_TO_MOVING_STATE:
            self.get_logger().warn(f'[StateManager] 알 수 없는 task_type: {task_type}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info('[StateManager] 취소 요청 수신')
        self._move.cancel_navigation()
        return CancelResponse.ACCEPT

    def _execute_move(self, goal_handle) -> MoveCommand.Result:
        task_type = goal_handle.request.task_type
        waypoints = goal_handle.request.waypoints

        self.get_logger().info(
            f'[StateManager] 실행: {task_type}, waypoints={len(waypoints)}개'
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

        # 전체 이동 완료 후 상태 전환
        if task_type == 'RETURN_HOME':
            success = self._do_docking()
            if not success:
                self._set_state('ERROR_RECOVERY')
                goal_handle.abort()
                return MoveCommand.Result(success=False, message='aruco docking failed')
            self._set_state('CHARGING')
        else:
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

    # ── ArUco 후진 도킹 ────────────────────────────────────────────────

    def _do_docking(self) -> bool:
        """RETURN_HOME 완료 후 ArUco 마커 기반 후진 도킹 및 위치 보정."""
        self.get_logger().info('[StateManager] ArUco 도킹 시작')
        return self._aruco.aruco_dock(
            self._aruco_id, self._standby_x, self._standby_y
        )

    # ── 센서 / TF 콜백 ────────────────────────────────────────────────

    def _update_pose(self) -> None:
        try:
            trans = self._tf_buffer.lookup_transform('map', 'base_link', Time())
            t = trans.transform
            with self._lock:
                self._pos_x = t.translation.x
                self._pos_y = t.translation.y
                self._pos_theta = quat_to_yaw(t.rotation)
        except Exception:
            pass

    def _battery_voltage_cb(self, msg: Float32) -> None:
        with self._lock:
            self._battery_pct = self._voltage_to_pct(msg.data)

    def _battery_state_cb(self, msg: BatteryState) -> None:
        with self._lock:
            self._battery_pct = self._voltage_to_pct(msg.voltage)

    def _voltage_to_pct(self, voltage: float) -> int:
        span = self._bat_full - self._bat_empty
        if span <= 0:
            return 100
        pct = int((voltage - self._bat_empty) / span * 100)
        return max(0, min(100, pct))

    # ── 주기 보고 ──────────────────────────────────────────────────────

    def _periodic_report(self) -> None:
        with self._lock:
            state = self._picky_state
        self._publish_state(state)
        self._report_to_server(state)

    def _report_to_server(self, state: str) -> None:
        with self._lock:
            payload = {
                'status': state,
                'battery_level': self._battery_pct,
                'pos_x': self._pos_x,
                'pos_y': self._pos_y,
                'pos_theta': self._pos_theta,
            }
        url = f'{self._url}/api/fleet/robots/{self._robot_id}'
        try:
            requests.patch(url, json=payload, timeout=3.0)
        except requests.exceptions.RequestException as e:
            self.get_logger().warn(f'[StateManager] 서버 보고 실패: {e}')


def main(args=None) -> None:
    rclpy.init(args=args)

    move_node = MoveToGoal()
    aruco_node = ArucoParking()
    state_mgr = StateManager(move_node, aruco_node)

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(move_node)
    executor.add_node(aruco_node)
    executor.add_node(state_mgr)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
