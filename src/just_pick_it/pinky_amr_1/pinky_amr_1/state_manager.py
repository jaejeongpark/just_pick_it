#!/usr/bin/env python3
import math
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from std_msgs.msg import Float32, String

from just_pick_it_interfaces.action import DockCommand, MoveCommand
from just_pick_it_interfaces.srv import EmergencyControl

from pinky_amr_1.emergency_latch import EmergencyLatch
from pinky_amr_1.move_to_goal import (
    MoveToGoal,
    STOP_NEAREST_90,
    STOP_NEAREST_Y,
    STOP_PLUS_Y,
    STOP_PLUS_X,
    STOP_MINUS_X,
)
from pinky_amr_1.reverse_docking import ReverseDocking


# task_type → 이동 시작 시 전환할 picky_state
TASK_TO_MOVING_STATE = {
    'MOVE_TO_PRODUCT': 'MOVING_TO_PRODUCT',
    'MOVE_TO_PICKUP':  'MOVING_TO_PICKUP',
    'MOVE_TO_STOCK':   'MOVING_TO_STOCK',
    'MOVE_TO_DISPLAY': 'MOVING_TO_DISPLAY',
    'RETURN_HOME':     'RETURNING',
}

# 목적지 도착 후 전환할 picky_state.
# RETURN_HOME 은 STANDBY_ZONE 도착까지만 담당하고 도킹은 DOCK_IN task 가 별도 수행한다.
ARRIVAL_STATE = {
    'MOVE_TO_PRODUCT': 'WAITING_FOR_COBOT',
    'MOVE_TO_PICKUP':  'WAITING_FOR_COBOT',
    'MOVE_TO_STOCK':   'WAITING_FOR_COBOT',
    'MOVE_TO_DISPLAY': 'WAITING_FOR_COBOT',
    'RETURN_HOME':     'STANDBY',
}

# task_type 별 최종 목적지 정지 자세(yaw) 정책. move_to_goal 의 final_mode 로 전달한다.
# MOVE_TO_PRODUCT/DISPLAY = 법선(±y) 중 회전 적은 쪽, PICKUP = -x, STOCK = +x,
# RETURN_HOME(standby) = +y. 그 외/미지정은 nearest-90 축 정렬 스냅.
STOP_MODE_BY_TASK = {
    'MOVE_TO_PRODUCT': STOP_NEAREST_Y,
    'MOVE_TO_DISPLAY': STOP_NEAREST_Y,
    'MOVE_TO_PICKUP':  STOP_MINUS_X,
    'MOVE_TO_STOCK':   STOP_PLUS_X,
    'RETURN_HOME':     STOP_PLUS_Y,
}

# 충전 중 배터리가 이 값(%)을 넘으면 picky_state 를 CHARGING -> STANDBY 로 바꾼다.
# 상태만 바꾸고 물리 이동(도크 이탈)은 하지 않는다. Fleet 의 작업 배정 게이트가
# picky_state STANDBY 를 요구하므로, 이 전환이 있어야 충전 후 새 주문을 받는다.
BATTERY_STANDBY_THRESHOLD = 30.0


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

    def __init__(
        self,
        move_node: MoveToGoal,
        reverse_docking_node: ReverseDocking,
        emergency_latch: EmergencyLatch,
    ) -> None:
        super().__init__('state_manager')

        self.declare_parameter('robot_id', 'PICKY1')
        self.declare_parameter('state_publish_interval_sec', 1.0)
        self.declare_parameter('dock_departure_distance', 0.20)

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
        # 비상 정지 래치. move_node / reverse_docking_node 와 같은 인스턴스를 공유한다.
        self._emergency = emergency_latch

        self._lock = threading.Lock()
        self._picky_state = 'CHARGING'
        # 물리적으로 충전 도크에 있는지. picky_state 와 분리한다. 배터리 임계 초과 시
        # 상태는 STANDBY 로 바뀌어도 도크에는 그대로 있으므로, move 수신 시 실제
        # 도크 이탈(undock) 여부 판정에 쓴다. 부팅 시 도크에서 시작한다고 가정.
        self._at_dock = True

        # Action과 타이머를 동시에 처리하기 위해 ReentrantCallbackGroup 사용
        cb_group = ReentrantCallbackGroup()

        # picky_state 퍼블리셔 (Traffic Manager가 구독).
        # 노드 namespace 가 'picky1' 이면 자동으로 /picky1/picky_state 가 된다.
        self._state_pub = self.create_publisher(String, 'picky_state', 10)
        # 도크 이탈 시 직접 구동용
        self._cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # 배터리 구독: 충전 중 배터리가 임계 초과면 CHARGING -> STANDBY 전환(상태만).
        self._battery_sub = self.create_subscription(
            Float32, 'battery/percent', self._on_battery, 10, callback_group=cb_group
        )

        # amcl pose 구독: 목적지 도착 후 정지 회전(사방향 스냅)에 현재 yaw 사용.
        self._cur_yaw = 0.0
        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_pose, 10,
            callback_group=cb_group,
        )

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

        # Fleet Manager 비상 정지/재개 수신 Service
        # 노드 namespace 가 'picky1' 이면 자동으로 /picky1/emergency_control 이 된다.
        self._emergency_service = self.create_service(
            EmergencyControl,
            'emergency_control',
            self._handle_emergency_control,
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
        if self._emergency.should_reject_goal():
            self.get_logger().warn(
                f'[StateManager] MOVE 거절: 비상 정지 중 reason={self._emergency.reason}'
            )
            return GoalResponse.REJECT
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
        # 실행 콜백에서 예외가 나면 rclpy 가 goal 을 ABORTED(기본 Result=빈 메시지)로
        # 처리해 "success=False, message=" 만 남고 원인이 사라진다. traceback 을 남기고
        # 의미있는 메시지로 반환하도록 전체를 감싼다.
        try:
            return self._execute_move_inner(goal_handle)
        except Exception as e:
            import traceback
            self.get_logger().error(
                f'[StateManager] MOVE 콜백 예외: {e}\n{traceback.format_exc()}'
            )
            try:
                if goal_handle.is_active:
                    goal_handle.abort()
            except Exception:
                pass
            self._set_state('ERROR_RECOVERY')
            return MoveCommand.Result(success=False, message=f'exception: {e}')

    def _execute_move_inner(self, goal_handle) -> MoveCommand.Result:
        task_type = goal_handle.request.task_type
        waypoints = goal_handle.request.waypoints

        self.get_logger().info(
            f'[StateManager] MOVE 실행: {task_type}, waypoints={len(waypoints)}개'
        )
        self.get_logger().info(
            '[PATHTRACE][StateMachine] 수신 waypoints=' + str(
                [(round(w.pose.position.x, 3), round(w.pose.position.y, 3)) for w in waypoints]
            )
        )

        # 도크 이탈은 'STANDBY 상태이면서 물리적으로 도크에 있을 때'만 수행한다.
        # 배터리 임계 초과로 CHARGING -> STANDBY 만 된 상태에서는 이동하지 않고,
        # 실제 move task 가 와야 여기서 이탈한다(STANDBY 상태에서만 undock).
        with self._lock:
            at_dock = self._at_dock
            current_state = self._picky_state
        if at_dock and current_state == 'STANDBY':
            self._depart_from_dock()
            with self._lock:
                self._at_dock = False

        self._set_state(TASK_TO_MOVING_STATE[task_type])

        feedback = MoveCommand.Feedback()
        feedback.total_waypoints = len(waypoints)

        # waypoint 순차 이동
        for i, wp in enumerate(waypoints):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return MoveCommand.Result(success=False, message='canceled')

            x = wp.pose.position.x
            y = wp.pose.position.y
            # zone 의 theta 는 쓰지 않는다. 중간 경유지는 통과만 하고, 마지막 목적지에서만
            # 정지 자세를 잡는다. 정지 yaw 는 task_type 별 정책(STOP_MODE_BY_TASK)으로 정한다.
            is_final = (i == len(waypoints) - 1)
            final_mode = (
                STOP_MODE_BY_TASK.get(task_type, STOP_NEAREST_90)
                if is_final else STOP_NEAREST_90
            )

            self.get_logger().info(
                f'[PATHTRACE][StateMachine->MoveToGoal] idx={i} 좌표=({x:.3f}, {y:.3f}) '
                f'final={is_final} final_mode={final_mode}'
            )

            if not self._move.move_to_goal(x, y, final=is_final, final_mode=final_mode):
                self._set_state('ERROR_RECOVERY')
                goal_handle.abort()
                return MoveCommand.Result(
                    success=False, message=f'navigation failed at waypoint {i}'
                )

            # waypoint i 에 '도착한 뒤에만' 진행 피드백을 발행한다. fleet TaskManager 가
            # 이 index 에 +1 을 해 TrafficManager 의 현재 경로를 trim 하므로, 이동 시작 전에
            # 발행하면 아직 떠나지 않은 현재 노드와 지금 주행 중인 엣지까지 차단이 풀려
            # 다른 로봇이 그 구간으로 진입해 충돌할 수 있다. 도착 후 발행하면 현재 노드는
            # 다음 도착 시점까지 차단이 유지된다.
            feedback.current_waypoint_index = i
            goal_handle.publish_feedback(feedback)

        # 전체 이동 완료. 정지 자세 회전(사방향 90° 스냅)은 move_to_goal 의 최종 목적지
        # (final=True) 처리로 옮겼다. 여기서는 추가 회전하지 않는다(중간 경유지도 회전 없음).

        # RETURN_HOME 도 여기서 STANDBY 로 종료한다. 도킹은 별도 DOCK_IN task 가 수행.
        self._set_state(ARRIVAL_STATE[task_type])

        # 주행·정지자세를 다 마쳤어도, 실행 중 goal 이 취소됐으면 succeed() 가 예외를 던진다
        # (취소된 goal 에 succeed 불가 → 빈 메시지 abort 의 유력 원인). goal 상태로 분기.
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().warn('[StateManager] MOVE 완료했으나 goal 취소 요청됨 → canceled')
            return MoveCommand.Result(success=False, message='canceled after arrival')
        if not goal_handle.is_active:
            self.get_logger().warn('[StateManager] MOVE 완료했으나 goal 비활성 → 결과만 반환')
            return MoveCommand.Result(success=True, message='ok (goal inactive)')
        goal_handle.succeed()
        return MoveCommand.Result(success=True, message='ok')

    # ── 정지 자세(사방향 스냅) 회전 ────────────────────────────────────

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        with self._lock:
            self._cur_yaw = quat_to_yaw(msg.pose.pose.orientation)

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
        if self._emergency.should_reject_goal():
            self.get_logger().warn(
                f'[StateManager] DOCK 거절: 비상 정지 중 reason={self._emergency.reason}'
            )
            return GoalResponse.REJECT
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

        with self._lock:
            self._at_dock = True
        self._set_state('CHARGING')
        goal_handle.succeed()
        return DockCommand.Result(success=True, message=f'docked at {dock_name}')

    # ── 배터리 기반 충전 완료 전환 ──────────────────────────────────────

    def _on_battery(self, msg: Float32) -> None:
        """충전 중 배터리가 임계를 넘으면 picky_state 를 STANDBY 로 바꾼다.

        상태만 바꾸고 도크 이탈(undock)은 하지 않는다. _at_dock 은 True 로 유지되어
        실제 move task 수신 시 _execute_move 에서 STANDBY 상태로 이탈한다.
        """
        if msg.data <= BATTERY_STANDBY_THRESHOLD:
            return
        with self._lock:
            is_charging = self._picky_state == 'CHARGING'
        if is_charging:
            self.get_logger().info(
                f'[StateManager] 배터리 {msg.data:.0f}% > '
                f'{BATTERY_STANDBY_THRESHOLD:.0f}% -> CHARGING 에서 STANDBY 로 전환(도크 유지)'
            )
            self._set_state('STANDBY')

    # ── 주기 상태 publish ──────────────────────────────────────────────

    def _periodic_publish(self) -> None:
        with self._lock:
            state = self._picky_state
        self._publish_state(state)

    # ── 비상 정지 / 재개 Service ───────────────────────────────────────

    def _handle_emergency_control(
        self,
        request: EmergencyControl.Request,
        response: EmergencyControl.Response,
    ) -> EmergencyControl.Response:
        """Fleet Manager 의 EmergencyControl 요청을 처리한다.

        emergency_stop=True: 래치를 걸고 즉시 정지 명령을 보낸다. 진행 중이던
            move/dock action 은 abort 하지 않는다(pause-continue). 주행 루프가
            래치를 보고 제자리에 멈춰 재개를 기다린다. picky_state 는 그대로
            둔다(이동 중이면 MOVING_* 유지) — 도착하면 정상 전이된다.
        emergency_stop=False: 래치를 풀어 멈춰 있던 주행 루프가 같은 동작을
            이어서 계속하게 한다.
        """
        if request.emergency_stop:
            self._emergency.stop(request.reason)
            # 즉시 정지 명령(주행 루프가 래치를 보기 전 latency 보강).
            self._cmd_vel_pub.publish(Twist())
            self._move.cancel_navigation()
            response.accepted = True
            response.status = 'EMERGENCY_STOP'
            response.message = (
                f'emergency stop accepted: reason={self._emergency.reason}, '
                f'task_id={request.task_id}, request_id={request.request_id}'
            )
            self.get_logger().warn(f'[StateManager] {response.message}')
            return response

        self._emergency.resume()
        response.accepted = True
        response.status = 'RESUMED'
        response.message = (
            f'resume accepted: reason={request.reason}, '
            f'task_id={request.task_id}, request_id={request.request_id}'
        )
        self.get_logger().info(f'[StateManager] {response.message}')
        return response


def main(args=None) -> None:
    rclpy.init(args=args)

    # 세 노드가 공유할 단일 비상 정지 래치. 서비스 콜백(state_mgr)이 걸면
    # 주행 루프(move_node/reverse_docking_node)가 같은 래치를 보고 멈춘다.
    emergency_latch = EmergencyLatch()

    move_node = MoveToGoal(emergency_latch)
    reverse_docking_node = ReverseDocking(emergency_latch)
    state_mgr = StateManager(move_node, reverse_docking_node, emergency_latch)

    # 노드별로 executor 를 분리한다. 셋을 하나의 MultiThreadedExecutor 에 모으면
    # rclpy(7.1.x)가 wait 마다 "세 노드 전체"의 wait set 을 재구성하는데, move_to_goal 의
    # /tf 구독이 ~58Hz 라 이 큰 재구성이 초당 58회 돌아 한 코어를 태운다(Dynamixel 시리얼까지
    # 굶겼던 원인). executor 를 쪼개면 /tf 가 깨우는 건 move_to_goal 의 작은 wait set 뿐이고,
    # state_manager 의 ActionServer 들은 저빈도 wait set 으로 빠져 비용이 크게 준다.
    #
    # move_to_goal / reverse_docking 의 blocking 메서드(move_to_goal(), reverse_dock())는
    # state_manager 의 executor 스레드(action 콜백)에서 호출되고, 두 노드는 각자 executor 에서
    # 계속 스핀하므로 _cur 위치 갱신·nav 액션 피드백이 막히지 않는다. 두 노드는 실행 중
    # cancel 동시처리가 필요 없어 SingleThreaded 로 충분하다. state_manager 는 ActionServer 가
    # 실행 중 cancel 을 받아야 하므로 MultiThreaded(Reentrant)를 유지한다.
    move_exec = SingleThreadedExecutor()
    move_exec.add_node(move_node)
    dock_exec = SingleThreadedExecutor()
    dock_exec.add_node(reverse_docking_node)
    state_exec = MultiThreadedExecutor(num_threads=2)
    state_exec.add_node(state_mgr)

    spin_threads = [
        threading.Thread(target=move_exec.spin, daemon=True),
        threading.Thread(target=dock_exec.spin, daemon=True),
    ]
    for t in spin_threads:
        t.start()

    try:
        state_exec.spin()
    finally:
        state_exec.shutdown()
        move_exec.shutdown()
        dock_exec.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
