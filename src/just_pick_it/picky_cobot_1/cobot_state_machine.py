#!/usr/bin/env python3
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import String

from just_pick_it_interfaces.action import ExecuteCobotTask
from just_pick_it_interfaces.srv import EmergencyControl


# task_type → 작업 단계에서 순서대로 거치는 cobot_state 목록.
# STOWING_ARM 은 모든 task 에서 공통으로 마지막에 발행된다.
# 상태값은 DB cobot_state ENUM 과 일치해야 한다.
TASK_PHASE_STATES = {
    'SORTING_AND_LOAD': ['SORTING', 'LOADING'],
    'INSPECTION':       ['INSPECTING'],
    'UNLOAD':           ['UNLOADING'],
    'DISPLAY_SCAN':     ['SCANNING'],
    'DISPLAY_PLACE':    ['PLACING'],
}


class CobotStateManager(Node):
    """
    Cobot cobot_state 상태 기계 노드.

    Task Manager로부터 ExecuteCobotTask Action으로 명령을 수신하여
    로봇팔 작업을 수행하고 cobot_state를 전환한다.
    EmergencyControl 서비스로 즉시 정지/재개를 처리한다.

    외부 인터페이스 (launch namespace 기준 상대경로):
      Action Server  : execute_cobot_task  (just_pick_it_interfaces/ExecuteCobotTask)
      Service Server : emergency_control   (just_pick_it_interfaces/EmergencyControl)
      Publisher      : cobot_state         (std_msgs/String)
    """

    def __init__(self) -> None:
        super().__init__('cobot_state_manager')

        self.declare_parameter('robot_id', 'COBOT1')
        self.declare_parameter('state_publish_interval_sec', 1.0)

        self._robot_id = self.get_parameter('robot_id').value

        self._lock = threading.Lock()
        self._cobot_state = 'STANDBY'
        self._emergency_stop = False

        # Action 과 서비스, 타이머를 동시에 처리하기 위해 ReentrantCallbackGroup 사용
        cb_group = ReentrantCallbackGroup()

        # cobot_state 퍼블리셔 (Fleet Manager / Traffic Manager 구독).
        # 노드 namespace 가 'cobot1' 이면 자동으로 /cobot1/cobot_state 가 된다.
        self._state_pub = self.create_publisher(String, 'cobot_state', 10)

        # Task Manager 코봇 작업 명령 수신 Action Server
        self._task_action_server = ActionServer(
            self,
            ExecuteCobotTask,
            'execute_cobot_task',
            execute_callback=self._execute_task,
            goal_callback=self._on_task_goal,
            cancel_callback=self._on_task_cancel,
            callback_group=cb_group,
        )

        # 이머전시 정지/재개 서비스 서버
        self._emergency_srv = self.create_service(
            EmergencyControl,
            'emergency_control',
            self._handle_emergency,
            callback_group=cb_group,
        )

        # 주기 상태 publish 타이머 (late subscriber 를 위한 cobot_state heartbeat)
        interval = self.get_parameter('state_publish_interval_sec').value
        self.create_timer(interval, self._periodic_publish, callback_group=cb_group)

        self.get_logger().info(
            f'[CobotStateManager] 시작 — robot_id={self._robot_id}, '
            f'namespace=/{self.get_namespace().strip("/")}'
        )

    # ── cobot_state 상태 전환 ─────────────────────────────────────────

    def _set_state(self, new_state: str) -> None:
        with self._lock:
            prev = self._cobot_state
            self._cobot_state = new_state

        if prev != new_state:
            self.get_logger().info(f'[CobotStateManager] {prev} -> {new_state}')

        self._publish_state(new_state)

    def _publish_state(self, state: str) -> None:
        msg = String()
        msg.data = state
        self._state_pub.publish(msg)

    # ── ExecuteCobotTask Action 콜백 ──────────────────────────────────

    def _on_task_goal(self, goal_request) -> GoalResponse:
        task_type = goal_request.task_type
        if task_type not in TASK_PHASE_STATES:
            self.get_logger().warn(f'[CobotStateManager] 알 수 없는 task_type: {task_type}')
            return GoalResponse.REJECT
        with self._lock:
            if self._emergency_stop:
                self.get_logger().warn('[CobotStateManager] 이머전시 정지 중 — goal 거부')
                return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _on_task_cancel(self, goal_handle) -> CancelResponse:
        self.get_logger().info('[CobotStateManager] TASK 취소 요청 수신')
        # [구현 필요] 코봇 제어기에 취소 신호 전달
        return CancelResponse.ACCEPT

    def _execute_task(self, goal_handle) -> ExecuteCobotTask.Result:
        request    = goal_handle.request
        task_type  = request.task_type
        task_id    = request.task_id
        robot_name = request.robot_name

        self.get_logger().info(
            f'[CobotStateManager] TASK 실행: task_id={task_id}, '
            f'task_type={task_type}, robot={robot_name}'
        )

        feedback = ExecuteCobotTask.Feedback()

        # ── ACCEPTED feedback 발행 ────────────────────────────────────
        feedback.state             = 'ACCEPTED'
        feedback.message           = f'task {task_id} accepted'
        feedback.progress          = 0.0
        feedback.detected_quantity = 0
        goal_handle.publish_feedback(feedback)

        # ── 작업 단계별 실행 ─────────────────────────────────────────
        phases       = TASK_PHASE_STATES[task_type]
        total_phases = len(phases)
        detected_qty = 0

        for idx, phase in enumerate(phases):
            if goal_handle.is_cancel_requested:
                self._set_state('STANDBY')
                goal_handle.canceled()
                return ExecuteCobotTask.Result(
                    success=False,
                    status='CANCELLED',
                    message='task cancelled',
                    detected_quantity=detected_qty,
                    stock_delta=0,
                )

            with self._lock:
                if self._emergency_stop:
                    self._set_state('SAFETY_STOPPED')
                    goal_handle.abort()
                    return ExecuteCobotTask.Result(
                        success=False,
                        status='FAILED',
                        message='emergency stop triggered',
                        detected_quantity=detected_qty,
                        stock_delta=0,
                    )

            self._set_state(phase)
            feedback.state             = phase
            feedback.message           = f'{phase} in progress'
            feedback.progress          = float(idx) / total_phases
            feedback.detected_quantity = detected_qty
            goal_handle.publish_feedback(feedback)

            success, detected_qty = self._run_phase(task_type, phase, request)

            if not success:
                self._set_state('SAFETY_STOPPED')
                goal_handle.abort()
                return ExecuteCobotTask.Result(
                    success=False,
                    status='FAILED',
                    message=f'{phase} failed',
                    detected_quantity=detected_qty,
                    stock_delta=0,
                )

            # 중간 인식 수량 feedback 갱신
            feedback.detected_quantity = detected_qty
            goal_handle.publish_feedback(feedback)

        # ── STOWING_ARM feedback 발행 (Fleet 다음 PICKY 경로 선계획 트리거) ──
        self._set_state('STOWING_ARM')
        feedback.state    = 'STOWING_ARM'
        feedback.message  = 'stowing arm'
        feedback.progress = 1.0
        goal_handle.publish_feedback(feedback)

        stow_success = self._stow_arm()

        if not stow_success:
            self._set_state('SAFETY_STOPPED')
            goal_handle.abort()
            return ExecuteCobotTask.Result(
                success=False,
                status='FAILED',
                message='arm stowing failed',
                detected_quantity=detected_qty,
                stock_delta=0,
            )

        # ── 최종 완료 — 팔 복귀까지 완전히 끝난 뒤에만 success=True ──
        self._set_state('STANDBY')
        goal_handle.succeed()
        return ExecuteCobotTask.Result(
            success=True,
            status='SUCCESS',
            message='ok',
            detected_quantity=detected_qty,
            stock_delta=0,  # [구현 필요] 실제 재고 반영 수량 반환
        )

    # ── phase별 코봇 제어 ─────────────────────────────────────────────

    def _run_phase(
        self,
        task_type: str,
        phase: str,
        request,
    ) -> tuple[bool, int]:
        """
        phase 에 해당하는 코봇 동작을 수행한다.
        반환값: (success, detected_quantity)
        """
        if phase == 'SORTING':
            # [구현 필요] SORTING_AND_LOAD — 분류 동작 수행
            pass

        elif phase == 'LOADING':
            # [구현 필요] SORTING_AND_LOAD — 적재 동작 수행
            pass

        elif phase == 'INSPECTING':
            # [구현 필요] INSPECTION — 검수 동작 수행
            pass

        elif phase == 'UNLOADING':
            # [구현 필요] UNLOAD — 하역 동작 수행
            pass

        elif phase == 'SCANNING':
            # [구현 필요] DISPLAY_SCAN — 진열대 빈자리 탐색 수행
            pass

        elif phase == 'PLACING':
            # [구현 필요] DISPLAY_PLACE — 진열 상품 진열 동작 수행
            pass

        detected_qty = 0  # [구현 필요] 실제 인식 수량 반환
        return True, detected_qty

    def _stow_arm(self) -> bool:
        """팔을 안전 복귀 자세로 이동한다."""
        # [구현 필요] 코봇 팔 복귀(stow) 동작 수행
        return True

    # ── EmergencyControl 서비스 콜백 ──────────────────────────────────

    def _handle_emergency(
        self,
        request: EmergencyControl.Request,
        response: EmergencyControl.Response,
    ) -> EmergencyControl.Response:
        # [구현 필요] EmergencyControl.Request 필드 확인 후 플래그 처리
        #   정지 요청 시: self._emergency_stop = True, cobot_state → SAFETY_STOPPED
        #                 robot.robot_status 는 Fleet Manager 가 EMERGENCY_STOP 으로 갱신
        #   재개 요청 시: self._emergency_stop = False, cobot_state → STANDBY
        with self._lock:
            # [구현 필요] self._emergency_stop 갱신 및 SAFETY_STOPPED / STANDBY 전환
            pass

        # [구현 필요] 코봇 하드웨어 즉시 정지 / 재개 명령 전달
        # [구현 필요] response 필드 채워서 반환
        return response

    # ── 주기 상태 publish ──────────────────────────────────────────────

    def _periodic_publish(self) -> None:
        with self._lock:
            state = self._cobot_state
        self._publish_state(state)


def main(args=None) -> None:
    rclpy.init(args=args)

    state_mgr = CobotStateManager()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(state_mgr)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
