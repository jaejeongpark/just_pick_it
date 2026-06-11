#!/usr/bin/env python3
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import Bool, Float64MultiArray, String

from just_pick_it_interfaces.action import ExecuteTask
from just_pick_it_interfaces.srv import EmergencyControl

# [확정 필요] Vision Service ROS2 인터페이스 타입 임포트
# from just_pick_it_interfaces.srv import VisionScanService

from .cobot_controller import CobotController


# task_type → 작업 단계에서 순서대로 거치는 cobot_state 목록.
# STOWING_ARM 은 모든 task 에서 공통으로 마지막에 발행된다.
# 상태값은 DB cobot_state ENUM 과 일치해야 한다.
TASK_PHASE_STATES = {
    'SORTING_AND_LOAD': ['SORTING', 'LOADING'],
    'INSPECTION':       ['INSPECTING'],
    'UNLOAD':           ['UNLOADING'],
    'DISPLAY_SCAN':     ['SCANNING'],
    'DISPLAY_PLACE':    ['PLACING'],
    # 임시 테스트용
    'SORTING_ONLY':     ['SORTING'], #나중에 삭제
}


class CobotStateManager(Node):
    """
    Cobot cobot_state 상태 기계 노드.

    Task Manager로부터 ExecuteTask Action으로 명령을 수신하여
    로봇팔 작업을 수행하고 cobot_state를 전환한다.
    EmergencyControl 서비스로 즉시 정지/재개를 처리한다.

    외부 인터페이스 (launch namespace 기준 상대경로):
      Action Server  : execute_task       (just_pick_it_interfaces/ExecuteTask)
      Service Server : emergency_control  (just_pick_it_interfaces/EmergencyControl)
      Publisher      : cobot_state        (std_msgs/String)
      Service Client : vision_service_name 파라미터로 지정한 Vision Service
    """

    def __init__(self) -> None:
        super().__init__('cobot_state_manager')

        self.declare_parameter('robot_id', 'COBOT1')
        self.declare_parameter('state_publish_interval_sec', 1.0)
        # Vision Service 는 Local AI Server 에서 실행되며 ROS2 cross-machine 으로 연결된다.
        # 두 머신의 ROS_DOMAIN_ID 가 동일해야 한다.
        self.declare_parameter('vision_service_name', '/vision/scan_empty_slot')  # [확정 필요]
        self.declare_parameter('vision_service_timeout_sec', 30.0)
        self.declare_parameter('centering_timeout_sec', 30.0)
        self.declare_parameter('servo_joints_topic', '/vision/servo_joints')    # [확정 필요]
        self.declare_parameter('centering_done_topic', '/vision/centering_done') # [확정 필요]
        self.declare_parameter('cobot_port', '/dev/ttyJETCOBOT')
        self.declare_parameter('cobot_baudrate', 1_000_000)

        self._robot_id          = self.get_parameter('robot_id').value
        self._vision_timeout    = self.get_parameter('vision_service_timeout_sec').value
        self._centering_timeout = self.get_parameter('centering_timeout_sec').value

        self._lock = threading.Lock()
        self._cobot_state    = 'STANDBY'
        self._emergency_stop = False
        # DISPLAY_SCAN 에서 받은 좌표를 DISPLAY_PLACE task 까지 보관한다.
        # 두 task 가 별도 Action goal 로 전달되므로 노드 내부에서 유지한다.
        self._scan_result = None

        # 센터링 루프 제어
        self._centering_active = False   # 센터링 구독 처리 활성 여부
        self._centering_done   = threading.Event()

        # Action 과 서비스, 타이머를 동시에 처리하기 위해 ReentrantCallbackGroup 사용
        cb_group = ReentrantCallbackGroup()

        # cobot_state 퍼블리셔 (Fleet Manager / Traffic Manager 구독).
        # 노드 namespace 가 'cobot1' 이면 자동으로 /cobot1/cobot_state 가 된다.
        self._state_pub = self.create_publisher(String, 'cobot_state', 10)

        # Task Manager 코봇 작업 명령 수신 Action Server
        self._task_action_server = ActionServer(
            self,
            ExecuteTask,
            f'{self._robot_id}/execute_task',
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

        # Vision Service 클라이언트 (Local AI Server, ROS2 cross-machine)
        # [확정 필요] VisionScanService 인터페이스 타입이 확정되면 주석 해제
        # vision_service_name = self.get_parameter('vision_service_name').value
        # self._vision_client = self.create_client(
        #     VisionScanService,
        #     vision_service_name,
        #     callback_group=cb_group,
        # )

        # 센터링 관절각 스트리밍 구독 (서버 publish 토픽, [확정 필요])
        servo_topic = self.get_parameter('servo_joints_topic').value
        self.create_subscription(
            Float64MultiArray,
            servo_topic,
            self._on_servo_joints,
            10,
            callback_group=cb_group,
        )

        # 센터링 완료 신호 구독 (서버 publish 토픽, [확정 필요])
        centering_done_topic = self.get_parameter('centering_done_topic').value
        self.create_subscription(
            Bool,
            centering_done_topic,
            self._on_centering_done,
            10,
            callback_group=cb_group,
        )

        # 코봇 하드웨어 제어기
        self._controller = CobotController(
            self,
            port=self.get_parameter('cobot_port').value,
            baudrate=self.get_parameter('cobot_baudrate').value,
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

    # ── Vision Service 호출 ───────────────────────────────────────────

    def _call_vision_service(self, request) -> tuple[bool, object]:
        """
        Vision Service 에 ROS2 서비스 요청을 보내고 응답을 기다린다.
        Local AI Server 와 cross-machine ROS2 로 통신한다.
        반환값: (success, response)
        """
        if not self._vision_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().error(
                '[CobotStateManager] Vision Service 연결 불가 — '
                'Local AI Server 의 ROS_DOMAIN_ID 와 네트워크 연결을 확인하세요'
            )
            return False, None

        future = self._vision_client.call_async(request)

        # MultiThreadedExecutor 안에서 future 를 기다리기 위해 threading.Event 사용.
        # rclpy.spin_until_future_complete 는 이미 spin 중인 executor 와 충돌하므로 사용하지 않는다.
        event = threading.Event()
        future.add_done_callback(lambda f: event.set())

        if not event.wait(timeout=self._vision_timeout):
            self.get_logger().error(
                f'[CobotStateManager] Vision Service 응답 타임아웃 '
                f'({self._vision_timeout}s)'
            )
            return False, None

        return True, future.result()

    # ── ExecuteTask Action 콜백 ──────────────────────────────────────

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

    def _execute_task(self, goal_handle) -> ExecuteTask.Result:
        request    = goal_handle.request
        task_type  = request.task_type
        task_id    = request.task_id
        

        self.get_logger().info(
            f'[CobotStateManager] TASK 실행: task_id={task_id}, '
            f'task_type={task_type}'
        )

        feedback = ExecuteTask.Feedback()

        # ── ACCEPTED feedback 발행 ────────────────────────────────────
        feedback.state             = 'ACCEPTED'
        feedback.message           = f'task {task_id} accepted'
        feedback.progress          = 0.0
        feedback.processed_quantity = 0
        goal_handle.publish_feedback(feedback)

        # ── 작업 단계별 실행 ─────────────────────────────────────────
        phases       = TASK_PHASE_STATES[task_type]
        total_phases = len(phases)
        detected_qty = 0

        for idx, phase in enumerate(phases):
            if goal_handle.is_cancel_requested:
                self._set_state('STANDBY')
                goal_handle.canceled()
                return ExecuteTask.Result(
                    success=False,
                    status='CANCELLED',
                    message='task cancelled',
                    processed_quantity=detected_qty,
                    stock_delta=0,
                )

            with self._lock:
                if self._emergency_stop:
                    self._set_state('SAFETY_STOPPED')
                    goal_handle.abort()
                    return ExecuteTask.Result(
                        success=False,
                        status='FAILED',
                        message='emergency stop triggered',
                        processed_quantity=detected_qty,
                        stock_delta=0,
                    )

            self._set_state(phase)
            feedback.state             = phase
            feedback.message           = f'{phase} in progress'
            feedback.progress          = float(idx) / total_phases
            feedback.processed_quantity = detected_qty
            goal_handle.publish_feedback(feedback)

            success, detected_qty = self._run_phase(task_type, phase, request)

            if not success:
                self._set_state('SAFETY_STOPPED')
                goal_handle.abort()
                return ExecuteTask.Result(
                    success=False,
                    status='FAILED',
                    message=f'{phase} failed',
                    processed_quantity=detected_qty,
                    stock_delta=0,
                )

            # 중간 인식 수량 feedback 갱신
            feedback.processed_quantity = detected_qty
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
            return ExecuteTask.Result(
                success=False,
                status='FAILED',
                message='arm stowing failed',
                processed_quantity=detected_qty,
                stock_delta=0,
            )

        # ── 최종 완료 — 팔 복귀까지 완전히 끝난 뒤에만 success=True ──
        self._set_state('STANDBY')
        goal_handle.succeed()
        return ExecuteTask.Result(
            success=True,
            status='SUCCESS',
            message='ok',
            processed_quantity=detected_qty,
            stock_delta=0,  # [구현 필요] 실제 재고 반영 수량 반환
        )

    # ── 센터링 스트리밍 콜백 ──────────────────────────────────────────────

    def _on_servo_joints(self, msg: Float64MultiArray) -> None:
        """서버 스트리밍 관절각 수신 — SORTING 센터링 활성 중에만 코봇에 반영."""
        if not self._centering_active:
            return
        self._controller.stream_joint_angles(list(msg.data))

    def _on_centering_done(self, msg: Bool) -> None:
        """서버 센터링 완료 신호 수신."""
        if msg.data:
            self._centering_done.set()

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
            return self._run_sorting_phase(request)

        elif phase == 'LOADING':
            return self._controller.run_loading(request)

        elif phase == 'INSPECTING':
            return self._controller.run_inspecting(request)

        elif phase == 'UNLOADING':
            return self._controller.run_unloading(request)

        elif phase == 'SCANNING':
            # [확정 필요] VisionScanService 인터페이스 타입 확정 후 아래 주석 해제
            #
            # req = VisionScanService.Request()
            # req.task_id          = request.task_id
            # req.target_zone_name = request.target_zone_name
            #
            # success, response = self._call_vision_service(req)
            # if not success:
            #     return False, 0
            #
            # self._scan_result = response
            return True, 0

        elif phase == 'PLACING':
            success, qty = self._controller.run_placing(self._scan_result, request)
            if success:
                self._scan_result = None
            return success, qty

        return True, 0

    def _run_sorting_phase(self, request) -> tuple[bool, int]:
        """
        1단계: 서버 스트리밍 관절각으로 물체를 카메라 중앙에 정렬
        2단계: 서버 학습 파지 궤적으로 파지 실행
        """
        self._centering_done.clear()
        self._centering_active = True
        self.get_logger().info('[CobotStateManager] SORTING: 센터링 대기 중...')

        try:
            centered = self._centering_done.wait(timeout=self._centering_timeout)
        finally:
            self._centering_active = False

        if not centered:
            self.get_logger().error(
                f'[CobotStateManager] SORTING 센터링 타임아웃 ({self._centering_timeout}s)'
            )
            return False, 0

        self.get_logger().info('[CobotStateManager] SORTING: 센터링 완료 — 파지 실행')

        # [구현 필요] 서버에서 학습 파지 궤적(waypoint 목록) 수신
        # 예: grasp_trajectory = self._call_grasp_service(request).trajectory
        grasp_trajectory: list[list[float]] = []

        return self._controller.run_sorting(grasp_trajectory)

    def _stow_arm(self) -> bool:
        """팔을 안전 복귀 자세로 이동한다."""
        return self._controller.stow_arm()

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
            # [구현 필요] EmergencyControl.Request 필드명 확정 후 조건 교체
            # 아래는 request.stop (bool) 필드를 가정한 예시
            stop = getattr(request, 'stop', True)
            self._emergency_stop = stop

        if stop:
            self._controller.emergency_stop()
            self._set_state('SAFETY_STOPPED')
        else:
            self._set_state('STANDBY')

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
