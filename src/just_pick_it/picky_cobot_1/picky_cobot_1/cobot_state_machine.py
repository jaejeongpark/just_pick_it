#!/usr/bin/env python3
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from std_msgs.msg import String

from std_srvs.srv import Trigger

from just_pick_it_interfaces.action import ExecuteTask
from just_pick_it_interfaces.srv import EmergencyControl

# [확정 필요] Vision Service ROS2 인터페이스 타입 임포트
# from just_pick_it_interfaces.srv import VisionScanService

from .cobot_controller import CobotController


# task_type → 작업 단계에서 순서대로 거치는 cobot_state 목록.
# STOWING_ARM 은 모든 task 에서 공통으로 마지막에 발행된다.
# 상태값은 DB cobot_state ENUM 과 일치해야 한다.
# DISPLAY_PLACE 는 picky 에 실린 해당 product 개수만큼 [슬롯 재파지 -> 빈자리 스캔 ->
# IBVS+NN 배치] 를 반복한다(스캔은 PLACING 내부 단계라 별도 task 가 아니다).
TASK_PHASE_STATES = {
    'SORTING_AND_LOAD': ['SORTING', 'LOADING'],
    'INSPECTION':       ['INSPECTING'],
    'UNLOAD':           ['UNLOADING'],
    'DISPLAY_PLACE':    ['PLACING']
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
        self.declare_parameter('dry_run', False)
        # 로봇 구동은 jetcobot_joint_subscriber 드라이버 토픽 경유. 드라이버 namespace 와 일치해야 한다.
        self.declare_parameter('robot_name', 'jetcobot1')
        # INSPECTION 검출 비교용 detection 토픽(local AI 컴퓨터의 yolo_seg_infer).
        self.declare_parameter('detection_topic', '/infer/tracked_objects')
        self.declare_parameter('inspect_min_confidence', 0.5)
        # IBVS+NN 픽 요청/결과 토픽. local 컴퓨터의 ibvs_nn_pick_agent 와 짝을 이룬다.
        self.declare_parameter('pick_timeout_sec', 120.0)
        self.declare_parameter('pick_request_topic', '/ibvs_nn_pick/request')
        self.declare_parameter('pick_result_topic', '/ibvs_nn_pick/result')

        self._robot_id       = self.get_parameter('robot_id').value
        self._vision_timeout = self.get_parameter('vision_service_timeout_sec').value

        self._lock = threading.Lock()
        self._cobot_state  = 'STANDBY'
        self._emergency_stop = False

        # Action 과 서비스, 타이머를 동시에 처리하기 위해 ReentrantCallbackGroup 사용
        cb_group = ReentrantCallbackGroup()

        # cobot_state 퍼블리셔 (Fleet Manager / Traffic Manager 구독).
        # 노드 namespace 가 'cobot1' 이면 자동으로 /cobot1/cobot_state 가 된다.
        self._state_pub = self.create_publisher(String, 'cobot_state', 10)

        # Task Manager 코봇 작업 명령 수신 Action Server
        self._task_action_server = ActionServer(
            self,
            ExecuteTask,
            f'{self._robot_id.lower()}/execute_task',
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

        # picky 적재 슬롯 수동 flush 서비스(UNLOADING 미연동 시 가득 참 해소용).
        self._flush_srv = self.create_service(
            Trigger,
            f'{self._robot_id.lower()}/flush_loadout',
            self._handle_flush_loadout,
            callback_group=cb_group,
        )

        # [디버그] 가상 적재 주입 토픽. SORTING_AND_LOAD 없이 INSPECTION/UNLOAD/DISPLAY_PLACE
        # 를 단독 테스트할 때, 쉼표로 구분한 상품 목록(예: "water,water,cream_bread")을 발행하면
        # 적재 DB 를 그 순서대로 채운다(빈 문자열 발행 시 초기화).
        self._seed_sub = self.create_subscription(
            String,
            f'{self._robot_id.lower()}/seed_loadout',
            self._handle_seed_loadout,
            10,
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

        # 코봇 제어기(토픽 기반). serial 은 jetcobot 드라이버가 점유하므로 제어기는
        # 드라이버 토픽으로 로봇을 구동한다. dry_run=True 면 발행 없이 시뮬레이션한다.
        # SORTING 픽은 IBVS+NN(IbvsNnPickClient)으로 위임한다.
        self._controller = CobotController(
            self,
            robot_name=self.get_parameter('robot_name').value,
            dry_run=self.get_parameter('dry_run').value,
            detection_topic=self.get_parameter('detection_topic').value,
            inspect_min_confidence=self.get_parameter('inspect_min_confidence').value,
            pick_timeout_sec=self.get_parameter('pick_timeout_sec').value,
            pick_request_topic=self.get_parameter('pick_request_topic').value,
            pick_result_topic=self.get_parameter('pick_result_topic').value,
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
        phases = TASK_PHASE_STATES[task_type]
        # SORTING_AND_LOAD 는 그리퍼 1개 제약상 '집기1 -> 적재1' 단위를 quantity 회
        # 번갈아 반복한다(집은 순서대로 picky 슬롯에 적재). 그 외 task 는 1회.
        units = max(1, int(request.quantity or 1)) if task_type == 'SORTING_AND_LOAD' else 1
        total_steps    = len(phases) * units
        step           = 0
        processed_qty  = 0
        last_phase_qty = 0

        for unit in range(units):
            for phase in phases:
                if goal_handle.is_cancel_requested:
                    self._set_state('STANDBY')
                    goal_handle.canceled()
                    return ExecuteTask.Result(
                        success=False,
                        status='CANCELLED',
                        message='task cancelled',
                        processed_quantity=processed_qty,
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
                            processed_quantity=processed_qty,
                            stock_delta=0,
                        )

                self._set_state(phase)
                feedback.state              = phase
                feedback.message            = f'{phase} in progress ({unit + 1}/{units})'
                feedback.progress           = float(step) / total_steps
                feedback.processed_quantity = processed_qty
                goal_handle.publish_feedback(feedback)

                success, last_phase_qty = self._run_phase(task_type, phase, request)

                if not success:
                    self._set_state('SAFETY_STOPPED')
                    goal_handle.abort()
                    return ExecuteTask.Result(
                        success=False,
                        status='FAILED',
                        message=f'{phase} failed',
                        processed_quantity=processed_qty,
                        stock_delta=0,
                    )

                step += 1
                # 단계 완료 feedback — 다음 단계 진입 전 task manager에 성공 알림
                feedback.state              = phase
                feedback.message            = f'{phase} complete ({unit + 1}/{units})'
                feedback.progress           = float(step) / total_steps
                feedback.processed_quantity = processed_qty
                goal_handle.publish_feedback(feedback)

            # 한 단위(집기+적재) 완료 시 처리 수량 +1.
            if task_type == 'SORTING_AND_LOAD':
                processed_qty += last_phase_qty
                # 더 집을 물건이 남아 있으면 STOWING 하지 않고 center 로 복귀(다음 픽 준비).
                # STOWING_ARM 은 quantity 가 모두 소진된 마지막에만 수행한다.
                if unit < units - 1:
                    self._set_state('SORTING')
                    if not self._controller.go_to_center():
                        self._set_state('SAFETY_STOPPED')
                        goal_handle.abort()
                        return ExecuteTask.Result(
                            success=False,
                            status='FAILED',
                            message='center 복귀 실패',
                            processed_quantity=processed_qty,
                            stock_delta=0,
                        )

        # 비 SORTING_AND_LOAD 는 마지막 phase 가 반환한 수량을 결과로 쓴다.
        if task_type != 'SORTING_AND_LOAD':
            processed_qty = last_phase_qty

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
                processed_quantity=processed_qty,
                stock_delta=0,
            )

        # ── 최종 완료 — 팔 복귀까지 완전히 끝난 뒤에만 success=True ──
        self._set_state('STANDBY')
        goal_handle.succeed()
        return ExecuteTask.Result(
            success=True,
            status='SUCCESS',
            message='ok',
            processed_quantity=processed_qty,
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
            # IBVS+NN 으로 request.product_name 1개를 집어 올린다(quantity 반복은 _execute_task).
            return self._controller.run_sorting(request.product_name)
        elif phase == 'LOADING':
            # 집은 상품을 picky 의 다음 빈 슬롯에 적재(집는 순서 = 슬롯 순서).
            return self._controller.run_loading(
                request.product_name, request.order_id, request.target_zone_name
            )

        elif phase == 'INSPECTING':
            # 4 SLOT 관측 자세에서 검출을 적재 기록과 비교(불일치면 abort).
            return self._controller.run_inspecting()

        elif phase == 'UNLOADING':
            # 적재된 모든 item 을 PICKUP SLOT 으로 이송 drop.
            return self._controller.run_unloading()

        elif phase == 'PLACING':
            # picky 에 실린 product 개수만큼 [슬롯 재파지 -> 빈자리 스캔 -> IBVS+NN 배치] 반복.
            return self._controller.run_placing(
                request.product_name, request.target_zone_name
            )

        return True, 0

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

    # ── 적재 슬롯 flush 서비스 콜백 ───────────────────────────────────────

    def _handle_flush_loadout(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        cleared = self._controller.flush_loadout()
        response.success = True
        response.message = f'{cleared} slot(s) cleared'
        self.get_logger().info(f'[CobotStateManager] 적재 flush — {cleared}개 슬롯 비움')
        return response

    # ── 가상 적재 주입 콜백(디버그) ──────────────────────────────────────
    def _handle_seed_loadout(self, msg: String) -> None:
        products = [p for p in msg.data.split(',') if p.strip()]
        seeded = self._controller.seed_loadout(products)
        self.get_logger().info(
            f'[CobotStateManager] 가상 적재 — {seeded}개: {products}')

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
