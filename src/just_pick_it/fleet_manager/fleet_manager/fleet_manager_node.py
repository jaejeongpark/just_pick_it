import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from fleet_manager.fleet_api_server import FleetApiServer
from fleet_manager.fleet_repository import FleetRepository
from fleet_manager.robot_command_gateway import RobotCommandGateway
from fleet_manager.robot_state_monitor import RobotStateMonitor
from fleet_manager.task_manager import CHARGE_BATTERY_THRESHOLD, TaskManager
from fleet_manager.traffic_manager import TrafficManager


# =====================================
# Fleet Manager node
# =====================================

class FleetManagerNode(Node):
    """Fleet Manager 메인 노드.

    각 기능 클래스는 별도 ROS2 Node가 아니라 이 노드에 조립되는 Python 객체다.

    - FleetRepository: DB 직접 접근
    - TrafficManager: PICKY 경로 탐색/예약
    - RobotStateMonitor: PICKY 상태 구독
    - RobotCommandGateway: 로봇 Action/Service 명령 송신
    - TaskManager: 주문/진열 요청을 task 흐름으로 변환
    - FleetApiServer: Web Gateway가 호출하는 HTTP/WebSocket API
    """

    def __init__(self) -> None:
        super().__init__('fleet_manager')

        self._declare_parameters()
        config = self._load_config()

        self.robot_ids = config["robot_ids"]
        picky_robot_ids = self._filter_picky_robot_ids(self.robot_ids)

        self.fleet_repo = FleetRepository(self)
        self.robot_gateway = RobotCommandGateway(self)
        # action 클라이언트 discovery 를 기동 구간에 끝내 첫 주문 dispatch timeout 방지.
        self.robot_gateway.prewarm(picky_robot_ids)
        self.traffic_manager = self._create_traffic_manager(picky_robot_ids)
        # TaskManager 를 먼저 만들어 RobotStateMonitor 의 battery hook 으로 넘긴다.
        self.task_manager = self._create_task_manager()
        # 재시작 복구(R1) 전까지 poll/dispatch 게이트를 닫는다. reconcile_timer 가 1회 해제한다.
        self.task_manager.arm_reconcile()
        self.robot_state_monitor = self._create_robot_state_monitor(picky_robot_ids, config)
        self.task_timer = self.create_timer(
            config["waiting_work_poll_period_sec"],
            self._poll_waiting_work_if_picky_idle,
        )
        self.api_server = self._create_api_server(config)
        # executor spin 이후(action server 탐색·텔레메트리 도착 보장) 1회 재시작 복구 수행.
        self.reconcile_timer = self.create_timer(
            config["reconcile_delay_sec"],
            self._run_startup_reconcile_once,
        )

        self.get_logger().info(
            f"[FleetManager] 노드 시작 — robots={self.robot_ids}, "
            f"picky={picky_robot_ids}, "
            f"waiting_work_poll={config['waiting_work_poll_period_sec']:.1f}s"
        )

    # =====================================
    # Config
    # =====================================

    def _declare_parameters(self) -> None:
        self.declare_parameter('robot_ids', ['PICKY1', 'PICKY2', 'COBOT1', 'COBOT2'])
        self.declare_parameter('waiting_work_poll_period_sec', 5.0)
        self.declare_parameter('robot_state_flush_period_sec', 1.0)
        self.declare_parameter('reconcile_delay_sec', 2.0)
        self.declare_parameter('api_enabled', True)
        self.declare_parameter('api_host', '0.0.0.0')
        self.declare_parameter('api_port', 8100)
        self.declare_parameter('api_push_interval_sec', 1.0)

    def _load_config(self) -> dict:
        return {
            "robot_ids": self.get_parameter('robot_ids').value,
            "waiting_work_poll_period_sec": self.get_parameter('waiting_work_poll_period_sec').value,
            "robot_state_flush_period_sec": self.get_parameter('robot_state_flush_period_sec').value,
            "reconcile_delay_sec": self.get_parameter('reconcile_delay_sec').value,
            "api_enabled": self.get_parameter('api_enabled').value,
            "api_host": self.get_parameter('api_host').value,
            "api_port": self.get_parameter('api_port').value,
            "api_push_interval_sec": self.get_parameter('api_push_interval_sec').value,
        }

    @staticmethod
    def _filter_picky_robot_ids(robot_ids: list[str]) -> list[str]:
        return [robot_id for robot_id in robot_ids if str(robot_id).upper().startswith('PICKY')]

    # =====================================
    # Component wiring
    # =====================================

    def _create_traffic_manager(self, picky_robot_ids: list[str]) -> TrafficManager:
        if not picky_robot_ids:
            self.get_logger().warn(
                '[FleetManager] robot_ids에 PICKY가 없습니다. TrafficManager/RobotStateMonitor가 비어 있습니다.'
            )

        zone_coords = self.fleet_repo.fetch_zone_coords()
        return TrafficManager(
            self,
            robot_ids=picky_robot_ids,
            zone_coords=zone_coords or None,
        )

    def _create_robot_state_monitor(
        self,
        picky_robot_ids: list[str],
        config: dict,
    ) -> RobotStateMonitor:
        return RobotStateMonitor(
            self,
            robot_ids=picky_robot_ids,
            fleet_repo=self.fleet_repo,
            on_state_change=self.traffic_manager.notify_state,
            on_battery_update=self.task_manager.handle_battery_update,
            db_flush_period_sec=config["robot_state_flush_period_sec"],
            battery_notify_threshold=CHARGE_BATTERY_THRESHOLD,
        )

    def _create_task_manager(self) -> TaskManager:
        return TaskManager(
            node=self,
            fleet_repo=self.fleet_repo,
            traffic_manager=self.traffic_manager,
            robot_gateway=self.robot_gateway,
            active_robot_ids=self.robot_ids,
        )

    def _create_api_server(self, config: dict) -> FleetApiServer | None:
        if not config["api_enabled"]:
            return None

        api_server = FleetApiServer(
            self,
            self.fleet_repo,
            host=config["api_host"],
            port=config["api_port"],
            push_interval_sec=config["api_push_interval_sec"],
            admin_snapshot_provider=self._build_admin_snapshot,
            debug_task_success_injector=self.task_manager.inject_running_robot_task_success,
        )
        api_server.start()
        return api_server

    def _build_admin_snapshot(self) -> dict | None:
        """DB snapshot에 TaskManager runtime route 정보를 붙여 관리자 UI로 보낸다."""
        snapshot = self.fleet_repo.get_snapshot()
        return self.task_manager.enrich_admin_snapshot_with_runtime_paths(snapshot)

    # =====================================
    # Waiting work polling
    # =====================================

    def _poll_waiting_work_if_picky_idle(self) -> None:
        """TaskManager scheduler를 주기 실행한다.

        신규 주문/진열 polling은 TaskManager 내부에서 가용 unit이 있을 때만 수행하고,
        기존 flow advance / CHARGE 정리 / ASSIGNED dispatch는 항상 재시도한다.
        """
        self.task_manager.check_waiting_work()

    def _run_startup_reconcile_once(self) -> None:
        """executor spin 이후 1회만 재시작 복구(R1)를 수행하고 타이머를 멈춘다."""
        self.reconcile_timer.cancel()
        self.task_manager.reconcile_on_startup()

    # =====================================
    # Emergency/resume
    # =====================================

    def _propagate_emergency(
        self,
        enabled: bool,
        *,
        reason: str = 'ADMIN',
        task_id: int = 0,
        request_id: str = '',
    ) -> dict:
        """robot EmergencyControl 전파 + TaskManager dispatch 제어."""
        results = self.robot_gateway.set_emergency_stop(
            self.robot_ids,
            enabled,
            reason=reason,
            task_id=task_id,
            request_id=request_id,
        )
        if enabled:
            self.task_manager.handle_emergency_stop()
        else:
            self.task_manager.handle_resume()
        return results

    def trigger_emergency_stop(self, enabled: bool, *, reason: str = 'ADMIN') -> dict:
        """API 경유 emergency/resume: DB 전이 + robot 전파 + TaskManager 처리."""
        if enabled:
            result = self.fleet_repo.apply_emergency_stop()
        else:
            result = self.fleet_repo.apply_resume()

        results = self._propagate_emergency(enabled, reason=reason)
        self.get_logger().info(
            f"[FleetManager] API emergency_control={enabled} db={result} 전파: {results}"
        )
        return result

    # =====================================
    # Lifecycle
    # =====================================

    def destroy_node(self) -> bool:
        """노드 종료 시 API 서버를 멈춘다."""
        if self.api_server is not None:
            self.api_server.stop()
        return super().destroy_node()


# =====================================
# Entry point
# =====================================

def main(args=None) -> None:
    rclpy.init(args=args)
    node = FleetManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
