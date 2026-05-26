import asyncio
import json
import threading

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from fleet_manager.fleet_repository import FleetRepository
from fleet_manager.robot_command_gateway import RobotCommandGateway
from fleet_manager.robot_state_monitor import RobotStateMonitor
from fleet_manager.task_manager import TaskManager
from fleet_manager.traffic_manager import TrafficManager


class FleetManagerNode(Node):
    """
    Fleet Manager 메인 노드 (조립자).

    내부 모듈(FleetRepository, TrafficManager, RobotStateMonitor, ...)은
    별도의 ROS2 Node 가 아닌 일반 Python 클래스이며, 필요한 경우 이 노드를
    인자로 받아 publisher/subscription/timer 를 생성한다.

    [구성]
        FleetRepository       : DB 직접 접근(주문/작업/로봇/zone/입고)
        TrafficManager        : 경로 탐색 / 충돌 회피 / 경로 예약
        RobotStateMonitor     : picky_state 토픽 구독 → TrafficManager 로 전달
        RobotCommandGateway   : task → 로봇 Action goal 전송
        TaskManager           : 주문/입고 → task 변환, 상태 전이
    """

    def __init__(self) -> None:
        super().__init__('fleet_manager')

        self.declare_parameter('robot_ids', ['PICKY1', 'PICKY2', 'COBOT1', 'COBOT2'])
        self.declare_parameter('server_base_url', 'http://192.168.4.1:8000')
        self.declare_parameter('waiting_work_poll_period_sec', 5.0)
        self.declare_parameter('fleet_event_ws_enabled', True)
        self.declare_parameter('fleet_event_reconnect_sec', 2.0)

        robot_ids: list[str] = self.get_parameter('robot_ids').value
        picky_robot_ids = self._filter_picky_robot_ids(robot_ids)
        server_url: str = self.get_parameter('server_base_url').value
        waiting_work_poll_period_sec: float = self.get_parameter('waiting_work_poll_period_sec').value
        fleet_event_ws_enabled: bool = self.get_parameter('fleet_event_ws_enabled').value
        fleet_event_reconnect_sec: float = self.get_parameter('fleet_event_reconnect_sec').value

        self.robot_ids = robot_ids
        self._fleet_event_stop = threading.Event()
        self._fleet_event_thread: threading.Thread | None = None

        self.fleet_repo = FleetRepository(self)
        self.robot_gateway = RobotCommandGateway(self)

        zone_coords = self.fleet_repo.fetch_zone_coords()
        self.traffic_manager = TrafficManager(
            self,
            robot_ids=picky_robot_ids,
            zone_coords=zone_coords or None,
        )

        self.robot_state_monitor = RobotStateMonitor(
            self,
            robot_ids=picky_robot_ids,
            on_state_change=self.traffic_manager.notify_state,
        )

        self.task_manager = TaskManager(
            node=self,
            fleet_repo=self.fleet_repo,
            traffic_manager=self.traffic_manager,
            robot_gateway=self.robot_gateway,
        )
        self.task_timer = self.create_timer(
            waiting_work_poll_period_sec,
            self._poll_waiting_work_if_picky_idle,
        )

        if fleet_event_ws_enabled:
            self._start_fleet_event_listener(server_url, fleet_event_reconnect_sec)

        self.get_logger().info(
            f'[FleetManager] 노드 시작 — robots={robot_ids}, picky={picky_robot_ids}, '
            f'waiting_work_poll={waiting_work_poll_period_sec:.1f}s'
        )

    def _filter_picky_robot_ids(self, robot_ids: list[str]) -> list[str]:
        """전체 Fleet 로봇 목록에서 TrafficManager가 관리할 PICKY만 추출한다."""
        picky_robot_ids = [robot_id for robot_id in robot_ids if str(robot_id).upper().startswith('PICKY')]
        if not picky_robot_ids:
            self.get_logger().warn(
                '[FleetManager] robot_ids에 PICKY가 없습니다. TrafficManager/RobotStateMonitor가 비어 있습니다.'
            )
        return picky_robot_ids

    # ==================================================================
    # Waiting work polling
    # ==================================================================

    def _poll_waiting_work_if_picky_idle(self) -> None:
        """PICKY가 IDLE/STANDBY일 때만 대기 주문/입고 polling을 수행한다."""
        if not self.task_manager.has_idle_picky_for_waiting_work():
            return
        self.task_manager.check_waiting_work()

    # ==================================================================
    # Control Server Fleet Event WebSocket
    # ==================================================================

    def _start_fleet_event_listener(self, server_url: str, reconnect_sec: float) -> None:
        """Control Server fleet event WebSocket listener를 background thread로 시작한다."""
        self._fleet_event_ws_url = self._to_fleet_event_ws_url(server_url)
        self._fleet_event_reconnect_sec = reconnect_sec
        self._fleet_event_thread = threading.Thread(
            target=self._run_fleet_event_listener,
            name='fleet_event_listener',
            daemon=True,
        )
        self._fleet_event_thread.start()

    def _run_fleet_event_listener(self) -> None:
        """async WebSocket loop를 thread 안에서 실행한다."""
        try:
            asyncio.run(self._fleet_event_loop())
        except Exception as exc:
            self.get_logger().warn(f'[FleetManager] fleet event listener 종료: {exc}')

    async def _fleet_event_loop(self) -> None:
        """Control Server fleet event를 수신하고 emergency/resume을 ROS service로 변환한다."""
        try:
            import websockets
        except ModuleNotFoundError:
            self.get_logger().warn(
                '[FleetManager] websockets 패키지가 없어 fleet event listener 비활성화'
            )
            return

        while not self._fleet_event_stop.is_set():
            try:
                async with websockets.connect(self._fleet_event_ws_url) as websocket:
                    self.get_logger().info(
                        f'[FleetManager] fleet event websocket 연결: {self._fleet_event_ws_url}'
                    )
                    while not self._fleet_event_stop.is_set():
                        try:
                            raw = await asyncio.wait_for(websocket.recv(), timeout=1.0)
                        except asyncio.TimeoutError:
                            continue
                        self._handle_fleet_event(raw)
            except Exception as exc:
                if not self._fleet_event_stop.is_set():
                    self.get_logger().warn(
                        f'[FleetManager] fleet event websocket 재연결 대기: {exc}'
                    )
                    await asyncio.sleep(self._fleet_event_reconnect_sec)

    def _handle_fleet_event(self, raw_event: str) -> None:
        """Control Server fleet event payload를 처리한다."""
        try:
            event = json.loads(raw_event)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f'[FleetManager] fleet event JSON 파싱 실패: {exc}')
            return

        event_name = event.get('event')
        if event_name == 'EMERGENCY_STOP':
            self._send_emergency_stop(True, event)
        elif event_name == 'RESUME':
            self._send_emergency_stop(False, event)

    def _send_emergency_stop(self, enabled: bool, event: dict) -> None:
        """Emergency/Resume fleet event를 robot EmergencyControl service 호출로 전파한다."""
        task_id = int(event.get('task_id') or 0)
        request_id = str(event.get('request_id') or event.get('event_id') or '')
        reason = str(event.get('reason') or event.get('event') or 'ADMIN')
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

        self.get_logger().info(
            f"[FleetManager] event={event.get('event')} emergency_control={enabled} 전파: {results}"
        )

    def _to_fleet_event_ws_url(self, server_url: str) -> str:
        """HTTP base URL을 fleet event WebSocket URL로 변환한다."""
        base = server_url.rstrip('/')
        if base.startswith('https://'):
            base = 'wss://' + base[len('https://'):]
        elif base.startswith('http://'):
            base = 'ws://' + base[len('http://'):]
        return f'{base}/api/fleet/ws/events'

    def destroy_node(self) -> bool:
        """노드 종료 시 fleet event listener를 멈춘다."""
        self._fleet_event_stop.set()
        return super().destroy_node()


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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
