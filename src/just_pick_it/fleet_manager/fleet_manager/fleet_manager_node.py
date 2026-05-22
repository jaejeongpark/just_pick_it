import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from fleet_manager.control_server_client import ControlServerClient
from fleet_manager.robot_state_monitor import RobotStateMonitor
from fleet_manager.traffic_manager import TrafficManager


class FleetManagerNode(Node):
    """
    Fleet Manager 메인 노드 (조립자).

    내부 모듈(ControlServerClient, TrafficManager, RobotStateMonitor, ...)은
    별도의 ROS2 Node 가 아닌 일반 Python 클래스이며, 필요한 경우 이 노드를
    인자로 받아 publisher/subscription/timer 를 생성한다.

    [Phase 1 구성]
        ControlServerClient   : Control Server HTTP API 통신
        TrafficManager        : 경로 탐색 / 충돌 회피 / 경로 예약
        RobotStateMonitor     : picky_state 토픽 구독 → TrafficManager 로 전달

    [추가 예정]
        TaskManager           : 주문/입고 → task 변환, 상태 전이 (Phase 2)
        RobotCommandGateway   : task → 로봇 Action goal 전송 (Phase 3)
    """

    def __init__(self) -> None:
        super().__init__('fleet_manager')

        self.declare_parameter('robot_ids', ['PICKY1', 'PICKY2'])
        self.declare_parameter('server_base_url', 'http://192.168.4.1:8000')

        robot_ids: list[str] = self.get_parameter('robot_ids').value
        server_url: str = self.get_parameter('server_base_url').value

        self.control_server = ControlServerClient(self, server_url)

        zone_coords = self.control_server.fetch_zone_coords()
        self.traffic_manager = TrafficManager(
            self,
            robot_ids=robot_ids,
            zone_coords=zone_coords or None,
        )

        self.robot_state_monitor = RobotStateMonitor(
            self,
            robot_ids=robot_ids,
            on_state_change=self.traffic_manager.notify_state,
        )

        self.get_logger().info(f'[FleetManager] 노드 시작 — 로봇: {robot_ids}')


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FleetManagerNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        executor.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
