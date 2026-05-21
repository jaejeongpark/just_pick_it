import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from fleet_manager.traffic_manager import TrafficManager


class FleetManagerNode(Node):
    """
    Fleet Manager 메인 노드.

    TrafficManager 와 TaskManager 를 함께 보유한다.
    TaskManager 가 경로가 필요할 때 TrafficManager 를 직접 호출하는 구조.

    [사용 흐름]
        waypoints = self.traffic_manager.find_path(source, target, robot_id)
        self.task_manager.send_move_command(robot_id, task_type, waypoints)
    """

    def __init__(self) -> None:
        super().__init__('fleet_manager')

        self.declare_parameter('robot_ids', ['AMR_001', 'AMR_002'])
        self.declare_parameter('server_base_url', 'http://192.168.4.1:8000')

        robot_ids: list[str] = self.get_parameter('robot_ids').value
        server_url: str = self.get_parameter('server_base_url').value

        self.traffic_manager = TrafficManager(self, robot_ids, server_url)

        # TaskManager 는 다른 팀 구현체를 여기에 추가한다.
        # self.task_manager = TaskManager(self, self.traffic_manager)

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
