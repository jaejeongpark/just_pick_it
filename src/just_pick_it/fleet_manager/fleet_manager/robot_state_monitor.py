from collections.abc import Callable

from rclpy.node import Node
from std_msgs.msg import String


StateCallback = Callable[[str, str], None]


class RobotStateMonitor:
    """PICKY 로봇들의 picky_state 토픽을 구독하고 콜백으로 상태 변화를 전달한다.

    상태 캐시는 보유하지 않으며, 콜백 수신자(TrafficManager 등)가 자체적으로 관리한다.
    """

    def __init__(
        self,
        node: Node,
        robot_ids: list[str],
        on_state_change: StateCallback,
    ) -> None:
        self._node = node

        for robot_id in robot_ids:
            ns = robot_id.lower()
            node.create_subscription(
                String,
                f'/{ns}/picky_state',
                lambda msg, rid=robot_id: on_state_change(rid, msg.data),
                10,
            )

        node.get_logger().info(
            f'[RobotStateMonitor] picky_state 구독 시작 — {robot_ids}'
        )
