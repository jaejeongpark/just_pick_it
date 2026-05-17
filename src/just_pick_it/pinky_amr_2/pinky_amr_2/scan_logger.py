"""
AMR2 scan 디버깅 노드.

이 파일은 LaserScan 메시지를 읽는 연습용 노드다.
LiDAR가 publish하는 scan 토픽을 받아 전체 최소 거리와 전방 최소 거리를 로그로 출력한다.

scan을 먼저 보는 이유는 obstacle stop, local obstacle avoidance, blocked 판단이
전부 LiDAR 거리값을 기반으로 시작되기 때문이다.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class Amr2ScanLogger(Node):
    """
    AMR2 LaserScan 디버깅 노드.

    LaserScan 메시지의 ranges에는 각도별 거리값이 배열로 들어온다.
    이 노드는 그중 유효한 거리만 골라서 다음 두 값을 출력한다.

    1. 전체 scan 중 최소 거리
    2. 로봇 정면 범위 안의 최소 거리

    front_angle_limit_rad는 LiDAR 하드웨어 고유값이 아니다.
    우리가 "정면"이라고 판단할 각도 범위를 정하는 정책값이다.
    그래서 코드에 박아두지 않고 ROS parameter로 선언한다.
    """

    def __init__(self):
        """노드 parameter를 선언하고 scan subscriber를 등록한다."""

        super().__init__("amr2_scan_logger")

        # declare_parameter는 이 노드가 받을 수 있는 설정값을 등록한다.
        # launch 파일이나 YAML에서 같은 이름의 parameter를 넘기면 그 값으로 바뀐다.
        self.declare_parameter("front_angle_limit_rad", 0.35)

        self.front_angle_limit_rad = (
            self.get_parameter("front_angle_limit_rad")
            .get_parameter_value()
            .double_value
        )

        # 실제 실행 토픽은 launch namespace 때문에 /amr2/scan이 된다.
        self.scan_sub = self.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,
            10,
        )

        self.get_logger().info(
            "scan logger started. "
            f"front_angle_limit_rad={self.front_angle_limit_rad:.3f}"
        )

    def scan_callback(self, msg):
        """
        LaserScan 메시지가 들어올 때마다 전체 최소 거리와 전방 최소 거리를 출력한다.

        LaserScan ranges에는 inf, nan, range_min보다 작은 값, range_max보다 큰 값이
        섞일 수 있다. 그래서 먼저 실제로 쓸 수 있는 거리만 valid_ranges에 담는다.
        """

        valid_ranges = []

        for distance in msg.ranges:
            if not math.isfinite(distance):
                continue

            if distance < msg.range_min:
                continue

            if distance > msg.range_max:
                continue

            valid_ranges.append(distance)

        if not valid_ranges:
            self.get_logger().warn("scan received, but no valid ranges")
            return

        min_distance = min(valid_ranges)
        front_min_distance = self.get_front_min_distance(msg)

        self.get_logger().info(
            f"scan min={min_distance:.3f}m, front_min={front_min_distance:.3f}m"
        )

    def get_front_min_distance(self, msg):
        """
        로봇 정면 각도 범위 안에 있는 유효 거리 중 최소값을 반환한다.

        LaserScan은 ranges 배열의 index만으로 각도를 바로 알 수 없다.
        각 index의 실제 각도는 아래 공식으로 계산한다.

        angle = angle_min + index * angle_increment
        """

        front_ranges = []

        for index, distance in enumerate(msg.ranges):
            if not math.isfinite(distance):
                continue

            if distance < msg.range_min:
                continue

            if distance > msg.range_max:
                continue

            angle = msg.angle_min + index * msg.angle_increment

            if -self.front_angle_limit_rad <= angle <= self.front_angle_limit_rad:
                front_ranges.append(distance)

        if not front_ranges:
            return float("inf")

        return min(front_ranges)


def main(args=None):
    """ROS2 Python 노드를 초기화하고 scan callback 처리를 시작한다."""

    rclpy.init(args=args)

    node = Amr2ScanLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
