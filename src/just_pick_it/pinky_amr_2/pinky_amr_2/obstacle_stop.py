"""
AMR2 obstacle stop 노드.

이 파일은 scan 기반 safety layer의 첫 버전이다.
상위 controller가 만든 속도 명령을 바로 cmd_vel로 보내지 않고,
먼저 LiDAR 전방 거리값을 확인한 뒤 안전할 때만 통과시킨다.

입력:
- scan: LaserScan. 전방 장애물 거리 확인용.
- cmd_vel_raw: 상위 controller가 만든 원본 속도 명령.

출력:
- cmd_vel: 실제 Pinky bringup으로 들어갈 최종 속도 명령.

나중에 go-to-goal이나 path tracker는 cmd_vel에 직접 publish하지 않고
cmd_vel_raw로 publish하게 만들면 된다. 그러면 이 노드가 마지막 안전 필터가 된다.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class Amr2ObstacleStop(Node):
    """
    AMR2 전방 장애물 정지 노드.

    이 노드는 planner나 controller가 아니다.
    목표 지점 계산, 경로 생성, 우회 판단은 하지 않는다.

    책임은 하나다.
    전방 장애물이 stop_distance_m 안에 있으면 최종 cmd_vel을 0으로 막는다.
    장애물이 없으면 cmd_vel_raw를 cmd_vel로 그대로 통과시킨다.
    """

    def __init__(self):
        """정지 판단 parameter, subscriber, publisher를 준비한다."""

        super().__init__("amr2_obstacle_stop")

        # front_angle_limit_rad:
        #   로봇 기준 정면으로 볼 좌우 각도 범위다.
        #   LiDAR 스펙값이 아니라 우리 safety 정책값이다.
        # stop_distance_m:
        #   이 거리 안에 장애물이 있으면 최종 속도를 0으로 만든다.
        self.declare_parameter("front_angle_limit_rad", 0.35)
        self.declare_parameter("stop_distance_m", 0.35)

        self.front_angle_limit_rad = (
            self.get_parameter("front_angle_limit_rad")
            .get_parameter_value()
            .double_value
        )
        self.stop_distance_m = (
            self.get_parameter("stop_distance_m")
            .get_parameter_value()
            .double_value
        )

        # scan이 아직 한 번도 안 들어온 상태에서는 막힌 것으로 보지 않는다.
        # 이후 실제 로봇 검증 단계에서 더 보수적으로 바꿀 수 있다.
        self.front_min_distance = float("inf")
        self.is_blocked = False

        # scan은 장애물 판단 입력이다. launch namespace 기준 실제 토픽은 /amr2/scan.
        self.scan_sub = self.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,
            10,
        )

        # cmd_vel_raw는 상위 주행 노드가 만든 원본 속도 명령이다.
        # obstacle_stop은 이 명령을 검사한 뒤 cmd_vel로 내보낸다.
        self.cmd_vel_raw_sub = self.create_subscription(
            Twist,
            "cmd_vel_raw",
            self.cmd_vel_raw_callback,
            10,
        )

        # cmd_vel은 실제 하드웨어 bringup이 구독할 최종 속도 명령이다.
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)

        self.get_logger().info(
            "obstacle stop started. "
            f"front_angle_limit_rad={self.front_angle_limit_rad:.3f}, "
            f"stop_distance_m={self.stop_distance_m:.3f}"
        )

    def scan_callback(self, msg):
        """
        LaserScan에서 전방 최소 거리를 갱신하고 blocked 상태 전환을 기록한다.

        scan은 계속 들어오고, cmd_vel_raw는 controller가 움직이려고 할 때 들어온다.
        그래서 scan callback에서는 현재 전방 상태를 최신값으로 유지한다.
        """

        previous_blocked = self.is_blocked

        self.front_min_distance = self.get_front_min_distance(msg)
        self.is_blocked = self.front_min_distance <= self.stop_distance_m

        if self.is_blocked and not previous_blocked:
            self.publish_stop()
            self.get_logger().warn(
                f"front obstacle detected. distance={self.front_min_distance:.3f}m"
            )

        if previous_blocked and not self.is_blocked:
            self.get_logger().info(
                f"front obstacle cleared. distance={self.front_min_distance:.3f}m"
            )

    def cmd_vel_raw_callback(self, msg):
        """
        상위 controller의 raw 속도 명령을 안전 검사 후 최종 cmd_vel로 내보낸다.

        blocked 상태면 원본 명령이 전진이든 회전이든 일단 완전 정지로 막는다.
        첫 버전에서는 단순 정지가 목표이고, 회전 허용/우회는 다음 단계에서 판단한다.
        """

        if self.is_blocked:
            self.publish_stop()
            return

        self.cmd_vel_pub.publish(msg)

    def get_front_min_distance(self, msg):
        """
        로봇 정면 각도 범위 안에 있는 유효 scan 거리 중 최소값을 반환한다.

        ranges 배열 안의 거리값 중 inf/nan, range_min보다 작은 값,
        range_max보다 큰 값은 장애물 판단에 쓰지 않는다.
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

    def publish_stop(self):
        """
        최종 cmd_vel에 완전 정지 명령을 publish한다.

        geometry_msgs/msg/Twist의 기본값은 모든 선속도/각속도가 0이다.
        즉 Twist()를 publish하면 정지 명령이 된다.
        """

        self.cmd_vel_pub.publish(Twist())


def main(args=None):
    """ROS2 Python 노드를 초기화하고 callback 처리를 시작한다."""

    rclpy.init(args=args)

    node = Amr2ObstacleStop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
