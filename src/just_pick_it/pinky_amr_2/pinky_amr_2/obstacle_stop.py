"""
PICKY2 obstacle stop 노드.

이 파일은 scan 기반 safety layer다.
상위 controller가 만든 속도 명령을 바로 cmd_vel로 보내지 않고,
먼저 LiDAR 전방 거리값을 확인한 뒤 전방 근접 장애물 앞에서는 전진 속도만
감속/차단한다.

입력:
- scan: LaserScan. 전방 장애물 거리 확인용.
- cmd_vel_raw: 상위 controller가 만든 원본 속도 명령.

출력:
- cmd_vel: 실제 Pinky bringup으로 들어갈 최종 속도 명령.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class Picky2ObstacleStop(Node):
    """
    PICKY2 전방 장애물 전진 차단 노드.

    이 노드는 planner나 controller가 아니다.
    목표 지점 계산, 경로 생성, 우회 판단은 하지 않는다.

    책임은 하나다.
    전방 장애물이 slow_distance_m 안에 있으면 전진 속도를 감속하고,
    stop_distance_m 안에 있으면 전진 속도만 0으로 막는다.
    회전은 유지해 좁은 통로/코너에서 방향 전환이 가능하게 한다.
    """

    def __init__(self):
        """정지 판단 parameter, subscriber, publisher를 준비한다."""

        super().__init__("picky2_obstacle_stop")

        self.declare_parameter("front_angle_center_rad", math.pi)
        self.declare_parameter("front_angle_limit_rad", 0.08)
        self.declare_parameter("slow_distance_m", 0.18)
        self.declare_parameter("stop_distance_m", 0.12)
        self.declare_parameter("hard_stop_distance_m", 0.08)
        self.declare_parameter("max_decel_mps2", 0.20)
        self.declare_parameter("allow_rotation_when_blocked", True)

        self.front_angle_center_rad = (
            self.get_parameter("front_angle_center_rad")
            .get_parameter_value()
            .double_value
        )
        self.front_angle_limit_rad = (
            self.get_parameter("front_angle_limit_rad")
            .get_parameter_value()
            .double_value
        )
        self.slow_distance_m = (
            self.get_parameter("slow_distance_m")
            .get_parameter_value()
            .double_value
        )
        self.stop_distance_m = (
            self.get_parameter("stop_distance_m")
            .get_parameter_value()
            .double_value
        )
        self.hard_stop_distance_m = (
            self.get_parameter("hard_stop_distance_m")
            .get_parameter_value()
            .double_value
        )
        self.max_decel_mps2 = (
            self.get_parameter("max_decel_mps2")
            .get_parameter_value()
            .double_value
        )
        self.allow_rotation_when_blocked = (
            self.get_parameter("allow_rotation_when_blocked")
            .get_parameter_value()
            .bool_value
        )

        # scan이 아직 한 번도 안 들어온 상태에서는 막힌 것으로 보지 않는다.
        # 이후 실제 로봇 검증 단계에서 더 보수적으로 바꿀 수 있다.
        self.front_min_distance = float("inf")
        self.is_blocked = False
        self.is_slowing = False
        self.last_output_linear_x = 0.0
        self.last_output_time = self.get_clock().now()

        # scan은 장애물 판단 입력이다. launch namespace 기준 실제 토픽은 /picky2/scan.
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
            f"front_angle_center_rad={self.front_angle_center_rad:.3f}, "
            f"front_angle_limit_rad={self.front_angle_limit_rad:.3f}, "
            f"slow_distance_m={self.slow_distance_m:.3f}, "
            f"stop_distance_m={self.stop_distance_m:.3f}, "
            f"hard_stop_distance_m={self.hard_stop_distance_m:.3f}, "
            f"max_decel_mps2={self.max_decel_mps2:.3f}, "
            f"allow_rotation_when_blocked={self.allow_rotation_when_blocked}"
        )

    def scan_callback(self, msg):
        """
        LaserScan에서 전방 최소 거리를 갱신하고 blocked/slow 상태 전환을 기록한다.

        scan은 계속 들어오고, cmd_vel_raw는 controller가 움직이려고 할 때 들어온다.
        그래서 scan callback에서는 현재 전방 상태를 최신값으로 유지한다.
        """

        previous_blocked = self.is_blocked
        previous_slowing = self.is_slowing

        self.front_min_distance = self.get_front_min_distance(msg)
        self.is_blocked = self.front_min_distance <= self.stop_distance_m
        self.is_slowing = self.front_min_distance <= self.slow_distance_m

        if self.is_blocked and not previous_blocked:
            self.get_logger().warn(
                f"front obstacle detected. distance={self.front_min_distance:.3f}m"
            )

        if previous_blocked and not self.is_blocked:
            self.get_logger().info(
                f"front obstacle cleared. distance={self.front_min_distance:.3f}m"
            )

        if self.is_slowing and not previous_slowing and not self.is_blocked:
            self.get_logger().info(
                f"front obstacle slowdown. distance={self.front_min_distance:.3f}m"
            )

    def cmd_vel_raw_callback(self, msg):
        """
        상위 controller의 raw 속도 명령을 안전 검사 후 최종 cmd_vel로 내보낸다.

        전방 근접 장애물은 전진 속도만 감속/차단한다.
        회전은 유지해 좁은 맵에서 코너를 빠져나올 수 있게 한다.
        """

        filtered = Twist()
        filtered.linear.x = msg.linear.x
        filtered.linear.y = msg.linear.y
        filtered.linear.z = msg.linear.z
        filtered.angular.x = msg.angular.x
        filtered.angular.y = msg.angular.y
        filtered.angular.z = msg.angular.z

        if msg.linear.x > 0.0 and self.is_slowing:
            target_linear_x = self.get_allowed_forward_speed(msg.linear.x)
            filtered.linear.x = self.apply_decel_limit(target_linear_x)
            if self.is_blocked and not self.allow_rotation_when_blocked:
                filtered.angular.z = 0.0
        else:
            self.last_output_linear_x = filtered.linear.x
            self.last_output_time = self.get_clock().now()

        self.cmd_vel_pub.publish(filtered)

    def get_allowed_forward_speed(self, requested_linear_x):
        """
        전방 장애물 거리 기반으로 허용 전진 속도를 계산한다.

        slow_distance_m 밖이면 원래 속도, stop_distance_m 안이면 0,
        그 사이는 거리 비례 감속이다.
        """

        if self.front_min_distance <= self.hard_stop_distance_m:
            return 0.0

        if self.front_min_distance <= self.stop_distance_m:
            return 0.0

        if self.front_min_distance >= self.slow_distance_m:
            return requested_linear_x

        span = self.slow_distance_m - self.stop_distance_m
        if span <= 0.0:
            return 0.0

        ratio = (self.front_min_distance - self.stop_distance_m) / span
        ratio = max(0.0, min(1.0, ratio))
        return requested_linear_x * ratio

    def apply_decel_limit(self, target_linear_x):
        """
        급격한 속도 차단을 줄이기 위해 max_decel_mps2 한도 안에서 감속한다.

        hard_stop_distance_m 안에서는 안전을 우선해 즉시 0으로 보낸다.
        """

        now = self.get_clock().now()
        dt = (now - self.last_output_time).nanoseconds / 1_000_000_000.0
        self.last_output_time = now

        if dt <= 0.0 or self.front_min_distance <= self.hard_stop_distance_m:
            self.last_output_linear_x = target_linear_x
            return target_linear_x

        if target_linear_x >= self.last_output_linear_x:
            self.last_output_linear_x = target_linear_x
            return target_linear_x

        max_delta = max(0.0, self.max_decel_mps2) * dt
        limited = max(target_linear_x, self.last_output_linear_x - max_delta)
        self.last_output_linear_x = limited
        return limited

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
            angle_error = self.get_shortest_angle_error(
                angle,
                self.front_angle_center_rad,
            )

            if abs(angle_error) <= self.front_angle_limit_rad:
                front_ranges.append(distance)

        if not front_ranges:
            return float("inf")

        return min(front_ranges)

    def get_shortest_angle_error(self, angle, target_angle):
        """
        두 각도의 가장 짧은 차이를 -pi ~ pi 범위로 반환한다.

        LaserScan에서 실제 전방이 pi 또는 -pi 근처에 있을 수 있다.
        단순히 target-limit <= angle <= target+limit 형태로 비교하면
        pi 경계에서 범위가 끊기므로, 각도 차이를 원형으로 계산한다.
        """

        return math.atan2(
            math.sin(angle - target_angle),
            math.cos(angle - target_angle),
        )


def main(args=None):
    """ROS2 Python 노드를 초기화하고 callback 처리를 시작한다."""

    rclpy.init(args=args)

    node = Picky2ObstacleStop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
