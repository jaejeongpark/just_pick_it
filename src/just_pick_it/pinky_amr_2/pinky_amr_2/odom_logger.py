"""
PICKY2 odom 디버깅 노드.

이 파일은 ROS2 Python 노드의 가장 기본 형태를 익히기 위한 첫 단계다.
역할은 하나다: odom 토픽을 구독해서 로봇의 현재 위치와 방향을 로그로 확인한다.

ROS2에서 odom은 보통 "로봇이 출발 지점 기준으로 얼마나 움직였는지"를 나타낸다.
Pinky bringup 쪽에서 바퀴 encoder 등을 이용해 odom을 publish하고,
이 노드는 그 값을 읽기만 한다.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class Picky2OdomLogger(Node):
    """
    PICKY2 Odometry 디버깅 노드.

    Node를 상속하면 이 클래스가 ROS2 노드가 된다.
    이 노드는 상대 토픽명 "odom"을 구독한다.

    launch 파일에서 namespace="picky2"를 주면 실제 토픽은 아래처럼 바뀐다.

    코드 안 토픽명: odom
    실제 토픽명: /picky2/odom

    이렇게 코드는 상대 토픽명으로 작성하고, namespace/remap은 launch에서 처리하는 방식이
    여러 로봇을 동시에 다룰 때 일반적으로 더 관리하기 좋다.
    """

    def __init__(self):
        """노드 이름을 정하고 odom subscriber를 등록한다."""

        super().__init__("picky2_odom_logger")

        # create_subscription 인자 순서:
        # 1. 메시지 타입: nav_msgs/msg/Odometry
        # 2. 구독할 토픽명: 상대 토픽 odom
        # 3. 메시지가 들어왔을 때 실행할 callback 함수
        # 4. QoS queue depth: 처리가 늦을 때 최대 몇 개까지 쌓아둘지
        self.odom_sub = self.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            10,
        )

        self.get_logger().info("odom logger started. waiting for odom...")

    def odom_callback(self, msg):
        """
        Odometry 메시지가 들어올 때마다 실행된다.

        msg.pose.pose.position은 x, y, z 위치다.
        msg.pose.pose.orientation은 quaternion 방향값이다.
        평면 주행 AMR에서는 보통 z축 회전인 yaw만 뽑아서 많이 본다.
        """

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        yaw = self.quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        self.get_logger().info(
            f"odom pose: x={position.x:.3f}, y={position.y:.3f}, yaw={yaw:.3f}"
        )

    def quaternion_to_yaw(self, x, y, z, w):
        """
        quaternion 방향값에서 yaw만 계산한다.

        ROS의 orientation은 roll/pitch/yaw를 직접 담지 않고 quaternion으로 담는다.
        AMR은 바닥 위에서 움직이므로 roll/pitch보다 yaw, 즉 좌우 회전각이 중요하다.
        반환값 단위는 radian이다.
        """

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    """ROS2 Python 노드를 초기화하고 spin으로 callback 처리를 시작한다."""

    rclpy.init(args=args)

    node = Picky2OdomLogger()

    try:
        # spin은 노드를 계속 실행하면서 subscriber callback을 호출하게 해준다.
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
