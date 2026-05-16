import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


class Amr2OdomLogger(Node):
    def __init__(self):
        super().__init__("amr2_odom_logger")

        # 코드에서는 상대 토픽명 "odom"만 사용한다.
        # 실제 토픽은 launch의 namespace 때문에 /amr2/odom이 된다.
        self.odom_sub = self.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            10,
        )

        self.get_logger().info("odom logger started. waiting for odom...")

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        yaw = self.quaternion_to_yaw(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        )

        self.get_logger().info(f"odom pose: x={position.x:.3f}, y={position.y:.3f}, yaw={yaw:.3f}")

    def quaternion_to_yaw(self, x, y, z, w):
        # ROS Odometry의 방향은 quaternion이므로, 평면 주행에 필요한 yaw만 계산한다.
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)

    node = Amr2OdomLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
