"""Republish PICKY2 odom with robot-prefixed TF frame ids."""

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def _prefixed(frame_prefix: str, frame_id: str) -> str:
    frame_id = frame_id.lstrip("/")
    if not frame_id:
        return frame_id
    if not frame_prefix:
        return frame_id
    if frame_id.startswith(frame_prefix):
        return frame_id
    return f"{frame_prefix}{frame_id}"


class Amr2TfFrameAdapter(Node):
    """Convert raw odom frame ids from the base driver to PICKY2 frame ids."""

    def __init__(self):
        super().__init__("amr2_tf_frame_adapter")

        self.declare_parameter("frame_prefix", "picky2/")
        self.declare_parameter("raw_odom_topic", "odom_raw")
        self.declare_parameter("odom_topic", "odom")

        self._frame_prefix = self.get_parameter("frame_prefix").value
        raw_odom_topic = self.get_parameter("raw_odom_topic").value
        odom_topic = self.get_parameter("odom_topic").value

        self._odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, raw_odom_topic, self._odom_cb, 20)

        self.get_logger().info(
            "TF frame adapter started: "
            f"{raw_odom_topic} -> {odom_topic}, prefix='{self._frame_prefix}'"
        )

    def _odom_cb(self, msg: Odometry) -> None:
        odom = Odometry()
        odom.header = msg.header
        odom.child_frame_id = _prefixed(self._frame_prefix, msg.child_frame_id)
        odom.pose = msg.pose
        odom.twist = msg.twist
        odom.header.frame_id = _prefixed(self._frame_prefix, msg.header.frame_id)
        self._odom_pub.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = odom.child_frame_id
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.translation.z = odom.pose.pose.position.z
        transform.transform.rotation = odom.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = Amr2TfFrameAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
