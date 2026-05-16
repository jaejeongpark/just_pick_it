from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pinky_amr_2",
                executable="odom_logger",
                namespace="amr2",
                name="amr2_odom_logger",
                output="screen",
            )
        ]
    )
