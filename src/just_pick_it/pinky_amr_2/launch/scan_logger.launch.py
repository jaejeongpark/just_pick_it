from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pinky_amr_2",
                executable="scan_logger",
                namespace="picky2",
                name="picky2_scan_logger",
                output="screen",
                parameters=[
                    {
                        "front_angle_limit_rad": 0.35,
                    }
                ],
            )
        ]
    )
