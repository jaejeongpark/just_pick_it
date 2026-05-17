from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pinky_amr_2",
                executable="obstacle_stop",
                namespace="amr2",
                name="amr2_obstacle_stop",
                output="screen",
                parameters=[
                    {
                        "front_angle_limit_rad": 0.35,
                        "stop_distance_m": 0.35,
                    }
                ],
            )
        ]
    )
