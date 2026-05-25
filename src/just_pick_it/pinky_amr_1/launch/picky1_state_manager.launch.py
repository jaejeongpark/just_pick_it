"""PICKY1용 State Manager launch.

state_manager 진입점은 한 프로세스 안에 state_manager / reverse_docking /
move_to_goal 세 노드를 동시에 띄운다. 모두 `/picky1` namespace 에 들어가
상대경로 토픽(cmd_vel, picky_state, move_command, dock_command,
battery/voltage, initialpose, camera/image_raw, navigate_to_pose 등)이
자동으로 `/picky1/...` 로 prefix 된다.

PICKY2 측 launch 는 pinky_amr_2 패키지가 별도로 관리한다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    server_base_url = LaunchConfiguration('server_base_url')

    reverse_docking_params = PathJoinSubstitution(
        [FindPackageShare('pinky_amr_1'), 'params', 'reverse_docking.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'server_base_url',
            default_value='http://192.168.4.1:8000',
            description='Control Server base URL for PATCH /api/fleet/robots/{id}',
        ),
        Node(
            package='pinky_amr_1',
            executable='state_manager',
            namespace='picky1',
            output='screen',
            parameters=[
                reverse_docking_params,
                {
                    'robot_id': 'PICKY1',
                    'server_base_url': server_base_url,
                },
            ],
        ),
    ])
