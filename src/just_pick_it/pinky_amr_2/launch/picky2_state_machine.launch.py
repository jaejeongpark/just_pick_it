"""PICKY2 State Machine launch.

`picky2_state_machine` 진입점은 한 프로세스 안에 state_machine,
reverse_docking, move_to_goal 세 노드를 동시에 띄운다. 모두 `namespace`
인자로 받은 namespace(기본 picky2)에 들어가 상대경로 토픽과 action/service가
자동으로 `/picky2/...` 로 prefix 된다.

사용 예:
  ros2 launch pinky_amr_2 picky2_state_machine.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    robot_id = LaunchConfiguration('robot_id')

    reverse_docking_params = PathJoinSubstitution(
        [FindPackageShare('pinky_amr_2'), 'params', 'reverse_docking.yaml']
    )
    move_to_goal_params = PathJoinSubstitution(
        [FindPackageShare('pinky_amr_2'), 'params', 'move_to_goal.yaml']
    )
    state_machine_params = PathJoinSubstitution(
        [FindPackageShare('pinky_amr_2'), 'params', 'state_machine.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='picky2',
            description='로봇 namespace. 드라이버의 /picky2/scan, /picky2/tf 와 일치해야 함.',
        ),
        DeclareLaunchArgument(
            'robot_id',
            default_value='PICKY2',
            description='DB robot_name. namespace 와 짝을 맞춘다.',
        ),
        Node(
            package='pinky_amr_2',
            executable='picky2_state_machine',
            namespace=namespace,
            output='screen',
            parameters=[
                state_machine_params,
                reverse_docking_params,
                move_to_goal_params,
                {
                    'robot_id': robot_id,
                },
            ],
            # tf2_ros TransformListener 는 기본적으로 절대경로 /tf, /tf_static 을
            # 구독하므로 namespace 기준 상대경로로 remap 한다.
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        ),
    ])
