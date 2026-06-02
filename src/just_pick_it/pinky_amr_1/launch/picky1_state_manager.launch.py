"""PICKY1용 State Manager launch.

state_manager 진입점은 한 프로세스 안에 state_manager / reverse_docking /
move_to_goal 세 노드를 동시에 띄운다. 모두 `/picky1` namespace 에 들어가
상대경로 토픽(cmd_vel, picky_state, move_command, dock_command,
battery/voltage, initialpose, camera/image_raw, navigate_to_pose 등)이
자동으로 `/picky1/...` 로 prefix 된다.

PICKY2 측 launch 는 pinky_amr_2 패키지가 별도로 관리한다.
"""

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    reverse_docking_params = PathJoinSubstitution(
        [FindPackageShare('pinky_amr_1'), 'params', 'reverse_docking.yaml']
    )

    return LaunchDescription([
        Node(
            package='pinky_amr_1',
            executable='state_manager',
            namespace='picky1',
            output='screen',
            parameters=[
                reverse_docking_params,
                {
                    'robot_id': 'PICKY1',
                },
            ],
            # move_to_goal / reverse_docking 의 TransformListener 는 namespace 와
            # 무관하게 절대경로 /tf, /tf_static 을 구독한다(tf2_ros 기본 동작).
            # 실제 TF 는 namespace 별 tf 토픽으로 나가므로, 절대경로 /tf 를 상대경로
            # tf 로 remap 해 node namespace 기준으로 해석되게 한다(picky1 이면
            # /picky1/tf, picky2 면 /picky2/tf). robot 별 하드코딩 없이 같은 코드를
            # 쓰기 위함. remap 없으면 위치를 못 읽어 move_to_goal 이 도착 판정을
            # 못 하고 nav_timeout 까지 멈춘다.
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        ),
    ])
