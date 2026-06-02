"""PICKY State Manager launch (namespace 파라미터화).

state_manager 진입점은 한 프로세스 안에 state_manager / reverse_docking /
move_to_goal 세 노드를 동시에 띄운다. 모두 `namespace` 인자로 받은 namespace
(기본 picky1) 에 들어가 상대경로 토픽(cmd_vel, picky_state, move_command,
dock_command, battery/voltage, initialpose, camera/image_raw, navigate_to_pose
등)이 자동으로 `/<namespace>/...` 로 prefix 된다.

robot 별 하드코딩 없이 picky1/picky2 양쪽에 같은 코드를 쓴다.
  ros2 launch pinky_amr_1 picky1_state_manager.launch.py                       # picky1
  ros2 launch pinky_amr_1 picky1_state_manager.launch.py namespace:=picky2 robot_id:=PICKY2
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
        [FindPackageShare('pinky_amr_1'), 'params', 'reverse_docking.yaml']
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='picky1',
            description='로봇 namespace (picky1 / picky2). 드라이버의 /<ns>/scan, /<ns>/tf 와 일치해야 함.'),
        DeclareLaunchArgument(
            'robot_id', default_value='PICKY1',
            description='DB robot_name (PICKY1 / PICKY2). namespace 와 짝을 맞춘다.'),
        Node(
            package='pinky_amr_1',
            executable='state_manager',
            namespace=namespace,
            output='screen',
            parameters=[
                reverse_docking_params,
                {
                    'robot_id': robot_id,
                },
            ],
            # move_to_goal / reverse_docking 의 TransformListener 는 namespace 와
            # 무관하게 절대경로 /tf, /tf_static 을 구독한다(tf2_ros 기본 동작).
            # 실제 TF 는 namespace 별 tf 토픽으로 나가므로, 절대경로 /tf 를 상대경로
            # tf 로 remap 해 node namespace 기준으로 해석되게 한다(picky1 이면
            # /picky1/tf, picky2 면 /picky2/tf). remap 없으면 위치를 못 읽어
            # move_to_goal 이 도착 판정을 못 하고 nav_timeout 까지 멈춘다.
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static'),
            ],
        ),
    ])
