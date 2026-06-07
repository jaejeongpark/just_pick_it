"""PICKY2 Nav2 (localization + navigation) launch.

pinky_navigation 의 XML bringup_launch.xml 은 namespace 를 push 하면서도
nav2_params.yaml 을 RewrittenYaml 없이 raw 로 로드한다. 그래서 노드 풀네임이
/picky2/controller_server 가 되면 params 파일 키(controller_server)와 매칭되지
않아 모든 nav2 노드가 기본값으로 떠 controller 가 기본 DWB(critics 없음)로
죽었다.

이 launch 는 nav2_bringup 표준 구현(localization_launch.py / navigation_launch.py)
을 그대로 따라 RewrittenYaml(root_key=namespace) 로 params 전체를 namespace
아래로 감싸 namespaced 노드에 정상 매칭시킨다.

단, Jazzy 표준 navigation_launch.py 는 route_server / collision_monitor /
docking_server 까지 띄우는데 pinky 의 nav2_params.yaml 에는 이들 설정이 없어
collision_monitor 가 configure 단계에서 죽고 nav 전체 bringup 이 abort 된다.
그래서 여기서는 pinky 가 설정한 노드만 띄운다(이 세 노드 제외). collision_monitor
를 빼면 cmd_vel_smoothed -> cmd_vel 최종 출력 단이 사라지므로 velocity_smoother
가 그 역할을 하도록 remap 한다.

사용 예:
  ros2 launch pinky_amr_2 picky2_nav.launch.py
  ros2 launch pinky_amr_2 picky2_nav.launch.py map:=/path/to/map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushROSNamespace, SetParameter
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    amr2_dir = get_package_share_directory('pinky_amr_2')
    pinky_nav_dir = get_package_share_directory('pinky_navigation')

    namespace = LaunchConfiguration('namespace')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    log_level = LaunchConfiguration('log_level')

    # 풀네임 토픽을 상대경로로 바꿔 namespace 가 prepend 되게 한다(tf 포함).
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # params 파일 전체를 root_key=namespace 아래로 감싸 /picky2/<node> 키로 만든다.
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={'autostart': autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )

    localization_nodes = ['map_server', 'amcl']
    navigation_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    declare_cmds = [
        DeclareLaunchArgument(
            'namespace', default_value='picky2',
            description='로봇 namespace (드라이버의 /picky2/scan, /picky2/tf 와 일치해야 함)',
        ),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(pinky_nav_dir, 'map', 'sync_map.yaml'),
            description='map yaml 전체 경로',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(amr2_dir, 'params', 'nav2_params.yaml'),
            description='PICKY2 전용 nav2 파라미터 파일 (RewrittenYaml 로 namespace 주입됨)',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('autostart', default_value='True'),
        DeclareLaunchArgument('log_level', default_value='info'),
        # 이 보드(ARM)는 composable 로드가 실패해 항상 standalone 으로 띄운다.
        # 호출부 호환을 위해 인자만 받고 사용하지 않는다.
        DeclareLaunchArgument('use_composition', default_value='False'),
    ]

    log_args = ['--ros-args', '--log-level', log_level]

    bringup = GroupAction([
        PushROSNamespace(namespace),
        SetParameter('use_sim_time', use_sim_time),

        # ===== localization =====
        Node(
            package='nav2_map_server', executable='map_server', name='map_server',
            output='screen',
            parameters=[configured_params, {'yaml_filename': map_yaml}],
            arguments=log_args, remappings=remappings,
        ),
        Node(
            package='nav2_amcl', executable='amcl', name='amcl', output='screen',
            parameters=[configured_params], arguments=log_args, remappings=remappings,
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen', arguments=log_args,
            parameters=[{'autostart': autostart}, {'node_names': localization_nodes}],
        ),

        # ===== navigation =====
        Node(
            package='nav2_controller', executable='controller_server', output='screen',
            parameters=[configured_params], arguments=log_args,
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_smoother', executable='smoother_server', name='smoother_server',
            output='screen', parameters=[configured_params], arguments=log_args,
            remappings=remappings,
        ),
        Node(
            package='nav2_planner', executable='planner_server', name='planner_server',
            output='screen', parameters=[configured_params], arguments=log_args,
            remappings=remappings,
        ),
        Node(
            package='nav2_behaviors', executable='behavior_server', name='behavior_server',
            output='screen', parameters=[configured_params], arguments=log_args,
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav')],
        ),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
            output='screen', parameters=[configured_params], arguments=log_args,
            remappings=remappings,
        ),
        Node(
            package='nav2_waypoint_follower', executable='waypoint_follower',
            name='waypoint_follower', output='screen',
            parameters=[configured_params], arguments=log_args, remappings=remappings,
        ),
        # collision_monitor 를 띄우지 않으므로 velocity_smoother 가 최종 cmd_vel 을
        # 발행한다. 입력 cmd_vel_nav(컨트롤러 출력), 출력 cmd_vel(드라이버 입력).
        Node(
            package='nav2_velocity_smoother', executable='velocity_smoother',
            name='velocity_smoother', output='screen',
            parameters=[configured_params], arguments=log_args,
            remappings=remappings + [('cmd_vel', 'cmd_vel_nav'),
                                     ('cmd_vel_smoothed', 'cmd_vel')],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_navigation', output='screen', arguments=log_args,
            parameters=[{'autostart': autostart}, {'node_names': navigation_nodes}],
        ),
    ])

    return LaunchDescription(declare_cmds + [bringup])
