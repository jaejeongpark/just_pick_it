"""PICKY2 SLAM(맵 빌딩) launch.

slam_toolbox online_sync 를 PICKY2 namespace 토픽 기준으로 띄운다. mapper params 는
pinky_amr_2 패키지의 전용 파일을 사용해서 PICKY1 튜닝과 분리한다.

사용 예:
  ros2 launch pinky_amr_2 picky2_slam.launch.py

저장:
  ros2 run nav2_map_server map_saver_cli -f <이름> -t /<namespace>/map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushROSNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    amr2_dir = get_package_share_directory('pinky_amr_2')

    namespace = LaunchConfiguration('namespace')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    slam_launch = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_sync_launch.py']
    )

    slam = GroupAction([
        PushROSNamespace(namespace),
        SetRemap(src='/scan', dst='scan'),
        SetRemap(src='/tf', dst='tf'),
        SetRemap(src='/tf_static', dst='tf_static'),
        SetRemap(src='/map', dst='map'),
        SetRemap(src='/map_metadata', dst='map_metadata'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'slam_params_file': slam_params_file,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='picky2',
            description='로봇 namespace.',
        ),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(amr2_dir, 'params', 'mapper_params.yaml'),
            description='PICKY2 전용 slam_toolbox 파라미터 파일.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        slam,
    ])
