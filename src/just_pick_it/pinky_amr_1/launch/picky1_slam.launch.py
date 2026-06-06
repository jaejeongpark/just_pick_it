"""PICKY SLAM(맵 빌딩) launch (namespace 파라미터화).

slam_toolbox online_sync 를 PICKY 토픽 기준으로 띄운다. 원본
pinky_navigation/map_building.launch.xml 은 글로벌 /scan, /tf 를 쓰는데,
드라이버는 namespace 별 /<ns>/scan, /<ns>/tf 로 발행하므로 remap 해서 맞춘다.
(picky1_bringup.launch.py 가 pinky_bringup 을 감싸는 방식과 동일.) robot 별
하드코딩 없이 namespace 인자(기본 picky1)로 picky1/picky2 둘 다 쓴다.

slam_toolbox 는 root namespace 의 단일 노드(/slam_toolbox)로 떠서 mapper_params
(slam_toolbox: 키)가 정상 바인딩된다. map 프레임 원점은 SLAM 시작 시점의
base_footprint 자세에 잡히므로, 로봇을 좌하단 코너에 +x 방향 정렬해 놓고 시작하면
축 정렬 + 코너 원점 맵이 나온다.

  ros2 launch pinky_amr_1 picky1_slam.launch.py                    # picky1
  ros2 launch pinky_amr_1 picky1_slam.launch.py namespace:=picky2  # picky2

저장:
  ros2 run nav2_map_server map_saver_cli -f <이름> -t /<namespace>/map
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')

    slam_launch = PathJoinSubstitution(
        [FindPackageShare('slam_toolbox'), 'launch', 'online_sync_launch.py']
    )
    mapper_params = PathJoinSubstitution(
        [FindPackageShare('pinky_navigation'), 'params', 'mapper_params.yaml']
    )

    def ns(topic):
        # 전역 토픽을 /<namespace>/<topic> 으로 remap (robot 별 하드코딩 제거)
        return ["/", namespace, "/", topic]

    slam = GroupAction([
        SetRemap(src='/scan', dst=ns('scan')),
        SetRemap(src='/tf', dst=ns('tf')),
        SetRemap(src='/tf_static', dst=ns('tf_static')),
        SetRemap(src='/map', dst=ns('map')),
        SetRemap(src='/map_metadata', dst=ns('map_metadata')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(slam_launch),
            launch_arguments={
                'slam_params_file': mapper_params,
                'use_sim_time': 'false',
            }.items(),
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='picky1',
            description='로봇 namespace (picky1 / picky2).'),
        slam,
    ])
