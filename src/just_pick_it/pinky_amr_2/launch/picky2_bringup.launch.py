"""PICKY2 Pinky bringup launch wrapper.

원본 `pinky_bringup` launch 파일은 수정하지 않는다. 대신 원본 bringup이
사용하는 전역 토픽을 `namespace` 인자로 받은 `/<namespace>/...` 토픽으로 remap해서
PICKY1/PICKY2가 같은 ROS_DOMAIN_ID에서 실행될 때 `/odom`, `/cmd_vel`, `/scan`
같은 전역 토픽이 충돌하지 않도록 한다.

사용 예:
  ros2 launch pinky_amr_2 picky2_bringup.launch.py
  ros2 launch pinky_amr_2 picky2_bringup.launch.py dest_ip:=<관제PC IP> enable_camera:=true
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """원본 `pinky_bringup`을 PICKY2 namespace/remap 기준으로 실행한다."""
    namespace = LaunchConfiguration('namespace')
    dest_ip = LaunchConfiguration('dest_ip')
    enable_camera = LaunchConfiguration('enable_camera')
    use_sim_time = LaunchConfiguration('use_sim_time')
    wheel_radius = LaunchConfiguration('wheel_radius')
    wheel_separation = LaunchConfiguration('wheel_separation')
    odom_stamp_offset_sec = LaunchConfiguration('odom_stamp_offset_sec')

    robot_description_launch = PathJoinSubstitution(
        [FindPackageShare("pinky_description"), "launch", "upload_robot.launch.py"]
    )
    lidar_launch = PathJoinSubstitution(
        [FindPackageShare("sllidar_ros2"), "launch", "sllidar_c1_launch.py"]
    )
    pinky_bringup_params = PathJoinSubstitution(
        [FindPackageShare("pinky_amr_2"), "params", "pinky_bringup.yaml"]
    )

    def ns(topic):
        return ["/", namespace, "/", topic]

    picky2_bringup = GroupAction(
        [
            SetRemap(src="/cmd_vel", dst=ns("cmd_vel")),
            SetRemap(src="/odom", dst=ns("odom")),
            SetRemap(src="/scan", dst=ns("scan")),
            SetRemap(src="/joint_states", dst=ns("joint_states")),
            SetRemap(src="/battery/percent", dst=ns("battery/percent")),
            SetRemap(src="/battery/voltage", dst=ns("battery/voltage")),
            SetRemap(src="/tf", dst=ns("tf")),
            SetRemap(src="/tf_static", dst=ns("tf_static")),
            SetRemap(src="/robot_description", dst=ns("robot_description")),
            SetRemap(src="/camera/image_raw", dst=ns("camera/image_raw")),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(robot_description_launch),
                launch_arguments={
                    "namespace": "",
                    "is_sim": use_sim_time,
                }.items(),
            ),
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(lidar_launch),
                launch_arguments={
                    "serial_port": "/dev/ttyS0",
                    "frame_id": "rplidar_link",
                    "inverted": "false",
                    "angle_compensate": "true",
                    "scan_mode": "DenseBoost",
                }.items(),
            ),
            Node(
                package="pinky_bringup",
                executable="bringup",
                parameters=[
                    pinky_bringup_params,
                    {"wheel_radius": ParameterValue(wheel_radius, value_type=float)},
                    {"wheel_separation": ParameterValue(wheel_separation, value_type=float)},
                    {
                        "odom_stamp_offset_sec": ParameterValue(
                            odom_stamp_offset_sec,
                            value_type=float,
                        )
                    },
                ],
            ),
            Node(
                package="pinky_bringup",
                executable="battery_publisher",
            ),
            Node(
                package="just_pick_it_perception",
                executable="udp_image_sender",
                name="pi_camera_udp_publisher",
                output="screen",
                condition=IfCondition(enable_camera),
                parameters=[
                    {"dest_port": 5002},
                    {"dest_ip": dest_ip},
                    {"width": 1280},
                    {"height": 720},
                    {"fps": 30},
                    {"jpeg_quality": 80},
                ],
            ),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='picky2',
            description='로봇 namespace.',
        ),
        DeclareLaunchArgument(
            'dest_ip',
            default_value='192.168.1.73',
            description='카메라 UDP 전송 대상 IP.',
        ),
        DeclareLaunchArgument(
            'enable_camera',
            default_value='false',
            description='카메라 UDP 스트리머 실행 여부. reverse docking 때만 true 권장.',
        ),
        DeclareLaunchArgument('use_sim_time', default_value='False'),
        DeclareLaunchArgument('wheel_radius', default_value='0.027'),
        DeclareLaunchArgument('wheel_separation', default_value='0.0961'),
        DeclareLaunchArgument('odom_stamp_offset_sec', default_value='0.0'),
        picky2_bringup,
    ])
