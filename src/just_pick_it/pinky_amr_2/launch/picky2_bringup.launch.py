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
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap, PushROSNamespace
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

    robot_xacro = PathJoinSubstitution(
        [FindPackageShare("pinky_description"), "urdf", "robot.urdf.xacro"]
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
            # 노드 이름을 /<namespace>/<node> 로 네임스페이스화(2대 동시 시연 시
            # /pinky_bringup, /sllidar_node, /battery_publisher 등 글로벌 노드 이름 충돌
            # 방지). 아래 RSP/JSP 는 PushROSNamespace 가 일괄 적용하도록 명시 namespace 를
            # 제거했다(중복 /picky2/picky2 방지). 프레임은 frame_prefix="" 라 무접두어 유지,
            # /tf 분리는 SetRemap(/tf -> /picky2/tf)이 담당(PushROSNamespace 는 절대 /tf 무관).
            PushROSNamespace(namespace),
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
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{
                    "ignore_timestamp": False,
                    "use_sim_time": use_sim_time,
                    "robot_description": Command([
                        "xacro ",
                        robot_xacro,
                        " namespace:=''",
                        " is_sim:=",
                        use_sim_time,
                        " cam_tilt_deg:=0",
                    ]),
                    "frame_prefix": "",
                }],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                parameters=[{
                    "source_list": ["joint_states"],
                    "rate": 20.0,
                    "use_sim_time": use_sim_time,
                }],
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
