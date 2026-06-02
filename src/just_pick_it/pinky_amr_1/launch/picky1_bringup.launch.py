"""PICKY Pinky bringup launch wrapper (namespace 파라미터화).

원본 `pinky_bringup` launch 파일은 수정하지 않는다. 대신 원본 bringup이
사용하는 전역 토픽을 `namespace` 인자로 받은 `/<namespace>/...` 토픽으로 remap해서
PICKY1/PICKY2가 같은 ROS_DOMAIN_ID에서 실행될 때 `/odom`, `/cmd_vel`, `/scan`
같은 전역 토픽이 충돌하지 않도록 한다. robot 별 하드코딩 없이 같은 코드를 쓴다.

  ros2 launch pinky_amr_1 picky1_bringup.launch.py                                  # picky1
  ros2 launch pinky_amr_1 picky1_bringup.launch.py namespace:=picky2 dest_ip:=<관제PC IP>
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap, Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """원본 `pinky_bringup`을 namespace/remap 기준으로 실행한다."""
    namespace = LaunchConfiguration('namespace')
    dest_ip = LaunchConfiguration('dest_ip')

    pinky_bringup_launch = PathJoinSubstitution(
        [FindPackageShare("pinky_bringup"), "launch", "bringup_robot.launch.xml"]
    )

    def ns(topic):
        # 전역 토픽을 /<namespace>/<topic> 으로 remap (robot 별 하드코딩 제거)
        return ["/", namespace, "/", topic]

    bringup = GroupAction(
        [
            SetRemap(src="/cmd_vel", dst=ns("cmd_vel")),
            SetRemap(src="/odom", dst=ns("odom")),
            SetRemap(src="/scan", dst=ns("scan")),
            SetRemap(src="/joint_states", dst=ns("joint_states")),
            SetRemap(src="/battery/percent", dst=ns("battery/percent")),
            SetRemap(src="/battery/voltage", dst=ns("battery/voltage")),
            SetRemap(src="/tf", dst=ns("tf")),
            SetRemap(src="/tf_static", dst=ns("tf_static")),
            SetRemap(src="/camera/image_raw", dst=ns("camera/image_raw")),
            IncludeLaunchDescription(AnyLaunchDescriptionSource(pinky_bringup_launch)),
            Node(
                package="just_pick_it_perception",
                executable="udp_image_sender",
                name="pi_camera_udp_publisher",
                output="screen",
                parameters=[
                    {"dest_port": 5001},
                    {"dest_ip": dest_ip},
                    {"width": 1280},
                    {"height": 720},
                    {"fps": 30},
                    {"jpeg_quality": 80},
                ]
            ),
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='picky1',
            description='로봇 namespace (picky1 / picky2).'),
        DeclareLaunchArgument(
            'dest_ip', default_value='192.168.1.73',
            description='카메라 UDP 전송 대상 IP (보통 관제 PC). robot/네트워크별로 바꿀 수 있다.'),
        bringup,
    ])
