"""PICKY1용 Pinky bringup launch wrapper.

원본 `pinky_bringup` launch 파일은 수정하지 않는다. 대신 원본 bringup이
사용하는 전역 토픽을 `/picky1/...` 토픽으로 remap해서 PICKY1/PICKY2가 같은
ROS_DOMAIN_ID에서 실행될 때 `/odom`, `/cmd_vel`, `/scan` 같은 전역 토픽이
충돌하지 않도록 한다.
"""

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import SetRemap, Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """원본 `pinky_bringup`을 PICKY1 namespace/remap 기준으로 실행한다."""
    pinky_bringup_launch = PathJoinSubstitution(
        [FindPackageShare("pinky_bringup"), "launch", "bringup_robot.launch.xml"]
    )

    picky1_bringup = GroupAction(
        [
            SetRemap(src="/cmd_vel", dst="/picky1/cmd_vel"),
            SetRemap(src="/odom", dst="/picky1/odom"),
            SetRemap(src="/scan", dst="/picky1/scan"),
            SetRemap(src="/joint_states", dst="/picky1/joint_states"),
            SetRemap(src="/battery/percent", dst="/picky1/battery/percent"),
            SetRemap(src="/battery/voltage", dst="/picky1/battery/voltage"),
            SetRemap(src="/tf", dst="/picky1/tf"),
            SetRemap(src="/tf_static", dst="/picky1/tf_static"),
            SetRemap(src="/camera/image_raw", dst="/picky1/camera/image_raw"),
            IncludeLaunchDescription(AnyLaunchDescriptionSource(pinky_bringup_launch)),
            Node(
                package="just_pick_it_perception",
                executable="udp_image_sender",
                name="pi_camera_udp_publisher",
                output="screen",
                parameters=[
                    {"dest_port": 5001},
                    {"dest_ip": "192.168.1.73"},
                    {"width": 1280},
                    {"height": 720},
                    {"fps": 30},
                    {"jpeg_quality": 80},
                ]
            ),
        ]
    )

    return LaunchDescription([picky1_bringup])
