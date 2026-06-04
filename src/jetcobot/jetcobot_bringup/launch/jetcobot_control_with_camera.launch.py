from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration("port")
    baudrate = LaunchConfiguration("baudrate")
    default_speed = LaunchConfiguration("default_speed")

    camera_device = LaunchConfiguration("camera_device")
    camera_dest_ip = LaunchConfiguration("camera_dest_ip")
    camera_dest_port = LaunchConfiguration("camera_dest_port")
    jpeg_quality = LaunchConfiguration("jpeg_quality")
    fps = LaunchConfiguration("fps")
    width = LaunchConfiguration("width")
    height = LaunchConfiguration("height")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "port",
                default_value="/dev/ttyJETCOBOT",
            ),
            DeclareLaunchArgument(
                "baudrate",
                default_value="1000000",
            ),
            DeclareLaunchArgument(
                "default_speed",
                default_value="20",
            ),
            DeclareLaunchArgument(
                "camera_device",
                default_value="/dev/jetcocam0",
            ),
            DeclareLaunchArgument(
                "camera_dest_ip",
                default_value="192.168.1.21",
            ),
            DeclareLaunchArgument(
                "camera_dest_port",
                default_value="5003",
            ),
            DeclareLaunchArgument(
                "jpeg_quality",
                default_value="80",
            ),
            DeclareLaunchArgument(
                "fps",
                default_value="30.0",
            ),
            DeclareLaunchArgument(
                "width",
                default_value="0",
            ),
            DeclareLaunchArgument(
                "height",
                default_value="0",
            ),
            Node(
                package="jetcobot_bringup",
                executable="jetcobot_joint_subscriber",
                name="jetcobot_joint_subscriber",
                output="screen",
                parameters=[
                    {"port": port},
                    {"baudrate": baudrate},
                    {"default_speed": default_speed},
                ],
            ),
            Node(
                package="jetcobot_bringup",
                executable="jetcobot_camera_udp_sender",
                name="jetcobot_camera_udp_sender",
                output="screen",
                parameters=[
                    {"camera_device": camera_device},
                    {"dest_ip": camera_dest_ip},
                    {"dest_port": camera_dest_port},
                    {"jpeg_quality": jpeg_quality},
                    {"fps": fps},
                    {"width": width},
                    {"height": height},
                ],
            ),
        ]
    )