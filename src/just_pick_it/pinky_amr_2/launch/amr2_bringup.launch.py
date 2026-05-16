from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 이 launch 파일의 목적:
    # - pinky_pro의 pinky_bringup 원본 launch는 수정하지 않는다.
    # - 대신 AMR2에서 사용할 토픽 이름만 /amr2/... 형태로 분리한다.
    # - 이렇게 하면 AMR1, AMR2가 같은 ROS_DOMAIN_ID에서 실행될 때
    #   /odom, /cmd_vel, /scan 같은 전역 토픽 충돌을 줄일 수 있다.

    # FindPackageShare("pinky_bringup")은 빌드/설치된 pinky_bringup 패키지의
    # share 디렉토리를 찾는다.
    # PathJoinSubstitution은 그 share 경로 아래의 launch 파일 경로를 조립한다.
    # 최종적으로 가리키는 파일은:
    #   pinky_bringup/launch/bringup_robot.launch.xml
    pinky_bringup_launch = PathJoinSubstitution(
        [FindPackageShare("pinky_bringup"), "launch", "bringup_robot.launch.xml"]
    )

    # GroupAction은 여러 launch action을 하나의 묶음으로 실행한다.
    # 여기서는 remap 설정들과 기존 pinky_bringup launch include를 한 그룹으로 묶는다.
    amr2_bringup = GroupAction(
        [
            # 아래 SetRemap들은 pinky_bringup 원본이 사용하는 전역 토픽을
            # AMR2 전용 토픽으로 바꿔준다.
            # 예를 들어 원본 노드가 /odom으로 publish하려 하면 /amr2/odom으로 바뀐다.
            SetRemap(src="/cmd_vel", dst="/amr2/cmd_vel"),
            SetRemap(src="/odom", dst="/amr2/odom"),
            SetRemap(src="/scan", dst="/amr2/scan"),
            SetRemap(src="/joint_states", dst="/amr2/joint_states"),
            SetRemap(src="/battery/percent", dst="/amr2/battery/percent"),
            SetRemap(src="/battery/voltage", dst="/amr2/battery/voltage"),
            # 기존 pinky_bringup launch를 이 launch 안에 포함해서 실행한다.
            # AnyLaunchDescriptionSource는 include 대상이 XML launch 파일이어도 읽을 수 있게 해준다.
            IncludeLaunchDescription(AnyLaunchDescriptionSource(pinky_bringup_launch)),
        ]
    )

    # ros2 launch는 generate_launch_description()이 반환하는 LaunchDescription을 실행한다.
    # 여기서는 AMR2 remap이 적용된 bringup 그룹 하나를 실행한다.
    return LaunchDescription([amr2_bringup])
