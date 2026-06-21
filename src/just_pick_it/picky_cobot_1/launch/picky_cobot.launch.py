#!/usr/bin/env python3
"""
Cobot 호스트(192.168.1.99)용 통합 launch — 이 하나로 cobot 호스트의 모든 노드를 올린다.

이 launch 하나면 다음이 모두 켜진다(별도로 카메라/드라이버 launch 를 띄울 필요 없음).
  1. jetcobot_joint_subscriber : /dev/ttyJETCOBOT serial 을 단독 점유하는 드라이버.
     IBVS/NN 및 state machine 이 발행하는 제어 토픽을 받아 로봇을 구동한다.
  2. jetcobot_camera_udp_sender : 카메라 프레임을 local AI 컴퓨터(192.168.1.70)로 UDP 송출.
     (위 두 노드는 jetcobot_control_with_camera.launch.py 를 include 해서 띄운다.)
  3. cobot_state_manager        : ExecuteTask 액션 서버. SORTING/LOADING/INSPECTION/
     UNLOAD/DISPLAY_PLACE 등 모든 cobot task 를 처리한다. serial 은 위
     드라이버가 점유하므로 dry_run=true 로 직접 제어를 생략하고, SORTING 픽은 IBVS+NN
     에 토픽으로 요청한다.

짝이 되는 local 컴퓨터(192.168.1.70) launch:
  ros2 launch picky_cobot_1 ibvs_nn_pick_agent.launch.py
  (detection + 픽 agent. cobot_state_manager 의 픽 요청을 받아 nn_inference 를 실행)

전제:
  - 두 머신의 ROS_DOMAIN_ID 가 동일하고 네트워크가 연결되어 있어야 한다.

흐름:
  fleet_manager -> ExecuteTask -> cobot_state_manager(cobot 호스트)
    -> SORTING: /ibvs_nn_pick/request 로 product_name 픽 요청
       -> ibvs_nn_pick_agent(local)가 nn_inference.launch.py 실행, 제어 토픽 발행
       -> jetcobot_joint_subscriber(cobot 호스트)가 토픽 받아 로봇 구동
       -> nn 이 grip 닫기 발행 -> agent 관측 -> /ibvs_nn_pick/result 로 success
    -> LOADING: place shell 로 내려놓기
    -> success=True 결과 반환
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    cobot_robot_id = LaunchConfiguration('cobot_robot_id')
    port = LaunchConfiguration('port')
    baudrate = LaunchConfiguration('baudrate')
    default_speed = LaunchConfiguration('default_speed')
    camera_dest_ip = LaunchConfiguration('camera_dest_ip')
    camera_dest_port = LaunchConfiguration('camera_dest_port')
    camera_dest_port_2 = LaunchConfiguration('camera_dest_port_2')
    dry_run = LaunchConfiguration('dry_run')
    detection_topic = LaunchConfiguration('detection_topic')
    pick_timeout_sec = LaunchConfiguration('pick_timeout_sec')
    pick_request_topic = LaunchConfiguration('pick_request_topic')
    pick_result_topic = LaunchConfiguration('pick_result_topic')

    args = [
        # jetcobot 드라이버 namespace. nn_inference.launch.py / agent 와 일치해야 한다.
        DeclareLaunchArgument('robot_name', default_value='jetcobot1'),
        # DB cobot_state ENUM / ExecuteTask 액션 이름에 쓰는 cobot 식별자.
        DeclareLaunchArgument('cobot_robot_id', default_value='COBOT1'),
        DeclareLaunchArgument('port', default_value='/dev/ttyJETCOBOT'),
        DeclareLaunchArgument('baudrate', default_value='1000000'),
        DeclareLaunchArgument('default_speed', default_value='20'),
        # 카메라 UDP 수신 측(local AI 컴퓨터) IP/포트. 운영 환경 다르면 camera_dest_ip:= 로 덮어쓴다.
        DeclareLaunchArgument('camera_dest_ip', default_value='192.168.0.5'),
        DeclareLaunchArgument('camera_dest_port', default_value='5003'),
        DeclareLaunchArgument('camera_dest_port_2', default_value='5004'),
        # true 면 토픽 발행 없이 시뮬레이션. 실제 로봇 구동은 false.
        DeclareLaunchArgument('dry_run', default_value='false'),
        # INSPECTION 검출 비교용 토픽(local AI 컴퓨터 yolo_seg_infer).
        DeclareLaunchArgument('detection_topic', default_value='/infer/tracked_objects'),
        DeclareLaunchArgument('pick_timeout_sec', default_value='120.0'),
        DeclareLaunchArgument('pick_request_topic', default_value='/ibvs_nn_pick/request'),
        DeclareLaunchArgument('pick_result_topic', default_value='/ibvs_nn_pick/result'),
    ]

    jetcobot_share = get_package_share_directory('jetcobot_bringup')
    driver_camera_launch = os.path.join(
        jetcobot_share, 'launch', 'jetcobot_control_with_camera.launch.py'
    )

    # serial 드라이버 + 카메라 UDP 송출(기존 launch 재사용).
    driver_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(driver_camera_launch),
        launch_arguments={
            'robot_name': robot_name,
            'port': port,
            'baudrate': baudrate,
            'default_speed': default_speed,
            'camera_dest_ip': camera_dest_ip,
            'camera_dest_port': camera_dest_port,
            'camera_dest_port_2': camera_dest_port_2,
        }.items(),
    )

    # ExecuteTask 액션 서버. 로봇 구동은 드라이버 토픽 경유(serial 은 드라이버가 점유).
    cobot_state_manager = Node(
        package='picky_cobot_1',
        executable='cobot_state_manager',
        name='cobot_state_manager',
        output='screen',
        parameters=[
            {'robot_id': cobot_robot_id},
            {'robot_name': robot_name},
            {'dry_run': ParameterValue(dry_run, value_type=bool)},
            {'detection_topic': detection_topic},
            {'pick_timeout_sec': ParameterValue(pick_timeout_sec, value_type=float)},
            {'pick_request_topic': pick_request_topic},
            {'pick_result_topic': pick_result_topic},
        ],
    )

    return LaunchDescription(args + [driver_camera, cobot_state_manager])
