#!/usr/bin/env python3
"""
Local AI 컴퓨터(192.168.1.70)용 launch — detection 파이프라인 + ibvs_nn_pick_agent.

구성(둘 다 상시 실행):
  1. yolo_seg_infer (detection)
     - cobot 호스트가 UDP 로 보낸 카메라 프레임을 수신해 YOLO-seg 추론.
     - /infer/tracked_objects (TrackedObjectArray)를 상시 발행한다.
     - IBVS/NN 은 이 토픽을 구독하므로 픽보다 먼저 떠 있어야 한다.
  2. ibvs_nn_pick_agent
     - cobot 호스트의 cobot_state_manager 가 보내는 픽 요청(/ibvs_nn_pick/request)을 받아
       이 머신에서 nn_inference.launch.py(사용자 ibvs+nn)를 on-demand 로 실행한다.

포트 정합 주의:
  - cobot 호스트 카메라 송신: camera_dest_port:=5003 (primary), camera_dest_port_2:=5004
  - 따라서 detection 수신 포트(udp_port)는 카메라 송신 포트와 일치해야 한다.
    기본값을 5003 으로 둔다. 운영 구성이 다르면 udp_port:= 로 덮어쓴다.

전제:
  - 두 머신의 ROS_DOMAIN_ID 가 동일해야 한다.
  - just_pick_it_perception / picky_cobot_1 패키지가 이 머신에 빌드/설치되어 있어야 한다.
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    udp_port = LaunchConfiguration('udp_port')
    detection_model_path = LaunchConfiguration('detection_model_path')
    target_classes = LaunchConfiguration('target_classes')
    detection_confidence = LaunchConfiguration('detection_confidence')
    pick_timeout_sec = LaunchConfiguration('pick_timeout_sec')
    pick_request_topic = LaunchConfiguration('pick_request_topic')
    pick_result_topic = LaunchConfiguration('pick_result_topic')
    desired_area_norm = LaunchConfiguration('desired_area_norm')

    home = os.path.expanduser('~')
    default_model_path = os.path.join(
        home,
        'just_pick_it/src/just_pick_it/just_pick_it_perception/result/jetcobot_1/best.pt',
    )

    args = [
        # nn_inference.launch.py 의 robot_name 및 cobot 호스트 드라이버와 일치해야 한다.
        DeclareLaunchArgument('robot_name', default_value='jetcobot1'),
        # 카메라 UDP 송신 포트(camera_dest_port)와 반드시 일치.
        DeclareLaunchArgument('udp_port', default_value='5003'),
        # 평소 yolo_seg_infer.launch.xml 실행 시 넘기던 모델 경로와 동일.
        DeclareLaunchArgument('detection_model_path', default_value=default_model_path),
        DeclareLaunchArgument(
            'target_classes',
            default_value='bread,choco_pie,cream_bread,fanta,water,watermelon',
        ),
        DeclareLaunchArgument('detection_confidence', default_value='0.5'),
        DeclareLaunchArgument('pick_timeout_sec', default_value='120.0'),
        DeclareLaunchArgument('pick_request_topic', default_value='/ibvs_nn_pick/request'),
        DeclareLaunchArgument('pick_result_topic', default_value='/ibvs_nn_pick/result'),
        # IBVS DONE 판정 area_norm 임계값. 클수록 물체에 더 가까이 접근한 뒤 grip.
        # 음수(기본값)면 nn_inference.launch.py 기본값(0.23)을 사용한다.
        DeclareLaunchArgument('desired_area_norm', default_value='-1.0'),
    ]

    perception_share = get_package_share_directory('just_pick_it_perception')
    detection_launch = os.path.join(
        perception_share, 'launch', 'yolo_seg_infer.launch.xml'
    )

    # detection(YOLO-seg) 상시 노드. /infer/tracked_objects 발행.
    detection = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(detection_launch),
        launch_arguments={
            'udp_port': udp_port,
            'model_path': detection_model_path,
            'target_classes': target_classes,
            'confidence': detection_confidence,
        }.items(),
    )

    # 픽 트리거 agent. 픽 요청 시 nn_inference.launch.py 를 로컬에서 on-demand 실행.
    agent = Node(
        package='picky_cobot_1',
        executable='ibvs_nn_pick_agent',
        name='ibvs_nn_pick_agent',
        output='screen',
        parameters=[
            {'robot_name': robot_name},
            {'pick_timeout_sec': ParameterValue(pick_timeout_sec, value_type=float)},
            {'request_topic': pick_request_topic},
            {'result_topic': pick_result_topic},
            {'desired_area_norm': ParameterValue(desired_area_norm, value_type=float)},
        ],
    )

    return LaunchDescription(args + [detection, agent])
