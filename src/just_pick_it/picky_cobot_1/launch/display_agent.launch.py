#!/usr/bin/env python3
"""
Local AI 컴퓨터(192.168.1.70)용 DISPLAY launch — detection + 빈자리 detector + 배치 agent.

픽(ibvs_nn_pick_agent.launch.py)과 동일한 구조다.
  1. yolo_seg_infer (detection, 상시)
     - cobot 호스트가 UDP 로 보낸 카메라 프레임을 YOLO-seg 추론 -> /infer/tracked_objects.
     - 빈자리 detector / place IBVS 가 이 토픽(또는 image_raw)을 쓰므로 먼저 떠 있어야 한다.
  2. empty_slot_detector (DISPLAY_PLACE 내부 스캔, 상시·수동 트리거)
     - 로봇을 구동하지 않는 수동 노드. DISPLAY_PLACE 가 unit 마다 호출하는
       cobot_controller.run_scanning 이 발행하는 /place/reset -> /place/capture_view ->
       /place/plan 트리거로만 동작해 /place/scan_result 를 발행한다. 켜둬도 idle 비용이
       거의 없어 yolo 처럼 상시 가동한다.
  3. display_place_agent (DISPLAY_PLACE, on-demand)
     - cobot 호스트가 보내는 배치 요청(/display_place/request)을 받아 이 머신에서
       place_servo.launch.py(csrt + IBVS + place release)를 on-demand 실행하고, place release
       의 gripper open 관측을 완료로 보고한 뒤 종료한다(픽 agent 와 동일한 수명 관리).

포트/도메인 주의(픽과 동일):
  - 카메라 송신 camera_dest_port:=5003 과 udp_port 일치, 두 머신 ROS_DOMAIN_ID 동일.

with_detection:
  - true(기본): 이 launch 가 yolo 도 띄운다(DISPLAY 단독 운영).
  - false: yolo 를 띄우지 않는다. 픽 launch(ibvs_nn_pick_agent.launch.py)가 이미 yolo 를
    띄운 채로 함께 돌릴 때 사용(같은 UDP 포트를 두 번 bind 하는 충돌 방지).
"""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    udp_port = LaunchConfiguration('udp_port')
    detection_model_path = LaunchConfiguration('detection_model_path')
    target_classes = LaunchConfiguration('target_classes')
    detection_confidence = LaunchConfiguration('detection_confidence')
    with_detection = LaunchConfiguration('with_detection')
    detector_params_file = LaunchConfiguration('detector_params_file')
    place_timeout_sec = LaunchConfiguration('place_timeout_sec')
    place_request_topic = LaunchConfiguration('place_request_topic')
    place_result_topic = LaunchConfiguration('place_result_topic')

    home = os.path.expanduser('~')
    default_model_path = os.path.join(
        home,
        'just_pick_it/src/just_pick_it/just_pick_it_perception/result/jetcobot_1/best.pt',
    )
    perception_share = get_package_share_directory('just_pick_it_perception')
    default_detector_params = os.path.join(
        perception_share, 'config', 'empty_slot_detector_params.yaml')

    args = [
        DeclareLaunchArgument('robot_name', default_value='jetcobot1'),
        DeclareLaunchArgument('udp_port', default_value='5003'),
        DeclareLaunchArgument('detection_model_path', default_value=default_model_path),
        DeclareLaunchArgument(
            'target_classes',
            default_value='bread,choco_pie,cream_bread,fanta,water,watermelon'),
        DeclareLaunchArgument('detection_confidence', default_value='0.5'),
        DeclareLaunchArgument('with_detection', default_value='true'),
        DeclareLaunchArgument('detector_params_file', default_value=default_detector_params),
        DeclareLaunchArgument('place_timeout_sec', default_value='120.0'),
        DeclareLaunchArgument('place_request_topic', default_value='/display_place/request'),
        DeclareLaunchArgument('place_result_topic', default_value='/display_place/result'),
    ]

    detection_launch = os.path.join(
        perception_share, 'launch', 'yolo_seg_infer.launch.xml')
    detector_launch = os.path.join(
        perception_share, 'launch', 'empty_slot_detector.launch.py')

    # 1. detection(YOLO-seg) 상시 노드 — with_detection=true 일 때만.
    detection = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(detection_launch),
        launch_arguments={
            'udp_port': udp_port,
            'model_path': detection_model_path,
            'target_classes': target_classes,
            'confidence': detection_confidence,
        }.items(),
        condition=IfCondition(with_detection),
    )

    # 2. 빈자리 detector 상시 노드(DISPLAY_PLACE 내부 스캔이 수동 트리거).
    detector = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(detector_launch),
        launch_arguments={'params_file': detector_params_file}.items(),
    )

    # 3. 배치 agent(DISPLAY_PLACE, on-demand 로 place_servo.launch.py 실행).
    place_agent = Node(
        package='picky_cobot_1',
        executable='display_place_agent',
        name='display_place_agent',
        output='screen',
        parameters=[
            {'robot_name': robot_name},
            {'place_timeout_sec': ParameterValue(place_timeout_sec, value_type=float)},
            {'request_topic': place_request_topic},
            {'result_topic': place_result_topic},
        ],
    )

    return LaunchDescription(args + [detection, detector, place_agent])
