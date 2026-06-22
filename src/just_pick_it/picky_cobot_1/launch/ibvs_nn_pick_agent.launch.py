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

NN/IBVS 튜닝 기본값:
  - 아래 nn_*/ibvs_*/grip_*/model_dir/max_fine_tune_steps/j6_* 기본값은 수동 검증에서
    가장 잘 동작한 nn_inference.launch.py 인자 조합과 동일하게 맞춰 두었다.
  - agent 가 픽 요청을 받으면 이 값들을 extra_launch_args 로 nn_inference.launch.py 에
    그대로 전달하므로, 별도 인자 없이 실행해도 수동 검증과 같은 동작을 재현한다.
  - 재튜닝이 필요하면 해당 인자만 nn_command_speed:= 처럼 덮어쓴다.

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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

# nn_inference.launch.py 로 그대로 전달할 튜닝 인자.
# (launch arg 이름, nn_inference.launch.py arg 이름) — 둘은 동일하게 둔다.
# robot_name / target_class_label / desired_area_norm 은 agent 가 직접 넣으므로 제외.
_FORWARDED_ARGS = [
    "model_dir",
    "grip_confidence_threshold",
    "grip_consecutive_required",
    "max_fine_tune_steps",
    "nn_control_rate_hz",
    "nn_command_speed",
    "nn_delta_scale",
    "nn_delta_smooth_alpha",
    "nn_settle_delta_deg",
    "nn_command_leash_deg",
    "nn_z_floor_enable",
    "nn_z_floor_mm",
    "nn_z_floor_margin_mm",
    "ibvs_command_speed",
    "j6_angle_sign",
    "j6_angle_offset_deg",
]


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name")
    udp_port = LaunchConfiguration("udp_port")
    detection_model_path = LaunchConfiguration("detection_model_path")
    target_classes = LaunchConfiguration("target_classes")
    detection_confidence = LaunchConfiguration("detection_confidence")
    pick_timeout_sec = LaunchConfiguration("pick_timeout_sec")
    pick_request_topic = LaunchConfiguration("pick_request_topic")
    pick_result_topic = LaunchConfiguration("pick_result_topic")
    desired_area_norm = LaunchConfiguration("desired_area_norm")

    perception_share = get_package_share_directory("just_pick_it_perception")
    detection_launch = os.path.join(
        perception_share, "launch", "yolo_seg_infer.launch.xml"
    )

    # detection(YOLO-seg) 상시 노드. /infer/tracked_objects 발행.
    detection = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(detection_launch),
        launch_arguments={
            "udp_port": udp_port,
            "model_path": detection_model_path,
            "target_classes": target_classes,
            "confidence": detection_confidence,
        }.items(),
    )

    # 검증된 튜닝 인자를 "key:=value" 목록으로 만들어 agent 에 전달한다.
    # agent 는 픽 요청 시 이 목록을 nn_inference.launch.py 명령에 그대로 덧붙인다.
    extra_launch_args = [
        f"{name}:={LaunchConfiguration(name).perform(context)}"
        for name in _FORWARDED_ARGS
    ]

    # 픽 트리거 agent. 픽 요청 시 nn_inference.launch.py 를 로컬에서 on-demand 실행.
    agent = Node(
        package="picky_cobot_1",
        executable="ibvs_nn_pick_agent",
        name="ibvs_nn_pick_agent",
        output="screen",
        parameters=[
            {"robot_name": robot_name},
            {"pick_timeout_sec": ParameterValue(pick_timeout_sec, value_type=float)},
            {"request_topic": pick_request_topic},
            {"result_topic": pick_result_topic},
            {"desired_area_norm": ParameterValue(desired_area_norm, value_type=float)},
            {"extra_launch_args": extra_launch_args},
        ],
    )

    return [detection, agent]


def generate_launch_description():
    home = os.path.expanduser("~")
    default_model_dir = os.path.join(
        home,
        "just_pick_it/src/just_pick_it/just_pick_it_perception/result/nn_controller",
    )
    default_detection_model = os.path.join(
        home,
        "just_pick_it/src/just_pick_it/just_pick_it_perception/result/jetcobot_1/best.pt",
    )

    args = [
        # nn_inference.launch.py 의 robot_name 및 cobot 호스트 드라이버와 일치해야 한다.
        DeclareLaunchArgument("robot_name", default_value="jetcobot1"),
        # 카메라 UDP 송신 포트(camera_dest_port)와 반드시 일치.
        DeclareLaunchArgument("udp_port", default_value="5003"),
        # 평소 yolo_seg_infer.launch.xml 실행 시 넘기던 모델 경로와 동일.
        DeclareLaunchArgument("detection_model_path", default_value=default_detection_model),
        DeclareLaunchArgument(
            "target_classes",
            default_value="bread,choco_pie,cream_bread,fanta,water,watermelon",
        ),
        DeclareLaunchArgument("detection_confidence", default_value="0.5"),
        DeclareLaunchArgument("pick_timeout_sec", default_value="120.0"),
        DeclareLaunchArgument("pick_request_topic", default_value="/ibvs_nn_pick/request"),
        DeclareLaunchArgument("pick_result_topic", default_value="/ibvs_nn_pick/result"),
        # IBVS DONE 판정 area_norm 임계값. 클수록 물체에 더 가까이 접근한 뒤 grip.
        # 음수(기본값)면 agent 가 전달하지 않아 nn_inference.launch.py 기본값(0.23)을 쓴다.
        DeclareLaunchArgument("desired_area_norm", default_value="-1.0"),
        # --- nn_inference.launch.py 로 전달할 검증된 튜닝 인자 ---
        # 기본값은 수동 검증에서 가장 잘 동작한 조합. 재튜닝 시 개별 덮어쓰기.
        DeclareLaunchArgument("model_dir", default_value=default_model_dir),
        DeclareLaunchArgument("grip_confidence_threshold", default_value="0.8"),
        DeclareLaunchArgument("grip_consecutive_required", default_value="8"),
        DeclareLaunchArgument("max_fine_tune_steps", default_value="300"),
        DeclareLaunchArgument("nn_control_rate_hz", default_value="6.0"),
        DeclareLaunchArgument("nn_command_speed", default_value="60"),
        DeclareLaunchArgument("nn_delta_scale", default_value="0.6"),
        DeclareLaunchArgument("nn_delta_smooth_alpha", default_value="0.5"),
        DeclareLaunchArgument("nn_settle_delta_deg", default_value="0.8"),
        DeclareLaunchArgument("nn_command_leash_deg", default_value="6.0"),
        DeclareLaunchArgument("nn_z_floor_enable", default_value="true"),
        DeclareLaunchArgument("nn_z_floor_mm", default_value="220.0"),
        DeclareLaunchArgument("nn_z_floor_margin_mm", default_value="0.0"),
        DeclareLaunchArgument("ibvs_command_speed", default_value="10"),
        DeclareLaunchArgument("j6_angle_sign", default_value="1.0"),
        DeclareLaunchArgument("j6_angle_offset_deg", default_value="0.0"),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
