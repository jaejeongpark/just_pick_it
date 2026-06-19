#!/usr/bin/env python3

"""
학습된 NN controller 테스트용 launch.

구성:
  1. ibvs_controller : center/left/right 탐색 -> align -> approach -> DONE 후 J6 장축 정렬
                       -> ibvs_done 발행
  2. nn_controller   : ibvs_done 수신 후 활성화. J1~J5 fine-tune(policy) + grip(Grip Success
                       Predictor 게이트). J6는 ibvs가 정렬해둔 값을 유지(passthrough).

주의:
  - 실제 로봇을 움직인다. 작업공간을 비우고 실행.
  - model_dir의 학습 산출물(nn_controller_policy.pt / grip_success_predictor.pt /
    nn_controller_config.json)을 사용한다. 새 파이프라인으로 수집·학습한 모델일수록 정확.
  - 추론 주기는 config의 target_control_hz를 따른다(학습과 일치).
"""

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    detection_topic = LaunchConfiguration("detection_topic").perform(context)
    target_class_label = LaunchConfiguration("target_class_label").perform(context)
    min_confidence = LaunchConfiguration("min_confidence").perform(context)
    image_width = LaunchConfiguration("image_width").perform(context)
    image_height = LaunchConfiguration("image_height").perform(context)
    model_dir = LaunchConfiguration("model_dir").perform(context)
    device = LaunchConfiguration("device").perform(context)
    grip_confidence_threshold = LaunchConfiguration("grip_confidence_threshold").perform(context)
    grip_consecutive_required = LaunchConfiguration("grip_consecutive_required").perform(context)
    max_fine_tune_steps = LaunchConfiguration("max_fine_tune_steps").perform(context)
    # NN 추론(정밀보정) 단계 제어.
    nn_control_rate_hz = LaunchConfiguration("nn_control_rate_hz").perform(context)
    nn_command_speed = LaunchConfiguration("nn_command_speed").perform(context)
    nn_delta_scale = LaunchConfiguration("nn_delta_scale").perform(context)
    nn_delta_smooth_alpha = LaunchConfiguration("nn_delta_smooth_alpha").perform(context)
    nn_settle_delta_deg = LaunchConfiguration("nn_settle_delta_deg").perform(context)
    nn_z_floor_enable = LaunchConfiguration("nn_z_floor_enable").perform(context)
    nn_z_floor_mm = LaunchConfiguration("nn_z_floor_mm").perform(context)
    nn_z_floor_margin_mm = LaunchConfiguration("nn_z_floor_margin_mm").perform(context)
    nn_status_poll_rate_hz = LaunchConfiguration("nn_status_poll_rate_hz").perform(context)
    nn_command_leash_deg = LaunchConfiguration("nn_command_leash_deg").perform(context)
    # IBVS(탐색/정렬/접근/J6) 단계 제어.
    ibvs_command_speed = LaunchConfiguration("ibvs_command_speed").perform(context)
    ibvs_pregrasp_speed = LaunchConfiguration("ibvs_pregrasp_speed").perform(context)
    ibvs_control_rate_hz = LaunchConfiguration("ibvs_control_rate_hz").perform(context)
    j6_angle_sign = LaunchConfiguration("j6_angle_sign").perform(context)
    j6_angle_offset_deg = LaunchConfiguration("j6_angle_offset_deg").perform(context)
    # IBVS DONE 판정 area_norm 임계값(클수록 물체에 더 가까이 접근 후 grip).
    desired_area_norm = LaunchConfiguration("desired_area_norm").perform(context)

    perception_share = get_package_share_directory("just_pick_it_perception")
    ibvs_launch_path = os.path.join(
        perception_share, "launch", "ibvs_controller.launch.py"
    )

    ibvs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ibvs_launch_path),
        launch_arguments={
            "robot_name": robot_name,
            "detection_topic": detection_topic,
            "target_class_label": target_class_label,
            "min_confidence": min_confidence,
            "image_width": image_width,
            "image_height": image_height,
            "j6_angle_sign": j6_angle_sign,
            "j6_angle_offset_deg": j6_angle_offset_deg,
            "desired_area_norm": desired_area_norm,
            # IBVS 단계 속도/주기 (NN 단계와 독립).
            "command_speed": ibvs_command_speed,
            "pregrasp_speed": ibvs_pregrasp_speed,
            "control_rate_hz": ibvs_control_rate_hz,
        }.items(),
    )

    nn_controller = Node(
        package="just_pick_it_perception",
        executable="nn_controller",
        name="nn_controller",
        output="screen",
        parameters=[
            {
                "robot_name": robot_name,
                "detection_topic": detection_topic,
                "target_class_label": target_class_label,
                "min_confidence": float(min_confidence),
                "image_width": float(image_width),
                "image_height": float(image_height),
                "model_dir": model_dir,
                "device": device,
                "grip_confidence_threshold": float(grip_confidence_threshold),
                "grip_consecutive_required": int(grip_consecutive_required),
                "max_fine_tune_steps": int(max_fine_tune_steps),
                # NN 단계: 0이면 학습 target_control_hz 사용, >0이면 override.
                "control_rate_hz": float(nn_control_rate_hz),
                "command_speed": int(nn_command_speed),
                "delta_scale": float(nn_delta_scale),
                "delta_smooth_alpha": float(nn_delta_smooth_alpha),
                "settle_delta_deg": float(nn_settle_delta_deg),
                "z_floor_enable": (nn_z_floor_enable.lower() in ("true", "1", "yes")),
                "z_floor_mm": float(nn_z_floor_mm),
                "z_floor_margin_mm": float(nn_z_floor_margin_mm),
                "status_poll_rate_hz": float(nn_status_poll_rate_hz),
                "command_leash_deg": float(nn_command_leash_deg),
            }
        ],
    )

    # 시작 시 gripper를 100(open)으로 만든다(드라이버 구독 시간 확보 후 발행).
    gripper_open = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "topic", "pub", "--once",
                    f"/{robot_name}/set_gripper",
                    "std_msgs/msg/Float64MultiArray",
                    "{data: [100.0, 50.0]}",
                ],
                output="screen",
            )
        ],
    )

    return [ibvs, nn_controller, gripper_open]


def generate_launch_description():
    home = os.path.expanduser("~")
    default_model_dir = os.path.join(
        home,
        "just_pick_it/src/just_pick_it/just_pick_it_perception/result/nn_controller/pick",
    )
    args = [
        DeclareLaunchArgument("robot_name", default_value="jetcobot1"),
        DeclareLaunchArgument("detection_topic", default_value="/infer/tracked_objects"),
        DeclareLaunchArgument("target_class_label", default_value="watermelon"),
        DeclareLaunchArgument("min_confidence", default_value="0.5"),
        DeclareLaunchArgument("image_width", default_value="640.0"),
        DeclareLaunchArgument("image_height", default_value="480.0"),
        DeclareLaunchArgument("model_dir", default_value=default_model_dir),
        DeclareLaunchArgument("device", default_value="cpu"),
        # grip go/no-go 게이트.
        DeclareLaunchArgument("grip_confidence_threshold", default_value="0.8"),
        DeclareLaunchArgument("grip_consecutive_required", default_value="3"),
        DeclareLaunchArgument("max_fine_tune_steps", default_value="100"),
        # --- NN 추론(정밀보정) 단계 제어 ---
        # 제어 주기(Hz). 0=학습 target_control_hz 사용. 로봇이 못 따라가 명령이 쌓이면
        # 낮춰서(예: 2.0) 천천히.
        DeclareLaunchArgument("nn_control_rate_hz", default_value="0.0"),
        DeclareLaunchArgument("nn_command_speed", default_value="10"),
        # policy 출력 스케일. 제어 주기를 올리고 이 값을 줄이면(예: 10Hz x 0.5)
        # 같은 속도로 step이 잘게 쪼개져 부드러워진다.
        DeclareLaunchArgument("nn_delta_scale", default_value="1.0"),
        # delta EMA 평활 (1.0=비활성). 노이즈성 방향 반전을 눌러 목표 주변 진동 억제.
        DeclareLaunchArgument("nn_delta_smooth_alpha", default_value="0.5"),
        # settle-hold: 평활 |delta|가 이 값(deg) 미만이면 명령 보류, 정지 상태로 grip 판정.
        DeclareLaunchArgument("nn_settle_delta_deg", default_value="0.8"),
        # z-floor: shelf 높이 아래로 end-effector가 못 내려가게 하는 hard constraint.
        # z_floor_mm은 로봇 get_coords와 같은 좌표계(mm, base 기준)의 shelf 표면 z.
        DeclareLaunchArgument("nn_z_floor_enable", default_value="false"),
        DeclareLaunchArgument("nn_z_floor_mm", default_value="0.0"),
        DeclareLaunchArgument("nn_z_floor_margin_mm", default_value="0.0"),
        # status 폴링 주기(Hz). 제어 주기와 분리(시리얼 정체 방지).
        DeclareLaunchArgument("nn_status_poll_rate_hz", default_value="5.0"),
        # 명령 적분 목표가 측정값보다 앞서갈 수 있는 한계(deg).
        DeclareLaunchArgument("nn_command_leash_deg", default_value="8.0"),
        # --- IBVS(탐색/정렬/접근/J6) 단계 제어 (NN과 독립) ---
        DeclareLaunchArgument("ibvs_command_speed", default_value="10"),
        DeclareLaunchArgument("ibvs_pregrasp_speed", default_value="15"),
        DeclareLaunchArgument("ibvs_control_rate_hz", default_value="5.0"),
        # J6 장축 정렬 보정(ibvs로 전달).
        DeclareLaunchArgument("j6_angle_sign", default_value="1.0"),
        DeclareLaunchArgument("j6_angle_offset_deg", default_value="0.0"),
        # DONE 판정 area_norm 임계값(ibvs_controller로 전달). 기본값은
        # ibvs_controller.launch.py 와 동일하게 0.23.
        DeclareLaunchArgument("desired_area_norm", default_value="0.23"),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
