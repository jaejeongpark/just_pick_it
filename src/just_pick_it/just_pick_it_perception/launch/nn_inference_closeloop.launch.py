#!/usr/bin/env python3

"""
Closed-loop NN controller 추론 launch (방식 1).

기존 nn_inference.launch.py 와 별개 파일(롤백 안전). 차이:
  - IBVS 가 학습(방식 1)과 동일하게 거친 접근 + 느슨한 정렬로 인계해야 추론 분포가
    학습과 일치한다. approach_center_threshold / desired_area_norm 기본값을 방식 1로 둔다.
  - nn_controller 는 config 의 model_kind=closeloop 를 자동 감지해 live 시각오차 입력 +
    closed/open hysteresis 로 동작한다. detection_topic / target_class_label 을 전달해
    런타임 live detection 선택을 학습과 동일(nearest_center)하게 맞춘다.

주의: 실제 로봇을 움직인다. 작업공간을 비우고 실행.
"""

import os

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
    # NN(closeloop) 단계 제어.
    nn_control_rate_hz = LaunchConfiguration("nn_control_rate_hz").perform(context)
    nn_command_speed = LaunchConfiguration("nn_command_speed").perform(context)
    nn_delta_scale = LaunchConfiguration("nn_delta_scale").perform(context)
    nn_delta_smooth_alpha = LaunchConfiguration("nn_delta_smooth_alpha").perform(context)
    nn_settle_delta_deg = LaunchConfiguration("nn_settle_delta_deg").perform(context)
    nn_command_leash_deg = LaunchConfiguration("nn_command_leash_deg").perform(context)
    nn_status_poll_rate_hz = LaunchConfiguration("nn_status_poll_rate_hz").perform(context)
    grip_confidence_threshold = LaunchConfiguration("grip_confidence_threshold").perform(context)
    grip_consecutive_required = LaunchConfiguration("grip_consecutive_required").perform(context)
    max_fine_tune_steps = LaunchConfiguration("max_fine_tune_steps").perform(context)
    # closeloop 전용. 0/음수면 모델 config 값 사용.
    det_valid_timeout = LaunchConfiguration("det_valid_timeout").perform(context)
    open_loop_lost_frames = LaunchConfiguration("open_loop_lost_frames").perform(context)
    on_low_confidence_action = LaunchConfiguration("on_low_confidence_action").perform(context)
    # z-floor: base_link 기준 z 하한(get_coords 좌표계, mm). end-effector 가 shelf 아래로
    # 못 내려가게 하는 hard constraint. effective floor = z_floor_mm + z_floor_margin_mm.
    nn_z_floor_enable = LaunchConfiguration("nn_z_floor_enable").perform(context)
    nn_z_floor_mm = LaunchConfiguration("nn_z_floor_mm").perform(context)
    nn_z_floor_margin_mm = LaunchConfiguration("nn_z_floor_margin_mm").perform(context)
    # IBVS(방식 1) 단계 제어.
    ibvs_command_speed = LaunchConfiguration("ibvs_command_speed").perform(context)
    ibvs_pregrasp_speed = LaunchConfiguration("ibvs_pregrasp_speed").perform(context)
    ibvs_control_rate_hz = LaunchConfiguration("ibvs_control_rate_hz").perform(context)
    j6_angle_sign = LaunchConfiguration("j6_angle_sign").perform(context)
    j6_angle_offset_deg = LaunchConfiguration("j6_angle_offset_deg").perform(context)
    j6_square_aspect_thresh = LaunchConfiguration("j6_square_aspect_thresh").perform(context)
    desired_area_norm = LaunchConfiguration("desired_area_norm").perform(context)
    approach_center_threshold = LaunchConfiguration("approach_center_threshold").perform(context)

    perception_share = get_package_share_directory("just_pick_it_perception")
    ibvs_launch_path = os.path.join(perception_share, "launch", "ibvs_controller.launch.py")

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
            "j6_square_aspect_thresh": j6_square_aspect_thresh,
            # 방식 1: 학습과 동일한 거친 접근 + 느슨한 정렬 인계.
            "desired_area_norm": desired_area_norm,
            "approach_center_threshold": approach_center_threshold,
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
                "control_rate_hz": float(nn_control_rate_hz),
                "command_speed": int(nn_command_speed),
                "delta_scale": float(nn_delta_scale),
                "delta_smooth_alpha": float(nn_delta_smooth_alpha),
                "settle_delta_deg": float(nn_settle_delta_deg),
                "command_leash_deg": float(nn_command_leash_deg),
                "status_poll_rate_hz": float(nn_status_poll_rate_hz),
                "grip_confidence_threshold": float(grip_confidence_threshold),
                "grip_consecutive_required": int(grip_consecutive_required),
                "max_fine_tune_steps": int(max_fine_tune_steps),
                # closeloop 전용.
                "det_valid_timeout": float(det_valid_timeout),
                "open_loop_lost_frames": int(open_loop_lost_frames),
                "on_low_confidence_action": on_low_confidence_action,
                # z-floor hard constraint (base_link 기준 z 하한).
                "z_floor_enable": (nn_z_floor_enable.lower() in ("true", "1", "yes")),
                "z_floor_mm": float(nn_z_floor_mm),
                "z_floor_margin_mm": float(nn_z_floor_margin_mm),
            }
        ],
    )

    # 시작 시 gripper open(100).
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
        # --- NN(closeloop) 단계 ---
        DeclareLaunchArgument("nn_control_rate_hz", default_value="0.0"),
        DeclareLaunchArgument("nn_command_speed", default_value="10"),
        DeclareLaunchArgument("nn_delta_scale", default_value="1.0"),
        DeclareLaunchArgument("nn_delta_smooth_alpha", default_value="0.5"),
        DeclareLaunchArgument("nn_settle_delta_deg", default_value="0.8"),
        DeclareLaunchArgument("nn_command_leash_deg", default_value="8.0"),
        DeclareLaunchArgument("nn_status_poll_rate_hz", default_value="5.0"),
        DeclareLaunchArgument("grip_confidence_threshold", default_value="0.8"),
        DeclareLaunchArgument("grip_consecutive_required", default_value="3"),
        DeclareLaunchArgument("max_fine_tune_steps", default_value="100"),
        # closeloop 전용. 0/음수면 모델 config 값을 따른다(학습 시 저장).
        DeclareLaunchArgument("det_valid_timeout", default_value="0.0"),
        DeclareLaunchArgument("open_loop_lost_frames", default_value="0"),
        DeclareLaunchArgument("on_low_confidence_action", default_value="grip"),
        # z-floor: base_link 기준 z 하한(get_coords 좌표계, mm). 기본 비활성.
        # z_floor_mm 은 gripper 가 닿으면 안 되는 표면(예: shelf) 의 z 측정값.
        DeclareLaunchArgument("nn_z_floor_enable", default_value="false"),
        DeclareLaunchArgument("nn_z_floor_mm", default_value="0.0"),
        DeclareLaunchArgument("nn_z_floor_margin_mm", default_value="0.0"),
        # --- IBVS(방식 1) 단계 ---
        DeclareLaunchArgument("ibvs_command_speed", default_value="20"),
        DeclareLaunchArgument("ibvs_pregrasp_speed", default_value="15"),
        DeclareLaunchArgument("ibvs_control_rate_hz", default_value="5.0"),
        DeclareLaunchArgument("j6_angle_sign", default_value="1.0"),
        DeclareLaunchArgument("j6_angle_offset_deg", default_value="0.0"),
        DeclareLaunchArgument("j6_square_aspect_thresh", default_value="1.2"),
        # 방식 1 인계: 학습 수집(nn_data_collection_closeloop)과 동일 기본값.
        DeclareLaunchArgument("desired_area_norm", default_value="0.14"),
        DeclareLaunchArgument("approach_center_threshold", default_value="0.25"),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
