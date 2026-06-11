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
    j6_angle_sign = LaunchConfiguration("j6_angle_sign").perform(context)
    j6_angle_offset_deg = LaunchConfiguration("j6_angle_offset_deg").perform(context)

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
        "just_pick_it/src/just_pick_it/just_pick_it_perception/result/nn_controller",
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
        # J6 장축 정렬 보정(ibvs로 전달).
        DeclareLaunchArgument("j6_angle_sign", default_value="1.0"),
        DeclareLaunchArgument("j6_angle_offset_deg", default_value="0.0"),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
