#!/usr/bin/env python3

import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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
    bag_base_dir = LaunchConfiguration("bag_base_dir").perform(context)
    episode_id = LaunchConfiguration("episode_id").perform(context)
    record_rate_hz = LaunchConfiguration("record_rate_hz").perform(context)
    record_mode = LaunchConfiguration("record_mode").perform(context)
    displacement_threshold_deg = LaunchConfiguration(
        "displacement_threshold_deg"
    ).perform(context)
    loop_episodes = LaunchConfiguration("loop_episodes").perform(context)
    done_status_poll_rate_hz = LaunchConfiguration("done_status_poll_rate_hz").perform(context)

    if not episode_id:
        episode_id = datetime.now().strftime("%Y%m%d_%H%M%S")

    print(f"[nn_data_collection] episode_id = {episode_id}")
    print(f"[nn_data_collection] bag_base_dir = {bag_base_dir}")

    perception_share = get_package_share_directory("just_pick_it_perception")
    ibvs_launch_path = os.path.join(
        perception_share, "launch", "ibvs_controller.launch.py"
    )

    # ibvs_controller는 기존 launch 파일을 재사용하고 공유 인자만 덮어쓴다.
    ibvs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ibvs_launch_path),
        launch_arguments={
            "robot_name": robot_name,
            "detection_topic": detection_topic,
            "target_class_label": target_class_label,
            "min_confidence": min_confidence,
            "image_width": image_width,
            "image_height": image_height,
            "done_status_poll_rate_hz": done_status_poll_rate_hz,
        }.items(),
    )

    # IBVS align+approach 구간 기록. ibvs_done 수신 시 bag만 닫고 노드는 유지한다.
    visual_servo_recorder = Node(
        package="just_pick_it_perception",
        executable="visual_servo_bag_recorder",
        name="visual_servo_bag_recorder",
        output="screen",
        parameters=[
            {
                "robot_name": robot_name,
                "detection_topic": detection_topic,
                "target_class_label": target_class_label,
                "min_confidence": float(min_confidence),
                "image_width": float(image_width),
                "image_height": float(image_height),
                "bag_base_dir": bag_base_dir,
                "episode_id": episode_id,
                "shutdown_on_stop": False,
            }
        ],
    )

    # ibvs_done 이후 free-drive + grip 구간 기록. 결과 확정 시 episode 디렉토리 이동.
    # 키 입력은 tkinter GUI 창으로 받는다(stdin 불필요).
    # 결과 기록(shutdown_on_done=True) 후 GUI가 닫히면 OnProcessExit으로 전체 launch 종료.
    human_recorder = Node(
        package="just_pick_it_perception",
        executable="human_interaction_recorder",
        name="human_interaction_recorder",
        output="screen",
        parameters=[
            {
                "robot_name": robot_name,
                "bag_base_dir": bag_base_dir,
                "episode_id": episode_id,
                "record_rate_hz": float(record_rate_hz),
                "record_mode": record_mode,
                "displacement_threshold_deg": float(displacement_threshold_deg),
                "loop_episodes": (loop_episodes.lower() in ("true", "1", "yes")),
                "shutdown_on_done": True,
            }
        ],
    )

    # 한 에피소드 = 한 launch 실행.
    # human_recorder가 결과 기록 후 종료되면 전체 launch를 종료한다.
    shutdown_on_human_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=human_recorder,
            on_exit=[
                EmitEvent(
                    event=Shutdown(reason="human_interaction_recorder finished")
                )
            ],
        )
    )

    # launch 시작 시 무조건 gripper를 100(open)으로 만든다.
    # jetcobot 드라이버가 set_gripper를 구독할 시간을 두기 위해 잠시 지연 후 발행한다.
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

    return [
        ibvs,
        visual_servo_recorder,
        human_recorder,
        shutdown_on_human_exit,
        gripper_open,
    ]


def generate_launch_description():
    args = [
        DeclareLaunchArgument("robot_name", default_value="jetcobot1"),
        DeclareLaunchArgument(
            "detection_topic", default_value="/infer/tracked_objects"
        ),
        DeclareLaunchArgument("target_class_label", default_value="watermelon"),
        DeclareLaunchArgument("min_confidence", default_value="0.5"),
        DeclareLaunchArgument("image_width", default_value="640.0"),
        DeclareLaunchArgument("image_height", default_value="480.0"),
        DeclareLaunchArgument("bag_base_dir", default_value="~/rosbags"),
        # 비워두면 launch 시점 타임스탬프로 자동 생성된다.
        DeclareLaunchArgument("episode_id", default_value=""),
        # 기록 충실도를 위해 10Hz로 높인다. 학습/추론 timestep은 train의
        # target_control_hz 다운샘플로 분리(예: 5Hz)한다.
        DeclareLaunchArgument("record_rate_hz", default_value="10.0"),
        # displacement(기본): J1~J5 최대 변위가 displacement_threshold_deg 이상일 때마다
        # waypoint 저장(시연 속도 무관, 관절 공간 등간격). fixed_rate: 기존 고정 주기.
        # displacement 모드 학습은 train --target-control-hz 0 으로 돌린다(기록 그대로 사용).
        DeclareLaunchArgument("record_mode", default_value="displacement"),
        DeclareLaunchArgument("displacement_threshold_deg", default_value="2.0"),
        # True면 S/F/ERROR 후 종료하지 않고 다음 episode로 자동 루프. 종료는 GUI X.
        DeclareLaunchArgument("loop_episodes", default_value="true"),
        # DONE(human phase) 동안 ibvs_controller가 status를 폴링하는 주파수.
        DeclareLaunchArgument("done_status_poll_rate_hz", default_value="10.0"),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
