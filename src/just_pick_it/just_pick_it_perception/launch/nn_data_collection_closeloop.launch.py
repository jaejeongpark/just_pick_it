#!/usr/bin/env python3

"""
Closed-loop NN 재수집(방식 1)용 데이터 수집 launch.

기존 nn_data_collection.launch.py 와 동일한 recorder 구성을 쓰되, IBVS 인계 방식만
바꾼다. 기존 launch/기본값은 건드리지 않으며, 문제가 있으면 이 파일만 지우면 된다.

방식 1 (거친 접근 + 느슨한 중앙정렬):
  - approach_center_threshold 를 높여(기본 0.09 대비 크게) IBVS 가 물체를 정중앙까지
    맞추지 않고 화면에서 다소 벗어난 상태로도 area 접근을 진행하게 한다. 그 결과 사람이
    인계받는 시점에 물체가 화면 여러 위치에 분포하여 closed-loop 시각오차가 다양하게
    수집된다(기존 데이터는 IBVS 가 항상 중앙에 맞춰 시각오차 분포가 거의 0이었다).
  - desired_area_norm 을 낮춰(기본 0.23 대비 작게) 물체가 화면에 잘 보이는 상태에서
    사람에게 인계한다. closed-loop 추적 구간(runway)이 길어진다.

수집 데이터는 기존 ~/rosbags 의 중앙정렬 데이터와 분포가 다르므로 섞지 않는다.
기본 저장 경로를 ~/rosbags_closeloop 로 분리한다.

실행:
  ros2 launch just_pick_it_perception nn_data_collection_closeloop.launch.py
"""

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
    # 방식 1 인계 파라미터 (ibvs_controller 로 전달).
    desired_area_norm = LaunchConfiguration("desired_area_norm").perform(context)
    approach_center_threshold = LaunchConfiguration("approach_center_threshold").perform(context)
    area_done_center_threshold = LaunchConfiguration("area_done_center_threshold").perform(context)

    if not episode_id:
        episode_id = datetime.now().strftime("%Y%m%d_%H%M%S")

    print(f"[nn_data_collection_closeloop] episode_id = {episode_id}")
    print(f"[nn_data_collection_closeloop] bag_base_dir = {bag_base_dir}")
    print(f"[nn_data_collection_closeloop] desired_area_norm = {desired_area_norm}, "
          f"approach_center_threshold = {approach_center_threshold}")

    perception_share = get_package_share_directory("just_pick_it_perception")
    ibvs_launch_path = os.path.join(
        perception_share, "launch", "ibvs_controller.launch.py"
    )

    # ibvs_controller 는 기존 launch 파일을 재사용하고 방식 1 인계 인자만 덮어쓴다.
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
            # 방식 1: 느슨한 정렬 + 조기 인계.
            "desired_area_norm": desired_area_norm,
            "approach_center_threshold": approach_center_threshold,
            "area_done_center_threshold": area_done_center_threshold,
        }.items(),
    )

    # IBVS align+approach 구간 기록. ibvs_done 수신 시 bag만 닫고 노드는 유지한다.
    # detection 이 사라져도 grip close 까지 계속 기록하므로 human 정밀보정 구간의
    # live 시각오차(current_cx/cy/area_norm)가 이 bag 에 함께 남는다.
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
    # human_recorder 가 결과 기록 후 종료되면 전체 launch 를 종료한다.
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

    # launch 시작 시 무조건 gripper 를 100(open)으로 만든다.
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
        # 방식 1 데이터는 중앙정렬 데이터와 섞지 않도록 별도 디렉토리에 모은다.
        DeclareLaunchArgument("bag_base_dir", default_value="~/rosbags_closeloop"),
        # 비워두면 launch 시점 타임스탬프로 자동 생성된다.
        DeclareLaunchArgument("episode_id", default_value=""),
        DeclareLaunchArgument("record_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("record_mode", default_value="displacement"),
        DeclareLaunchArgument("displacement_threshold_deg", default_value="2.0"),
        DeclareLaunchArgument("loop_episodes", default_value="true"),
        DeclareLaunchArgument("done_status_poll_rate_hz", default_value="10.0"),
        # --- 방식 1 인계 파라미터 (ibvs_controller 로 전달) ---
        # 조기 인계: 기본 0.23 보다 작게. 물체가 화면에 잘 보이는 상태에서 사람에게 넘김.
        DeclareLaunchArgument("desired_area_norm", default_value="0.14"),
        # 느슨한 정렬: 기본 0.09 보다 크게. 물체가 화면에서 다소 벗어난 채로 접근/인계되어
        # closed-loop 시각오차가 다양하게 수집된다.
        DeclareLaunchArgument("approach_center_threshold", default_value="0.25"),
        # DONE center 임계. 음수이면 ibvs_controller 가 approach_center_threshold 를 쓴다.
        DeclareLaunchArgument("area_done_center_threshold", default_value="-1.0"),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
