#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("robot_name", default_value="jetcobot1"),

        DeclareLaunchArgument("status_topic", default_value=""),
        DeclareLaunchArgument("request_status_topic", default_value=""),
        DeclareLaunchArgument("detection_topic", default_value="/infer/tracked_objects"),

        DeclareLaunchArgument("target_class_label", default_value="watermelon"),
        DeclareLaunchArgument("min_confidence", default_value="0.5"),

        DeclareLaunchArgument("image_width", default_value="640.0"),
        DeclareLaunchArgument("image_height", default_value="480.0"),

        DeclareLaunchArgument("center_source", default_value="bbox"),
        DeclareLaunchArgument("bbox_xy_mode", default_value="center"),

        DeclareLaunchArgument("desired_cx", default_value="-1.0"),
        DeclareLaunchArgument("desired_cy", default_value="-1.0"),

        DeclareLaunchArgument("sample_rate_hz", default_value="10.0"),
        DeclareLaunchArgument("status_timeout_sec", default_value="1.0"),
        DeclareLaunchArgument("detection_timeout_sec", default_value="1.0"),

        DeclareLaunchArgument("start_after_first_detection", default_value="true"),
        DeclareLaunchArgument("use_last_valid_when_lost", default_value="true"),

        DeclareLaunchArgument("terminal_trigger_area_norm", default_value="0.06"),
        DeclareLaunchArgument("terminal_trigger_center_norm", default_value="0.035"),
        DeclareLaunchArgument("terminal_ready_frames", default_value="5"),
        DeclareLaunchArgument("terminal_nominal_duration_sec", default_value="1.0"),

        DeclareLaunchArgument("bag_uri", default_value=""),
        DeclareLaunchArgument("bag_base_dir", default_value="~/rosbags"),
        DeclareLaunchArgument("bag_name_prefix", default_value="visual_servo"),
        DeclareLaunchArgument("bag_topic", default_value="/nn_controller/training_sample"),
        DeclareLaunchArgument("storage_id", default_value="sqlite3"),

        DeclareLaunchArgument("stop_on_gripper_close", default_value="true"),
        DeclareLaunchArgument("gripper_close_mode", default_value="le"),
        DeclareLaunchArgument("gripper_close_threshold", default_value="20.0"),
        DeclareLaunchArgument("require_open_before_close", default_value="true"),

        DeclareLaunchArgument(
            "manual_stop_topic",
            default_value="/visual_servo_bag_recorder/stop",
        ),
        DeclareLaunchArgument("shutdown_on_stop", default_value="true"),
    ]

    node = Node(
        package="just_pick_it_perception",
        executable="visual_servo_bag_recorder",
        name="visual_servo_bag_recorder",
        output="screen",
        parameters=[
            {
                "robot_name": LaunchConfiguration("robot_name"),

                "status_topic": LaunchConfiguration("status_topic"),
                "request_status_topic": LaunchConfiguration("request_status_topic"),
                "detection_topic": LaunchConfiguration("detection_topic"),

                "target_class_label": LaunchConfiguration("target_class_label"),
                "min_confidence": LaunchConfiguration("min_confidence"),

                "image_width": LaunchConfiguration("image_width"),
                "image_height": LaunchConfiguration("image_height"),

                "center_source": LaunchConfiguration("center_source"),
                "bbox_xy_mode": LaunchConfiguration("bbox_xy_mode"),

                "desired_cx": LaunchConfiguration("desired_cx"),
                "desired_cy": LaunchConfiguration("desired_cy"),

                "sample_rate_hz": LaunchConfiguration("sample_rate_hz"),
                "status_timeout_sec": LaunchConfiguration("status_timeout_sec"),
                "detection_timeout_sec": LaunchConfiguration("detection_timeout_sec"),

                "start_after_first_detection": LaunchConfiguration(
                    "start_after_first_detection"
                ),
                "use_last_valid_when_lost": LaunchConfiguration(
                    "use_last_valid_when_lost"
                ),

                "terminal_trigger_area_norm": LaunchConfiguration(
                    "terminal_trigger_area_norm"
                ),
                "terminal_trigger_center_norm": LaunchConfiguration(
                    "terminal_trigger_center_norm"
                ),
                "terminal_ready_frames": LaunchConfiguration(
                    "terminal_ready_frames"
                ),
                "terminal_nominal_duration_sec": LaunchConfiguration(
                    "terminal_nominal_duration_sec"
                ),

                "bag_uri": LaunchConfiguration("bag_uri"),
                "bag_base_dir": LaunchConfiguration("bag_base_dir"),
                "bag_name_prefix": LaunchConfiguration("bag_name_prefix"),
                "bag_topic": LaunchConfiguration("bag_topic"),
                "storage_id": LaunchConfiguration("storage_id"),

                "stop_on_gripper_close": LaunchConfiguration("stop_on_gripper_close"),
                "gripper_close_mode": LaunchConfiguration("gripper_close_mode"),
                "gripper_close_threshold": LaunchConfiguration(
                    "gripper_close_threshold"
                ),
                "require_open_before_close": LaunchConfiguration(
                    "require_open_before_close"
                ),

                "manual_stop_topic": LaunchConfiguration("manual_stop_topic"),
                "shutdown_on_stop": LaunchConfiguration("shutdown_on_stop"),
            }
        ],
    )

    return LaunchDescription(args + [node])