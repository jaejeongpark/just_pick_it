#!/usr/bin/env python3

import ast

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import OpaqueFunction


def _parse_float_list(s):
    try:
        result = ast.literal_eval(s)
        return [float(v) for v in result]
    except Exception:
        raise ValueError(f"Cannot parse float list from: {s!r}")


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration("robot_name").perform(context)
    detection_topic = LaunchConfiguration("detection_topic").perform(context)
    target_class_label = LaunchConfiguration("target_class_label").perform(context)
    center_source = LaunchConfiguration("center_source").perform(context)
    detection_timeout_sec = float(
        LaunchConfiguration("detection_timeout_sec").perform(context)
    )
    min_confidence = float(LaunchConfiguration("min_confidence").perform(context))
    image_width = float(LaunchConfiguration("image_width").perform(context))
    image_height = float(LaunchConfiguration("image_height").perform(context))

    lambda_gain = float(LaunchConfiguration("lambda_gain").perform(context))
    damping = float(LaunchConfiguration("damping").perform(context))
    max_delta_deg = float(LaunchConfiguration("max_delta_deg").perform(context))
    control_rate_hz = float(LaunchConfiguration("control_rate_hz").perform(context))
    jacobian_delta_deg = float(
        LaunchConfiguration("jacobian_delta_deg").perform(context)
    )
    jacobian_settle_sec = float(
        LaunchConfiguration("jacobian_settle_sec").perform(context)
    )
    stop_error = float(LaunchConfiguration("stop_error").perform(context))
    max_steps = int(LaunchConfiguration("max_steps").perform(context))

    pregrasp_speed = int(LaunchConfiguration("pregrasp_speed").perform(context))
    ibvs_speed = int(LaunchConfiguration("ibvs_speed").perform(context))

    lock_track_id = LaunchConfiguration("lock_track_id").perform(context).lower() in (
        "true",
        "1",
    )
    use_status_for_q0 = LaunchConfiguration(
        "use_status_for_q0"
    ).perform(context).lower() in ("true", "1")
    status_timeout_sec = float(
        LaunchConfiguration("status_timeout_sec").perform(context)
    )
    pregrasp_wait_sec = float(LaunchConfiguration("pregrasp_wait_sec").perform(context))

    pregrasp_angles = _parse_float_list(
        LaunchConfiguration("pregrasp_angles").perform(context)
    )

    node = Node(
        package="just_pick_it_perception",
        executable="ibvs_controller",
        output="screen",
        parameters=[
            {
                "robot_name": robot_name,
                "detection_topic": detection_topic,
                "target_class_label": target_class_label,
                "center_source": center_source,
                "detection_timeout_sec": detection_timeout_sec,
                "min_confidence": min_confidence,
                "image_width": image_width,
                "image_height": image_height,
                "lambda_gain": lambda_gain,
                "damping": damping,
                "max_delta_deg": max_delta_deg,
                "control_rate_hz": control_rate_hz,
                "jacobian_delta_deg": jacobian_delta_deg,
                "jacobian_settle_sec": jacobian_settle_sec,
                "stop_error": stop_error,
                "max_steps": max_steps,
                "pregrasp_speed": pregrasp_speed,
                "ibvs_speed": ibvs_speed,
                "lock_track_id": lock_track_id,
                "use_status_for_q0": use_status_for_q0,
                "status_timeout_sec": status_timeout_sec,
                "pregrasp_wait_sec": pregrasp_wait_sec,
                "pregrasp_angles": pregrasp_angles,
            }
        ],
    )
    return [node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_name", default_value="jetcobot1"),
            DeclareLaunchArgument(
                "detection_topic", default_value="/infer/tracked_objects"
            ),
            DeclareLaunchArgument("target_class_label", default_value="watermelon"),
            DeclareLaunchArgument("center_source", default_value="bbox"),
            DeclareLaunchArgument("detection_timeout_sec", default_value="2.0"),
            DeclareLaunchArgument("min_confidence", default_value="0.5"),
            DeclareLaunchArgument("image_width", default_value="640.0"),
            DeclareLaunchArgument("image_height", default_value="480.0"),
            DeclareLaunchArgument("lambda_gain", default_value="0.8"),
            DeclareLaunchArgument("damping", default_value="0.04"),
            DeclareLaunchArgument("max_delta_deg", default_value="0.5"),
            DeclareLaunchArgument("control_rate_hz", default_value="10.0"),
            DeclareLaunchArgument("jacobian_delta_deg", default_value="2.0"),
            DeclareLaunchArgument("jacobian_settle_sec", default_value="1.2"),
            DeclareLaunchArgument("stop_error", default_value="0.01"),
            DeclareLaunchArgument("max_steps", default_value="150"),
            DeclareLaunchArgument("pregrasp_speed", default_value="15"),
            DeclareLaunchArgument("ibvs_speed", default_value="10"),
            DeclareLaunchArgument("lock_track_id", default_value="true"),
            DeclareLaunchArgument("use_status_for_q0", default_value="true"),
            DeclareLaunchArgument("status_timeout_sec", default_value="1.0"),
            DeclareLaunchArgument("pregrasp_wait_sec", default_value="3.0"),
            DeclareLaunchArgument(
                "pregrasp_angles",
                default_value="[82.8, 56.5, -19.3, -93.6, 24.9, -121.4]",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
