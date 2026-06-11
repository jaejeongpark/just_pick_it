#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument("robot_name", default_value="jetcobot1"),
        DeclareLaunchArgument("detection_topic", default_value="/infer/tracked_objects"),
        DeclareLaunchArgument("target_class_label", default_value="watermelon"),
        DeclareLaunchArgument("min_confidence", default_value="0.5"),
        DeclareLaunchArgument("detection_timeout_sec", default_value="2.0"),
        DeclareLaunchArgument("lock_track_id", default_value="true"),

        DeclareLaunchArgument("center_source", default_value="bbox"),
        DeclareLaunchArgument("bbox_xy_mode", default_value="center"),
        DeclareLaunchArgument("image_width", default_value="640.0"),
        DeclareLaunchArgument("image_height", default_value="480.0"),
        DeclareLaunchArgument("desired_cx", default_value="-1.0"),
        DeclareLaunchArgument("desired_cy", default_value="-1.0"),

        # Joint split. 0-indexed: 0=J1, 1=J2, ..., 5=J6.
        DeclareLaunchArgument("align_joints", default_value="[0,3,4]"),
        DeclareLaunchArgument("approach_joints", default_value="[1,2]"),
        DeclareLaunchArgument(
            "pregrasp_angles",
            default_value="[107.75,29.17,-31.11,-71.63,2.90,-134.12]",
        ),

        DeclareLaunchArgument(
            "center_pregrasp_angles",
            default_value="[114.78,-5.09,-9.05,-75.49,9.05,-107.31]",
        ),
        DeclareLaunchArgument(
            "left_pregrasp_angles",
            default_value="[147.48,-8.96,-24.08,-59.85,4.39,-73.12]",
        ),
        DeclareLaunchArgument(
            "right_pregrasp_angles",
            default_value="[94.39,1.31,-26.19,-62.84,3.51,-127.08]",
        ),
        DeclareLaunchArgument("search_timeout_sec", default_value="3.0"),

        DeclareLaunchArgument("handoff_area_norm", default_value="-1.0"),
        DeclareLaunchArgument("handoff_area_ratio", default_value="0.6"),
        DeclareLaunchArgument("done_status_poll_rate_hz", default_value="5.0"),

        DeclareLaunchArgument("pregrasp_speed", default_value="15"),
        DeclareLaunchArgument("command_speed", default_value="20"),
        DeclareLaunchArgument("pregrasp_wait_sec", default_value="3.0"),
        DeclareLaunchArgument("use_status_for_q0", default_value="true"),
        DeclareLaunchArgument("status_timeout_sec", default_value="1.0"),

        # Align Jacobian measurement.
        DeclareLaunchArgument("jacobian_delta_deg", default_value="1.0"),
        DeclareLaunchArgument("jacobian_settle_sec", default_value="0.5"),

        # Align controller.
        DeclareLaunchArgument("lambda_gain", default_value="0.8"),
        DeclareLaunchArgument("damping", default_value="0.04"),
        DeclareLaunchArgument("max_align_delta_deg", default_value="1.0"),
        DeclareLaunchArgument("max_align_offset_deg", default_value="20.0"),
        DeclareLaunchArgument("control_rate_hz", default_value="5.0"),

        # ALIGN stuck recovery / active-set re-Jacobian.
        DeclareLaunchArgument("enable_align_stuck_recovery", default_value="true"),
        DeclareLaunchArgument("enable_align_active_set", default_value="true"),
        DeclareLaunchArgument("align_stuck_frames", default_value="8"),
        DeclareLaunchArgument("align_stuck_min_improvement", default_value="0.002"),
        DeclareLaunchArgument("align_stuck_cmd_delta_deg", default_value="0.05"),
        DeclareLaunchArgument("align_stuck_saturation_ratio", default_value="0.95"),
        DeclareLaunchArgument("max_align_rejacobian_count", default_value="5"),
        DeclareLaunchArgument("align_rejacobian_cooldown_sec", default_value="1.0"),
        DeclareLaunchArgument("align_rejacobian_after_approach_steps", default_value="3"),

        DeclareLaunchArgument("approach_center_threshold", default_value="0.09"),

        # DONE condition.
        # If area_done_center_threshold is negative, the node uses approach_center_threshold.
        DeclareLaunchArgument("desired_area_norm", default_value="0.23"),
        DeclareLaunchArgument("area_done_center_threshold", default_value="-1.0"),

        # Area Jacobian approach.
        DeclareLaunchArgument("approach_step_deg", default_value="3.0"),
        DeclareLaunchArgument("approach_wait_sec", default_value="0.6"),
        DeclareLaunchArgument("max_approach_steps", default_value="250"),
        DeclareLaunchArgument("area_jacobian_delta_deg", default_value="3.0"),
        DeclareLaunchArgument("area_jacobian_settle_sec", default_value="0.8"),
        DeclareLaunchArgument("area_window_size", default_value="5"),
        DeclareLaunchArgument("area_jacobian_min_grad", default_value="0.00001"),

        # Area direction reuse.
        # 1 = old behavior, measure every approach step.
        # 3 = one measured approach + up to two cached approach steps.
        DeclareLaunchArgument("area_jacobian_reuse_steps", default_value="3"),
        DeclareLaunchArgument("area_min_gain_for_reuse", default_value="0.001"),
        DeclareLaunchArgument("area_drop_tolerance", default_value="0.003"),

        # General safety / filter.
        DeclareLaunchArgument("max_total_steps", default_value="500"),
        DeclareLaunchArgument("hard_stop_below_center_error", default_value="-1.0"),
        DeclareLaunchArgument("filter_alpha", default_value="0.5"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]

    node = Node(
        package="just_pick_it_perception",
        executable="ibvs_controller",
        name="ibvs_controller",
        output="screen",
        parameters=[
            {
                "robot_name": LaunchConfiguration("robot_name"),
                "detection_topic": LaunchConfiguration("detection_topic"),
                "target_class_label": LaunchConfiguration("target_class_label"),
                "min_confidence": LaunchConfiguration("min_confidence"),
                "detection_timeout_sec": LaunchConfiguration("detection_timeout_sec"),
                "lock_track_id": LaunchConfiguration("lock_track_id"),

                "center_source": LaunchConfiguration("center_source"),
                "bbox_xy_mode": LaunchConfiguration("bbox_xy_mode"),
                "image_width": LaunchConfiguration("image_width"),
                "image_height": LaunchConfiguration("image_height"),
                "desired_cx": LaunchConfiguration("desired_cx"),
                "desired_cy": LaunchConfiguration("desired_cy"),

                "align_joints": LaunchConfiguration("align_joints"),
                "approach_joints": LaunchConfiguration("approach_joints"),
                "pregrasp_angles": LaunchConfiguration("pregrasp_angles"),

                "center_pregrasp_angles": LaunchConfiguration("center_pregrasp_angles"),
                "left_pregrasp_angles": LaunchConfiguration("left_pregrasp_angles"),
                "right_pregrasp_angles": LaunchConfiguration("right_pregrasp_angles"),
                "search_timeout_sec": LaunchConfiguration("search_timeout_sec"),
                "handoff_area_norm": LaunchConfiguration("handoff_area_norm"),
                "handoff_area_ratio": LaunchConfiguration("handoff_area_ratio"),
                "done_status_poll_rate_hz": LaunchConfiguration("done_status_poll_rate_hz"),

                "pregrasp_speed": LaunchConfiguration("pregrasp_speed"),
                "command_speed": LaunchConfiguration("command_speed"),
                "pregrasp_wait_sec": LaunchConfiguration("pregrasp_wait_sec"),
                "use_status_for_q0": LaunchConfiguration("use_status_for_q0"),
                "status_timeout_sec": LaunchConfiguration("status_timeout_sec"),

                "jacobian_delta_deg": LaunchConfiguration("jacobian_delta_deg"),
                "jacobian_settle_sec": LaunchConfiguration("jacobian_settle_sec"),

                "lambda_gain": LaunchConfiguration("lambda_gain"),
                "damping": LaunchConfiguration("damping"),
                "max_align_delta_deg": LaunchConfiguration("max_align_delta_deg"),
                "max_align_offset_deg": LaunchConfiguration("max_align_offset_deg"),
                "control_rate_hz": LaunchConfiguration("control_rate_hz"),
                "enable_align_stuck_recovery": LaunchConfiguration("enable_align_stuck_recovery"),
                "enable_align_active_set": LaunchConfiguration("enable_align_active_set"),
                "align_stuck_frames": LaunchConfiguration("align_stuck_frames"),
                "align_stuck_min_improvement": LaunchConfiguration("align_stuck_min_improvement"),
                "align_stuck_cmd_delta_deg": LaunchConfiguration("align_stuck_cmd_delta_deg"),
                "align_stuck_saturation_ratio": LaunchConfiguration("align_stuck_saturation_ratio"),
                "max_align_rejacobian_count": LaunchConfiguration("max_align_rejacobian_count"),
                "align_rejacobian_cooldown_sec": LaunchConfiguration("align_rejacobian_cooldown_sec"),
                "align_rejacobian_after_approach_steps": LaunchConfiguration("align_rejacobian_after_approach_steps"),
                "approach_center_threshold": LaunchConfiguration("approach_center_threshold"),

                "desired_area_norm": LaunchConfiguration("desired_area_norm"),
                "area_done_center_threshold": LaunchConfiguration("area_done_center_threshold"),

                "approach_step_deg": LaunchConfiguration("approach_step_deg"),
                "approach_wait_sec": LaunchConfiguration("approach_wait_sec"),
                "max_approach_steps": LaunchConfiguration("max_approach_steps"),
                "area_jacobian_delta_deg": LaunchConfiguration("area_jacobian_delta_deg"),
                "area_jacobian_settle_sec": LaunchConfiguration("area_jacobian_settle_sec"),
                "area_window_size": LaunchConfiguration("area_window_size"),
                "area_jacobian_min_grad": LaunchConfiguration("area_jacobian_min_grad"),
                "area_jacobian_reuse_steps": LaunchConfiguration("area_jacobian_reuse_steps"),
                "area_min_gain_for_reuse": LaunchConfiguration("area_min_gain_for_reuse"),
                "area_drop_tolerance": LaunchConfiguration("area_drop_tolerance"),

                "max_total_steps": LaunchConfiguration("max_total_steps"),
                "hard_stop_below_center_error": LaunchConfiguration("hard_stop_below_center_error"),
                "filter_alpha": LaunchConfiguration("filter_alpha"),
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    return LaunchDescription(args + [node])