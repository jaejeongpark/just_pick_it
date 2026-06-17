#!/usr/bin/env python3
"""PICKY2 reverse docking.

Docking is split from normal Nav2 driving:
  - RETURN_HOME moves the robot to the STANDBY zone with Nav2.
  - DOCK_IN uses this node to align to the AprilTag/ArUco marker and
    reverse slowly into the charging dock.

This MVP uses the marker for alignment and stop distance. During reverse
insertion, the blue guide lane can add a small yaw correction by comparing the
lane centerline against the marker-normal axis. The current physical marker is
on the horizontal wall above the standby zones, so the marker is a local
alignment reference, not the final stopping target.
"""

import math
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image


def quat_to_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class ReverseDocking(Node):
    def __init__(self):
        super().__init__("reverse_docking")

        # AprilTag/ArUco marker. The physical tags are shared with PICKY1.
        self.declare_parameter("aruco_marker_dict", "DICT_APRILTAG_36h11")
        self.declare_parameter("marker_size_m", 0.05)

        # PICKY2 camera calibration, 1280x720.
        self.declare_parameter("camera_matrix", [
            1564.8861778174316, 0.0, 634.6279547011992,
            0.0, 1557.9751956153086, 386.3538829840184,
            0.0, 0.0, 1.0,
        ])
        self.declare_parameter("dist_coeffs", [
            -0.03862007925270209,
            1.9953590527035414,
            -0.004035341465584413,
            0.007809686398255311,
            -10.752768150812287,
        ])

        # Camera is off during normal driving. For docking, open it directly.
        # Use "ros_topic" only when an external /camera/image_raw publisher is
        # intentionally running for bench tests.
        self.declare_parameter("camera_source", "picamera2")
        self.declare_parameter("camera_topic", "camera/image_raw")
        self.declare_parameter("camera_width", 1280)
        self.declare_parameter("camera_height", 720)
        self.declare_parameter("flip_camera_180", False)
        self.declare_parameter("camera_warmup_sec", 1.5)

        # Debug image is published only while docking phases request marker
        # detection. It shows the exact frame used by this controller.
        self.declare_parameter("debug_publish_image", True)
        self.declare_parameter("debug_image_topic", "docking/debug_image")
        self.declare_parameter("debug_image_period_sec", 0.05)
        self.declare_parameter("debug_save_failure_images", True)
        self.declare_parameter("debug_save_dir", "~/just_pick_it/bags/docking_debug")
        self.declare_parameter("debug_save_frame_count", 12)

        # Blue guide lane debug/control. It overlays the marker-normal docking
        # axis and the detected lane pair; reverse insertion may use the
        # marker-axis vs lane-centerline error as a yaw correction.
        self.declare_parameter("lane_debug_enabled", True)
        self.declare_parameter("lane_roi_y_min_ratio", 0.52)
        self.declare_parameter("lane_hsv_lower", [75, 35, 40])
        self.declare_parameter("lane_hsv_upper", [105, 255, 255])
        self.declare_parameter("lane_min_area_px", 500.0)
        self.declare_parameter("lane_morph_kernel_px", 5)
        self.declare_parameter("lane_min_pair_separation_px", 160.0)
        self.declare_parameter("lane_min_centerline_length_px", 40.0)

        # Phase timing and motion.
        self.declare_parameter("marker_timeout_sec", 15.0)
        self.declare_parameter("align_timeout_sec", 10.0)
        self.declare_parameter("reverse_timeout_sec", 20.0)
        self.declare_parameter("odom_timeout_sec", 5.0)
        self.declare_parameter("settle_sec", 0.3)
        self.declare_parameter("marker_search_step_sec", 0.30)
        self.declare_parameter("marker_search_settle_sec", 0.20)
        self.declare_parameter("align_rotation_step_sec", 0.18)
        self.declare_parameter("marker_search_hint_timeout_sec", 6.0)
        self.declare_parameter("marker_0_default_search_direction", 1.0)
        self.declare_parameter("marker_1_default_search_direction", -1.0)

        self.declare_parameter("acquire_rotate_speed", 0.25)
        self.declare_parameter("align_lateral_tolerance_m", 0.03)
        self.declare_parameter("align_stable_frames", 2)
        self.declare_parameter("align_kp", 1.2)
        self.declare_parameter("align_reacquire_rotate_speed", 0.15)
        self.declare_parameter("reverse_speed", 0.035)
        self.declare_parameter("reverse_stop_mode", "marker_distance")
        self.declare_parameter("reverse_target_marker_distance_m", 0.46)
        self.declare_parameter("reverse_marker_distance_tolerance_m", 0.02)
        self.declare_parameter("reverse_marker_lost_timeout_sec", 0.35)
        self.declare_parameter("reverse_distance_m", 0.32)
        self.declare_parameter("reverse_success_max_marker_lat_m", 0.05)
        self.declare_parameter("reverse_success_max_axis_error_px", 180.0)
        self.declare_parameter("yaw_hold_kp", 1.2)
        self.declare_parameter("reverse_lane_axis_enabled", True)
        self.declare_parameter("reverse_lane_axis_kp", 0.0025)
        self.declare_parameter("reverse_lane_axis_angle_kp", 0.0)
        self.declare_parameter("reverse_lane_axis_max_correction", 0.22)
        self.declare_parameter("reverse_lane_axis_max_age_sec", 1.5)
        self.declare_parameter("reverse_lane_axis_control_max_error_px", 350.0)
        self.declare_parameter("reverse_marker_progress_timeout_sec", 2.0)
        self.declare_parameter("reverse_marker_progress_min_delta_m", 0.015)
        self.declare_parameter("reverse_retry_count", 1)
        self.declare_parameter("retry_escape_distance_m", 0.08)
        self.declare_parameter("retry_escape_speed", 0.035)
        self.declare_parameter("retry_escape_timeout_sec", 4.0)
        # Default 0 keeps reverse insertion odom-yaw based. Increase only after
        # logs show consistent lateral drift while reversing.
        self.declare_parameter("reverse_marker_lat_kp", 0.0)
        self.declare_parameter("max_angular_vel", 0.35)

        dict_id = self._resolve_aruco_dict(
            self.get_parameter("aruco_marker_dict").value
        )
        self._marker_size = float(self.get_parameter("marker_size_m").value)
        self._cam_matrix = np.array(
            self.get_parameter("camera_matrix").value,
            dtype=np.float64,
        ).reshape(3, 3)
        self._dist_coeffs = np.array(
            self.get_parameter("dist_coeffs").value,
            dtype=np.float64,
        )

        self._camera_source = str(self.get_parameter("camera_source").value)
        self._camera_width = int(self.get_parameter("camera_width").value)
        self._camera_height = int(self.get_parameter("camera_height").value)
        self._flip_180 = bool(self.get_parameter("flip_camera_180").value)
        self._camera_warmup_sec = max(
            0.0,
            float(self.get_parameter("camera_warmup_sec").value),
        )
        self._debug_publish_image = bool(
            self.get_parameter("debug_publish_image").value
        )
        self._debug_image_topic = str(
            self.get_parameter("debug_image_topic").value
        )
        self._debug_image_period = max(
            0.05,
            float(self.get_parameter("debug_image_period_sec").value),
        )
        self._debug_save_failure_images = bool(
            self.get_parameter("debug_save_failure_images").value
        )
        self._debug_save_dir = str(
            self.get_parameter("debug_save_dir").value
        )
        self._debug_save_frame_count = max(
            1,
            int(self.get_parameter("debug_save_frame_count").value),
        )
        self._debug_enabled = (
            self._debug_publish_image or self._debug_save_failure_images
        )
        self._lane_debug_enabled = bool(
            self.get_parameter("lane_debug_enabled").value
        )
        self._lane_roi_y_min_ratio = float(
            self.get_parameter("lane_roi_y_min_ratio").value
        )
        self._lane_hsv_lower = np.array(
            self.get_parameter("lane_hsv_lower").value,
            dtype=np.uint8,
        )
        self._lane_hsv_upper = np.array(
            self.get_parameter("lane_hsv_upper").value,
            dtype=np.uint8,
        )
        self._lane_min_area = float(
            self.get_parameter("lane_min_area_px").value
        )
        self._lane_morph_kernel = max(
            1,
            int(self.get_parameter("lane_morph_kernel_px").value),
        )
        self._lane_min_pair_separation = float(
            self.get_parameter("lane_min_pair_separation_px").value
        )
        self._lane_min_centerline_length = float(
            self.get_parameter("lane_min_centerline_length_px").value
        )

        self._marker_timeout = float(self.get_parameter("marker_timeout_sec").value)
        self._align_timeout = float(self.get_parameter("align_timeout_sec").value)
        self._reverse_timeout = float(self.get_parameter("reverse_timeout_sec").value)
        self._odom_timeout = float(self.get_parameter("odom_timeout_sec").value)
        self._settle_sec = float(self.get_parameter("settle_sec").value)
        self._marker_search_step_sec = max(
            0.0,
            float(self.get_parameter("marker_search_step_sec").value),
        )
        self._marker_search_settle_sec = max(
            0.0,
            float(self.get_parameter("marker_search_settle_sec").value),
        )
        self._align_rotation_step_sec = max(
            0.0,
            float(self.get_parameter("align_rotation_step_sec").value),
        )
        self._marker_search_hint_timeout = max(
            0.0,
            float(self.get_parameter("marker_search_hint_timeout_sec").value),
        )
        self._marker_search_directions = {
            0: float(
                self.get_parameter("marker_0_default_search_direction").value
            ),
            1: float(
                self.get_parameter("marker_1_default_search_direction").value
            ),
        }

        self._acquire_rotate_speed = float(
            self.get_parameter("acquire_rotate_speed").value
        )
        self._align_tol = float(self.get_parameter("align_lateral_tolerance_m").value)
        self._align_stable_frames = int(
            self.get_parameter("align_stable_frames").value
        )
        self._align_kp = float(self.get_parameter("align_kp").value)
        self._align_reacquire_rotate_speed = float(
            self.get_parameter("align_reacquire_rotate_speed").value
        )
        self._reverse_speed = float(self.get_parameter("reverse_speed").value)
        self._reverse_stop_mode = str(
            self.get_parameter("reverse_stop_mode").value
        ).strip().lower()
        if self._reverse_stop_mode not in ("marker_distance", "odom_distance"):
            self.get_logger().warn(
                "Unknown reverse_stop_mode="
                f"{self._reverse_stop_mode!r}; using marker_distance"
            )
            self._reverse_stop_mode = "marker_distance"
        self._reverse_target_marker_distance = float(
            self.get_parameter("reverse_target_marker_distance_m").value
        )
        self._reverse_marker_distance_tolerance = float(
            self.get_parameter("reverse_marker_distance_tolerance_m").value
        )
        self._reverse_marker_lost_timeout = float(
            self.get_parameter("reverse_marker_lost_timeout_sec").value
        )
        self._reverse_distance = float(
            self.get_parameter("reverse_distance_m").value
        )
        self._reverse_success_max_marker_lat = float(
            self.get_parameter("reverse_success_max_marker_lat_m").value
        )
        self._reverse_success_max_axis_error = float(
            self.get_parameter("reverse_success_max_axis_error_px").value
        )
        self._yaw_hold_kp = float(self.get_parameter("yaw_hold_kp").value)
        self._reverse_lane_axis_enabled = bool(
            self.get_parameter("reverse_lane_axis_enabled").value
        )
        self._reverse_lane_axis_kp = float(
            self.get_parameter("reverse_lane_axis_kp").value
        )
        self._reverse_lane_axis_angle_kp = float(
            self.get_parameter("reverse_lane_axis_angle_kp").value
        )
        self._reverse_lane_axis_max_correction = float(
            self.get_parameter("reverse_lane_axis_max_correction").value
        )
        self._reverse_lane_axis_max_age = float(
            self.get_parameter("reverse_lane_axis_max_age_sec").value
        )
        self._reverse_lane_axis_control_max_error = float(
            self.get_parameter("reverse_lane_axis_control_max_error_px").value
        )
        self._reverse_marker_progress_timeout = float(
            self.get_parameter("reverse_marker_progress_timeout_sec").value
        )
        self._reverse_marker_progress_min_delta = float(
            self.get_parameter("reverse_marker_progress_min_delta_m").value
        )
        self._reverse_retry_count = max(
            0,
            int(self.get_parameter("reverse_retry_count").value),
        )
        self._retry_escape_distance = float(
            self.get_parameter("retry_escape_distance_m").value
        )
        self._retry_escape_speed = float(
            self.get_parameter("retry_escape_speed").value
        )
        self._retry_escape_timeout = float(
            self.get_parameter("retry_escape_timeout_sec").value
        )
        self._reverse_marker_lat_kp = float(
            self.get_parameter("reverse_marker_lat_kp").value
        )
        self._max_ang = float(self.get_parameter("max_angular_vel").value)

        self._aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self._aruco_params = cv2.aruco.DetectorParameters()
        self._detector = None
        if hasattr(cv2.aruco, "ArucoDetector"):
            self._detector = cv2.aruco.ArucoDetector(
                self._aruco_dict,
                self._aruco_params,
            )

        h = self._marker_size / 2.0
        self._marker_obj_pts = np.array([
            [-h,  h, 0.0],
            [ h,  h, 0.0],
            [ h, -h, 0.0],
            [-h, -h, 0.0],
        ], dtype=np.float64)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_frame = None
        self._latest_odom = None
        self._cancel_requested = False
        self._picam2 = None
        self._debug_last_frame_time = 0.0
        self._debug_frames = deque(maxlen=self._debug_save_frame_count)
        self._debug_image_pub = None
        self._last_lane_axis_error_px = None
        self._last_lane_axis_angle_error_deg = None
        self._last_lane_axis_error_time = 0.0
        self._last_seen_marker_id = None
        self._last_seen_marker_time = 0.0
        self._last_target_marker_id = None
        self._last_target_lat_err = None
        self._last_target_seen_time = 0.0

        if self._camera_source == "ros_topic":
            cam_topic = self.get_parameter("camera_topic").value
            self.create_subscription(Image, cam_topic, self._image_cb, 10)

        self.create_subscription(Odometry, "odom", self._odom_cb, 20)

        self._cmd_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self._init_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            "initialpose",
            10,
        )
        if self._debug_publish_image:
            self._debug_image_pub = self.create_publisher(
                Image,
                self._debug_image_topic,
                2,
            )

        self.get_logger().info(
            "ReverseDocking ready "
            f"(camera_source={self._camera_source}, "
            f"marker_size={self._marker_size:.3f}m, "
            f"debug_image={self._debug_publish_image}, "
            f"lane_debug={self._lane_debug_enabled})"
        )

    # ------------------------------------------------------------------ #
    # Public interface
    # ------------------------------------------------------------------ #

    def reverse_dock(
        self,
        marker_id: int,
        dock_map_x: float,
        dock_map_y: float,
        dock_map_yaw: float,
    ) -> bool:
        """Align to marker, reverse into the dock, and correct AMCL pose."""
        self.get_logger().info(
            f"reverse_dock: marker={marker_id}, "
            f"target=({dock_map_x:.3f}, {dock_map_y:.3f}, "
            f"{math.degrees(dock_map_yaw):.1f}deg)"
        )
        self._reset_debug_frames()

        if not self._start_camera():
            self._stop()
            return False

        try:
            if not self._wait_for_odom():
                self.get_logger().error("reverse_dock: FAILED at WAIT_ODOM")
                self._save_debug_failure_images("WAIT_ODOM")
                return False

            for attempt in range(self._reverse_retry_count + 1):
                if attempt > 0:
                    self.get_logger().info(
                        "reverse_dock: retrying docking "
                        f"attempt={attempt + 1}/{self._reverse_retry_count + 1}"
                    )

                phases = [
                    ("OBSERVE_MARKER", lambda: self._phase_observe_marker(marker_id)),
                    (
                        "ALIGN_MARKER_CENTER",
                        lambda: self._phase_align_marker_center(marker_id),
                    ),
                    ("REVERSE_INSERT", lambda: self._phase_reverse_insert(marker_id)),
                ]
                failed_phase = None
                for name, fn in phases:
                    if not fn():
                        self._stop()
                        failed_phase = name
                        break

                if failed_phase is None:
                    self._stop()
                    time.sleep(self._settle_sec)
                    self._publish_pose_correction(
                        dock_map_x,
                        dock_map_y,
                        dock_map_yaw,
                    )
                    self.get_logger().info("reverse_dock: SUCCESS")
                    return True

                self.get_logger().error(f"reverse_dock: FAILED at {failed_phase}")
                self._save_debug_failure_images(failed_phase)
                if (
                    failed_phase != "REVERSE_INSERT"
                    or attempt >= self._reverse_retry_count
                ):
                    return False

                if not self._phase_retry_escape():
                    self.get_logger().error("reverse_dock: FAILED at RETRY_ESCAPE")
                    self._save_debug_failure_images("RETRY_ESCAPE")
                    return False

            return False
        finally:
            self._stop()
            self._stop_camera()

    def cancel(self) -> None:
        """Request cancellation; the active phase will stop on the next loop."""
        with self._lock:
            self._cancel_requested = True
        self._stop()

    def clear_cancel(self) -> None:
        with self._lock:
            self._cancel_requested = False

    # ------------------------------------------------------------------ #
    # Phases
    # ------------------------------------------------------------------ #

    def _phase_observe_marker(self, marker_id: int) -> bool:
        """Wait until the expected marker is visible."""
        deadline = time.time() + self._marker_timeout
        last_log = 0.0
        while time.time() < deadline:
            if self._is_cancel_requested():
                return False

            result = self._detect_from_latest_frame(marker_id, "OBSERVE_MARKER")
            if result is None:
                hint_w = self._marker_search_hint(marker_id)
                twist = Twist()
                if hint_w is not None:
                    twist.angular.z = hint_w
                    reason = f"wrong_marker={self._last_seen_marker_id}"
                else:
                    twist.angular.z = self._default_marker_search_cmd(marker_id)
                    reason = "scan"

                now = time.time()
                if now - last_log > 1.0:
                    self.get_logger().info(
                        "OBSERVE_MARKER: searching target "
                        f"id={marker_id}, cmd_w={twist.angular.z:.3f}, "
                        f"reason={reason}, "
                        f"step={self._marker_search_step_sec:.2f}s"
                    )
                    last_log = now
                self._rotate_step(
                    twist.angular.z,
                    self._marker_search_step_sec,
                    self._marker_search_settle_sec,
                )
                continue

            tvec, rvec, center = result
            self._stop()
            time.sleep(self._settle_sec)
            self.get_logger().info(
                "OBSERVE_MARKER: "
                f"id={marker_id}, center=({center[0]:.1f},{center[1]:.1f}), "
                f"tvec=({tvec[0]:.3f},{tvec[1]:.3f},{tvec[2]:.3f}), "
                f"rvec=({rvec[0]:.3f},{rvec[1]:.3f},{rvec[2]:.3f})"
            )
            return True

        self.get_logger().warn(f"OBSERVE_MARKER: timeout waiting marker_id={marker_id}")
        return False

    def _phase_align_marker_center(self, marker_id: int) -> bool:
        """Rotate in place until the marker is centered laterally."""
        deadline = time.time() + self._align_timeout
        stable = 0
        last_log = 0.0
        had_target = False
        last_cmd_w = 0.0

        while time.time() < deadline:
            if self._is_cancel_requested():
                return False

            result = self._detect_from_latest_frame(marker_id, "ALIGN_MARKER_CENTER")
            if result is None:
                twist = Twist()
                target_w = self._recent_target_alignment_cmd(marker_id)
                hint_w = self._marker_search_hint(marker_id)
                if target_w is not None:
                    twist.angular.z = target_w
                    reason = "recent_target"
                elif hint_w is not None:
                    twist.angular.z = hint_w
                    reason = f"wrong_marker={self._last_seen_marker_id}"
                elif had_target and abs(last_cmd_w) > 1e-6:
                    twist.angular.z = -math.copysign(
                        abs(self._align_reacquire_rotate_speed),
                        last_cmd_w,
                    )
                    reason = "overshoot"
                else:
                    twist.angular.z = self._default_marker_search_cmd(marker_id)
                    reason = "scan"
                stable = 0

                now = time.time()
                if now - last_log > 1.0:
                    self.get_logger().info(
                        "ALIGN_MARKER_CENTER: target lost, "
                        f"reacquire_cmd_w={twist.angular.z:.3f}, "
                        f"reason={reason}, "
                        f"step={self._align_rotation_step_sec:.2f}s"
                    )
                    last_log = now
                self._rotate_step(
                    twist.angular.z,
                    self._align_rotation_step_sec,
                    self._marker_search_settle_sec,
                )
                continue

            tvec, _, center = result
            had_target = True
            lat_err = float(tvec[0])

            if abs(lat_err) <= self._align_tol:
                stable += 1
                self._stop()
                if stable >= self._align_stable_frames:
                    self.get_logger().info(
                        "ALIGN_MARKER_CENTER: done "
                        f"lat_err={lat_err:.3f}m, center_x={center[0]:.1f}"
                    )
                    return True
                time.sleep(0.05)
                continue

            stable = 0
            twist = Twist()
            # If the marker is to the camera's right, rotate clockwise.
            cmd_w = -self._align_kp * lat_err
            if abs(cmd_w) < self._align_reacquire_rotate_speed:
                cmd_w = math.copysign(self._align_reacquire_rotate_speed, cmd_w)
            twist.angular.z = self._clamp(cmd_w)
            last_cmd_w = twist.angular.z

            now = time.time()
            if now - last_log > 1.0:
                self.get_logger().info(
                    "ALIGN_MARKER_CENTER: "
                    f"lat_err={lat_err:.3f}m, "
                    f"cmd_w={twist.angular.z:.3f}, "
                    f"step={self._align_rotation_step_sec:.2f}s"
                )
                last_log = now
            self._rotate_step(
                twist.angular.z,
                self._align_rotation_step_sec,
                self._marker_search_settle_sec,
            )

        self._stop()
        self.get_logger().warn("ALIGN_MARKER_CENTER: timeout")
        return False

    def _phase_reverse_insert(self, marker_id: int) -> bool:
        """Reverse while holding yaw, stopping by marker distance or odom."""
        start = self._get_odom_pose()
        if start is None:
            self.get_logger().warn("REVERSE_INSERT: no odom")
            return False

        start_x, start_y, theta_ref = start
        deadline = time.time() + self._reverse_timeout
        last_log = 0.0
        last_marker_time = time.time()
        last_marker_distance = None
        best_marker_distance = None
        last_marker_progress_time = time.time()
        use_marker_distance = self._reverse_stop_mode == "marker_distance"
        self._last_lane_axis_error_px = None
        self._last_lane_axis_angle_error_deg = None
        self._last_lane_axis_error_time = 0.0

        self.get_logger().info(
            "REVERSE_INSERT: start "
            f"odom=({start_x:.3f},{start_y:.3f},{math.degrees(theta_ref):.1f}deg), "
            f"stop_mode={self._reverse_stop_mode}, "
            f"target_marker_distance={self._reverse_target_marker_distance:.3f}m, "
            f"safety_odom_distance={self._reverse_distance:.3f}m"
        )

        while time.time() < deadline:
            if self._is_cancel_requested():
                return False

            cur = self._get_odom_pose()
            if cur is None:
                self._stop()
                self.get_logger().warn("REVERSE_INSERT: lost odom")
                return False

            x, y, yaw = cur
            moved = math.hypot(x - start_x, y - start_y)
            yaw_err = normalize_angle(theta_ref - yaw)
            marker_lat = 0.0
            lane_axis_error = None
            lane_axis_angle_error = None
            lane_axis_correction = 0.0
            lane_axis_used = False
            marker_lat_correction = 0.0
            if (
                use_marker_distance
                or self._reverse_marker_lat_kp != 0.0
                or self._reverse_lane_axis_enabled
                or self._debug_frame_due()
            ):
                result = self._detect_from_latest_frame(marker_id, "REVERSE_INSERT")
                if result is not None:
                    tvec = result[0]
                    marker_lat = float(tvec[0])
                    marker_distance = float(tvec[2])
                    last_marker_distance = marker_distance
                    last_marker_time = time.time()
                    if best_marker_distance is None:
                        best_marker_distance = marker_distance
                        last_marker_progress_time = time.time()
                    elif (
                        marker_distance
                        >= best_marker_distance + self._reverse_marker_progress_min_delta
                    ):
                        best_marker_distance = marker_distance
                        last_marker_progress_time = time.time()

                    if (
                        use_marker_distance
                        and marker_distance
                        >= (
                            self._reverse_target_marker_distance
                            - self._reverse_marker_distance_tolerance
                        )
                    ):
                        success_axis_error, _ = self._recent_lane_axis_error()
                        if moved >= self._reverse_distance:
                            self._stop()
                            self.get_logger().warn(
                                "REVERSE_INSERT: safety odom distance reached "
                                "before aligned marker-distance success "
                                f"(moved={moved:.3f}/"
                                f"{self._reverse_distance:.3f}m, "
                                f"marker_z={marker_distance:.3f}/"
                                f"{self._reverse_target_marker_distance:.3f}m)"
                            )
                            return False
                        if not self._reverse_success_alignment_ok(
                            marker_lat,
                            success_axis_error,
                        ):
                            self._stop()
                            axis_text = (
                                f"{success_axis_error:.1f}px"
                                if success_axis_error is not None
                                else "none"
                            )
                            self.get_logger().warn(
                                "REVERSE_INSERT: marker distance reached but "
                                "alignment is not acceptable "
                                f"(marker_lat={marker_lat:.3f}/"
                                f"{self._reverse_success_max_marker_lat:.3f}m, "
                                f"axis_err={axis_text}, "
                                f"marker_z={marker_distance:.3f}/"
                                f"{self._reverse_target_marker_distance:.3f}m)"
                            )
                            return False
                        self._stop()
                        self.get_logger().info(
                            "REVERSE_INSERT: done by marker distance "
                            f"moved={moved:.3f}m, "
                            f"marker_z={marker_distance:.3f}/"
                            f"{self._reverse_target_marker_distance:.3f}m, "
                            f"yaw={math.degrees(yaw):.1f}deg"
                        )
                        return True
                elif (
                    use_marker_distance
                    and time.time() - last_marker_time
                    >= self._reverse_marker_lost_timeout
                ):
                    self._stop()
                    last_marker_text = (
                        f"{last_marker_distance:.3f}m"
                        if last_marker_distance is not None
                        else "none"
                    )
                    self.get_logger().warn(
                        "REVERSE_INSERT: marker lost while using marker "
                        f"distance stop (last_marker_z={last_marker_text})"
                    )
                    return False
                if (
                    use_marker_distance
                    and best_marker_distance is not None
                    and self._reverse_marker_progress_timeout > 0.0
                    and time.time() - last_marker_progress_time
                    >= self._reverse_marker_progress_timeout
                ):
                    self._stop()
                    self.get_logger().warn(
                        "REVERSE_INSERT: marker distance is not progressing "
                        f"(best_marker_z={best_marker_distance:.3f}m, "
                        f"last_marker_z={last_marker_distance:.3f}m, "
                        f"timeout={self._reverse_marker_progress_timeout:.1f}s)"
                    )
                    return False

            if self._reverse_lane_axis_enabled:
                lane_axis_error, lane_axis_angle_error = self._recent_lane_axis_error()
                if lane_axis_error is not None:
                    if abs(lane_axis_error) <= self._reverse_lane_axis_control_max_error:
                        lane_axis_used = True
                    else:
                        self.get_logger().warn(
                            "REVERSE_INSERT: lane axis ignored for control "
                            f"(axis_err={lane_axis_error:.1f}px > "
                            f"{self._reverse_lane_axis_control_max_error:.1f}px)"
                        )

                if lane_axis_used:
                    lane_axis_raw_correction = (
                        -self._reverse_lane_axis_kp * lane_axis_error
                    )
                    if lane_axis_angle_error is not None:
                        lane_axis_raw_correction += (
                            self._reverse_lane_axis_angle_kp
                            * lane_axis_angle_error
                        )
                    lane_axis_correction = self._clamp_abs(
                        lane_axis_raw_correction,
                        self._reverse_lane_axis_max_correction,
                    )

            if not lane_axis_used:
                marker_lat_correction = -self._reverse_marker_lat_kp * marker_lat

            if moved >= self._reverse_distance:
                self._stop()
                if use_marker_distance:
                    last_marker_text = (
                        f"{last_marker_distance:.3f}m"
                        if last_marker_distance is not None
                        else "none"
                    )
                    self.get_logger().warn(
                        "REVERSE_INSERT: safety odom distance reached before "
                        "marker target "
                        f"(moved={moved:.3f}/{self._reverse_distance:.3f}m, "
                        f"last_marker_z={last_marker_text})"
                    )
                    return False
                self.get_logger().info(
                    "REVERSE_INSERT: done by odom distance "
                    f"moved={moved:.3f}m, yaw={math.degrees(yaw):.1f}deg"
                )
                return True

            twist = Twist()
            twist.linear.x = -self._reverse_speed
            twist.angular.z = self._clamp(
                self._yaw_hold_kp * yaw_err
                + marker_lat_correction
                + lane_axis_correction
            )
            self._cmd_pub.publish(twist)

            now = time.time()
            if now - last_log > 1.0:
                marker_text = ""
                if use_marker_distance:
                    marker_text = (
                        f", marker_z={last_marker_distance:.3f}/"
                        f"{self._reverse_target_marker_distance:.3f}m"
                        if last_marker_distance is not None
                        else ", marker_z=none"
                    )
                lane_text = (
                    f", axis_err={lane_axis_error:.1f}px"
                    + (
                        f", axis_angle={lane_axis_angle_error:.1f}deg"
                        if lane_axis_angle_error is not None
                        else ""
                    )
                    + f", axis_cmd={lane_axis_correction:.3f}"
                    + f", axis_used={lane_axis_used}"
                    if lane_axis_error is not None
                    else ""
                )
                marker_lat_text = (
                    f", marker_lat={marker_lat:.3f}m,"
                    f" marker_lat_cmd={marker_lat_correction:.3f}"
                    if self._reverse_marker_lat_kp != 0.0
                    else ""
                )
                self.get_logger().info(
                    "REVERSE_INSERT: "
                    f"moved={moved:.3f}/{self._reverse_distance:.3f}m, "
                    f"yaw_err={math.degrees(yaw_err):.1f}deg, "
                    f"cmd=({twist.linear.x:.3f},{twist.angular.z:.3f})"
                    f"{marker_text}"
                    f"{lane_text}"
                    f"{marker_lat_text}"
                )
                last_log = now
            time.sleep(0.05)

        self._stop()
        self.get_logger().warn("REVERSE_INSERT: timeout")
        return False

    def _phase_retry_escape(self) -> bool:
        """Move forward a little before retrying marker alignment."""
        if self._retry_escape_distance <= 0.0:
            self.get_logger().info("RETRY_ESCAPE: skipped")
            return True

        start = self._get_odom_pose()
        if start is None:
            self.get_logger().warn("RETRY_ESCAPE: no odom")
            return False

        start_x, start_y, theta_ref = start
        deadline = time.time() + self._retry_escape_timeout
        last_log = 0.0
        self.get_logger().info(
            "RETRY_ESCAPE: start "
            f"distance={self._retry_escape_distance:.3f}m, "
            f"speed={self._retry_escape_speed:.3f}m/s"
        )

        while time.time() < deadline:
            if self._is_cancel_requested():
                return False

            cur = self._get_odom_pose()
            if cur is None:
                self._stop()
                self.get_logger().warn("RETRY_ESCAPE: lost odom")
                return False

            x, y, yaw = cur
            moved = math.hypot(x - start_x, y - start_y)
            if moved >= self._retry_escape_distance:
                self._stop()
                time.sleep(self._settle_sec)
                self.get_logger().info(
                    f"RETRY_ESCAPE: done moved={moved:.3f}m"
                )
                return True

            yaw_err = normalize_angle(theta_ref - yaw)
            twist = Twist()
            twist.linear.x = abs(self._retry_escape_speed)
            twist.angular.z = self._clamp(self._yaw_hold_kp * yaw_err)
            self._cmd_pub.publish(twist)

            now = time.time()
            if now - last_log > 1.0:
                self.get_logger().info(
                    "RETRY_ESCAPE: "
                    f"moved={moved:.3f}/{self._retry_escape_distance:.3f}m, "
                    f"cmd=({twist.linear.x:.3f},{twist.angular.z:.3f})"
                )
                last_log = now
            time.sleep(0.05)

        self._stop()
        self.get_logger().warn("RETRY_ESCAPE: timeout")
        return False

    def _recent_lane_axis_error(self):
        axis_age = time.time() - self._last_lane_axis_error_time
        if (
            self._last_lane_axis_error_px is not None
            and axis_age <= self._reverse_lane_axis_max_age
        ):
            return (
                self._last_lane_axis_error_px,
                self._last_lane_axis_angle_error_deg,
            )
        return None, None

    def _reverse_success_alignment_ok(
        self,
        marker_lat: float,
        lane_axis_error,
    ) -> bool:
        if abs(marker_lat) > self._reverse_success_max_marker_lat:
            return False
        if lane_axis_error is None:
            return True
        return abs(lane_axis_error) <= self._reverse_success_max_axis_error

    # ------------------------------------------------------------------ #
    # Detection and camera
    # ------------------------------------------------------------------ #

    def _detect_from_latest_frame(self, marker_id: int, phase: str):
        frame = self._get_latest_frame()
        if frame is None:
            return None
        return self._detect_aruco(frame, marker_id, phase)

    def _detect_aruco(self, frame, target_id: int, phase: str):
        """Return (tvec, rvec, center_px) for the target marker, or None."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray,
                self._aruco_dict,
                parameters=self._aruco_params,
            )
        if ids is None:
            self._record_debug_frame(
                frame,
                phase,
                target_id,
                corners=[],
                ids=None,
                detection=None,
                status="marker not detected",
            )
            return None

        for i, marker_id in enumerate(ids.flatten()):
            if int(marker_id) != int(target_id):
                continue
            self._remember_visible_marker(int(marker_id))
            marker_corners = corners[i][0].astype(np.float64)
            ok, rvec, tvec = cv2.solvePnP(
                self._marker_obj_pts,
                marker_corners,
                self._cam_matrix,
                self._dist_coeffs,
            )
            if not ok:
                self._record_debug_frame(
                    frame,
                    phase,
                    target_id,
                    corners=corners,
                    ids=ids,
                    detection=None,
                    status="solvePnP failed",
                )
                return None
            center = marker_corners.mean(axis=0)
            detection = (tvec.flatten(), rvec.flatten(), center)
            self._remember_target_detection(int(marker_id), float(detection[0][0]))
            alignment = None
            if phase == "REVERSE_INSERT" and self._reverse_lane_axis_enabled:
                alignment = self._update_lane_axis_error(
                    frame,
                    corners,
                    ids,
                    target_id,
                )
            self._record_debug_frame(
                frame,
                phase,
                target_id,
                corners=corners,
                ids=ids,
                detection=detection,
                status="target detected",
                alignment=alignment,
            )
            return detection

        visible_ids = [int(marker_id) for marker_id in ids.flatten()]
        if visible_ids:
            self._remember_visible_marker(visible_ids[0])
        self._record_debug_frame(
            frame,
            phase,
            target_id,
            corners=corners,
            ids=ids,
            detection=None,
            status="target id not found",
        )
        return None

    def _remember_visible_marker(self, marker_id: int) -> None:
        self._last_seen_marker_id = int(marker_id)
        self._last_seen_marker_time = time.time()

    def _remember_target_detection(self, marker_id: int, lat_err: float) -> None:
        self._last_target_marker_id = int(marker_id)
        self._last_target_lat_err = float(lat_err)
        self._last_target_seen_time = time.time()

    def _recent_target_alignment_cmd(self, target_id: int):
        """Center using the last target pose before falling back to scan."""
        if self._last_target_marker_id != int(target_id):
            return None
        if time.time() - self._last_target_seen_time > 1.0:
            return None
        if self._last_target_lat_err is None:
            return None

        lat_err = float(self._last_target_lat_err)
        if abs(lat_err) <= self._align_tol:
            return 0.0

        cmd = -self._align_kp * lat_err
        if abs(cmd) < self._align_reacquire_rotate_speed:
            cmd = math.copysign(self._align_reacquire_rotate_speed, cmd)
        return self._clamp(cmd)

    def _marker_search_hint(self, target_id: int):
        """Use the other standby marker as a direction hint while searching."""
        if time.time() - self._last_seen_marker_time > self._marker_search_hint_timeout:
            return None

        seen_id = self._last_seen_marker_id
        if seen_id is None:
            return None
        speed = abs(self._acquire_rotate_speed)
        if int(target_id) == 1 and int(seen_id) == 0:
            return -speed
        if int(target_id) == 0 and int(seen_id) == 1:
            return speed
        return None

    def _default_marker_search_cmd(self, target_id: int) -> float:
        """Return the preferred blind scan direction for the target dock marker."""
        direction = self._marker_search_directions.get(int(target_id), 1.0)
        if abs(direction) < 1e-6:
            direction = 1.0
        return math.copysign(abs(self._acquire_rotate_speed), direction)

    def _start_camera(self) -> bool:
        with self._lock:
            self._latest_frame = None

        if self._camera_source == "ros_topic":
            self.get_logger().info("Camera source: ros_topic")
            return True

        if self._camera_source != "picamera2":
            self.get_logger().error(f"Unknown camera_source={self._camera_source!r}")
            return False

        if self._picam2 is not None:
            return True

        try:
            from picamera2 import Picamera2

            self._picam2 = Picamera2()
            config = self._picam2.create_video_configuration(
                main={
                    "size": (self._camera_width, self._camera_height),
                    "format": "RGB888",
                }
            )
            self._picam2.configure(config)
            self._picam2.start()
            self._warmup_camera()
            self.get_logger().info(
                "Camera opened for docking "
                f"({self._camera_width}x{self._camera_height})"
            )
            return True
        except Exception as exc:
            self.get_logger().error(f"Failed to open Picamera2: {exc}")
            self._stop_camera()
            return False

    def _stop_camera(self) -> None:
        if self._picam2 is None:
            return
        try:
            self._picam2.stop()
        except Exception:
            pass
        try:
            self._picam2.close()
        except Exception:
            pass
        self._picam2 = None
        self.get_logger().info("Camera closed after docking")

    # ------------------------------------------------------------------ #
    # ROS callbacks and pose helpers
    # ------------------------------------------------------------------ #

    def _image_cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if self._flip_180:
                frame = cv2.flip(frame, -1)
            with self._lock:
                self._latest_frame = frame
        except Exception as exc:
            self.get_logger().warn(f"Image conversion error: {exc}")

    def _odom_cb(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        with self._lock:
            self._latest_odom = (
                float(pose.position.x),
                float(pose.position.y),
                quat_to_yaw(pose.orientation),
                time.time(),
            )

    def _get_latest_frame(self):
        if self._camera_source == "picamera2":
            if self._picam2 is None:
                return None
            try:
                frame = self._picam2.capture_array()
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                if self._flip_180:
                    frame = cv2.flip(frame, -1)
                return frame
            except Exception as exc:
                self.get_logger().warn(f"Camera capture failed: {exc}")
                return None

        with self._lock:
            return self._latest_frame.copy() if self._latest_frame is not None else None

    def _wait_for_odom(self) -> bool:
        deadline = time.time() + self._odom_timeout
        while time.time() < deadline:
            if self._is_cancel_requested():
                return False
            if self._get_odom_pose() is not None:
                return True
            time.sleep(0.05)
        return False

    def _get_odom_pose(self):
        with self._lock:
            if self._latest_odom is None:
                return None
            x, y, yaw, stamp = self._latest_odom
        if time.time() - stamp > 1.0:
            return None
        return x, y, yaw

    def _is_cancel_requested(self) -> bool:
        with self._lock:
            return self._cancel_requested

    # ------------------------------------------------------------------ #
    # Pose correction and utilities
    # ------------------------------------------------------------------ #

    def _publish_pose_correction(self, x: float, y: float, yaw: float) -> None:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        half = yaw / 2.0
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)
        msg.pose.covariance[0] = 0.01
        msg.pose.covariance[7] = 0.01
        msg.pose.covariance[35] = 0.005
        self._init_pose_pub.publish(msg)
        self.get_logger().info(
            f"Pose correction: ({x:.3f}, {y:.3f}, {math.degrees(yaw):.1f}deg)"
        )

    def _resolve_aruco_dict(self, value) -> int:
        if isinstance(value, str):
            if not hasattr(cv2.aruco, value):
                raise ValueError(f"Unknown cv2.aruco dictionary: {value}")
            return int(getattr(cv2.aruco, value))
        return int(value)

    def _clamp(self, val: float) -> float:
        return max(min(val, self._max_ang), -self._max_ang)

    def _clamp_abs(self, val: float, limit: float) -> float:
        limit = abs(limit)
        return max(min(val, limit), -limit)

    def _stop(self) -> None:
        self._cmd_pub.publish(Twist())

    def _rotate_step(self, angular_z: float, duration: float, settle: float) -> None:
        """Rotate briefly, then stop before the next camera frame is evaluated."""
        duration = max(0.0, float(duration))
        settle = max(0.0, float(settle))
        angular_z = self._clamp(float(angular_z))

        if duration > 0.0 and abs(angular_z) > 1e-6:
            twist = Twist()
            twist.angular.z = angular_z
            self._cmd_pub.publish(twist)

            deadline = time.time() + duration
            while time.time() < deadline:
                if self._is_cancel_requested():
                    break
                time.sleep(min(0.05, max(0.0, deadline - time.time())))

        self._stop()
        if settle > 0.0 and not self._is_cancel_requested():
            time.sleep(settle)

    def _warmup_camera(self) -> None:
        if self._picam2 is None or self._camera_warmup_sec <= 0.0:
            return

        deadline = time.time() + self._camera_warmup_sec
        frames = 0
        while time.time() < deadline:
            try:
                self._picam2.capture_array()
                frames += 1
            except Exception:
                time.sleep(0.05)
        self.get_logger().info(
            "Camera warmup complete "
            f"(duration={self._camera_warmup_sec:.1f}s, discarded_frames={frames})"
        )

    # ------------------------------------------------------------------ #
    # Lane debug
    # ------------------------------------------------------------------ #

    def _detect_lane_center(self, frame):
        """Detect blue guide-lane components in the lower camera ROI."""
        if not self._lane_debug_enabled:
            return None

        height, width = frame.shape[:2]
        y0 = int(height * self._lane_roi_y_min_ratio)
        y0 = max(0, min(height - 1, y0))
        roi = frame[y0:height, :]
        if roi.size == 0:
            return None

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, self._lane_hsv_lower, self._lane_hsv_upper)

        if self._lane_morph_kernel > 1:
            kernel = np.ones(
                (self._lane_morph_kernel, self._lane_morph_kernel),
                dtype=np.uint8,
            )
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        components = []
        for contour in contours:
            component = self._lane_component_from_contour(contour, y0)
            if component is not None:
                components.append(component)

        components.sort(key=lambda item: item["area"], reverse=True)
        lane_center_point = None
        lane_centerline = None
        error_px = None
        pair_separation = None
        pair_valid = False
        selected = []
        if len(components) >= 2:
            # Use the leftmost and rightmost among the largest few components,
            # which is robust when a line is split into smaller pieces.
            candidates = components[:4]
            left = min(candidates, key=lambda item: item["cx"])
            right = max(candidates, key=lambda item: item["cx"])
            selected = [left, right]
            pair_separation = abs(right["cx"] - left["cx"])
            if pair_separation >= self._lane_min_pair_separation:
                lane_centerline = self._lane_centerline_from_pair(
                    left,
                    right,
                    y0,
                    width,
                    height,
                )
                if lane_centerline is not None:
                    pair_valid = True
                    lane_center_point = lane_centerline["control_point"]
                    error_px = lane_center_point[0] - (width / 2.0)
        elif len(components) == 1:
            selected = [components[0]]

        return {
            "roi_y0": y0,
            "components": components,
            "selected": selected,
            "pair_valid": pair_valid,
            "pair_separation": pair_separation,
            "lane_center_point": lane_center_point,
            "lane_centerline": lane_centerline,
            "error_px": error_px,
        }

    def _lane_component_from_contour(self, contour, roi_y0: int):
        area = float(cv2.contourArea(contour))
        if area < self._lane_min_area:
            return None

        moments = cv2.moments(contour)
        if moments["m00"] <= 0.0:
            return None

        shifted = contour.copy()
        shifted[:, 0, 1] += roi_y0
        x, y, w, h = cv2.boundingRect(shifted)
        return {
            "area": area,
            "cx": float(moments["m10"] / moments["m00"]),
            "cy": float(roi_y0 + moments["m01"] / moments["m00"]),
            "x_min": float(x),
            "x_max": float(x + w - 1),
            "y_min": float(y),
            "y_max": float(y + h - 1),
            "contour": shifted,
        }

    def _fit_lane_component_line(self, component):
        points = component["contour"].reshape(-1, 2).astype(np.float32)
        if len(points) < 2:
            return None

        vx, vy, x0, y0 = cv2.fitLine(
            points,
            cv2.DIST_L2,
            0,
            0.01,
            0.01,
        ).flatten()
        vx = float(vx)
        vy = float(vy)
        norm = math.hypot(vx, vy)
        if norm < 1e-6:
            return None

        vx /= norm
        vy /= norm
        if vy < 0.0:
            vx = -vx
            vy = -vy

        return {
            "vx": vx,
            "vy": vy,
            "x0": float(x0),
            "y0": float(y0),
        }

    def _line_x_at_y(self, line, y: float):
        if line is None or abs(line["vy"]) < 1e-6:
            return None
        t = (float(y) - line["y0"]) / line["vy"]
        return line["x0"] + t * line["vx"]

    def _lane_centerline_from_pair(
        self,
        left,
        right,
        roi_y0: int,
        width: int,
        height: int,
    ):
        left_line = self._fit_lane_component_line(left)
        right_line = self._fit_lane_component_line(right)
        if left_line is None or right_line is None:
            return None

        top_y = max(float(roi_y0), left["y_min"], right["y_min"])
        bottom_y = min(float(height - 1), left["y_max"], right["y_max"])
        if bottom_y - top_y < self._lane_min_centerline_length:
            mid_y = (left["cy"] + right["cy"]) / 2.0
            half = max(self._lane_min_centerline_length / 2.0, 30.0)
            top_y = max(float(roi_y0), mid_y - half)
            bottom_y = min(float(height - 1), mid_y + half)

        if bottom_y - top_y < 1.0:
            return None

        points = {}
        for key, y in (
            ("top", top_y),
            ("bottom", bottom_y),
            ("control", bottom_y),
        ):
            left_x = self._line_x_at_y(left_line, y)
            right_x = self._line_x_at_y(right_line, y)
            if left_x is None or right_x is None:
                return None
            if right_x < left_x:
                left_x, right_x = right_x, left_x
            center_x = (left_x + right_x) / 2.0
            points[key] = (center_x, y, left_x, right_x)

        top = np.array(points["top"][:2], dtype=np.float64)
        bottom = np.array(points["bottom"][:2], dtype=np.float64)
        direction = bottom - top
        direction_norm = float(np.linalg.norm(direction))
        if direction_norm < 1e-6:
            return None
        direction /= direction_norm
        if direction[1] < 0.0:
            direction = -direction
            top, bottom = bottom, top

        control_x, control_y, _, _ = points["control"]
        if control_x < -width or control_x > width * 2:
            return None

        return {
            "left_line": left_line,
            "right_line": right_line,
            "left_segment": (
                (points["top"][2], top_y),
                (points["bottom"][2], bottom_y),
            ),
            "right_segment": (
                (points["top"][3], top_y),
                (points["bottom"][3], bottom_y),
            ),
            "top": tuple(top),
            "bottom": tuple(bottom),
            "control_point": (float(control_x), float(control_y)),
            "direction": direction,
        }

    def _target_marker_corners(self, corners, ids, target_id: int):
        if ids is None or len(corners) == 0:
            return None

        for i, marker_id in enumerate(ids.flatten()):
            if int(marker_id) == int(target_id):
                return corners[i][0].astype(np.float64)
        return None

    def _marker_normal_axis_px(self, marker_corners):
        """Return a 2D image-space axis from marker center toward the dock lane."""
        if marker_corners is None or len(marker_corners) != 4:
            return None

        top_mid = (marker_corners[0] + marker_corners[1]) / 2.0
        bottom_mid = (marker_corners[2] + marker_corners[3]) / 2.0
        direction = bottom_mid - top_mid
        norm = float(np.linalg.norm(direction))
        if norm < 1e-6:
            top_edge = marker_corners[1] - marker_corners[0]
            direction = np.array([-top_edge[1], top_edge[0]], dtype=np.float64)
            norm = float(np.linalg.norm(direction))
            if norm < 1e-6:
                return None

        if direction[1] < 0.0:
            direction = -direction

        marker_center = marker_corners.mean(axis=0)
        return marker_center, direction / norm

    def _ray_end_in_image(self, origin, direction, width: int, height: int):
        candidates = []
        ox, oy = float(origin[0]), float(origin[1])
        dx, dy = float(direction[0]), float(direction[1])

        if abs(dx) > 1e-6:
            for x_edge in (0.0, float(width - 1)):
                t = (x_edge - ox) / dx
                y = oy + t * dy
                if t > 0.0 and 0.0 <= y <= height - 1:
                    candidates.append((t, np.array([x_edge, y], dtype=np.float64)))

        if abs(dy) > 1e-6:
            for y_edge in (0.0, float(height - 1)):
                t = (y_edge - oy) / dy
                x = ox + t * dx
                if t > 0.0 and 0.0 <= x <= width - 1:
                    candidates.append((t, np.array([x, y_edge], dtype=np.float64)))

        if not candidates:
            return origin + direction * float(height)
        return min(candidates, key=lambda item: item[0])[1]

    def _axis_error_px(self, point, axis_origin, axis_direction):
        point = np.array(point, dtype=np.float64)
        origin = np.array(axis_origin, dtype=np.float64)
        direction = np.array(axis_direction, dtype=np.float64)
        delta = point - origin
        along = float(np.dot(delta, direction))
        closest = origin + direction * along
        perp = np.array([-direction[1], direction[0]], dtype=np.float64)
        error = float(np.dot(delta, perp))
        return error, closest

    def _axis_angle_error_deg(self, lane_direction, axis_direction):
        lane = np.array(lane_direction, dtype=np.float64)
        axis = np.array(axis_direction, dtype=np.float64)
        lane_norm = float(np.linalg.norm(lane))
        axis_norm = float(np.linalg.norm(axis))
        if lane_norm < 1e-6 or axis_norm < 1e-6:
            return None

        lane /= lane_norm
        axis /= axis_norm
        if lane[1] < 0.0:
            lane = -lane
        if axis[1] < 0.0:
            axis = -axis

        dot = float(np.clip(np.dot(axis, lane), -1.0, 1.0))
        cross = float(axis[0] * lane[1] - axis[1] * lane[0])
        return math.degrees(math.atan2(cross, dot))

    def _compute_lane_axis_alignment(self, frame, corners, ids, target_id: int):
        lane = self._detect_lane_center(frame)
        axis = None
        axis_error = None
        axis_angle_error = None
        closest = None

        target_corners = self._target_marker_corners(corners, ids, target_id)
        marker_axis = self._marker_normal_axis_px(target_corners)
        if marker_axis is not None:
            axis_origin, axis_direction = marker_axis
            axis = (axis_origin, axis_direction)
            if lane is not None and lane["lane_center_point"] is not None:
                axis_error, closest = self._axis_error_px(
                    lane["lane_center_point"],
                    axis_origin,
                    axis_direction,
                )
                if lane["lane_centerline"] is not None:
                    axis_angle_error = self._axis_angle_error_deg(
                        lane["lane_centerline"]["direction"],
                        axis_direction,
                    )

        return {
            "lane": lane,
            "axis": axis,
            "axis_error": axis_error,
            "axis_angle_error": axis_angle_error,
            "closest": closest,
        }

    def _update_lane_axis_error(self, frame, corners, ids, target_id: int):
        alignment = self._compute_lane_axis_alignment(frame, corners, ids, target_id)
        axis_error = alignment["axis_error"]
        self._last_lane_axis_error_px = axis_error
        self._last_lane_axis_angle_error_deg = (
            alignment["axis_angle_error"] if axis_error is not None else None
        )
        self._last_lane_axis_error_time = time.time() if axis_error is not None else 0.0
        return alignment

    # ------------------------------------------------------------------ #
    # Debug image helpers
    # ------------------------------------------------------------------ #

    def _reset_debug_frames(self) -> None:
        if not self._debug_enabled:
            return
        with self._lock:
            self._debug_frames.clear()
        self._debug_last_frame_time = 0.0

    def _debug_frame_due(self) -> bool:
        return (
            self._debug_enabled
            and time.time() - self._debug_last_frame_time >= self._debug_image_period
        )

    def _record_debug_frame(
        self,
        frame,
        phase: str,
        target_id: int,
        corners,
        ids,
        detection,
        status: str,
        alignment=None,
    ) -> None:
        if not self._debug_enabled:
            return

        now = time.time()
        if now - self._debug_last_frame_time < self._debug_image_period:
            return
        self._debug_last_frame_time = now

        overlay = frame.copy()
        height, width = overlay.shape[:2]
        center_x = width // 2
        center_y = height // 2
        if alignment is None:
            alignment = self._compute_lane_axis_alignment(
                frame,
                corners,
                ids,
                target_id,
            )
        lane = alignment["lane"]

        cv2.line(overlay, (center_x, 0), (center_x, height), (0, 255, 255), 1)
        cv2.line(overlay, (0, center_y), (width, center_y), (80, 80, 80), 1)

        if lane is not None:
            roi_y0 = lane["roi_y0"]
            cv2.rectangle(
                overlay,
                (0, roi_y0),
                (width - 1, height - 1),
                (255, 180, 0),
                2,
            )
            for component in lane["components"]:
                cv2.drawContours(
                    overlay,
                    [component["contour"]],
                    -1,
                    (0, 180, 255),
                    2,
                )
                cv2.circle(
                    overlay,
                    (int(component["cx"]), int(component["cy"])),
                    5,
                    (0, 180, 255),
                    -1,
                )

            for component in lane["selected"]:
                cv2.circle(
                    overlay,
                    (int(component["cx"]), int(component["cy"])),
                    8,
                    (255, 0, 0),
                    2,
                )

            if lane["lane_center_point"] is not None:
                lane_point = (
                    int(lane["lane_center_point"][0]),
                    int(lane["lane_center_point"][1]),
                )
                centerline = lane["lane_centerline"]
                if centerline is not None:
                    for segment_name in ("left_segment", "right_segment"):
                        start, end = centerline[segment_name]
                        cv2.line(
                            overlay,
                            (int(start[0]), int(start[1])),
                            (int(end[0]), int(end[1])),
                            (255, 255, 0),
                            2,
                        )
                    cv2.line(
                        overlay,
                        (
                            int(centerline["top"][0]),
                            int(centerline["top"][1]),
                        ),
                        (
                            int(centerline["bottom"][0]),
                            int(centerline["bottom"][1]),
                        ),
                        (255, 255, 0),
                        3,
                    )
                else:
                    cv2.line(
                        overlay,
                        (
                            int(lane["selected"][0]["cx"]),
                            int(lane["selected"][0]["cy"]),
                        ),
                        (
                            int(lane["selected"][1]["cx"]),
                            int(lane["selected"][1]["cy"]),
                        ),
                        (255, 255, 0),
                        2,
                    )
                cv2.circle(overlay, lane_point, 9, (0, 255, 255), -1)

        if ids is not None and len(corners) > 0:
            cv2.aruco.drawDetectedMarkers(overlay, corners, ids)

        axis_error = None
        axis_angle_error = None
        if detection is not None and alignment["axis"] is not None:
            axis_origin, axis_direction = alignment["axis"]
            axis_error = alignment["axis_error"]
            axis_angle_error = alignment["axis_angle_error"]
            axis_end = self._ray_end_in_image(
                axis_origin,
                axis_direction,
                width,
                height,
            )
            cv2.line(
                overlay,
                tuple(axis_origin.astype(int)),
                tuple(axis_end.astype(int)),
                (0, 255, 0),
                3,
            )
            cv2.putText(
                overlay,
                "marker normal axis",
                tuple((axis_origin + axis_direction * 36.0).astype(int)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
                cv2.LINE_AA,
            )

            if (
                lane is not None
                and lane["lane_center_point"] is not None
                and alignment["closest"] is not None
            ):
                closest = alignment["closest"]
                lane_point = np.array(
                    lane["lane_center_point"],
                    dtype=np.float64,
                )
                cv2.line(
                    overlay,
                    tuple(lane_point.astype(int)),
                    tuple(closest.astype(int)),
                    (255, 255, 255),
                    2,
                )
                cv2.circle(
                    overlay,
                    tuple(closest.astype(int)),
                    7,
                    (255, 255, 255),
                    -1,
                )

        lines = [
            f"phase={phase}",
            f"target_id={target_id} status={status}",
            f"source={self._camera_source} flip_180={self._flip_180}",
        ]

        if lane is not None:
            if lane["lane_center_point"] is None:
                sep_text = (
                    f", sep={lane['pair_separation']:.1f}px"
                    if lane["pair_separation"] is not None
                    else ""
                )
                lines.append(
                    f"lane: components={len(lane['components'])}, "
                    f"pair=not_ready{sep_text}"
                )
            else:
                sep_text = (
                    f", sep={lane['pair_separation']:.1f}px"
                    if lane["pair_separation"] is not None
                    else ""
                )
                axis_text = (
                    f", axis_err={axis_error:.1f}px"
                    if axis_error is not None
                    else ""
                )
                angle_text = (
                    f", axis_angle={axis_angle_error:.1f}deg"
                    if axis_angle_error is not None
                    else ""
                )
                lines.append(
                    f"lane: components={len(lane['components'])}, "
                    f"center={lane['lane_center_point'][0]:.1f}, "
                    f"screen_err={lane['error_px']:.1f}px"
                    f"{sep_text}{axis_text}{angle_text}"
                )

        if detection is not None:
            tvec, rvec, marker_center = detection
            marker_px = (int(marker_center[0]), int(marker_center[1]))
            cv2.circle(overlay, marker_px, 6, (0, 0, 255), -1)
            cv2.line(overlay, (center_x, center_y), marker_px, (255, 0, 255), 2)
            lines.extend([
                f"center=({marker_center[0]:.1f},{marker_center[1]:.1f})",
                f"tvec=({tvec[0]:.3f},{tvec[1]:.3f},{tvec[2]:.3f})",
                f"rvec=({rvec[0]:.3f},{rvec[1]:.3f},{rvec[2]:.3f})",
            ])

        for idx, text in enumerate(lines):
            y = 28 + idx * 24
            cv2.putText(
                overlay,
                text,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 0),
                3,
                cv2.LINE_AA,
            )
            cv2.putText(
                overlay,
                text,
                (12, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )

        with self._lock:
            self._debug_frames.append((now, phase, overlay.copy()))

        if self._debug_image_pub is None:
            return

        try:
            msg = self._bridge.cv2_to_imgmsg(overlay, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "front_camera_link"
            self._debug_image_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f"Debug image publish failed: {exc}")

    def _save_debug_failure_images(self, failed_phase: str) -> None:
        if not self._debug_save_failure_images:
            return

        with self._lock:
            frames = list(self._debug_frames)
        if not frames:
            self.get_logger().warn(
                f"No docking debug frames to save for {failed_phase}"
            )
            return

        base_dir = Path(self._debug_save_dir).expanduser()
        if not base_dir.is_absolute():
            base_dir = Path.cwd() / base_dir

        run_dir = base_dir / (
            datetime.now().strftime("%Y%m%d_%H%M%S")
            + f"_{self._safe_name(failed_phase)}"
        )
        run_dir.mkdir(parents=True, exist_ok=True)

        saved = 0
        for idx, (stamp, phase, frame) in enumerate(frames, start=1):
            stamp_ms = int(stamp * 1000)
            path = run_dir / (
                f"{idx:02d}_{stamp_ms}_{self._safe_name(phase)}.png"
            )
            if cv2.imwrite(str(path), frame):
                saved += 1

        self.get_logger().warn(
            f"Saved {saved} docking debug frames for {failed_phase}: {run_dir}"
        )

    def _safe_name(self, value: str) -> str:
        return "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in value)

    def destroy_node(self):
        self._stop_camera()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ReverseDocking()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
