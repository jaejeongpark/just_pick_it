#!/usr/bin/env python3
"""PICKY2 reverse docking.

Docking is split from normal Nav2 driving:
  - RETURN_HOME moves the robot to the STANDBY zone with Nav2.
  - DOCK_IN uses this node to align to the AprilTag/ArUco marker and
    reverse slowly into the charging dock.

This MVP intentionally does not use the yellow lane or blue stop tape. The
current physical marker is on the horizontal wall above the standby zones, so
the marker is a local alignment reference, not the final stopping target.
"""

import math
import threading
import time

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

        # Phase timing and motion.
        self.declare_parameter("marker_timeout_sec", 15.0)
        self.declare_parameter("align_timeout_sec", 10.0)
        self.declare_parameter("reverse_timeout_sec", 20.0)
        self.declare_parameter("odom_timeout_sec", 5.0)
        self.declare_parameter("settle_sec", 0.3)

        self.declare_parameter("acquire_rotate_speed", 0.25)
        self.declare_parameter("align_lateral_tolerance_m", 0.02)
        self.declare_parameter("align_stable_frames", 3)
        self.declare_parameter("align_kp", 1.2)
        self.declare_parameter("reverse_speed", 0.035)
        self.declare_parameter("reverse_distance_m", 0.32)
        self.declare_parameter("yaw_hold_kp", 1.2)
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

        self._marker_timeout = float(self.get_parameter("marker_timeout_sec").value)
        self._align_timeout = float(self.get_parameter("align_timeout_sec").value)
        self._reverse_timeout = float(self.get_parameter("reverse_timeout_sec").value)
        self._odom_timeout = float(self.get_parameter("odom_timeout_sec").value)
        self._settle_sec = float(self.get_parameter("settle_sec").value)

        self._acquire_rotate_speed = float(
            self.get_parameter("acquire_rotate_speed").value
        )
        self._align_tol = float(self.get_parameter("align_lateral_tolerance_m").value)
        self._align_stable_frames = int(
            self.get_parameter("align_stable_frames").value
        )
        self._align_kp = float(self.get_parameter("align_kp").value)
        self._reverse_speed = float(self.get_parameter("reverse_speed").value)
        self._reverse_distance = float(
            self.get_parameter("reverse_distance_m").value
        )
        self._yaw_hold_kp = float(self.get_parameter("yaw_hold_kp").value)
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

        self.get_logger().info(
            "ReverseDocking ready "
            f"(camera_source={self._camera_source}, marker_size={self._marker_size:.3f}m)"
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

        if not self._start_camera():
            self._stop()
            return False

        try:
            if not self._wait_for_odom():
                self.get_logger().error("reverse_dock: FAILED at WAIT_ODOM")
                return False

            phases = [
                ("OBSERVE_MARKER", lambda: self._phase_observe_marker(marker_id)),
                ("ALIGN_MARKER_CENTER", lambda: self._phase_align_marker_center(marker_id)),
                ("REVERSE_INSERT", lambda: self._phase_reverse_insert(marker_id)),
            ]
            for name, fn in phases:
                if not fn():
                    self._stop()
                    self.get_logger().error(f"reverse_dock: FAILED at {name}")
                    return False

            self._stop()
            time.sleep(self._settle_sec)
            self._publish_pose_correction(dock_map_x, dock_map_y, dock_map_yaw)
            self.get_logger().info("reverse_dock: SUCCESS")
            return True
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
        while time.time() < deadline:
            if self._is_cancel_requested():
                return False

            result = self._detect_from_latest_frame(marker_id)
            if result is None:
                time.sleep(0.05)
                continue

            tvec, rvec, center = result
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

        while time.time() < deadline:
            if self._is_cancel_requested():
                return False

            result = self._detect_from_latest_frame(marker_id)
            if result is None:
                twist = Twist()
                twist.angular.z = self._acquire_rotate_speed
                self._cmd_pub.publish(twist)
                stable = 0
                time.sleep(0.05)
                continue

            tvec, _, center = result
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
            twist.angular.z = self._clamp(-self._align_kp * lat_err)
            self._cmd_pub.publish(twist)

            now = time.time()
            if now - last_log > 1.0:
                self.get_logger().info(
                    "ALIGN_MARKER_CENTER: "
                    f"lat_err={lat_err:.3f}m, cmd_w={twist.angular.z:.3f}"
                )
                last_log = now
            time.sleep(0.05)

        self._stop()
        self.get_logger().warn("ALIGN_MARKER_CENTER: timeout")
        return False

    def _phase_reverse_insert(self, marker_id: int) -> bool:
        """Reverse by odom distance while holding the starting yaw."""
        start = self._get_odom_pose()
        if start is None:
            self.get_logger().warn("REVERSE_INSERT: no odom")
            return False

        start_x, start_y, theta_ref = start
        deadline = time.time() + self._reverse_timeout
        last_log = 0.0

        self.get_logger().info(
            "REVERSE_INSERT: start "
            f"odom=({start_x:.3f},{start_y:.3f},{math.degrees(theta_ref):.1f}deg), "
            f"target_distance={self._reverse_distance:.3f}m"
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
            if moved >= self._reverse_distance:
                self._stop()
                self.get_logger().info(
                    "REVERSE_INSERT: done "
                    f"moved={moved:.3f}m, yaw={math.degrees(yaw):.1f}deg"
                )
                return True

            yaw_err = normalize_angle(theta_ref - yaw)
            marker_lat = 0.0
            if self._reverse_marker_lat_kp != 0.0:
                result = self._detect_from_latest_frame(marker_id)
                if result is not None:
                    marker_lat = float(result[0][0])

            twist = Twist()
            twist.linear.x = -self._reverse_speed
            twist.angular.z = self._clamp(
                self._yaw_hold_kp * yaw_err
                - self._reverse_marker_lat_kp * marker_lat
            )
            self._cmd_pub.publish(twist)

            now = time.time()
            if now - last_log > 1.0:
                self.get_logger().info(
                    "REVERSE_INSERT: "
                    f"moved={moved:.3f}/{self._reverse_distance:.3f}m, "
                    f"yaw_err={math.degrees(yaw_err):.1f}deg, "
                    f"cmd=({twist.linear.x:.3f},{twist.angular.z:.3f})"
                )
                last_log = now
            time.sleep(0.05)

        self._stop()
        self.get_logger().warn("REVERSE_INSERT: timeout")
        return False

    # ------------------------------------------------------------------ #
    # Detection and camera
    # ------------------------------------------------------------------ #

    def _detect_from_latest_frame(self, marker_id: int):
        frame = self._get_latest_frame()
        if frame is None:
            return None
        return self._detect_aruco(frame, marker_id)

    def _detect_aruco(self, frame, target_id: int):
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
            return None

        for i, marker_id in enumerate(ids.flatten()):
            if int(marker_id) != int(target_id):
                continue
            marker_corners = corners[i][0].astype(np.float64)
            ok, rvec, tvec = cv2.solvePnP(
                self._marker_obj_pts,
                marker_corners,
                self._cam_matrix,
                self._dist_coeffs,
            )
            if not ok:
                return None
            center = marker_corners.mean(axis=0)
            return tvec.flatten(), rvec.flatten(), center

        return None

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
            time.sleep(0.5)
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

    def _stop(self) -> None:
        self._cmd_pub.publish(Twist())

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
