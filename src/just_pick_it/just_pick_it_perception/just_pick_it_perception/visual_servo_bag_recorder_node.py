#!/usr/bin/env python3

import math
import os
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message

import rosbag2_py

from std_msgs.msg import Empty
from std_msgs.msg import Float64MultiArray

from just_pick_it_interfaces.msg import TrackedObjectArray
from just_pick_it_interfaces.msg import VisualServoSample


PHASE_WAIT_TARGET = 0
PHASE_VISUAL_SERVO = 1
PHASE_TERMINAL_APPROACH = 2
PHASE_GRIP_DONE = 3


class VisualServoBagRecorderNode(Node):
    """
    Visual servoing NN controller dataset recorder.

    Subscribes:
        /{robot_name}/status
            Float64MultiArray

            Expected current Jetcobot status layout:
                data[0:6]    = tool_reference
                data[6:12]   = world_reference
                data[12]     = reference_frame
                data[13]     = end_type
                data[14:20]  = current joint angles
                data[20:26]  = current coords
                data[26]     = gripper value

        detection_topic
            just_pick_it_interfaces/msg/TrackedObjectArray

    Publishes:
        /{robot_name}/request_status
            std_msgs/msg/Empty

    Writes rosbag:
        bag_topic
            just_pick_it_interfaces/msg/VisualServoSample

    Recording logic:
        1. Wait until target class is detected at least once.
        2. Record current joint angles and detection feature.
        3. Select the target-class object closest to image center.
        4. If area_norm and center_norm satisfy stable condition for N frames,
           save terminal anchor state and switch to TERMINAL_APPROACH phase.
        5. Continue recording even when detection disappears.
        6. Stop when gripper close transition is detected.
    """

    def __init__(self):
        super().__init__("visual_servo_bag_recorder")

        # ============================================================
        # Basic topics
        # ============================================================
        self.declare_parameter("robot_name", "jetcobot1")

        self.declare_parameter("status_topic", "")
        self.declare_parameter("request_status_topic", "")
        self.declare_parameter("detection_topic", "/infer/tracked_objects")

        # ============================================================
        # Detection target
        # ============================================================
        self.declare_parameter("target_class_label", "watermelon")
        self.declare_parameter("min_confidence", 0.5)

        self.declare_parameter("image_width", 640.0)
        self.declare_parameter("image_height", 480.0)

        # "bbox" or "mask"
        self.declare_parameter("center_source", "bbox")

        # "center" or "topleft"
        # Your current detector uses bbox_x, bbox_y as bbox center.
        self.declare_parameter("bbox_xy_mode", "center")

        # Negative means image center.
        self.declare_parameter("desired_cx", -1.0)
        self.declare_parameter("desired_cy", -1.0)

        # ============================================================
        # Recording behavior
        # ============================================================
        self.declare_parameter("sample_rate_hz", 10.0)
        self.declare_parameter("status_timeout_sec", 1.0)
        self.declare_parameter("detection_timeout_sec", 1.0)

        # If true, recorder waits until first target detection before writing samples.
        self.declare_parameter("start_after_first_detection", True)

        # If detection disappears after target was seen, keep last valid cx/cy/area.
        self.declare_parameter("use_last_valid_when_lost", True)

        # ============================================================
        # Terminal anchor trigger
        # ============================================================
        self.declare_parameter("terminal_trigger_area_norm", 0.06)
        self.declare_parameter("terminal_trigger_center_norm", 0.035)
        self.declare_parameter("terminal_ready_frames", 5)

        # Used only to provide normalized terminal_progress in recorded samples.
        self.declare_parameter("terminal_nominal_duration_sec", 1.0)

        # ============================================================
        # Rosbag output
        # ============================================================
        self.declare_parameter("bag_uri", "")
        self.declare_parameter("bag_base_dir", str(Path.home() / "rosbags"))
        self.declare_parameter("bag_name_prefix", "visual_servo")
        self.declare_parameter("bag_topic", "/nn_controller/training_sample")
        self.declare_parameter("storage_id", "sqlite3")

        # ============================================================
        # Gripper stop condition
        # ============================================================
        self.declare_parameter("stop_on_gripper_close", True)

        # gripper_close_mode:
        #   "le": closed if gripper_value <= threshold
        #   "ge": closed if gripper_value >= threshold
        #
        # Many myCobot gripper setups use:
        #   0   = closed
        #   100 = open
        #
        # But verify with your hardware.
        self.declare_parameter("gripper_close_mode", "le")
        self.declare_parameter("gripper_close_threshold", 20.0)

        # If true, recorder does not stop immediately when it starts
        # while the gripper is already closed.
        self.declare_parameter("require_open_before_close", True)

        self.declare_parameter("manual_stop_topic", "/visual_servo_bag_recorder/stop")
        self.declare_parameter("shutdown_on_stop", True)

        # ============================================================
        # Load parameters
        # ============================================================
        self.robot_name = str(self.get_parameter("robot_name").value)

        status_topic = str(self.get_parameter("status_topic").value)
        request_status_topic = str(self.get_parameter("request_status_topic").value)

        self.status_topic = (
            status_topic if status_topic else f"/{self.robot_name}/status"
        )
        self.request_status_topic = (
            request_status_topic
            if request_status_topic
            else f"/{self.robot_name}/request_status"
        )

        self.detection_topic = str(self.get_parameter("detection_topic").value)

        self.target_class_label = str(
            self.get_parameter("target_class_label").value
        )
        self.min_confidence = float(self.get_parameter("min_confidence").value)

        self.image_w = float(self.get_parameter("image_width").value)
        self.image_h = float(self.get_parameter("image_height").value)

        self.center_source = str(self.get_parameter("center_source").value).lower()
        if self.center_source not in ["bbox", "mask"]:
            self.get_logger().warn(
                f"Invalid center_source={self.center_source}. Use bbox."
            )
            self.center_source = "bbox"

        self.bbox_xy_mode = str(self.get_parameter("bbox_xy_mode").value).lower()
        if self.bbox_xy_mode not in ["center", "topleft"]:
            self.get_logger().warn(
                f"Invalid bbox_xy_mode={self.bbox_xy_mode}. Use center."
            )
            self.bbox_xy_mode = "center"

        self.desired_cx = float(self.get_parameter("desired_cx").value)
        self.desired_cy = float(self.get_parameter("desired_cy").value)

        if self.desired_cx < 0.0:
            self.desired_cx = self.image_w * 0.5
        if self.desired_cy < 0.0:
            self.desired_cy = self.image_h * 0.5

        self.sample_rate_hz = float(self.get_parameter("sample_rate_hz").value)
        self.status_timeout_sec = float(
            self.get_parameter("status_timeout_sec").value
        )
        self.detection_timeout_sec = float(
            self.get_parameter("detection_timeout_sec").value
        )

        self.start_after_first_detection = self.parse_bool(
            self.get_parameter("start_after_first_detection").value
        )
        self.use_last_valid_when_lost = self.parse_bool(
            self.get_parameter("use_last_valid_when_lost").value
        )

        self.terminal_trigger_area_norm = float(
            self.get_parameter("terminal_trigger_area_norm").value
        )
        self.terminal_trigger_center_norm = float(
            self.get_parameter("terminal_trigger_center_norm").value
        )
        self.terminal_ready_frames = int(
            self.get_parameter("terminal_ready_frames").value
        )
        self.terminal_nominal_duration_sec = float(
            self.get_parameter("terminal_nominal_duration_sec").value
        )

        self.bag_uri = str(self.get_parameter("bag_uri").value)
        self.bag_base_dir = str(self.get_parameter("bag_base_dir").value)
        self.bag_name_prefix = str(self.get_parameter("bag_name_prefix").value)
        self.bag_topic = str(self.get_parameter("bag_topic").value)
        self.storage_id = str(self.get_parameter("storage_id").value)

        self.stop_on_gripper_close = self.parse_bool(
            self.get_parameter("stop_on_gripper_close").value
        )
        self.gripper_close_mode = str(
            self.get_parameter("gripper_close_mode").value
        ).lower()
        self.gripper_close_threshold = float(
            self.get_parameter("gripper_close_threshold").value
        )
        self.require_open_before_close = self.parse_bool(
            self.get_parameter("require_open_before_close").value
        )

        self.manual_stop_topic = str(self.get_parameter("manual_stop_topic").value)
        self.shutdown_on_stop = self.parse_bool(
            self.get_parameter("shutdown_on_stop").value
        )

        if self.gripper_close_mode not in ["le", "ge"]:
            self.get_logger().warn(
                f"Invalid gripper_close_mode={self.gripper_close_mode}. Use le."
            )
            self.gripper_close_mode = "le"

        if self.sample_rate_hz <= 0.0:
            raise ValueError("sample_rate_hz must be > 0.0")

        if self.terminal_ready_frames < 1:
            raise ValueError("terminal_ready_frames must be >= 1")

        if self.terminal_nominal_duration_sec <= 0.0:
            self.terminal_nominal_duration_sec = 1.0

        # ============================================================
        # Runtime state
        # ============================================================
        self.latest_angles = None
        self.latest_gripper_value = math.nan
        self.latest_status_time = None

        self.current_target = None
        self.current_target_time = None

        self.last_valid_target = None
        self.last_valid_target_time = None
        self.has_seen_target = False

        self.phase = PHASE_WAIT_TARGET

        self.terminal_ready_count = 0
        self.terminal_anchor_valid = False
        self.q_anchor = [math.nan] * 6
        self.anchor_target = None

        self.terminal_start_time = None
        self.terminal_step_count = 0

        self.prev_gripper_closed = None
        self.seen_gripper_open = False
        self.manual_stop_requested = False

        self.recording = True
        self.sample_index = 0

        # ============================================================
        # ROS interfaces
        # ============================================================
        self.status_sub = self.create_subscription(
            Float64MultiArray,
            self.status_topic,
            self.status_callback,
            10,
        )

        self.detection_sub = self.create_subscription(
            TrackedObjectArray,
            self.detection_topic,
            self.detection_callback,
            10,
        )

        self.request_status_pub = self.create_publisher(
            Empty,
            self.request_status_topic,
            10,
        )

        self.manual_stop_sub = self.create_subscription(
            Empty,
            self.manual_stop_topic,
            self.manual_stop_callback,
            10,
        )

        # ============================================================
        # Rosbag writer
        # ============================================================
        self.writer = None
        self.resolved_bag_uri = self.resolve_bag_uri()
        self.open_bag_writer()

        period = 1.0 / self.sample_rate_hz
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info("VisualServoBagRecorderNode started")
        self.get_logger().info(f"status_topic={self.status_topic}")
        self.get_logger().info(f"request_status_topic={self.request_status_topic}")
        self.get_logger().info(f"detection_topic={self.detection_topic}")
        self.get_logger().info(f"target_class_label={self.target_class_label}")
        self.get_logger().info(f"center_source={self.center_source}")
        self.get_logger().info(f"bbox_xy_mode={self.bbox_xy_mode}")
        self.get_logger().info(
            f"desired_point=({self.desired_cx:.1f}, {self.desired_cy:.1f})"
        )
        self.get_logger().info(
            f"terminal trigger: area_norm>={self.terminal_trigger_area_norm:.4f}, "
            f"center_norm<={self.terminal_trigger_center_norm:.4f}, "
            f"frames={self.terminal_ready_frames}"
        )
        self.get_logger().info(f"bag_uri={self.resolved_bag_uri}")
        self.get_logger().info(f"bag_topic={self.bag_topic}")
        self.get_logger().info(
            f"gripper close: mode={self.gripper_close_mode}, "
            f"threshold={self.gripper_close_threshold}"
        )

    # ============================================================
    # Helpers
    # ============================================================
    @staticmethod
    def parse_bool(value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ["true", "1", "yes", "y"]
        return bool(value)

    @staticmethod
    def nan_target() -> Dict:
        return {
            "class_label": "",
            "class_id": -1,
            "track_id": -1,
            "confidence": 0.0,
            "cx": math.nan,
            "cy": math.nan,
            "bbox_x": math.nan,
            "bbox_y": math.nan,
            "bbox_w": math.nan,
            "bbox_h": math.nan,
            "bbox_cx": math.nan,
            "bbox_cy": math.nan,
            "mask_cx": math.nan,
            "mask_cy": math.nan,
            "area_norm": math.nan,
            "orientation_angle": math.nan,
            "frame_id": "camera_link",
        }

    def center_error_from_target(self, target: Dict):
        cx = float(target["cx"])
        cy = float(target["cy"])

        if not math.isfinite(cx) or not math.isfinite(cy):
            return math.nan, math.nan, math.nan

        u = (cx - self.desired_cx) / self.image_w
        v = (cy - self.desired_cy) / self.image_h
        center_norm = math.sqrt(u * u + v * v)
        return float(u), float(v), float(center_norm)

    # ============================================================
    # Bag writer
    # ============================================================
    def resolve_bag_uri(self) -> str:
        if self.bag_uri:
            path = Path(os.path.expanduser(self.bag_uri))
        else:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = Path(os.path.expanduser(self.bag_base_dir)) / (
                f"{self.bag_name_prefix}_{timestamp}"
            )

        path.parent.mkdir(parents=True, exist_ok=True)

        if path.exists():
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = path.with_name(f"{path.name}_{timestamp}")

        return str(path)

    def open_bag_writer(self):
        storage_options = rosbag2_py.StorageOptions(
            uri=self.resolved_bag_uri,
            storage_id=self.storage_id,
        )

        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        )

        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(storage_options, converter_options)

        msg_type = "just_pick_it_interfaces/msg/VisualServoSample"

        try:
            topic_info = rosbag2_py.TopicMetadata(
                0,
                self.bag_topic,
                "just_pick_it_interfaces/msg/VisualServoSample",
                "cdr",
                [],
                "",
            )
        except TypeError:
            topic_info = rosbag2_py.TopicMetadata(
                name=self.bag_topic,
                type=msg_type,
                serialization_format="cdr",
            )

        self.writer.create_topic(topic_info)

    def close_bag_writer(self):
        if self.writer is not None:
            self.get_logger().info(
                f"Closing rosbag. samples={self.sample_index}, "
                f"bag_uri={self.resolved_bag_uri}"
            )
            self.writer = None

    # ============================================================
    # Status callback
    # ============================================================
    def status_callback(self, msg: Float64MultiArray):
        data = list(msg.data)

        try:
            if len(data) >= 27:
                angles = [float(v) for v in data[14:20]]
                gripper_value = float(data[26])

            elif len(data) >= 7:
                angles = [float(v) for v in data[0:6]]
                gripper_value = float(data[6])
                self.get_logger().warn(
                    "Status length shorter than expected. "
                    "Fallback layout: data[0:6]=angles, data[6]=gripper."
                )

            elif len(data) >= 6:
                angles = [float(v) for v in data[0:6]]
                gripper_value = math.nan
                self.get_logger().warn(
                    "Status length shorter than expected. "
                    "Fallback layout: data[0:6]=angles only."
                )

            else:
                self.get_logger().warn(
                    f"Invalid status length={len(data)}. Need at least 6."
                )
                return

        except Exception as exc:
            self.get_logger().warn(f"Failed to parse status: {exc}")
            return

        self.latest_angles = angles
        self.latest_gripper_value = gripper_value
        self.latest_status_time = self.get_clock().now()

    def has_fresh_status(self) -> bool:
        if self.latest_status_time is None:
            return False

        age = (self.get_clock().now() - self.latest_status_time).nanoseconds * 1e-9
        return age <= self.status_timeout_sec

    # ============================================================
    # Detection callback
    # ============================================================
    def detection_callback(self, msg: TrackedObjectArray):
        target = self.select_nearest_target_to_image_center(msg)

        if target is None:
            self.current_target = None
            self.current_target_time = self.get_clock().now()
            return

        self.current_target = target
        self.current_target_time = self.get_clock().now()

        self.last_valid_target = target.copy()
        self.last_valid_target_time = self.current_target_time
        self.has_seen_target = True

    def select_nearest_target_to_image_center(
        self,
        msg: TrackedObjectArray,
    ) -> Optional[Dict]:
        best_target = None
        best_dist2 = float("inf")

        desired_x = self.image_w * 0.5
        desired_y = self.image_h * 0.5

        for obj in msg.objects:
            class_label = str(obj.class_label)
            confidence = float(obj.confidence)

            if self.target_class_label and class_label != self.target_class_label:
                continue

            if confidence < self.min_confidence:
                continue

            parsed = self.parse_tracked_object(obj, msg.header.frame_id)
            if parsed is None:
                continue

            cx = parsed["cx"]
            cy = parsed["cy"]

            dist2 = (cx - desired_x) ** 2 + (cy - desired_y) ** 2

            if dist2 < best_dist2:
                best_dist2 = dist2
                best_target = parsed

        return best_target

    def parse_tracked_object(self, obj, frame_id: str) -> Optional[Dict]:
        bbox_x = float(obj.bbox_x)
        bbox_y = float(obj.bbox_y)
        bbox_w = float(obj.bbox_w)
        bbox_h = float(obj.bbox_h)

        if self.bbox_xy_mode == "center":
            bbox_cx = bbox_x
            bbox_cy = bbox_y
        else:
            bbox_cx = bbox_x + bbox_w * 0.5
            bbox_cy = bbox_y + bbox_h * 0.5

        mask_cx = float(obj.mask_cx)
        mask_cy = float(obj.mask_cy)

        bbox_valid = (
            math.isfinite(bbox_cx)
            and math.isfinite(bbox_cy)
            and math.isfinite(bbox_w)
            and math.isfinite(bbox_h)
            and bbox_w > 0.0
            and bbox_h > 0.0
            and 0.0 <= bbox_cx <= self.image_w
            and 0.0 <= bbox_cy <= self.image_h
        )

        mask_valid = (
            math.isfinite(mask_cx)
            and math.isfinite(mask_cy)
            and 0.0 <= mask_cx <= self.image_w
            and 0.0 <= mask_cy <= self.image_h
        )

        if self.center_source == "mask" and mask_valid:
            cx = mask_cx
            cy = mask_cy
        elif bbox_valid:
            cx = bbox_cx
            cy = bbox_cy
        elif mask_valid:
            cx = mask_cx
            cy = mask_cy
        else:
            return None

        image_area = max(self.image_w * self.image_h, 1.0)
        area_norm = (bbox_w * bbox_h) / image_area

        if not math.isfinite(area_norm) or area_norm <= 0.0:
            return None

        return {
            "class_label": str(obj.class_label),
            "class_id": int(obj.class_id),
            "track_id": int(obj.track_id),
            "confidence": float(obj.confidence),
            "cx": float(cx),
            "cy": float(cy),
            "bbox_x": bbox_x,
            "bbox_y": bbox_y,
            "bbox_w": bbox_w,
            "bbox_h": bbox_h,
            "bbox_cx": bbox_cx,
            "bbox_cy": bbox_cy,
            "mask_cx": mask_cx,
            "mask_cy": mask_cy,
            "area_norm": float(area_norm),
            "orientation_angle": float(obj.orientation_angle),
            "frame_id": str(frame_id) if frame_id else "camera_link",
        }

    def has_fresh_current_detection(self) -> bool:
        if self.current_target is None:
            return False

        if self.current_target_time is None:
            return False

        age = (self.get_clock().now() - self.current_target_time).nanoseconds * 1e-9
        return age <= self.detection_timeout_sec

    def time_since_last_detection(self) -> float:
        if self.last_valid_target_time is None:
            return math.nan

        return float(
            (self.get_clock().now() - self.last_valid_target_time).nanoseconds * 1e-9
        )

    # ============================================================
    # Gripper / stop
    # ============================================================
    def manual_stop_callback(self, _msg: Empty):
        self.get_logger().info("Manual stop requested.")
        self.manual_stop_requested = True

    def compute_gripper_closed(self) -> bool:
        if not math.isfinite(self.latest_gripper_value):
            return False

        if self.gripper_close_mode == "le":
            return self.latest_gripper_value <= self.gripper_close_threshold

        return self.latest_gripper_value >= self.gripper_close_threshold

    def compute_stop_trigger(self, gripper_closed: bool) -> bool:
        if self.manual_stop_requested:
            return True

        if not self.stop_on_gripper_close:
            return False

        if self.prev_gripper_closed is None:
            self.prev_gripper_closed = gripper_closed

            if not gripper_closed:
                self.seen_gripper_open = True

            return False

        if not gripper_closed:
            self.seen_gripper_open = True

        close_transition = gripper_closed and not self.prev_gripper_closed

        if self.require_open_before_close:
            return close_transition and self.seen_gripper_open

        return close_transition

    # ============================================================
    # Terminal anchor logic
    # ============================================================
    def update_terminal_trigger(self, detected: bool) -> bool:
        """
        Returns:
            terminal_ready_now
        """
        if self.phase != PHASE_VISUAL_SERVO:
            return False

        if not detected or self.current_target is None:
            self.terminal_ready_count = 0
            return False

        target = self.current_target
        u, v, center_norm = self.center_error_from_target(target)
        area_norm = float(target["area_norm"])

        area_ready = area_norm >= self.terminal_trigger_area_norm
        center_ready = center_norm <= self.terminal_trigger_center_norm

        terminal_ready_now = bool(area_ready and center_ready)

        if terminal_ready_now:
            self.terminal_ready_count += 1
        else:
            self.terminal_ready_count = 0

        if self.terminal_ready_count >= self.terminal_ready_frames:
            self.capture_terminal_anchor(target)
            self.phase = PHASE_TERMINAL_APPROACH
            self.terminal_start_time = self.get_clock().now()
            self.terminal_step_count = 0

            self.get_logger().info(
                "Terminal anchor captured. "
                f"cx={target['cx']:.1f}, cy={target['cy']:.1f}, "
                f"area_norm={target['area_norm']:.5f}, "
                f"center_norm={center_norm:.4f}, "
                f"q_anchor={self.q_anchor}"
            )

        return terminal_ready_now

    def capture_terminal_anchor(self, target: Dict):
        self.terminal_anchor_valid = True

        if self.latest_angles is not None:
            self.q_anchor = [float(v) for v in self.latest_angles]
        else:
            self.q_anchor = [math.nan] * 6

        self.anchor_target = target.copy()

    def terminal_time_and_progress(self):
        if not self.terminal_anchor_valid or self.terminal_start_time is None:
            return 0.0, 0.0

        elapsed = (
            self.get_clock().now() - self.terminal_start_time
        ).nanoseconds * 1e-9

        progress = min(elapsed / self.terminal_nominal_duration_sec, 1.0)
        return float(elapsed), float(progress)

    # ============================================================
    # Sample construction
    # ============================================================
    def select_sample_target(self, detected: bool) -> Dict:
        if detected and self.current_target is not None:
            return self.current_target

        if self.use_last_valid_when_lost and self.last_valid_target is not None:
            return self.last_valid_target

        return self.nan_target()

    def fill_target_fields(
        self,
        sample: VisualServoSample,
        target: Dict,
        detected: bool,
    ):
        sample.target_class_label = str(target["class_label"])
        sample.target_class_id = int(target["class_id"])
        sample.track_id = int(target["track_id"])
        sample.confidence = float(target["confidence"])

        sample.cx = float(target["cx"])
        sample.cy = float(target["cy"])

        if detected and self.current_target is not None:
            sample.current_cx = float(self.current_target["cx"])
            sample.current_cy = float(self.current_target["cy"])
            sample.current_area_norm = float(self.current_target["area_norm"])
        else:
            sample.current_cx = math.nan
            sample.current_cy = math.nan
            sample.current_area_norm = math.nan

        if self.last_valid_target is not None:
            sample.last_valid_cx = float(self.last_valid_target["cx"])
            sample.last_valid_cy = float(self.last_valid_target["cy"])
            sample.last_valid_area_norm = float(
                self.last_valid_target["area_norm"]
            )
        else:
            sample.last_valid_cx = math.nan
            sample.last_valid_cy = math.nan
            sample.last_valid_area_norm = math.nan

        sample.bbox_x = float(target["bbox_x"])
        sample.bbox_y = float(target["bbox_y"])
        sample.bbox_w = float(target["bbox_w"])
        sample.bbox_h = float(target["bbox_h"])

        sample.mask_cx = float(target["mask_cx"])
        sample.mask_cy = float(target["mask_cy"])

        u, v, center_norm = self.center_error_from_target(target)
        sample.center_error_u = float(u)
        sample.center_error_v = float(v)
        sample.center_norm = float(center_norm)

        sample.area_norm = float(target["area_norm"])
        sample.orientation_angle = float(target["orientation_angle"])

    def fill_anchor_fields(self, sample: VisualServoSample):
        sample.terminal_anchor_valid = bool(self.terminal_anchor_valid)

        if self.terminal_anchor_valid:
            sample.q_anchor = [float(v) for v in self.q_anchor]

            if self.anchor_target is not None:
                anchor = self.anchor_target
            else:
                anchor = self.nan_target()

            sample.anchor_cx = float(anchor["cx"])
            sample.anchor_cy = float(anchor["cy"])

            u, v, center_norm = self.center_error_from_target(anchor)
            sample.anchor_center_error_u = float(u)
            sample.anchor_center_error_v = float(v)
            sample.anchor_center_norm = float(center_norm)

            sample.anchor_area_norm = float(anchor["area_norm"])
            sample.anchor_bbox_w = float(anchor["bbox_w"])
            sample.anchor_bbox_h = float(anchor["bbox_h"])
            sample.anchor_confidence = float(anchor["confidence"])

        else:
            sample.q_anchor = [math.nan] * 6

            sample.anchor_cx = math.nan
            sample.anchor_cy = math.nan
            sample.anchor_center_error_u = math.nan
            sample.anchor_center_error_v = math.nan
            sample.anchor_center_norm = math.nan
            sample.anchor_area_norm = math.nan
            sample.anchor_bbox_w = math.nan
            sample.anchor_bbox_h = math.nan
            sample.anchor_confidence = math.nan

        elapsed, progress = self.terminal_time_and_progress()
        sample.time_since_terminal_start = float(elapsed)
        sample.terminal_progress = float(progress)
        sample.terminal_step_count = int(self.terminal_step_count)

    def build_sample(
        self,
        detected: bool,
        terminal_ready_now: bool,
        stop_signal: bool,
    ) -> VisualServoSample:
        now = self.get_clock().now()

        if stop_signal:
            phase_for_sample = PHASE_GRIP_DONE
        else:
            phase_for_sample = self.phase

        target = self.select_sample_target(detected)

        sample = VisualServoSample()
        sample.header.stamp = now.to_msg()
        sample.header.frame_id = str(target.get("frame_id", "camera_link"))

        sample.phase = int(phase_for_sample)
        sample.sample_index = int(self.sample_index)

        sample.detected = bool(detected)
        sample.has_seen_target = bool(self.has_seen_target)

        sample.image_width = float(self.image_w)
        sample.image_height = float(self.image_h)

        self.fill_target_fields(sample, target, detected)

        if self.last_valid_target_time is None:
            sample.time_since_last_detection = math.nan
        else:
            sample.time_since_last_detection = float(self.time_since_last_detection())

        if self.latest_angles is not None:
            sample.joint_angles = [float(v) for v in self.latest_angles]
        else:
            sample.joint_angles = [math.nan] * 6

        sample.terminal_ready = bool(terminal_ready_now)
        sample.terminal_ready_count = int(self.terminal_ready_count)

        self.fill_anchor_fields(sample)

        gripper_closed = self.compute_gripper_closed()
        sample.gripper_value = float(self.latest_gripper_value)
        sample.gripper_closed = bool(gripper_closed)
        sample.stop_signal = bool(stop_signal)

        return sample

    def write_sample(self, sample: VisualServoSample):
        if self.writer is None:
            return

        timestamp_ns = self.get_clock().now().nanoseconds
        self.writer.write(self.bag_topic, serialize_message(sample), timestamp_ns)
        self.sample_index += 1

    # ============================================================
    # Main timer
    # ============================================================
    def timer_callback(self):
        if not self.recording:
            return

        self.request_status_pub.publish(Empty())

        if not self.has_fresh_status():
            self.get_logger().warn("Waiting for fresh robot status...")
            return

        detected = self.has_fresh_current_detection()

        if self.phase == PHASE_WAIT_TARGET and self.has_seen_target:
            self.phase = PHASE_VISUAL_SERVO
            self.get_logger().info("First target detected. Start VISUAL_SERVO phase.")

        if self.start_after_first_detection and not self.has_seen_target:
            return

        terminal_ready_now = self.update_terminal_trigger(detected)

        gripper_closed = self.compute_gripper_closed()
        stop_signal = self.compute_stop_trigger(gripper_closed)

        sample = self.build_sample(
            detected=detected,
            terminal_ready_now=terminal_ready_now,
            stop_signal=stop_signal,
        )

        self.write_sample(sample)

        self.get_logger().info(
            f"sample={sample.sample_index}, "
            f"phase={sample.phase}, "
            f"detected={sample.detected}, "
            f"class={sample.target_class_label}, "
            f"cx={sample.cx:.1f}, cy={sample.cy:.1f}, "
            f"area={sample.area_norm:.5f}, "
            f"center_norm={sample.center_norm:.4f}, "
            f"ready_count={sample.terminal_ready_count}, "
            f"anchor={sample.terminal_anchor_valid}, "
            f"terminal_step={sample.terminal_step_count}, "
            f"gripper={sample.gripper_value:.1f}, "
            f"closed={sample.gripper_closed}, "
            f"stop={sample.stop_signal}"
        )

        if self.phase == PHASE_TERMINAL_APPROACH:
            self.terminal_step_count += 1

        if stop_signal:
            self.get_logger().info("Stop signal detected. Closing rosbag.")
            self.recording = False
            self.close_bag_writer()

            if self.shutdown_on_stop:
                self.get_logger().info("shutdown_on_stop=true. Shutting down.")
                rclpy.shutdown()
                return

        self.prev_gripper_closed = gripper_closed


def main(args=None):
    rclpy.init(args=args)

    node = VisualServoBagRecorderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt. Closing rosbag.")
    finally:
        node.recording = False
        node.close_bag_writer()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()