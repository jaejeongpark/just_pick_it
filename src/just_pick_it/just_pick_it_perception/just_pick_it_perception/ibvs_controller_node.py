#!/usr/bin/env python3

import ast
import math
from collections import deque
from enum import Enum

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, Empty

from just_pick_it_interfaces.msg import TrackedObjectArray


CMD_JOINT = 0


class Phase(Enum):
    INIT = 0
    MOVE_PREGRASP = 1
    WAIT_PREGRASP = 2
    WAIT_Q0_STATUS = 3

    ALIGN_JAC_PLUS_SEND = 10
    ALIGN_JAC_PLUS_WAIT = 11
    ALIGN_JAC_MINUS_SEND = 12
    ALIGN_JAC_MINUS_WAIT = 13
    ALIGN_JAC_BACK_SEND = 14
    ALIGN_JAC_BACK_WAIT = 15

    RUN = 20

    AREA_JAC_PLUS_SEND = 30
    AREA_JAC_PLUS_WAIT = 31
    AREA_JAC_MINUS_SEND = 32
    AREA_JAC_MINUS_WAIT = 33
    AREA_JAC_BACK_SEND = 34
    AREA_JAC_BACK_WAIT = 35

    APPROACH_WAIT = 40
    DONE = 90
    ERROR = 99


class AreaJacobianIBVSNode(Node):
    """
    Split active-joint IBVS controller with simplified area-Jacobian approach.

    Main design:
      - align_joints are used only for image center alignment.
      - approach_joints are used only for area-increasing approach.
      - No near_grasp_angles.
      - No taught approach direction.
      - No approach_direction_hint.
      - No near-grasp distance limit.

    DONE condition:
      area_norm >= desired_area_norm
      center_norm <= area_done_center_threshold

    Safety stop:
      max_approach_steps
      max_total_steps
      detection loss before DONE
    """

    def __init__(self):
        super().__init__("ibvs_controller")

        # ============================================================
        # Robot / detection parameters
        # ============================================================
        self.declare_parameter("robot_name", "jetcobot1")
        self.declare_parameter("image_width", 640.0)
        self.declare_parameter("image_height", 480.0)
        self.declare_parameter("detection_topic", "/infer/tracked_objects")
        self.declare_parameter("detection_timeout_sec", 2.0)
        self.declare_parameter("min_confidence", 0.5)
        self.declare_parameter("target_class_label", "watermelon")
        self.declare_parameter("lock_track_id", True)
        self.declare_parameter("center_source", "bbox")
        self.declare_parameter("bbox_xy_mode", "center")
        self.declare_parameter("desired_cx", -1.0)
        self.declare_parameter("desired_cy", -1.0)

        # ============================================================
        # Joint split parameters
        # ============================================================
        self.declare_parameter("align_joints", [0, 3, 4])
        self.declare_parameter("approach_joints", [1, 2])
        self.declare_parameter(
            "pregrasp_angles",
            [107.75, 29.17, -31.11, -71.63, 2.90, -134.12],
        )

        # ============================================================
        # Motion parameters
        # ============================================================
        self.declare_parameter("pregrasp_speed", 15)
        self.declare_parameter("command_speed", 10)
        self.declare_parameter("pregrasp_wait_sec", 3.0)
        self.declare_parameter("use_status_for_q0", True)
        self.declare_parameter("status_timeout_sec", 1.0)

        # ============================================================
        # Align Jacobian parameters
        # ============================================================
        self.declare_parameter("jacobian_delta_deg", 2.0)
        self.declare_parameter("jacobian_settle_sec", 1.2)

        # ============================================================
        # Align controller parameters
        # ============================================================
        self.declare_parameter("lambda_gain", 0.8)
        self.declare_parameter("damping", 0.04)
        self.declare_parameter("max_align_delta_deg", 1.0)
        self.declare_parameter("max_align_offset_deg", 20.0)
        self.declare_parameter("control_rate_hz", 5.0)

        # ALIGN stuck recovery / re-Jacobian.
        # These prevent a soft-offset saturation from repeatedly publishing
        # almost identical commands, e.g. J4 offset stuck at +max_align_offset_deg.
        self.declare_parameter("enable_align_stuck_recovery", True)
        self.declare_parameter("enable_align_active_set", True)
        self.declare_parameter("align_stuck_frames", 8)
        self.declare_parameter("align_stuck_min_improvement", 0.002)
        self.declare_parameter("align_stuck_cmd_delta_deg", 0.05)
        self.declare_parameter("align_stuck_saturation_ratio", 0.95)
        self.declare_parameter("max_align_rejacobian_count", 5)
        self.declare_parameter("align_rejacobian_cooldown_sec", 1.0)
        self.declare_parameter("align_rejacobian_after_approach_steps", 3)

        # Loose threshold for allowing area approach.
        self.declare_parameter("approach_center_threshold", 0.09)

        # DONE threshold for center. If negative, approach_center_threshold is used.
        self.declare_parameter("area_done_center_threshold", -1.0)

        # ============================================================
        # Area-Jacobian approach parameters
        # ============================================================
        self.declare_parameter("approach_step_deg", 3.0)
        self.declare_parameter("approach_wait_sec", 0.6)
        self.declare_parameter("max_approach_steps", 250)
        self.declare_parameter("desired_area_norm", 0.23)
        self.declare_parameter("area_jacobian_delta_deg", 3.0)
        self.declare_parameter("area_jacobian_settle_sec", 0.8)
        self.declare_parameter("area_window_size", 5)
        self.declare_parameter("area_jacobian_min_grad", 1e-5)

        # Reuse the last measured area-gradient direction to avoid repeated
        # plus/minus/back perturbations on every approach step.
        # 1 = old behavior: measure area Jacobian for every approach step.
        # 3 = one measured approach + up to two cached approach steps.
        self.declare_parameter("area_jacobian_reuse_steps", 3)
        self.declare_parameter("area_min_gain_for_reuse", 0.001)
        self.declare_parameter("area_drop_tolerance", 0.003)

        # ============================================================
        # General safety / filter
        # ============================================================
        self.declare_parameter("max_total_steps", 500)
        self.declare_parameter("hard_stop_below_center_error", -1.0)
        self.declare_parameter("filter_alpha", 0.5)

        # ============================================================
        # Load parameters
        # ============================================================
        self.robot_name = str(self.get_parameter("robot_name").value)
        self.ns = f"/{self.robot_name}"

        self.image_w = float(self.get_parameter("image_width").value)
        self.image_h = float(self.get_parameter("image_height").value)

        self.detection_topic = str(self.get_parameter("detection_topic").value)
        self.detection_timeout_sec = float(
            self.get_parameter("detection_timeout_sec").value
        )
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.target_class_label = str(
            self.get_parameter("target_class_label").value
        )
        self.lock_track_id = self.parse_bool_parameter(
            self.get_parameter("lock_track_id").value
        )

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

        self.align_joints = self.parse_int_list_parameter(
            self.get_parameter("align_joints").value,
            default=[0, 3, 4],
        )
        self.approach_joints = self.parse_int_list_parameter(
            self.get_parameter("approach_joints").value,
            default=[1, 2],
        )
        self.pregrasp_angles = self.parse_float_list_parameter(
            self.get_parameter("pregrasp_angles").value,
            default=[107.75, 29.17, -31.11, -71.63, 2.90, -134.12],
        )

        self.validate_joint_config()

        self.pregrasp_speed = int(self.get_parameter("pregrasp_speed").value)
        self.command_speed = int(self.get_parameter("command_speed").value)
        self.pregrasp_wait_sec = float(self.get_parameter("pregrasp_wait_sec").value)
        self.use_status_for_q0 = self.parse_bool_parameter(
            self.get_parameter("use_status_for_q0").value
        )
        self.status_timeout_sec = float(self.get_parameter("status_timeout_sec").value)

        self.jacobian_delta_deg = float(self.get_parameter("jacobian_delta_deg").value)
        self.jacobian_settle_sec = float(
            self.get_parameter("jacobian_settle_sec").value
        )

        self.lambda_gain = float(self.get_parameter("lambda_gain").value)
        self.damping = float(self.get_parameter("damping").value)
        self.max_align_delta_deg = float(
            self.get_parameter("max_align_delta_deg").value
        )
        self.max_align_offset_deg = float(
            self.get_parameter("max_align_offset_deg").value
        )
        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)

        self.enable_align_stuck_recovery = self.parse_bool_parameter(
            self.get_parameter("enable_align_stuck_recovery").value
        )
        self.enable_align_active_set = self.parse_bool_parameter(
            self.get_parameter("enable_align_active_set").value
        )
        self.align_stuck_frames = int(
            self.get_parameter("align_stuck_frames").value
        )
        self.align_stuck_frames = max(1, self.align_stuck_frames)
        self.align_stuck_min_improvement = float(
            self.get_parameter("align_stuck_min_improvement").value
        )
        self.align_stuck_cmd_delta_deg = float(
            self.get_parameter("align_stuck_cmd_delta_deg").value
        )
        self.align_stuck_saturation_ratio = float(
            self.get_parameter("align_stuck_saturation_ratio").value
        )
        self.align_stuck_saturation_ratio = max(0.1, min(1.0, self.align_stuck_saturation_ratio))
        self.max_align_rejacobian_count = int(
            self.get_parameter("max_align_rejacobian_count").value
        )
        self.max_align_rejacobian_count = max(0, self.max_align_rejacobian_count)
        self.align_rejacobian_cooldown_sec = float(
            self.get_parameter("align_rejacobian_cooldown_sec").value
        )
        self.align_rejacobian_cooldown_sec = max(0.0, self.align_rejacobian_cooldown_sec)
        self.align_rejacobian_after_approach_steps = int(
            self.get_parameter("align_rejacobian_after_approach_steps").value
        )
        self.align_rejacobian_after_approach_steps = max(0, self.align_rejacobian_after_approach_steps)

        self.approach_center_threshold = float(
            self.get_parameter("approach_center_threshold").value
        )
        self.area_done_center_threshold = float(
            self.get_parameter("area_done_center_threshold").value
        )
        if self.area_done_center_threshold < 0.0:
            self.area_done_center_threshold = self.approach_center_threshold

        self.approach_step_deg = float(self.get_parameter("approach_step_deg").value)
        self.approach_wait_sec = float(self.get_parameter("approach_wait_sec").value)
        self.max_approach_steps = int(self.get_parameter("max_approach_steps").value)
        self.desired_area_norm = float(self.get_parameter("desired_area_norm").value)
        self.area_jacobian_delta_deg = float(
            self.get_parameter("area_jacobian_delta_deg").value
        )
        self.area_jacobian_settle_sec = float(
            self.get_parameter("area_jacobian_settle_sec").value
        )
        self.area_window_size = int(self.get_parameter("area_window_size").value)
        self.area_window_size = max(1, self.area_window_size)
        self.area_jacobian_min_grad = float(
            self.get_parameter("area_jacobian_min_grad").value
        )
        self.area_jacobian_reuse_steps = int(
            self.get_parameter("area_jacobian_reuse_steps").value
        )
        self.area_jacobian_reuse_steps = max(1, self.area_jacobian_reuse_steps)
        self.area_min_gain_for_reuse = float(
            self.get_parameter("area_min_gain_for_reuse").value
        )
        self.area_drop_tolerance = float(
            self.get_parameter("area_drop_tolerance").value
        )

        self.max_total_steps = int(self.get_parameter("max_total_steps").value)
        self.hard_stop_below_center_error = float(
            self.get_parameter("hard_stop_below_center_error").value
        )
        self.filter_alpha = float(self.get_parameter("filter_alpha").value)
        self.filter_alpha = max(0.0, min(1.0, self.filter_alpha))

        if self.desired_area_norm <= 0.0:
            raise ValueError("desired_area_norm must be > 0.0")
        if self.approach_step_deg <= 0.0:
            raise ValueError("approach_step_deg must be > 0.0")
        if self.area_jacobian_delta_deg <= 0.0:
            raise ValueError("area_jacobian_delta_deg must be > 0.0")
        if self.area_jacobian_reuse_steps < 1:
            raise ValueError("area_jacobian_reuse_steps must be >= 1")
        if self.area_drop_tolerance < 0.0:
            raise ValueError("area_drop_tolerance must be >= 0.0")

        # ============================================================
        # Publishers / subscribers
        # ============================================================
        self.target_pose_pub = self.create_publisher(
            Float64MultiArray,
            f"{self.ns}/target_pose",
            10,
        )
        self.status_request_pub = self.create_publisher(
            Empty,
            f"{self.ns}/request_status",
            10,
        )
        self.status_sub = self.create_subscription(
            Float64MultiArray,
            f"{self.ns}/status",
            self.status_callback,
            10,
        )
        self.detection_sub = self.create_subscription(
            TrackedObjectArray,
            self.detection_topic,
            self.detection_callback,
            10,
        )

        # ============================================================
        # Detection state
        # ============================================================
        self.latest_valid = False
        self.latest_cx = 0.0
        self.latest_cy = 0.0
        self.latest_conf = 0.0
        self.latest_detection_time = None

        self.filtered_initialized = False
        self.filtered_cx = 0.0
        self.filtered_cy = 0.0
        self.filtered_area_norm = 0.0
        self.area_window = deque(maxlen=self.area_window_size)

        self.target_track_id = None
        self.latest_track_id = -1
        self.latest_class_id = -1
        self.latest_class_label = ""
        self.latest_bbox_x = 0.0
        self.latest_bbox_y = 0.0
        self.latest_bbox_w = 0.0
        self.latest_bbox_h = 0.0
        self.latest_bbox_cx = 0.0
        self.latest_bbox_cy = 0.0
        self.latest_mask_cx = 0.0
        self.latest_mask_cy = 0.0
        self.latest_area_norm = 0.0
        self.latest_orientation_angle = 0.0

        # ============================================================
        # Status state
        # ============================================================
        self.latest_status_time = None
        self.latest_angles = None
        self.latest_coords = None
        self.latest_gripper_value = None

        # ============================================================
        # Controller state
        # ============================================================
        self.phase = Phase.INIT
        self.phase_start_time = self.get_clock().now()

        self.q0 = None
        self.q_base = np.array(self.pregrasp_angles, dtype=np.float64)
        self.q_align_offset = np.zeros(6, dtype=np.float64)
        self.q_last_cmd = self.q_base.copy()
        self.approach_start_q = self.q_base.copy()

        # Align Jacobian state.
        self.align_jacobian_cols = []
        self.current_align_jac_local_idx = 0
        self.align_f_plus = None
        self.align_f_minus = None
        self.J_align = None

        # ALIGN stuck / rebase state.
        self.prev_align_center_norm = None
        self.align_stuck_count = 0
        self.align_rejacobian_count = 0
        self.last_align_rejacobian_time = None
        self.last_align_jacobian_approach_step = 0
        self.last_align_blocked_joints = []
        self.last_align_free_joints = []

        # Area Jacobian state.
        self.area_jac_base_q = None
        self.area_jacobian_cols = []
        self.current_area_jac_local_idx = 0
        self.area_f_plus = None
        self.area_f_minus = None
        self.area_gradient = None

        self.total_step_count = 0
        self.approach_step_count = 0
        self.last_control_time = None
        self.area_before_approach = None

        # Cached area approach direction. This removes repeated
        # J2+/J2-/back, J3+/J3-/back cycles while the last measured
        # direction is still improving area and keeping the target centered.
        self.cached_area_direction_valid = False
        self.cached_area_direction_full = None
        self.cached_area_gradient = None
        self.cached_area_grad_norm = 0.0
        self.cached_area_direction_use_count = 0
        self.last_approach_source = ""

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("Area-Jacobian split IBVS node started")
        self.get_logger().info(f"robot_name={self.robot_name}")
        self.get_logger().info(f"namespace={self.ns}")
        self.get_logger().info(f"detection_topic={self.detection_topic}")
        self.get_logger().info(f"target_class_label={self.target_class_label}")
        self.get_logger().info(f"center_source={self.center_source}")
        self.get_logger().info(f"bbox_xy_mode={self.bbox_xy_mode}")
        self.get_logger().info(
            f"desired_point=({self.desired_cx:.1f}, {self.desired_cy:.1f})"
        )
        self.get_logger().info(f"align_joints={self.align_joints}")
        self.get_logger().info(f"approach_joints={self.approach_joints}")
        self.get_logger().info(f"pregrasp_angles={self.pregrasp_angles}")
        self.get_logger().info(
            f"DONE condition: area_norm >= {self.desired_area_norm:.5f}, "
            f"center_norm <= {self.area_done_center_threshold:.5f}"
        )
        self.get_logger().info(
            f"Area Jacobian: delta={self.area_jacobian_delta_deg}, "
            f"settle={self.area_jacobian_settle_sec}, "
            f"window={self.area_window_size}, "
            f"min_grad={self.area_jacobian_min_grad}, "
            f"reuse_steps={self.area_jacobian_reuse_steps}, "
            f"min_gain_for_reuse={self.area_min_gain_for_reuse}, "
            f"drop_tolerance={self.area_drop_tolerance}"
        )
        self.get_logger().info(
            f"Align recovery: enabled={self.enable_align_stuck_recovery}, "
            f"active_set={self.enable_align_active_set}, "
            f"stuck_frames={self.align_stuck_frames}, "
            f"min_improvement={self.align_stuck_min_improvement}, "
            f"cmd_delta_th={self.align_stuck_cmd_delta_deg}, "
            f"sat_ratio={self.align_stuck_saturation_ratio}, "
            f"max_rejac={self.max_align_rejacobian_count}, "
            f"cooldown={self.align_rejacobian_cooldown_sec}, "
            f"after_approach_steps={self.align_rejacobian_after_approach_steps}"
        )

    # ============================================================
    # Parameter parsing / validation
    # ============================================================
    @staticmethod
    def parse_bool_parameter(value):
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ["true", "1", "yes", "y"]
        return bool(value)

    @staticmethod
    def parse_float_list_parameter(value, default):
        if isinstance(value, str):
            text = value.strip()
            try:
                parsed = ast.literal_eval(text)
                if isinstance(parsed, (list, tuple)):
                    return [float(v) for v in parsed]
            except Exception:
                pass
            if "," in text:
                return [float(v.strip()) for v in text.split(",") if v.strip() != ""]
            return [float(v) for v in default]

        try:
            return [float(v) for v in list(value)]
        except Exception:
            return [float(v) for v in default]

    @staticmethod
    def parse_int_list_parameter(value, default):
        if isinstance(value, str):
            text = value.strip()
            try:
                parsed = ast.literal_eval(text)
                if isinstance(parsed, (list, tuple)):
                    return [int(v) for v in parsed]
            except Exception:
                pass
            if "," in text:
                return [int(v.strip()) for v in text.split(",") if v.strip() != ""]
            return [int(v) for v in default]

        try:
            return [int(v) for v in list(value)]
        except Exception:
            return [int(v) for v in default]

    def validate_joint_config(self):
        if len(self.pregrasp_angles) != 6:
            raise ValueError("pregrasp_angles must have length 6")

        for idx in self.align_joints:
            if idx < 0 or idx > 5:
                raise ValueError(f"Invalid align joint index: {idx}")
        for idx in self.approach_joints:
            if idx < 0 or idx > 5:
                raise ValueError(f"Invalid approach joint index: {idx}")

        overlap = set(self.align_joints).intersection(set(self.approach_joints))
        if overlap:
            raise ValueError(
                f"align_joints and approach_joints must not overlap. Overlap={overlap}"
            )

        if len(self.align_joints) == 0:
            raise ValueError("align_joints must not be empty")
        if len(self.approach_joints) == 0:
            raise ValueError("approach_joints must not be empty")

    # ============================================================
    # Command publisher / status
    # ============================================================
    def publish_joint_command(self, angles, speed):
        if len(angles) != 6:
            self.get_logger().error(f"Invalid angles length: {len(angles)}")
            return

        speed = int(max(1, min(100, speed)))
        msg = Float64MultiArray()
        msg.data = [float(CMD_JOINT)] + [float(v) for v in angles] + [float(speed)]
        self.target_pose_pub.publish(msg)
        self.q_last_cmd = np.array(angles, dtype=np.float64)

        self.get_logger().info(
            f"Publish joint command: angles={np.round(self.q_last_cmd, 3).tolist()}, "
            f"speed={speed}"
        )

    def compose_q_cmd(self):
        return self.q_base + self.q_align_offset

    def request_status(self):
        self.status_request_pub.publish(Empty())

    def status_callback(self, msg: Float64MultiArray):
        data = list(msg.data)
        if len(data) < 27:
            self.get_logger().warn(
                f"Invalid status length: {len(data)}. Expected at least 27."
            )
            return

        try:
            angles = [float(v) for v in data[14:20]]
            coords = [float(v) for v in data[20:26]]
            gripper_value = float(data[26])
        except Exception as exc:
            self.get_logger().warn(f"Failed to parse status: {exc}")
            return

        self.latest_angles = np.array(angles, dtype=np.float64)
        self.latest_coords = np.array(coords, dtype=np.float64)
        self.latest_gripper_value = gripper_value
        self.latest_status_time = self.get_clock().now()

    def has_fresh_status(self):
        if self.latest_status_time is None:
            return False
        age = (self.get_clock().now() - self.latest_status_time).nanoseconds * 1e-9
        return age <= self.status_timeout_sec

    # ============================================================
    # Detection callback
    # ============================================================
    def detection_callback(self, msg: TrackedObjectArray):
        if len(msg.objects) == 0:
            self.latest_valid = False
            self.latest_detection_time = self.get_clock().now()
            return

        best_obj = None
        best_score = -1.0

        if self.lock_track_id and self.target_track_id is not None:
            for obj in msg.objects:
                if int(obj.track_id) == int(self.target_track_id):
                    if float(obj.confidence) >= self.min_confidence:
                        best_obj = obj
                    break

        if best_obj is None:
            for obj in msg.objects:
                class_label = str(obj.class_label)
                conf = float(obj.confidence)

                if self.target_class_label != "" and class_label != self.target_class_label:
                    continue
                if conf < self.min_confidence:
                    continue
                if conf > best_score:
                    best_score = conf
                    best_obj = obj

            if best_obj is not None and self.lock_track_id:
                self.target_track_id = int(best_obj.track_id)
                self.get_logger().info(
                    f"Locked target track_id={self.target_track_id}, "
                    f"class={best_obj.class_label}, conf={best_obj.confidence:.3f}"
                )

        if best_obj is None:
            self.latest_valid = False
            self.latest_detection_time = self.get_clock().now()
            return

        bbox_x = float(best_obj.bbox_x)
        bbox_y = float(best_obj.bbox_y)
        bbox_w = float(best_obj.bbox_w)
        bbox_h = float(best_obj.bbox_h)

        if self.bbox_xy_mode == "center":
            bbox_cx = bbox_x
            bbox_cy = bbox_y
        else:
            bbox_cx = bbox_x + bbox_w * 0.5
            bbox_cy = bbox_y + bbox_h * 0.5

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

        mask_cx = float(best_obj.mask_cx)
        mask_cy = float(best_obj.mask_cy)

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
            self.latest_valid = False
            self.latest_detection_time = self.get_clock().now()
            self.get_logger().warn("Invalid bbox/mask center. Detection ignored.")
            return

        image_area = max(self.image_w * self.image_h, 1.0)
        bbox_area_norm = (bbox_w * bbox_h) / image_area
        if not math.isfinite(bbox_area_norm) or bbox_area_norm <= 0.0:
            self.latest_valid = False
            self.latest_detection_time = self.get_clock().now()
            self.get_logger().warn("Invalid bbox area. Detection ignored.")
            return

        if not self.filtered_initialized or self.filter_alpha >= 1.0:
            filtered_cx = cx
            filtered_cy = cy
            filtered_area_norm = bbox_area_norm
            self.filtered_initialized = True
        else:
            alpha = self.filter_alpha
            filtered_cx = alpha * cx + (1.0 - alpha) * self.filtered_cx
            filtered_cy = alpha * cy + (1.0 - alpha) * self.filtered_cy
            filtered_area_norm = alpha * bbox_area_norm + (1.0 - alpha) * self.filtered_area_norm

        self.filtered_cx = filtered_cx
        self.filtered_cy = filtered_cy
        self.filtered_area_norm = filtered_area_norm

        self.latest_valid = True
        self.latest_cx = filtered_cx
        self.latest_cy = filtered_cy
        self.latest_conf = float(best_obj.confidence)
        self.latest_detection_time = self.get_clock().now()

        self.latest_track_id = int(best_obj.track_id)
        self.latest_class_id = int(best_obj.class_id)
        self.latest_class_label = str(best_obj.class_label)
        self.latest_bbox_x = bbox_x
        self.latest_bbox_y = bbox_y
        self.latest_bbox_w = bbox_w
        self.latest_bbox_h = bbox_h
        self.latest_bbox_cx = bbox_cx
        self.latest_bbox_cy = bbox_cy
        self.latest_mask_cx = mask_cx
        self.latest_mask_cy = mask_cy
        self.latest_area_norm = filtered_area_norm
        self.latest_orientation_angle = float(best_obj.orientation_angle)

        self.area_window.append(float(filtered_area_norm))

    # ============================================================
    # Detection utilities
    # ============================================================
    def has_fresh_detection(self):
        if self.latest_detection_time is None:
            return False
        age = (self.get_clock().now() - self.latest_detection_time).nanoseconds * 1e-9
        if age > self.detection_timeout_sec:
            return False
        if not self.latest_valid:
            return False
        if self.latest_conf < self.min_confidence:
            return False
        return True

    def get_center_feature(self):
        u = (self.latest_cx - self.desired_cx) / self.image_w
        v = (self.latest_cy - self.desired_cy) / self.image_h
        return np.array([u, v], dtype=np.float64)

    def get_center_norm(self):
        if not self.latest_valid:
            return float("inf")
        return float(np.linalg.norm(self.get_center_feature()))

    def get_area_avg(self):
        values = [v for v in self.area_window if math.isfinite(v)]
        if len(values) == 0:
            return None
        return float(np.mean(values))

    def reset_area_window(self):
        self.area_window.clear()

    def check_done_condition(self, area_value=None, center_norm=None, source=""):
        if area_value is None:
            area_value = float(self.latest_area_norm)
        else:
            area_value = float(area_value)

        if center_norm is None:
            center_norm = self.get_center_norm()
        else:
            center_norm = float(center_norm)

        area_ok = area_value >= self.desired_area_norm
        center_ok = center_norm <= self.area_done_center_threshold

        if area_ok and center_ok:
            self.get_logger().info(
                f"DONE: area+center condition reached at {source}. "
                f"area={area_value:.5f} >= desired={self.desired_area_norm:.5f}, "
                f"center_norm={center_norm:.5f} <= "
                f"area_done_center_threshold={self.area_done_center_threshold:.5f}"
            )
            self.set_phase(Phase.DONE)
            return True

        if area_ok and not center_ok:
            self.get_logger().info(
                f"DONE not triggered at {source}: area ok but center not ok. "
                f"area={area_value:.5f} >= desired={self.desired_area_norm:.5f}, "
                f"center_norm={center_norm:.5f} > "
                f"area_done_center_threshold={self.area_done_center_threshold:.5f}"
            )
        elif center_ok and not area_ok:
            self.get_logger().info(
                f"DONE not triggered at {source}: center ok but area not ok. "
                f"area={area_value:.5f} < desired={self.desired_area_norm:.5f}, "
                f"center_norm={center_norm:.5f} <= "
                f"area_done_center_threshold={self.area_done_center_threshold:.5f}"
            )

        return False

    # ============================================================
    # Math
    # ============================================================
    def damped_pseudo_inverse(self, J):
        J = np.asarray(J, dtype=np.float64)
        m = J.shape[0]
        identity = np.eye(m)
        return J.T @ np.linalg.inv(J @ J.T + (self.damping ** 2) * identity)

    def compute_align_delta_full(self):
        if self.J_align is None:
            raise RuntimeError("Align Jacobian is not estimated yet.")

        feature = self.get_center_feature()
        self.last_align_blocked_joints = []
        self.last_align_free_joints = list(self.align_joints)

        # First solve using all align joints. This gives the desired direction and
        # also tells us which saturated joint is being pushed farther into its limit.
        J_full = np.asarray(self.J_align, dtype=np.float64)
        J_pinv = self.damped_pseudo_inverse(J_full)
        delta_active_raw = -self.lambda_gain * (J_pinv @ feature)

        if not self.enable_align_active_set:
            delta_active = np.clip(
                delta_active_raw,
                -self.max_align_delta_deg,
                self.max_align_delta_deg,
            )
            delta_full = np.zeros(6, dtype=np.float64)
            for local_idx, joint_idx in enumerate(self.align_joints):
                delta_full[joint_idx] = delta_active[local_idx]
            return delta_full

        sat_limit = self.max_align_offset_deg * self.align_stuck_saturation_ratio
        blocked_local_indices = []
        free_local_indices = []

        for local_idx, joint_idx in enumerate(self.align_joints):
            offset = float(self.q_align_offset[joint_idx])
            desired_delta = float(delta_active_raw[local_idx])
            pushing_positive_limit = offset >= sat_limit and desired_delta > 0.0
            pushing_negative_limit = offset <= -sat_limit and desired_delta < 0.0

            if pushing_positive_limit or pushing_negative_limit:
                blocked_local_indices.append(local_idx)
            else:
                free_local_indices.append(local_idx)

        if len(blocked_local_indices) == 0:
            delta_active = np.clip(
                delta_active_raw,
                -self.max_align_delta_deg,
                self.max_align_delta_deg,
            )
            delta_full = np.zeros(6, dtype=np.float64)
            for local_idx, joint_idx in enumerate(self.align_joints):
                delta_full[joint_idx] = delta_active[local_idx]
            return delta_full

        self.last_align_blocked_joints = [
            self.align_joints[i] for i in blocked_local_indices
        ]
        self.last_align_free_joints = [
            self.align_joints[i] for i in free_local_indices
        ]

        if len(free_local_indices) == 0:
            self.get_logger().warn(
                "ALIGN active-set: all align joints are blocked by soft offset limits. "
                f"blocked_joints={[j + 1 for j in self.last_align_blocked_joints]}, "
                f"q_align_offset={np.round(self.q_align_offset, 3).tolist()}"
            )
            return np.zeros(6, dtype=np.float64)

        # Re-solve with the saturated/pushing joints removed. This redistributes
        # correction to the remaining joints instead of throwing it away at clip().
        J_free = J_full[:, free_local_indices]
        J_free_pinv = self.damped_pseudo_inverse(J_free)
        delta_free = -self.lambda_gain * (J_free_pinv @ feature)
        delta_free = np.clip(
            delta_free,
            -self.max_align_delta_deg,
            self.max_align_delta_deg,
        )

        delta_full = np.zeros(6, dtype=np.float64)
        for out_idx, local_idx in enumerate(free_local_indices):
            joint_idx = self.align_joints[local_idx]
            delta_full[joint_idx] = delta_free[out_idx]

        self.get_logger().warn(
            "ALIGN active-set: saturated/pushing joint(s) excluded. "
            f"blocked_joints={[j + 1 for j in self.last_align_blocked_joints]}, "
            f"free_joints={[j + 1 for j in self.last_align_free_joints]}, "
            f"raw_delta={np.round(delta_active_raw, 4).tolist()}, "
            f"redistributed_delta={np.round(delta_full, 4).tolist()}"
        )
        return delta_full

    # ============================================================
    # State machine helpers
    # ============================================================
    def set_phase(self, new_phase):
        self.phase = new_phase
        self.phase_start_time = self.get_clock().now()
        self.get_logger().info(f"Phase -> {self.phase.name}")

    def elapsed_in_phase(self):
        return (self.get_clock().now() - self.phase_start_time).nanoseconds * 1e-9

    # ============================================================
    # Main timer state machine
    # ============================================================
    def timer_callback(self):
        try:
            if self.phase == Phase.INIT:
                self.set_phase(Phase.MOVE_PREGRASP)

            elif self.phase == Phase.MOVE_PREGRASP:
                self.get_logger().info(f"Moving to pregrasp: {self.pregrasp_angles}")
                self.publish_joint_command(self.pregrasp_angles, self.pregrasp_speed)
                self.set_phase(Phase.WAIT_PREGRASP)

            elif self.phase == Phase.WAIT_PREGRASP:
                if self.elapsed_in_phase() < self.pregrasp_wait_sec:
                    return
                if not self.has_fresh_detection():
                    self.get_logger().warn(
                        "Waiting for fresh detection before align Jacobian estimation..."
                    )
                    return
                if self.use_status_for_q0:
                    self.get_logger().info("Requesting status for q0...")
                    self.request_status()
                    self.set_phase(Phase.WAIT_Q0_STATUS)
                else:
                    self.q0 = np.array(self.pregrasp_angles, dtype=np.float64)
                    self.initialize_controller_states()
                    self.prepare_align_jacobian_estimation()

            elif self.phase == Phase.WAIT_Q0_STATUS:
                if self.has_fresh_status() and self.latest_angles is not None:
                    self.q0 = self.latest_angles.copy()
                    self.get_logger().info(
                        f"q0 from status: {np.round(self.q0, 3).tolist()}"
                    )
                    self.initialize_controller_states()
                    self.prepare_align_jacobian_estimation()
                    return

                if self.elapsed_in_phase() > self.status_timeout_sec:
                    self.get_logger().warn(
                        "Status timeout. Use pregrasp_angles as q0 fallback."
                    )
                    self.q0 = np.array(self.pregrasp_angles, dtype=np.float64)
                    self.initialize_controller_states()
                    self.prepare_align_jacobian_estimation()
                    return

            elif self.phase == Phase.ALIGN_JAC_PLUS_SEND:
                joint_idx = self.align_joints[self.current_align_jac_local_idx]
                q_plus = self.q0.copy()
                q_plus[joint_idx] += self.jacobian_delta_deg
                self.get_logger().info(
                    f"Align Jacobian J{joint_idx + 1}: send +{self.jacobian_delta_deg} deg"
                )
                self.publish_joint_command(q_plus.tolist(), self.command_speed)
                self.set_phase(Phase.ALIGN_JAC_PLUS_WAIT)

            elif self.phase == Phase.ALIGN_JAC_PLUS_WAIT:
                if self.elapsed_in_phase() < self.jacobian_settle_sec:
                    return
                if not self.has_fresh_detection():
                    self.get_logger().error("Detection lost at ALIGN_JAC_PLUS_WAIT")
                    self.set_phase(Phase.ERROR)
                    return
                self.align_f_plus = self.get_center_feature()
                self.get_logger().info(
                    f"align_f_plus={np.round(self.align_f_plus, 5).tolist()}"
                )
                self.set_phase(Phase.ALIGN_JAC_MINUS_SEND)

            elif self.phase == Phase.ALIGN_JAC_MINUS_SEND:
                joint_idx = self.align_joints[self.current_align_jac_local_idx]
                q_minus = self.q0.copy()
                q_minus[joint_idx] -= self.jacobian_delta_deg
                self.get_logger().info(
                    f"Align Jacobian J{joint_idx + 1}: send -{self.jacobian_delta_deg} deg"
                )
                self.publish_joint_command(q_minus.tolist(), self.command_speed)
                self.set_phase(Phase.ALIGN_JAC_MINUS_WAIT)

            elif self.phase == Phase.ALIGN_JAC_MINUS_WAIT:
                if self.elapsed_in_phase() < self.jacobian_settle_sec:
                    return
                if not self.has_fresh_detection():
                    self.get_logger().error("Detection lost at ALIGN_JAC_MINUS_WAIT")
                    self.set_phase(Phase.ERROR)
                    return
                self.align_f_minus = self.get_center_feature()
                self.get_logger().info(
                    f"align_f_minus={np.round(self.align_f_minus, 5).tolist()}"
                )
                col = (self.align_f_plus - self.align_f_minus) / (
                    2.0 * self.jacobian_delta_deg
                )
                self.align_jacobian_cols.append(col)
                joint_idx = self.align_joints[self.current_align_jac_local_idx]
                self.get_logger().info(
                    f"Align Jacobian column J{joint_idx + 1}: "
                    f"{np.round(col, 6).tolist()}"
                )
                self.set_phase(Phase.ALIGN_JAC_BACK_SEND)

            elif self.phase == Phase.ALIGN_JAC_BACK_SEND:
                self.get_logger().info("Return to q0 after align Jacobian perturbation")
                self.publish_joint_command(self.q0.tolist(), self.command_speed)
                self.set_phase(Phase.ALIGN_JAC_BACK_WAIT)

            elif self.phase == Phase.ALIGN_JAC_BACK_WAIT:
                if self.elapsed_in_phase() < self.jacobian_settle_sec:
                    return
                self.current_align_jac_local_idx += 1
                if self.current_align_jac_local_idx >= len(self.align_joints):
                    self.J_align = np.stack(self.align_jacobian_cols, axis=1)
                    self.get_logger().info("Estimated align Jacobian J_align:")
                    self.get_logger().info("\n" + str(self.J_align))
                    self.last_align_jacobian_approach_step = self.approach_step_count
                    self.prev_align_center_norm = None
                    self.align_stuck_count = 0
                    self.last_control_time = None
                    self.set_phase(Phase.RUN)
                else:
                    self.set_phase(Phase.ALIGN_JAC_PLUS_SEND)

            elif self.phase == Phase.RUN:
                self.run_control_step()

            elif self.phase == Phase.AREA_JAC_PLUS_SEND:
                joint_idx = self.approach_joints[self.current_area_jac_local_idx]
                q_plus = self.area_jac_base_q.copy()
                q_plus[joint_idx] += self.area_jacobian_delta_deg
                self.reset_area_window()
                self.get_logger().info(
                    f"Area Jacobian J{joint_idx + 1}: send +{self.area_jacobian_delta_deg} deg"
                )
                self.publish_joint_command(q_plus.tolist(), self.command_speed)
                self.set_phase(Phase.AREA_JAC_PLUS_WAIT)

            elif self.phase == Phase.AREA_JAC_PLUS_WAIT:
                if self.elapsed_in_phase() < self.area_jacobian_settle_sec:
                    return
                if not self.has_fresh_detection():
                    self.get_logger().error("Detection lost at AREA_JAC_PLUS_WAIT")
                    self.set_phase(Phase.ERROR)
                    return
                self.area_f_plus = self.get_area_avg()
                if self.area_f_plus is None:
                    self.get_logger().error("No valid area average at AREA_JAC_PLUS_WAIT")
                    self.set_phase(Phase.ERROR)
                    return
                self.get_logger().info(f"area_f_plus_avg={self.area_f_plus:.6f}")
                if self.check_done_condition(
                    area_value=self.area_f_plus,
                    center_norm=self.get_center_norm(),
                    source="AREA_JAC_PLUS_WAIT",
                ):
                    return
                self.set_phase(Phase.AREA_JAC_MINUS_SEND)

            elif self.phase == Phase.AREA_JAC_MINUS_SEND:
                joint_idx = self.approach_joints[self.current_area_jac_local_idx]
                q_minus = self.area_jac_base_q.copy()
                q_minus[joint_idx] -= self.area_jacobian_delta_deg
                self.reset_area_window()
                self.get_logger().info(
                    f"Area Jacobian J{joint_idx + 1}: send -{self.area_jacobian_delta_deg} deg"
                )
                self.publish_joint_command(q_minus.tolist(), self.command_speed)
                self.set_phase(Phase.AREA_JAC_MINUS_WAIT)

            elif self.phase == Phase.AREA_JAC_MINUS_WAIT:
                if self.elapsed_in_phase() < self.area_jacobian_settle_sec:
                    return
                if not self.has_fresh_detection():
                    self.get_logger().error("Detection lost at AREA_JAC_MINUS_WAIT")
                    self.set_phase(Phase.ERROR)
                    return
                self.area_f_minus = self.get_area_avg()
                if self.area_f_minus is None:
                    self.get_logger().error("No valid area average at AREA_JAC_MINUS_WAIT")
                    self.set_phase(Phase.ERROR)
                    return
                if self.check_done_condition(
                    area_value=self.area_f_minus,
                    center_norm=self.get_center_norm(),
                    source="AREA_JAC_MINUS_WAIT",
                ):
                    return
                col = (self.area_f_plus - self.area_f_minus) / (
                    2.0 * self.area_jacobian_delta_deg
                )
                self.area_jacobian_cols.append(float(col))
                joint_idx = self.approach_joints[self.current_area_jac_local_idx]
                self.get_logger().info(
                    f"Area Jacobian column J{joint_idx + 1}: "
                    f"plus={self.area_f_plus:.6f}, "
                    f"minus={self.area_f_minus:.6f}, "
                    f"darea/dq={col:.8f}"
                )
                self.set_phase(Phase.AREA_JAC_BACK_SEND)

            elif self.phase == Phase.AREA_JAC_BACK_SEND:
                self.get_logger().info("Return to area_jac_base_q")
                self.publish_joint_command(self.area_jac_base_q.tolist(), self.command_speed)
                self.set_phase(Phase.AREA_JAC_BACK_WAIT)

            elif self.phase == Phase.AREA_JAC_BACK_WAIT:
                if self.elapsed_in_phase() < self.area_jacobian_settle_sec:
                    return
                self.current_area_jac_local_idx += 1
                if self.current_area_jac_local_idx >= len(self.approach_joints):
                    self.area_gradient = np.array(self.area_jacobian_cols, dtype=np.float64)
                    self.get_logger().info(
                        f"Estimated area gradient over approach_joints={self.approach_joints}: "
                        f"{np.round(self.area_gradient, 8).tolist()}"
                    )
                    self.execute_area_jacobian_approach_step()
                else:
                    self.set_phase(Phase.AREA_JAC_PLUS_SEND)

            elif self.phase == Phase.APPROACH_WAIT:
                if self.elapsed_in_phase() >= self.approach_wait_sec:
                    if self.has_fresh_detection():
                        if self.check_done_condition(
                            area_value=self.latest_area_norm,
                            center_norm=self.get_center_norm(),
                            source="APPROACH_WAIT",
                        ):
                            return

                    self.validate_cached_area_direction_after_approach()
                    self.area_before_approach = None
                    self.last_approach_source = ""
                    self.last_control_time = None
                    self.set_phase(Phase.RUN)

            elif self.phase == Phase.DONE:
                return

            elif self.phase == Phase.ERROR:
                return

        except Exception as exc:
            self.get_logger().error(f"Exception in timer_callback: {exc}")
            self.set_phase(Phase.ERROR)

    def initialize_controller_states(self):
        self.q_base = self.q0.copy()
        self.q_align_offset = np.zeros(6, dtype=np.float64)
        self.q_last_cmd = self.q_base.copy()
        self.approach_start_q = self.q0.copy()
        self.total_step_count = 0
        self.approach_step_count = 0
        self.area_before_approach = None
        self.last_control_time = None
        self.prev_align_center_norm = None
        self.align_stuck_count = 0
        self.align_rejacobian_count = 0
        self.last_align_rejacobian_time = None
        self.last_align_jacobian_approach_step = 0
        self.last_align_blocked_joints = []
        self.last_align_free_joints = list(self.align_joints)
        self.invalidate_cached_area_direction("controller initialized", log=False)

    def prepare_align_jacobian_estimation(self):
        self.align_jacobian_cols = []
        self.current_align_jac_local_idx = 0
        self.align_f_plus = None
        self.align_f_minus = None
        self.prev_align_center_norm = None
        self.align_stuck_count = 0
        self.last_control_time = None
        self.invalidate_cached_area_direction("align Jacobian estimation started")
        self.get_logger().info(
            f"Base q0 for align Jacobian: {np.round(self.q0, 3).tolist()}"
        )
        self.set_phase(Phase.ALIGN_JAC_PLUS_SEND)

    def prepare_area_jacobian_estimation(self):
        self.area_jac_base_q = self.compose_q_cmd().copy()
        self.area_jacobian_cols = []
        self.current_area_jac_local_idx = 0
        self.area_f_plus = None
        self.area_f_minus = None
        self.area_gradient = None
        self.get_logger().info(
            f"Prepare area Jacobian at q={np.round(self.area_jac_base_q, 3).tolist()}"
        )
        self.set_phase(Phase.AREA_JAC_PLUS_SEND)

    # ============================================================
    # Main control
    # ============================================================
    def run_control_step(self):
        now = self.get_clock().now()
        if self.last_control_time is not None:
            dt = (now - self.last_control_time).nanoseconds * 1e-9
            min_dt = 1.0 / self.control_rate_hz
            if dt < min_dt:
                return
        self.last_control_time = now

        if not self.has_fresh_detection():
            self.get_logger().error("Detection lost during RUN before area+center DONE.")
            self.set_phase(Phase.ERROR)
            return

        center_feature = self.get_center_feature()
        center_norm = float(np.linalg.norm(center_feature))
        v_error = float(center_feature[1])

        self.get_logger().info(
            f"RUN step={self.total_step_count}, approach_step={self.approach_step_count}, "
            f"track={self.latest_track_id}, class={self.latest_class_label}, "
            f"cx={self.latest_cx:.1f}, cy={self.latest_cy:.1f}, "
            f"area_norm={self.latest_area_norm:.5f}, desired_area={self.desired_area_norm:.5f}, "
            f"center_feature={np.round(center_feature, 5).tolist()}, "
            f"center_norm={center_norm:.4f}, area_jac=True"
        )

        if (
            self.hard_stop_below_center_error > 0.0
            and v_error > self.hard_stop_below_center_error
        ):
            self.get_logger().error(
                f"Hard stop: object too far below center. v_error={v_error:.4f}, "
                f"threshold={self.hard_stop_below_center_error:.4f}"
            )
            self.set_phase(Phase.ERROR)
            return

        if self.check_done_condition(
            area_value=self.latest_area_norm,
            center_norm=center_norm,
            source="RUN",
        ):
            return

        if self.total_step_count >= self.max_total_steps:
            self.get_logger().error(
                "ERROR: max_total_steps reached before area+center DONE condition."
            )
            self.set_phase(Phase.ERROR)
            return

        if self.approach_step_count >= self.max_approach_steps:
            self.get_logger().error(
                "ERROR: max_approach_steps reached before area+center DONE condition."
            )
            self.set_phase(Phase.ERROR)
            return

        if center_norm > self.approach_center_threshold:
            self.invalidate_cached_area_direction(
                f"center outside approach threshold: {center_norm:.5f} > "
                f"{self.approach_center_threshold:.5f}"
            )

            if self.should_remeasure_align_jacobian_after_approach():
                self.trigger_align_rebase_and_remeasure(
                    reason=(
                        "approach changed camera geometry: "
                        f"approach_step_count={self.approach_step_count}, "
                        f"last_align_jacobian_approach_step="
                        f"{self.last_align_jacobian_approach_step}"
                    )
                )
                return

            self.execute_align_step(center_norm)
            return

        if self.try_execute_cached_area_approach_step(center_norm):
            return

        self.prepare_area_jacobian_estimation()

    def execute_align_step(self, center_norm):
        q_before = self.compose_q_cmd().copy()
        delta_align = self.compute_align_delta_full()
        self.q_align_offset += delta_align

        for joint_idx in self.align_joints:
            self.q_align_offset[joint_idx] = float(
                np.clip(
                    self.q_align_offset[joint_idx],
                    -self.max_align_offset_deg,
                    self.max_align_offset_deg,
                )
            )

        q_cmd = self.compose_q_cmd()
        cmd_delta_norm = float(np.linalg.norm(q_cmd - q_before))
        saturated_joints = self.get_saturated_align_joints()

        improvement = None
        if self.prev_align_center_norm is not None:
            improvement = float(self.prev_align_center_norm - center_norm)

        stuck_candidate = False
        if improvement is not None:
            no_improvement = improvement < self.align_stuck_min_improvement
            tiny_command = cmd_delta_norm < self.align_stuck_cmd_delta_deg
            saturated_or_blocked = (
                len(saturated_joints) > 0 or len(self.last_align_blocked_joints) > 0
            )
            stuck_candidate = no_improvement and (tiny_command or saturated_or_blocked)

        if stuck_candidate:
            self.align_stuck_count += 1
        else:
            self.align_stuck_count = 0

        improvement_text = "None" if improvement is None else f"{improvement:.5f}"

        self.get_logger().info(
            f"ALIGN: center_norm={center_norm:.4f} > "
            f"approach_center_threshold={self.approach_center_threshold:.4f}, "
            f"delta_align={np.round(delta_align, 3).tolist()}, "
            f"q_align_offset={np.round(self.q_align_offset, 3).tolist()}, "
            f"q_cmd={np.round(q_cmd, 2).tolist()}, "
            f"cmd_delta_norm={cmd_delta_norm:.4f}, "
            f"improvement={improvement_text}, "
            f"stuck_count={self.align_stuck_count}/{self.align_stuck_frames}, "
            f"saturated_joints={[j + 1 for j in saturated_joints]}, "
            f"blocked_joints={[j + 1 for j in self.last_align_blocked_joints]}"
        )

        self.prev_align_center_norm = center_norm

        if (
            self.enable_align_stuck_recovery
            and self.align_stuck_count >= self.align_stuck_frames
        ):
            self.trigger_align_rebase_and_remeasure(
                reason=(
                    "ALIGN stuck: "
                    f"center_norm={center_norm:.5f}, "
                    f"cmd_delta_norm={cmd_delta_norm:.5f}, "
                    f"saturated_joints={[j + 1 for j in saturated_joints]}, "
                    f"blocked_joints={[j + 1 for j in self.last_align_blocked_joints]}, "
                    f"q_align_offset={np.round(self.q_align_offset, 3).tolist()}"
                )
            )
            return

        self.publish_joint_command(q_cmd.tolist(), self.command_speed)
        self.total_step_count += 1

    def get_saturated_align_joints(self):
        sat_limit = self.max_align_offset_deg * self.align_stuck_saturation_ratio
        return [
            joint_idx
            for joint_idx in self.align_joints
            if abs(float(self.q_align_offset[joint_idx])) >= sat_limit
        ]

    def can_trigger_align_rejacobian(self):
        if not self.enable_align_stuck_recovery:
            return False

        if self.align_rejacobian_count >= self.max_align_rejacobian_count:
            return False

        if self.last_align_rejacobian_time is None:
            return True

        elapsed = (
            self.get_clock().now() - self.last_align_rejacobian_time
        ).nanoseconds * 1e-9
        return elapsed >= self.align_rejacobian_cooldown_sec

    def should_remeasure_align_jacobian_after_approach(self):
        if self.align_rejacobian_after_approach_steps <= 0:
            return False

        delta_steps = (
            self.approach_step_count - self.last_align_jacobian_approach_step
        )
        if delta_steps < self.align_rejacobian_after_approach_steps:
            return False

        if not self.can_trigger_align_rejacobian():
            return False

        return True

    def get_rebase_q(self):
        if self.use_status_for_q0 and self.has_fresh_status() and self.latest_angles is not None:
            return self.latest_angles.copy(), "fresh_status"
        return self.compose_q_cmd().copy(), "current_command"

    def trigger_align_rebase_and_remeasure(self, reason):
        if not self.can_trigger_align_rejacobian():
            self.get_logger().warn(
                "ALIGN rebase/re-Jacobian requested but skipped. "
                f"count={self.align_rejacobian_count}/"
                f"{self.max_align_rejacobian_count}, reason={reason}"
            )
            return False

        q_rebase, q_source = self.get_rebase_q()
        self.q0 = q_rebase.copy()
        self.q_base = q_rebase.copy()
        self.q_align_offset = np.zeros(6, dtype=np.float64)
        self.q_last_cmd = q_rebase.copy()

        # For diagnostic distance only; no near-grasp limit is used.
        self.approach_start_q = self.q_base.copy()

        self.align_rejacobian_count += 1
        self.last_align_rejacobian_time = self.get_clock().now()
        self.prev_align_center_norm = None
        self.align_stuck_count = 0
        self.last_align_blocked_joints = []
        self.last_align_free_joints = list(self.align_joints)
        self.last_control_time = None
        self.area_before_approach = None
        self.last_approach_source = ""
        self.invalidate_cached_area_direction("align rebase/re-Jacobian")

        self.get_logger().warn(
            "ALIGN REBASE + RE-JACOBIAN: "
            f"reason={reason}, "
            f"q_source={q_source}, "
            f"new_q0={np.round(self.q0, 3).tolist()}, "
            f"rejac_count={self.align_rejacobian_count}/"
            f"{self.max_align_rejacobian_count}"
        )

        self.prepare_align_jacobian_estimation()
        return True

    def set_cached_area_direction(self, direction_full, gradient, grad_norm):
        self.cached_area_direction_valid = True
        self.cached_area_direction_full = np.array(direction_full, dtype=np.float64)
        self.cached_area_gradient = np.array(gradient, dtype=np.float64)
        self.cached_area_grad_norm = float(grad_norm)
        self.cached_area_direction_use_count = 0
        self.get_logger().info(
            "AREA_DIRECTION_CACHE set: "
            f"direction={np.round(self.cached_area_direction_full, 5).tolist()}, "
            f"gradient={np.round(self.cached_area_gradient, 8).tolist()}, "
            f"grad_norm={self.cached_area_grad_norm:.8f}, "
            f"reuse_budget={self.area_jacobian_reuse_steps} approach move(s)"
        )

    def invalidate_cached_area_direction(self, reason, log=True):
        if log and self.cached_area_direction_valid:
            self.get_logger().info(f"AREA_DIRECTION_CACHE invalidated: {reason}")
        self.cached_area_direction_valid = False
        self.cached_area_direction_full = None
        self.cached_area_gradient = None
        self.cached_area_grad_norm = 0.0
        self.cached_area_direction_use_count = 0

    def try_execute_cached_area_approach_step(self, center_norm):
        if not self.cached_area_direction_valid:
            return False

        if self.cached_area_direction_full is None:
            self.invalidate_cached_area_direction("cache direction is None")
            return False

        if self.cached_area_direction_use_count >= self.area_jacobian_reuse_steps:
            self.invalidate_cached_area_direction(
                f"reuse budget consumed: "
                f"{self.cached_area_direction_use_count}/{self.area_jacobian_reuse_steps}"
            )
            return False

        if center_norm > self.approach_center_threshold:
            self.invalidate_cached_area_direction(
                f"center outside approach threshold before cached approach: "
                f"{center_norm:.5f} > {self.approach_center_threshold:.5f}"
            )
            return False

        self.cached_area_direction_use_count += 1
        self.execute_area_approach_with_direction(
            direction_full=self.cached_area_direction_full,
            source="CACHE",
        )
        return True

    def validate_cached_area_direction_after_approach(self):
        if not self.has_fresh_detection():
            self.invalidate_cached_area_direction("no fresh detection after approach")
            return

        if self.area_before_approach is None:
            return

        area_now = float(self.latest_area_norm)
        area_gain = area_now - float(self.area_before_approach)
        area_ratio = area_gain / max(float(self.area_before_approach), 1e-6)
        center_norm = self.get_center_norm()

        self.get_logger().info(
            f"APPROACH result ({self.last_approach_source}): "
            f"area_before={self.area_before_approach:.5f}, "
            f"area_now={area_now:.5f}, "
            f"area_gain={area_gain:.5f}, "
            f"area_ratio={area_ratio:.4f}, "
            f"center_norm={center_norm:.5f}, "
            f"cache_valid={self.cached_area_direction_valid}, "
            f"cache_use={self.cached_area_direction_use_count}/"
            f"{self.area_jacobian_reuse_steps}"
        )

        if not self.cached_area_direction_valid:
            return

        if center_norm > self.approach_center_threshold:
            self.invalidate_cached_area_direction(
                f"center drifted outside threshold after approach: "
                f"{center_norm:.5f} > {self.approach_center_threshold:.5f}"
            )
            return

        if area_gain < -self.area_drop_tolerance:
            self.invalidate_cached_area_direction(
                f"area dropped too much: gain={area_gain:.5f} < "
                f"-{self.area_drop_tolerance:.5f}"
            )
            return

        if area_gain < self.area_min_gain_for_reuse:
            self.invalidate_cached_area_direction(
                f"area gain too small for reuse: gain={area_gain:.5f} < "
                f"min_gain={self.area_min_gain_for_reuse:.5f}"
            )
            return

        if self.cached_area_direction_use_count >= self.area_jacobian_reuse_steps:
            self.invalidate_cached_area_direction(
                f"reuse budget consumed after approach: "
                f"{self.cached_area_direction_use_count}/"
                f"{self.area_jacobian_reuse_steps}"
            )
            return

        self.get_logger().info(
            "AREA_DIRECTION_CACHE keep: "
            f"area_gain={area_gain:.5f}, "
            f"center_norm={center_norm:.5f}, "
            f"next_cached_use={self.cached_area_direction_use_count + 1}/"
            f"{self.area_jacobian_reuse_steps}"
        )

    def execute_area_approach_with_direction(self, direction_full, source):
        if self.approach_step_count >= self.max_approach_steps:
            self.get_logger().error(
                "ERROR: max_approach_steps reached before area+center DONE condition."
            )
            self.set_phase(Phase.ERROR)
            return

        direction_full = np.array(direction_full, dtype=np.float64)
        direction_norm_active = float(
            np.linalg.norm(direction_full[self.approach_joints])
        )
        if direction_norm_active < 1e-9:
            self.get_logger().error(
                f"Invalid approach direction for source={source}: "
                f"{np.round(direction_full, 6).tolist()}"
            )
            self.invalidate_cached_area_direction("invalid approach direction")
            self.set_phase(Phase.ERROR)
            return

        # Normalize again for safety, but only over approach joints.
        direction_full = direction_full.copy()
        direction_full[self.approach_joints] /= direction_norm_active

        step = self.approach_step_deg
        for joint_idx in self.approach_joints:
            self.q_base[joint_idx] += step * direction_full[joint_idx]

        q_cmd = self.compose_q_cmd()
        self.area_before_approach = float(self.latest_area_norm)
        self.last_approach_source = source

        # Distance is only diagnostic now. It is not a limit.
        approach_delta = self.q_base - self.approach_start_q
        approach_distance_diag = float(np.linalg.norm(approach_delta[self.approach_joints]))

        if source == "CACHE":
            cache_info = (
                f"cache_use={self.cached_area_direction_use_count}/"
                f"{self.area_jacobian_reuse_steps}"
            )
        else:
            cache_info = (
                f"cache_use={self.cached_area_direction_use_count}/"
                f"{self.area_jacobian_reuse_steps}"
            )

        self.get_logger().info(
            f"APPROACH_AREA_{source}: "
            f"direction={np.round(direction_full, 5).tolist()}, "
            f"step={step:.3f}, "
            f"{cache_info}, "
            f"approach_distance_diag={approach_distance_diag:.3f}, "
            f"area_before={self.area_before_approach:.5f}, "
            f"q_base={np.round(self.q_base, 3).tolist()}, "
            f"q_align_offset={np.round(self.q_align_offset, 3).tolist()}, "
            f"q_cmd={np.round(q_cmd, 2).tolist()}"
        )

        self.publish_joint_command(q_cmd.tolist(), self.command_speed)
        self.approach_step_count += 1
        self.total_step_count += 1
        self.set_phase(Phase.APPROACH_WAIT)

    def execute_area_jacobian_approach_step(self):
        if self.approach_step_count >= self.max_approach_steps:
            self.get_logger().error(
                "ERROR: max_approach_steps reached before area+center DONE condition."
            )
            self.set_phase(Phase.ERROR)
            return

        if self.area_gradient is None or len(self.area_gradient) != len(self.approach_joints):
            self.get_logger().error("Invalid area gradient.")
            self.set_phase(Phase.ERROR)
            return

        grad_norm = float(np.linalg.norm(self.area_gradient))
        if grad_norm < self.area_jacobian_min_grad:
            self.get_logger().error(
                f"ERROR: area gradient too small. norm={grad_norm:.8f}, "
                f"threshold={self.area_jacobian_min_grad:.8f}"
            )
            self.invalidate_cached_area_direction("area gradient too small")
            self.set_phase(Phase.ERROR)
            return

        direction_active = self.area_gradient / grad_norm
        direction_full = np.zeros(6, dtype=np.float64)
        for local_idx, joint_idx in enumerate(self.approach_joints):
            direction_full[joint_idx] = direction_active[local_idx]

        self.get_logger().info(
            f"APPROACH_AREA_MEASURED_GRADIENT: "
            f"gradient={np.round(self.area_gradient, 8).tolist()}, "
            f"grad_norm={grad_norm:.8f}, "
            f"direction={np.round(direction_full, 5).tolist()}"
        )

        self.set_cached_area_direction(direction_full, self.area_gradient, grad_norm)
        self.cached_area_direction_use_count = 1
        self.execute_area_approach_with_direction(
            direction_full=direction_full,
            source="MEASURED",
        )

def main(args=None):
    rclpy.init(args=args)
    node = AreaJacobianIBVSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()