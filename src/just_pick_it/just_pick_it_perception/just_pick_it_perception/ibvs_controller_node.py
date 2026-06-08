#!/usr/bin/env python3

import math
from enum import Enum

import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, Empty

# TODO:
# 실제 인터페이스 패키지 이름이 다르면 이 import만 수정하세요.
# 예:
# from jetcobot_interfaces.msg import TrackedObjectArray
# from your_package_name.msg import TrackedObjectArray
from just_pick_it_interfaces.msg import TrackedObjectArray


CMD_JOINT = 0


class Phase(Enum):
    INIT = 0
    MOVE_PREGRASP = 1
    WAIT_PREGRASP = 2
    REQUEST_Q0_STATUS = 3
    WAIT_Q0_STATUS = 4

    JAC_PLUS_SEND = 5
    JAC_PLUS_WAIT = 6
    JAC_MINUS_SEND = 7
    JAC_MINUS_WAIT = 8
    JAC_BACK_SEND = 9
    JAC_BACK_WAIT = 10

    RUN_IBVS = 11
    DONE = 12
    ERROR = 13


class SimpleIBVSPublisherNode(Node):
    """
    AI server side IBVS node.

    This node does NOT connect to MyCobot280 directly.

    It publishes joint commands to:

        /{robot_name}/target_pose

    Message format expected by the Jetcobot command subscriber:

        Float64MultiArray.data =
        [
            CMD_JOINT,
            q1, q2, q3, q4, q5, q6,
            speed
        ]

    It subscribes to detection:

        TrackedObjectArray

    and optionally uses status:

        /{robot_name}/status
        /{robot_name}/request_status

    Status layout from your subscriber:

        data[0:6]    tool_reference
        data[6:12]   world_reference
        data[12]     reference_frame
        data[13]     end_type
        data[14:20]  angles
        data[20:26]  coords
        data[26]     gripper_value
    """

    def __init__(self):
        super().__init__("simple_ibvs_publisher_node")

        # ============================================================
        # Parameters
        # ============================================================
        self.declare_parameter("robot_name", "jetcobot1")

        self.declare_parameter("image_width", 640.0)
        self.declare_parameter("image_height", 480.0)

        self.declare_parameter("detection_topic", "/infer/tracked_objects")
        self.declare_parameter("detection_timeout_sec", 0.5)
        self.declare_parameter("min_confidence", 0.3)

        # ""이면 class filtering 안 함
        self.declare_parameter("target_class_label", "watermelon")

        # True면 첫 target의 track_id를 고정 추적
        self.declare_parameter("lock_track_id", True)

        # "bbox" 또는 "mask"
        # 초기 IBVS는 bbox 추천
        self.declare_parameter("center_source", "bbox")

        # q1, q2, q3만 사용
        # index 기준: 0=q1, 1=q2, 2=q3, ...
        self.declare_parameter("active_joints", [0, 1, 2])

        self.declare_parameter(
            "pregrasp_angles",
            [51.74, 55.12, -30.44, -77.64, 34.60, -135.11],
        )

        self.declare_parameter("pregrasp_speed", 15)
        self.declare_parameter("ibvs_speed", 10)

        # empirical image Jacobian 측정용
        self.declare_parameter("jacobian_delta_deg", 1.0)
        self.declare_parameter("jacobian_settle_sec", 0.8)

        # IBVS gain
        self.declare_parameter("lambda_gain", 0.5)
        self.declare_parameter("damping", 0.08)
        self.declare_parameter("max_delta_deg", 0.2)

        self.declare_parameter("control_rate_hz", 3.0)
        self.declare_parameter("stop_error", 0.03)
        self.declare_parameter("max_steps", 100)

        # pregrasp 이동 후 기다리는 시간
        self.declare_parameter("pregrasp_wait_sec", 3.0)

        # status를 q0 초기화에 사용할지 여부
        # False면 pregrasp_angles를 q0로 사용
        self.declare_parameter("use_status_for_q0", True)
        self.declare_parameter("status_timeout_sec", 1.0)

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
        self.lock_track_id = bool(self.get_parameter("lock_track_id").value)
        self.center_source = str(self.get_parameter("center_source").value).lower()

        if self.center_source not in ["bbox", "mask"]:
            self.get_logger().warn(
                f"Invalid center_source='{self.center_source}'. "
                "Use 'bbox' as fallback."
            )
            self.center_source = "bbox"

        self.active_joints = list(self.get_parameter("active_joints").value)
        self.pregrasp_angles = list(self.get_parameter("pregrasp_angles").value)

        self.pregrasp_speed = int(self.get_parameter("pregrasp_speed").value)
        self.ibvs_speed = int(self.get_parameter("ibvs_speed").value)

        self.jacobian_delta_deg = float(
            self.get_parameter("jacobian_delta_deg").value
        )
        self.jacobian_settle_sec = float(
            self.get_parameter("jacobian_settle_sec").value
        )

        self.lambda_gain = float(self.get_parameter("lambda_gain").value)
        self.damping = float(self.get_parameter("damping").value)
        self.max_delta_deg = float(self.get_parameter("max_delta_deg").value)

        self.control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.stop_error = float(self.get_parameter("stop_error").value)
        self.max_steps = int(self.get_parameter("max_steps").value)

        self.pregrasp_wait_sec = float(self.get_parameter("pregrasp_wait_sec").value)

        self.use_status_for_q0 = bool(self.get_parameter("use_status_for_q0").value)
        self.status_timeout_sec = float(self.get_parameter("status_timeout_sec").value)

        # ============================================================
        # Publishers / Subscribers
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

        self.target_track_id = None

        self.latest_track_id = -1
        self.latest_class_id = -1
        self.latest_class_label = ""
        self.latest_bbox_x = 0.0
        self.latest_bbox_y = 0.0
        self.latest_bbox_w = 0.0
        self.latest_bbox_h = 0.0
        self.latest_mask_cx = 0.0
        self.latest_mask_cy = 0.0
        self.latest_orientation_angle = 0.0

        # ============================================================
        # Status state
        # ============================================================
        self.latest_status_time = None
        self.latest_angles = None
        self.latest_coords = None
        self.latest_gripper_value = None

        # 현재 명령 기준 추정 관절값
        # Jetcobot 내부 subscriber가 joint command 후 status를 자동 publish하지 않으므로,
        # IBVS loop에서는 commanded angle을 q estimate로 유지한다.
        self.q_est = np.array(self.pregrasp_angles, dtype=np.float64)

        # ============================================================
        # IBVS state
        # ============================================================
        self.phase = Phase.INIT
        self.phase_start_time = self.get_clock().now()

        self.q0 = None

        self.jacobian_cols = []
        self.current_jac_joint_local_idx = 0

        self.f_plus = None
        self.f_minus = None
        self.J = None

        self.ibvs_step = 0
        self.last_control_time = None

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.get_logger().info("Simple IBVS publisher node started")
        self.get_logger().info(f"robot_name={self.robot_name}")
        self.get_logger().info(f"namespace={self.ns}")
        self.get_logger().info(f"Pub: {self.ns}/target_pose")
        self.get_logger().info(f"Pub: {self.ns}/request_status")
        self.get_logger().info(f"Sub: {self.ns}/status")
        self.get_logger().info(f"Sub detection: {self.detection_topic}")
        self.get_logger().info(f"target_class_label={self.target_class_label}")
        self.get_logger().info(f"center_source={self.center_source}")
        self.get_logger().info(f"active_joints={self.active_joints}")

    # ============================================================
    # Command publisher
    # ============================================================
    def publish_joint_command(self, angles, speed):
        if len(angles) != 6:
            self.get_logger().error(f"Invalid angles length: {len(angles)}")
            return

        speed = int(max(1, min(100, speed)))

        msg = Float64MultiArray()
        msg.data = [float(CMD_JOINT)] + [float(v) for v in angles] + [float(speed)]

        self.target_pose_pub.publish(msg)

        self.q_est = np.array(angles, dtype=np.float64)

        self.get_logger().info(
            f"Publish joint command: angles={np.round(self.q_est, 3).tolist()}, "
            f"speed={speed}"
        )

    def request_status(self):
        self.status_request_pub.publish(Empty())

    # ============================================================
    # Status callback
    # ============================================================
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

        self.get_logger().debug(
            f"Status received: angles={np.round(self.latest_angles, 3).tolist()}, "
            f"coords={np.round(self.latest_coords, 3).tolist()}, "
            f"gripper={self.latest_gripper_value:.1f}"
        )

    def has_fresh_status(self):
        if self.latest_status_time is None:
            return False

        now = self.get_clock().now()
        age = (now - self.latest_status_time).nanoseconds * 1e-9

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

        # ------------------------------------------------------------
        # 1. 이미 target_track_id가 고정되어 있으면 해당 track 우선 추적
        # ------------------------------------------------------------
        if self.lock_track_id and self.target_track_id is not None:
            for obj in msg.objects:
                if int(obj.track_id) == int(self.target_track_id):
                    if float(obj.confidence) >= self.min_confidence:
                        best_obj = obj
                    break

        # ------------------------------------------------------------
        # 2. target track을 못 찾았거나 아직 없으면 새 target 선택
        # ------------------------------------------------------------
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
                    f"class={best_obj.class_label}, "
                    f"conf={best_obj.confidence:.3f}"
                )

        if best_obj is None:
            self.latest_valid = False
            self.latest_detection_time = self.get_clock().now()
            return

        # ------------------------------------------------------------
        # 3. bbox center 계산
        # ------------------------------------------------------------
        bbox_x = float(best_obj.bbox_x)
        bbox_y = float(best_obj.bbox_y)
        bbox_w = float(best_obj.bbox_w)
        bbox_h = float(best_obj.bbox_h)

        bbox_cx = bbox_x
        bbox_cy = bbox_y

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

        # ------------------------------------------------------------
        # 4. mask center 계산
        # ------------------------------------------------------------
        mask_cx = float(best_obj.mask_cx)
        mask_cy = float(best_obj.mask_cy)

        mask_valid = (
            math.isfinite(mask_cx)
            and math.isfinite(mask_cy)
            and 0.0 <= mask_cx <= self.image_w
            and 0.0 <= mask_cy <= self.image_h
        )

        # ------------------------------------------------------------
        # 5. 사용할 center 선택
        # ------------------------------------------------------------
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

        # ------------------------------------------------------------
        # 6. latest detection 저장
        # ------------------------------------------------------------
        self.latest_valid = True
        self.latest_cx = cx
        self.latest_cy = cy
        self.latest_conf = float(best_obj.confidence)
        self.latest_detection_time = self.get_clock().now()

        self.latest_track_id = int(best_obj.track_id)
        self.latest_class_id = int(best_obj.class_id)
        self.latest_class_label = str(best_obj.class_label)

        self.latest_bbox_x = bbox_x
        self.latest_bbox_y = bbox_y
        self.latest_bbox_w = bbox_w
        self.latest_bbox_h = bbox_h
        self.latest_mask_cx = mask_cx
        self.latest_mask_cy = mask_cy
        self.latest_orientation_angle = float(best_obj.orientation_angle)

        self.get_logger().debug(
            f"target track={self.latest_track_id}, "
            f"class={self.latest_class_label}, "
            f"center_source={self.center_source}, "
            f"used_center=({cx:.1f}, {cy:.1f}), "
            f"bbox_center=({bbox_cx:.1f}, {bbox_cy:.1f}), "
            f"mask_center=({mask_cx:.1f}, {mask_cy:.1f}), "
            f"conf={self.latest_conf:.3f}"
        )

    # ============================================================
    # Detection utilities
    # ============================================================
    def has_fresh_detection(self):
        if self.latest_detection_time is None:
            return False

        now = self.get_clock().now()
        age = (now - self.latest_detection_time).nanoseconds * 1e-9

        if age > self.detection_timeout_sec:
            return False

        if not self.latest_valid:
            return False

        if self.latest_conf < self.min_confidence:
            return False

        return True

    def get_center_error(self):
        u = (self.latest_cx - self.image_w / 2.0) / self.image_w
        v = (self.latest_cy - self.image_h / 2.0) / self.image_h

        return np.array([u, v], dtype=np.float64)

    # ============================================================
    # Math
    # ============================================================
    def damped_pseudo_inverse(self, J):
        J = np.asarray(J, dtype=np.float64)
        m = J.shape[0]
        identity = np.eye(m)

        return J.T @ np.linalg.inv(J @ J.T + (self.damping ** 2) * identity)

    def compute_delta_q_full(self, error):
        if self.J is None:
            raise RuntimeError("Jacobian is not estimated yet.")

        error = np.asarray(error, dtype=np.float64).reshape(-1)

        J_pinv = self.damped_pseudo_inverse(self.J)

        delta_active = -self.lambda_gain * (J_pinv @ error)

        delta_active = np.clip(
            delta_active,
            -self.max_delta_deg,
            self.max_delta_deg,
        )

        delta_full = np.zeros(6, dtype=np.float64)

        for local_idx, joint_idx in enumerate(self.active_joints):
            delta_full[joint_idx] = delta_active[local_idx]

        return delta_full

    # ============================================================
    # State machine helpers
    # ============================================================
    def set_phase(self, new_phase):
        self.phase = new_phase
        self.phase_start_time = self.get_clock().now()
        self.get_logger().info(f"Phase -> {self.phase.name}")

    def elapsed_in_phase(self):
        now = self.get_clock().now()
        return (now - self.phase_start_time).nanoseconds * 1e-9

    # ============================================================
    # Main timer state machine
    # ============================================================
    def timer_callback(self):
        try:
            if self.phase == Phase.INIT:
                self.set_phase(Phase.MOVE_PREGRASP)

            elif self.phase == Phase.MOVE_PREGRASP:
                self.get_logger().info(
                    f"Moving to pregrasp: {self.pregrasp_angles}"
                )
                self.publish_joint_command(self.pregrasp_angles, self.pregrasp_speed)
                self.set_phase(Phase.WAIT_PREGRASP)

            elif self.phase == Phase.WAIT_PREGRASP:
                if self.elapsed_in_phase() < self.pregrasp_wait_sec:
                    return

                if not self.has_fresh_detection():
                    self.get_logger().warn(
                        "Waiting for fresh detection before Jacobian estimation..."
                    )
                    return

                if self.use_status_for_q0:
                    self.get_logger().info("Requesting status for q0...")
                    self.request_status()
                    self.set_phase(Phase.WAIT_Q0_STATUS)
                else:
                    self.q0 = np.array(self.pregrasp_angles, dtype=np.float64)
                    self.q_est = self.q0.copy()
                    self.prepare_jacobian_estimation()

            elif self.phase == Phase.WAIT_Q0_STATUS:
                if self.has_fresh_status() and self.latest_angles is not None:
                    self.q0 = self.latest_angles.copy()
                    self.q_est = self.q0.copy()

                    self.get_logger().info(
                        f"q0 from status: {np.round(self.q0, 3).tolist()}"
                    )
                    self.prepare_jacobian_estimation()
                    return

                if self.elapsed_in_phase() > self.status_timeout_sec:
                    self.get_logger().warn(
                        "Status timeout. Use pregrasp_angles as q0 fallback."
                    )
                    self.q0 = np.array(self.pregrasp_angles, dtype=np.float64)
                    self.q_est = self.q0.copy()
                    self.prepare_jacobian_estimation()
                    return

            elif self.phase == Phase.JAC_PLUS_SEND:
                joint_idx = self.active_joints[self.current_jac_joint_local_idx]

                q_plus = self.q0.copy()
                q_plus[joint_idx] += self.jacobian_delta_deg

                self.get_logger().info(
                    f"Jacobian q{joint_idx + 1}: "
                    f"send +{self.jacobian_delta_deg} deg"
                )
                self.publish_joint_command(q_plus.tolist(), self.ibvs_speed)
                self.set_phase(Phase.JAC_PLUS_WAIT)

            elif self.phase == Phase.JAC_PLUS_WAIT:
                if self.elapsed_in_phase() < self.jacobian_settle_sec:
                    return

                if not self.has_fresh_detection():
                    self.get_logger().error("Detection lost at JAC_PLUS_WAIT")
                    self.set_phase(Phase.ERROR)
                    return

                self.f_plus = self.get_center_error()
                self.get_logger().info(
                    f"f_plus={np.round(self.f_plus, 5).tolist()}"
                )
                self.set_phase(Phase.JAC_MINUS_SEND)

            elif self.phase == Phase.JAC_MINUS_SEND:
                joint_idx = self.active_joints[self.current_jac_joint_local_idx]

                q_minus = self.q0.copy()
                q_minus[joint_idx] -= self.jacobian_delta_deg

                self.get_logger().info(
                    f"Jacobian q{joint_idx + 1}: "
                    f"send -{self.jacobian_delta_deg} deg"
                )
                self.publish_joint_command(q_minus.tolist(), self.ibvs_speed)
                self.set_phase(Phase.JAC_MINUS_WAIT)

            elif self.phase == Phase.JAC_MINUS_WAIT:
                if self.elapsed_in_phase() < self.jacobian_settle_sec:
                    return

                if not self.has_fresh_detection():
                    self.get_logger().error("Detection lost at JAC_MINUS_WAIT")
                    self.set_phase(Phase.ERROR)
                    return

                self.f_minus = self.get_center_error()
                self.get_logger().info(
                    f"f_minus={np.round(self.f_minus, 5).tolist()}"
                )

                col = (self.f_plus - self.f_minus) / (
                    2.0 * self.jacobian_delta_deg
                )
                self.jacobian_cols.append(col)

                joint_idx = self.active_joints[self.current_jac_joint_local_idx]
                self.get_logger().info(
                    f"Jacobian column q{joint_idx + 1}: "
                    f"{np.round(col, 6).tolist()}"
                )

                self.set_phase(Phase.JAC_BACK_SEND)

            elif self.phase == Phase.JAC_BACK_SEND:
                self.get_logger().info("Return to q0")
                self.publish_joint_command(self.q0.tolist(), self.ibvs_speed)
                self.set_phase(Phase.JAC_BACK_WAIT)

            elif self.phase == Phase.JAC_BACK_WAIT:
                if self.elapsed_in_phase() < self.jacobian_settle_sec:
                    return

                self.current_jac_joint_local_idx += 1

                if self.current_jac_joint_local_idx >= len(self.active_joints):
                    self.J = np.stack(self.jacobian_cols, axis=1)

                    self.get_logger().info("Estimated image Jacobian J:")
                    self.get_logger().info("\n" + str(self.J))

                    self.ibvs_step = 0
                    self.last_control_time = None
                    self.q_est = self.q0.copy()
                    self.set_phase(Phase.RUN_IBVS)
                else:
                    self.set_phase(Phase.JAC_PLUS_SEND)

            elif self.phase == Phase.RUN_IBVS:
                self.run_ibvs_step()

            elif self.phase == Phase.DONE:
                return

            elif self.phase == Phase.ERROR:
                return

        except Exception as exc:
            self.get_logger().error(f"Exception in timer_callback: {exc}")
            self.set_phase(Phase.ERROR)

    def prepare_jacobian_estimation(self):
        self.jacobian_cols = []
        self.current_jac_joint_local_idx = 0

        self.get_logger().info(
            f"Base q0 for Jacobian: {np.round(self.q0, 3).tolist()}"
        )
        self.set_phase(Phase.JAC_PLUS_SEND)

    # ============================================================
    # IBVS control loop
    # ============================================================
    def run_ibvs_step(self):
        now = self.get_clock().now()

        if self.last_control_time is not None:
            dt = (now - self.last_control_time).nanoseconds * 1e-9
            min_dt = 1.0 / self.control_rate_hz
            if dt < min_dt:
                return

        self.last_control_time = now

        if not self.has_fresh_detection():
            self.get_logger().error("Detection lost during IBVS. Stop.")
            self.set_phase(Phase.ERROR)
            return

        error = self.get_center_error()
        err_norm = float(np.linalg.norm(error))

        self.get_logger().info(
            f"IBVS step={self.ibvs_step}, "
            f"track={self.latest_track_id}, "
            f"class={self.latest_class_label}, "
            f"cx={self.latest_cx:.1f}, "
            f"cy={self.latest_cy:.1f}, "
            f"conf={self.latest_conf:.2f}, "
            f"error=({error[0]:.4f}, {error[1]:.4f}), "
            f"norm={err_norm:.4f}"
        )

        if err_norm < self.stop_error:
            self.get_logger().info("IBVS success: center aligned.")
            self.set_phase(Phase.DONE)
            return

        if self.ibvs_step >= self.max_steps:
            self.get_logger().warn("IBVS stopped: max_steps reached.")
            self.set_phase(Phase.DONE)
            return

        q_current = self.q_est.copy()
        delta_q = self.compute_delta_q_full(error)
        q_cmd = q_current + delta_q

        self.get_logger().info(
            f"q_est={np.round(q_current, 2).tolist()}, "
            f"delta_q={np.round(delta_q, 3).tolist()}, "
            f"q_cmd={np.round(q_cmd, 2).tolist()}"
        )

        self.publish_joint_command(q_cmd.tolist(), self.ibvs_speed)

        self.ibvs_step += 1


def main(args=None):
    rclpy.init(args=args)

    node = SimpleIBVSPublisherNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()