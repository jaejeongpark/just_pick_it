#!/usr/bin/env python3

"""
NN Controller 추론 노드 (PLAN.md Task 8, 1단계).

역할 (D1): align + approach + J6 정렬은 ibvs_controller가 수행하고, ibvs_done 수신 후
활성화되어 J1~J5 fine-tune + grip을 담당한다.

설계 (확정):
  - 입력 53차원 = anchor[3] (cx,cy,area) + (J1~J5 정규화[5] + dJ1~dJ5 스케일[5]) × window5.
    detection은 활성화 시점 마지막 유효값으로 동결한다(D3).
  - J6는 NN에서 제외한다. ibvs_controller가 ibvs_done 직전에 OBB 장축으로 J6를
    결정론적 정렬·고정하므로, 이 노드는 J6를 현재값으로 passthrough(유지)만 한다.
  - Policy는 J1~J5 delta만 출력. grip(0/1)은 Grip Success Predictor가 전담:
    P(success)가 grip_confidence_threshold를 grip_consecutive_required step 연속 넘으면
    1회 grip(latch). max_fine_tune_steps 안전장치.
  - 추론 중 arm은 powered 상태 유지(release_all_servos 호출 안 함).

구독:
  /{robot_name}/ibvs_done (Empty)             : 활성화 트리거
  detection_topic (TrackedObjectArray)        : 활성화 직전 마지막 유효 detection 동결용
  /{robot_name}/status (Float64MultiArray)    : 현재 관절/그리퍼

발행:
  /{robot_name}/target_pose (Float64MultiArray): [CMD_JOINT, q1..q6, speed]
  /{robot_name}/set_gripper (Float64MultiArray): [0, speed] (close)
  /{robot_name}/request_status (Empty)
"""

import math
from collections import deque
from enum import Enum
from pathlib import Path

import numpy as np

import rclpy
from rclpy.node import Node

import torch

from std_msgs.msg import Empty, Float64MultiArray

from just_pick_it_interfaces.msg import TrackedObjectArray

from just_pick_it_perception.nn_controller_model import (
    FeatureBuilder,
    load_config,
    load_policy,
    load_grip_predictor,
    N_JOINTS,
)


CMD_JOINT = 0


class Phase(Enum):
    IDLE = 0
    RUN = 3
    GRIP = 4
    GRIP_WAIT = 5
    DONE = 6
    ERROR = 99


class NNControllerNode(Node):
    def __init__(self):
        super().__init__("nn_controller")

        # ============================================================
        # Parameters
        # ============================================================
        self.declare_parameter("robot_name", "jetcobot1")
        self.declare_parameter("detection_topic", "/infer/tracked_objects")
        self.declare_parameter("target_class_label", "")
        self.declare_parameter("min_confidence", 0.5)
        self.declare_parameter("image_width", 640.0)
        self.declare_parameter("image_height", 480.0)

        self.declare_parameter(
            "model_dir",
            str(Path.home()
                / "just_pick_it/src/just_pick_it/just_pick_it_perception"
                / "result/nn_controller"),
        )
        self.declare_parameter("device", "cpu")

        # 기록(human recorder)과 동일하게 5Hz 고정.
        self.declare_parameter("control_rate_hz", 5.0)
        self.declare_parameter("status_timeout_sec", 1.0)
        self.declare_parameter("command_speed", 10)
        self.declare_parameter("gripper_speed", 50)

        # Grip gate.
        self.declare_parameter("grip_confidence_threshold", 0.8)
        self.declare_parameter("grip_consecutive_required", 3)
        self.declare_parameter("max_fine_tune_steps", 100)
        self.declare_parameter("on_max_steps_action", "grip")  # "grip" or "error"
        self.declare_parameter("grip_wait_sec", 0.5)

        self.robot_name = str(self.get_parameter("robot_name").value)
        self.ns = f"/{self.robot_name}"
        self.detection_topic = str(self.get_parameter("detection_topic").value)
        self.target_class_label = str(self.get_parameter("target_class_label").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.image_w = float(self.get_parameter("image_width").value)
        self.image_h = float(self.get_parameter("image_height").value)

        self.model_dir = str(self.get_parameter("model_dir").value)
        self.device = str(self.get_parameter("device").value)

        self.control_rate_hz = max(0.5, float(self.get_parameter("control_rate_hz").value))
        self.status_timeout_sec = float(self.get_parameter("status_timeout_sec").value)
        self.command_speed = int(self.get_parameter("command_speed").value)
        self.gripper_speed = int(self.get_parameter("gripper_speed").value)

        self.grip_confidence_threshold = float(
            self.get_parameter("grip_confidence_threshold").value
        )
        self.grip_consecutive_required = max(
            1, int(self.get_parameter("grip_consecutive_required").value)
        )
        self.max_fine_tune_steps = int(self.get_parameter("max_fine_tune_steps").value)
        self.on_max_steps_action = str(self.get_parameter("on_max_steps_action").value).lower()
        self.grip_wait_sec = float(self.get_parameter("grip_wait_sec").value)

        # ============================================================
        # Model / feature builder
        # ============================================================
        self.config = load_config(self.model_dir)
        self.builder = FeatureBuilder(self.config)
        self.window_size = int(self.config["window"])
        self.max_delta_deg = float(self.config["max_delta_deg"])
        self.joint_limits = [tuple(v) for v in self.config["joint_limits"]]
        self.controlled_joints = list(self.builder.controlled_joints)

        # 학습 timestep(target_control_hz)과 추론 주기를 일치시킨다(일관성).
        # config에 있으면 control_rate_hz 파라미터보다 우선한다.
        cfg_hz = float(self.config.get("target_control_hz", 0.0) or 0.0)
        if cfg_hz > 0.0:
            self.control_rate_hz = cfg_hz

        self.policy = load_policy(self.model_dir, self.config, self.device)
        self.grip_net = load_grip_predictor(self.model_dir, self.config, self.device)
        self.get_logger().info(
            f"Loaded models from {self.model_dir} (input_dim={self.config['input_dim']}, "
            f"window={self.window_size}, controlled_joints={self.controlled_joints}, "
            f"device={self.device})"
        )

        # ============================================================
        # Detection / status state
        # ============================================================
        self.latest_valid_det = None   # (cx, cy, area_norm)
        self.target_track_id = None

        self.latest_angles = None
        self.latest_gripper_value = math.nan
        self.latest_status_time = None

        # ============================================================
        # Controller state
        # ============================================================
        self.phase = Phase.IDLE
        self.phase_start_time = self.get_clock().now()
        self._activated = False

        self.anchor_vec = None
        self.window = deque(maxlen=self.window_size)
        self.prev_step_angles = None
        self.step_count = 0
        self.grip_consec = 0

        # ============================================================
        # Publishers / subscribers
        # ============================================================
        self.target_pose_pub = self.create_publisher(
            Float64MultiArray, f"{self.ns}/target_pose", 10
        )
        self.set_gripper_pub = self.create_publisher(
            Float64MultiArray, f"{self.ns}/set_gripper", 10
        )
        self.status_request_pub = self.create_publisher(
            Empty, f"{self.ns}/request_status", 10
        )

        self.status_sub = self.create_subscription(
            Float64MultiArray, f"{self.ns}/status", self.status_callback, 10
        )
        self.detection_sub = self.create_subscription(
            TrackedObjectArray, self.detection_topic, self.detection_callback, 10
        )
        self.ibvs_done_sub = self.create_subscription(
            Empty, f"{self.ns}/ibvs_done", self.ibvs_done_callback, 1
        )

        self.timer = self.create_timer(1.0 / self.control_rate_hz, self.timer_callback)
        self.get_logger().info("NNControllerNode started. Waiting for ibvs_done...")

    # ============================================================
    # Callbacks
    # ============================================================
    def status_callback(self, msg: Float64MultiArray):
        data = list(msg.data)
        if len(data) < 27:
            return
        try:
            angles = [float(v) for v in data[14:20]]
            gripper_value = float(data[26])
        except Exception:
            return
        self.latest_angles = np.array(angles, dtype=np.float64)
        self.latest_gripper_value = gripper_value
        self.latest_status_time = self.get_clock().now()

    def detection_callback(self, msg: TrackedObjectArray):
        if len(msg.objects) == 0:
            return
        best = None
        best_score = -1.0
        if self.target_track_id is not None:
            for obj in msg.objects:
                if int(obj.track_id) == int(self.target_track_id):
                    if float(obj.confidence) >= self.min_confidence:
                        best = obj
                    break
        if best is None:
            for obj in msg.objects:
                if self.target_class_label and str(obj.class_label) != self.target_class_label:
                    continue
                conf = float(obj.confidence)
                if conf < self.min_confidence:
                    continue
                if conf > best_score:
                    best_score = conf
                    best = obj
            if best is not None:
                self.target_track_id = int(best.track_id)
        if best is None:
            return

        cx = float(best.bbox_x)
        cy = float(best.bbox_y)
        area_norm = (float(best.bbox_w) * float(best.bbox_h)) / max(self.image_w * self.image_h, 1.0)
        if not (math.isfinite(cx) and math.isfinite(cy) and math.isfinite(area_norm)) or area_norm <= 0.0:
            return
        self.latest_valid_det = (cx, cy, area_norm)

    def ibvs_done_callback(self, _msg: Empty):
        if self._activated:
            return
        self._activated = True
        self.get_logger().info("ibvs_done received. Activating NN controller.")

    # ============================================================
    # Helpers
    # ============================================================
    def has_fresh_status(self):
        if self.latest_status_time is None or self.latest_angles is None:
            return False
        age = (self.get_clock().now() - self.latest_status_time).nanoseconds * 1e-9
        return age <= self.status_timeout_sec

    def elapsed_in_phase(self):
        return (self.get_clock().now() - self.phase_start_time).nanoseconds * 1e-9

    def set_phase(self, new_phase):
        self.phase = new_phase
        self.phase_start_time = self.get_clock().now()
        self.get_logger().info(f"Phase -> {new_phase.name}")

    def publish_joint_command(self, angles, speed):
        speed = int(max(1, min(100, speed)))
        msg = Float64MultiArray()
        msg.data = [float(CMD_JOINT)] + [float(v) for v in angles] + [float(speed)]
        self.target_pose_pub.publish(msg)

    def publish_grip_close(self):
        msg = Float64MultiArray()
        msg.data = [0.0, float(self.gripper_speed)]
        self.set_gripper_pub.publish(msg)

    def clip_joint(self, j, value):
        lo, hi = self.joint_limits[j]
        return float(np.clip(value, lo, hi))

    # ============================================================
    # State machine
    # ============================================================
    def timer_callback(self):
        try:
            # 활성화 전에는 명령을 발행하지 않는다. status만 폴링한다.
            self.status_request_pub.publish(Empty())

            if self.phase == Phase.IDLE:
                if not self._activated:
                    return
                if not self.has_fresh_status():
                    return
                if self.latest_valid_det is None:
                    self.get_logger().warn(
                        "Activated but no valid detection captured yet. Waiting..."
                    )
                    return
                self.begin_run()

            elif self.phase == Phase.RUN:
                if self.has_fresh_status():
                    self.run_step()

            elif self.phase == Phase.GRIP:
                self.publish_grip_close()
                self.get_logger().info("Grip command sent (close).")
                self.set_phase(Phase.GRIP_WAIT)

            elif self.phase == Phase.GRIP_WAIT:
                if self.elapsed_in_phase() >= self.grip_wait_sec:
                    self.set_phase(Phase.DONE)

            elif self.phase in (Phase.DONE, Phase.ERROR):
                return

        except Exception as exc:
            self.get_logger().error(f"Exception in timer_callback: {exc}")
            self.set_phase(Phase.ERROR)

    def begin_run(self):
        # detection anchor 동결 (J6는 ibvs_controller가 이미 정렬했으므로 여기선 미관여).
        cx, cy, area_norm = self.latest_valid_det
        self.anchor_vec = self.builder.anchor_vec(cx, cy, area_norm, self.image_w, self.image_h)

        q = self.latest_angles.copy()
        seed = self.builder.step_feat(q, np.zeros(N_JOINTS))
        self.window.clear()
        for _ in range(self.window_size):
            self.window.append(seed)
        self.prev_step_angles = q
        self.step_count = 0
        self.grip_consec = 0
        self.get_logger().info(
            f"Begin RUN. anchor(cx,cy,area)=({cx:.0f},{cy:.0f},{area_norm:.3f}), "
            f"q0={np.round(q, 1).tolist()}"
        )
        self.set_phase(Phase.RUN)

    def run_step(self):
        q = self.latest_angles.copy()
        delta = q - self.prev_step_angles
        self.window.append(self.builder.step_feat(q, delta))
        self.prev_step_angles = q

        inp = self.builder.build_input(self.anchor_vec, self.window)
        x = torch.from_numpy(inp).unsqueeze(0).to(self.device)
        with torch.no_grad():
            delta_norm = self.policy(x)[0].cpu().numpy()       # (n_ctrl,)
            p_success = float(torch.sigmoid(self.grip_net(x)[0]).cpu().item())

        delta_deg = delta_norm * self.max_delta_deg

        # 제어 관절(J1~J5)만 delta 적용. 나머지(J6 등)는 현재값 passthrough.
        q_cmd = q.copy()
        for k, j in enumerate(self.controlled_joints):
            q_cmd[j] = self.clip_joint(j, q[j] + float(delta_deg[k]))

        self.publish_joint_command(q_cmd.tolist(), self.command_speed)

        if p_success >= self.grip_confidence_threshold:
            self.grip_consec += 1
        else:
            self.grip_consec = 0
        self.step_count += 1

        self.get_logger().info(
            f"RUN step={self.step_count}/{self.max_fine_tune_steps}, "
            f"P(success)={p_success:.3f} "
            f"(consec={self.grip_consec}/{self.grip_consecutive_required}), "
            f"delta_deg(J1~J5)={np.round(delta_deg, 2).tolist()}, "
            f"q_cmd={np.round(q_cmd, 1).tolist()}"
        )

        if self.grip_consec >= self.grip_consecutive_required:
            self.get_logger().info(
                f"Grip gate passed (P>={self.grip_confidence_threshold} for "
                f"{self.grip_consecutive_required} steps). Gripping."
            )
            self.set_phase(Phase.GRIP)
            return

        if self.step_count >= self.max_fine_tune_steps:
            if self.on_max_steps_action == "error":
                self.get_logger().error(
                    "max_fine_tune_steps reached without grip confidence. ERROR."
                )
                self.set_phase(Phase.ERROR)
            else:
                self.get_logger().warn("max_fine_tune_steps reached. Forcing grip.")
                self.set_phase(Phase.GRIP)


def main(args=None):
    rclpy.init(args=args)
    node = NNControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
