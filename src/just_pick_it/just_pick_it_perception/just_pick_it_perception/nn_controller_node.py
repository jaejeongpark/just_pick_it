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

        # 추론 제어 주기(Hz). 0이면 학습 모델의 target_control_hz를 따른다(일관성).
        # >0으로 주면 그 값으로 override(튜닝용: 로봇이 못 따라가 명령이 쌓이면 낮춘다).
        self.declare_parameter("control_rate_hz", 0.0)
        # policy 출력 스케일. to_goal 모델은 목표 방향장이라 작은 step으로 잘게
        # 적분해도 방향이 유지된다. 제어 주기를 올리고 이 값을 줄이면(예: 10Hz x 0.5)
        # 같은 속도로 더 부드럽게 움직인다.
        self.declare_parameter("delta_scale", 1.0)
        # status 폴링 주기(Hz). 제어 주기와 분리한다. 제어 tick마다 요청하면
        # 시리얼이 정체되어 status가 오히려 느려진다(읽기 1회당 수백 ms).
        self.declare_parameter("status_poll_rate_hz", 5.0)
        # status(측정)가 제어 주기보다 느릴 때, 같은 stale q로 명령을 반복 적분하면
        # 과적분 -> overshoot -> 진동(limit cycle)이 생긴다. true면 측정값이 실제로
        # 갱신됐을 때만 제어를 1 step 진행한다(타이머는 빠르게 돌되 명령은 측정 주기).
        self.declare_parameter("require_fresh_status", True)
        # delta EMA 평활 계수 (1.0=비활성). 측정 노이즈로 인한 step 간 방향 반전을
        # 눌러 목표 주변 진동(limit cycle)을 줄인다. delta_f = a*new + (1-a)*prev.
        self.declare_parameter("delta_smooth_alpha", 0.5)
        # settle-hold: 평활된 |delta| 최대치가 이 값(deg) 미만이면 목표 근방으로
        # 판단하고 명령을 보류한다(정지 상태에서 grip 판정). 0이면 비활성.
        self.declare_parameter("settle_delta_deg", 0.8)
        # z-floor hard constraint. shelf 높이 아래로 end-effector가 내려가지 못하게 한다.
        # 좌표계/단위는 로봇 get_coords와 동일(mm, base 기준). z_floor_enable=true일 때만.
        #   효과 floor = z_floor_mm + z_floor_margin_mm.
        # dz/dq(관절→z 야코비안)는 런타임에 측정 z로 RLS 추정해 get_coords 좌표계와
        # 자동 일치시킨다(해석적 FK 불필요). 추정이 충분히 쌓이면 명령 delta를
        # gradient projection으로 보정해 commanded z가 floor 아래로 못 가게 만든다.
        self.declare_parameter("z_floor_enable", False)
        self.declare_parameter("z_floor_mm", 0.0)
        self.declare_parameter("z_floor_margin_mm", 0.0)
        # RLS: 야코비안 추정에 필요한 최소 업데이트 수와 step당 관절 변위 하한(deg).
        self.declare_parameter("z_jac_min_updates", 5)
        self.declare_parameter("z_jac_excite_deg", 0.3)
        self.declare_parameter("status_timeout_sec", 1.0)
        self.declare_parameter("command_speed", 10)
        self.declare_parameter("gripper_speed", 50)

        # 명령 적분: q_cmd = 직전 명령 + delta (측정값 재기준 대신).
        # 측정값 재기준(q_cmd = 측정 + delta)은 로봇이 못 따라간 만큼 목표가 같이
        # 끌려와 실현율이 극히 낮아진다(실측 ~6%). 적분 방식은 목표가 연속 전진해
        # 로봇이 따라붙는다. command_leash_deg: 측정값보다 이 이상 앞서가지 않게 제한.
        self.declare_parameter("command_integration", True)
        self.declare_parameter("command_leash_deg", 8.0)
        # anchor용 detection 선택 규칙. 학습 anchor의 출처(visual_servo_bag_recorder)는
        # "이미지 중심에서 가장 가까운 target"이므로 기본값을 동일하게 맞춘다.
        # confidence: 기존 방식(track lock + 최고 confidence)으로 되돌리기.
        self.declare_parameter("anchor_select", "nearest_center")

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

        self._control_rate_hz_param = float(self.get_parameter("control_rate_hz").value)
        self.delta_scale = float(self.get_parameter("delta_scale").value)
        self.delta_smooth_alpha = float(
            np.clip(self.get_parameter("delta_smooth_alpha").value, 0.05, 1.0)
        )
        self.settle_delta_deg = float(self.get_parameter("settle_delta_deg").value)
        self.require_fresh_status = bool(self.get_parameter("require_fresh_status").value)
        self._last_proc_status_ns = None
        self._delta_filt = None

        self.z_floor_enable = bool(self.get_parameter("z_floor_enable").value)
        self.z_floor_mm = float(self.get_parameter("z_floor_mm").value)
        self.z_floor_margin_mm = float(self.get_parameter("z_floor_margin_mm").value)
        self.z_jac_min_updates = int(self.get_parameter("z_jac_min_updates").value)
        self.z_jac_excite_deg = float(self.get_parameter("z_jac_excite_deg").value)
        self.latest_z = math.nan
        # RLS 상태 (begin_run에서 초기화).
        self._jz = None
        self._jz_P = None
        self._jz_n = 0
        self._prev_est_q = None
        self._prev_est_z = math.nan
        self._z_floor_warned = False
        self.status_poll_rate_hz = max(
            0.5, float(self.get_parameter("status_poll_rate_hz").value)
        )
        self._last_status_req_time = None
        self.status_timeout_sec = float(self.get_parameter("status_timeout_sec").value)
        self.command_speed = int(self.get_parameter("command_speed").value)
        self.gripper_speed = int(self.get_parameter("gripper_speed").value)

        self.command_integration = bool(self.get_parameter("command_integration").value)
        self.command_leash_deg = float(self.get_parameter("command_leash_deg").value)
        self.anchor_select = str(self.get_parameter("anchor_select").value).strip().lower()
        if self.anchor_select not in ("nearest_center", "confidence"):
            self.get_logger().warn(
                f"Unknown anchor_select '{self.anchor_select}'. Using 'nearest_center'."
            )
            self.anchor_select = "nearest_center"

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
        # zero: anchor 3차원을 0으로 고정(학습과 동일). detection 동결 불필요.
        # 구버전 config(키 없음)는 frozen(기존 동작).
        self.anchor_mode = str(self.config.get("anchor_mode", "frozen")).lower()
        self.window_size = int(self.config["window"])
        self.max_delta_deg = float(self.config["max_delta_deg"])
        self.joint_limits = [tuple(v) for v in self.config["joint_limits"]]
        self.controlled_joints = list(self.builder.controlled_joints)

        # 추론 제어 주기 결정 (우선순위):
        #   1) control_rate_hz 파라미터 > 0 이면 그 값(launch 튜닝)
        #   2) 아니면 학습 모델의 target_control_hz (학습/추론 일관성)
        #   3) 둘 다 없으면 5.0 fallback
        cfg_hz = float(self.config.get("target_control_hz", 0.0) or 0.0)
        if self._control_rate_hz_param > 0.0:
            self.control_rate_hz = self._control_rate_hz_param
            self.get_logger().warn(
                f"control_rate_hz overridden to {self.control_rate_hz} Hz via param "
                f"(trained target_control_hz={cfg_hz}). delta 의미가 학습과 달라질 수 있음."
            )
        elif cfg_hz > 0.0:
            self.control_rate_hz = cfg_hz
        else:
            self.control_rate_hz = 5.0
        self.control_rate_hz = max(0.5, self.control_rate_hz)

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
        self.start_angles = None       # 활성화 시점 자세(진행도 기준)
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
            # coords = data[20:26] = [x, y, z, rx, ry, rz] (mm/deg, get_coords).
            z_val = float(data[22])
        except Exception:
            return
        self.latest_angles = np.array(angles, dtype=np.float64)
        self.latest_gripper_value = gripper_value
        self.latest_z = z_val
        self.latest_status_time = self.get_clock().now()

    def detection_callback(self, msg: TrackedObjectArray):
        if len(msg.objects) == 0:
            return
        best = None
        if self.anchor_select == "nearest_center":
            # 학습 anchor의 출처(visual_servo_bag_recorder.select_nearest_target_to_image_center)
            # 와 동일한 선택 규칙: class/confidence 필터 후 이미지 중심에 가장 가까운 target.
            best_d2 = float("inf")
            for obj in msg.objects:
                if self.target_class_label and str(obj.class_label) != self.target_class_label:
                    continue
                if float(obj.confidence) < self.min_confidence:
                    continue
                d2 = ((float(obj.bbox_x) - self.image_w * 0.5) ** 2
                      + (float(obj.bbox_y) - self.image_h * 0.5) ** 2)
                if d2 < best_d2:
                    best_d2 = d2
                    best = obj
        else:
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
            # 폴링은 제어 주기와 분리해 status_poll_rate_hz로 제한한다(시리얼 정체 방지).
            now = self.get_clock().now()
            if (
                self._last_status_req_time is None
                or (now - self._last_status_req_time).nanoseconds * 1e-9
                >= 1.0 / self.status_poll_rate_hz
            ):
                self._last_status_req_time = now
                self.status_request_pub.publish(Empty())

            if self.phase == Phase.IDLE:
                if not self._activated:
                    return
                if not self.has_fresh_status():
                    return
                if self.anchor_mode != "zero" and self.latest_valid_det is None:
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
        if self.anchor_mode == "zero":
            # anchor 미사용 모델: 학습과 동일하게 0 벡터.
            cx, cy, area_norm = float("nan"), float("nan"), float("nan")
            self.anchor_vec = np.zeros(3, dtype=np.float32)
        else:
            # detection anchor 동결 (J6는 ibvs_controller가 이미 정렬했으므로 미관여).
            cx, cy, area_norm = self.latest_valid_det
            self.anchor_vec = self.builder.anchor_vec(
                cx, cy, area_norm, self.image_w, self.image_h
            )

        q = self.latest_angles.copy()
        # 진행도 기준점 = 활성화 시점의 현재 자세.
        self.start_angles = q.copy()
        seed = self.builder.step_feat(q, np.zeros(N_JOINTS))
        self.window.clear()
        for _ in range(self.window_size):
            self.window.append(seed)
        self.prev_step_angles = q
        # 명령 적분 기준점. 비제어 관절(J6 등)은 활성화 시점 값으로 고정 유지.
        self.cmd_ref = q.copy()
        self._delta_filt = None
        self._last_proc_status_ns = None
        self.step_count = 0
        self.grip_consec = 0

        # z-floor 야코비안 RLS 초기화. dz/dq는 제어 관절(J1~J5) 기준 5-vector.
        n_ctrl = len(self.controlled_joints)
        self._jz = np.zeros(n_ctrl)
        self._jz_P = np.eye(n_ctrl) * 1.0e3
        self._jz_n = 0
        self._prev_est_q = q[self.controlled_joints].copy()
        self._prev_est_z = self.latest_z
        self._z_floor_warned = False
        self.get_logger().info(
            f"Begin RUN. anchor(cx,cy,area)=({cx:.0f},{cy:.0f},{area_norm:.3f}), "
            f"q0={np.round(q, 1).tolist()}"
        )
        self.set_phase(Phase.RUN)

    def _apply_z_floor(self, q, delta_deg):
        # z-floor: dz/dq를 측정 z로 RLS 추정한 뒤, commanded z가 floor 아래로 가면
        # 최소 노름 보정(gradient projection)으로 floor에 맞춘다. 측정 z가 이미
        # floor 아래면 같은 식이 위로 들어올린다(closed-loop 회복).
        if not self.z_floor_enable or not math.isfinite(self.latest_z):
            return delta_deg

        # --- RLS 업데이트: 직전 step 이후 측정된 (dq, dz)로 dz/dq 추정 ---
        cur_q = q[self.controlled_joints]
        if self._prev_est_q is not None and math.isfinite(self._prev_est_z):
            dq = cur_q - self._prev_est_q
            dz = self.latest_z - self._prev_est_z
            # 충분한 여기(excitation)와 outlier(좌표 글리치) 배제 시에만 갱신.
            if float(np.max(np.abs(dq))) >= self.z_jac_excite_deg and abs(dz) < 200.0:
                P, jz, lam = self._jz_P, self._jz, 0.99
                Px = P @ dq
                denom = lam + float(dq @ Px)
                if denom > 1e-9:
                    K = Px / denom
                    self._jz = jz + K * (dz - float(dq @ jz))
                    self._jz_P = (P - np.outer(K, Px)) / lam
                    self._jz_n += 1
        self._prev_est_q = cur_q.copy()
        self._prev_est_z = self.latest_z

        z_eff = self.z_floor_mm + self.z_floor_margin_mm
        jz = self._jz
        jz_norm2 = float(jz @ jz)
        confident = self._jz_n >= self.z_jac_min_updates and jz_norm2 > 1e-6

        if not confident:
            # 야코비안 미확정. 접근은 보통 floor 한참 위에서 시작하므로 곧 확정된다.
            # 만약 이미 floor 아래라면 안전하게 추가 이동을 막고 1회 경고한다.
            if self.latest_z < z_eff and not self._z_floor_warned:
                self._z_floor_warned = True
                self.get_logger().warn(
                    f"z below floor ({self.latest_z:.1f} < {z_eff:.1f}mm) but dz/dq not "
                    f"estimated yet. Holding to avoid unsafe descent."
                )
                return np.zeros_like(delta_deg)
            return delta_deg

        z_pred = self.latest_z + float(jz @ delta_deg)
        if z_pred < z_eff:
            corr = (z_eff - z_pred) / jz_norm2
            delta_deg = delta_deg + corr * jz
            self.get_logger().info(
                f"z-floor active: z_now={self.latest_z:.1f} z_pred={z_pred:.1f} "
                f"-> clamp to {z_eff:.1f}mm (corr lift)."
            )
        return delta_deg

    def run_step(self):
        # 측정값이 갱신됐을 때만 1 step 진행한다. stale q로 반복 적분 시 과적분으로
        # overshoot/진동이 생기므로(status가 제어 주기보다 느릴 때), fresh일 때만 명령.
        if self.require_fresh_status and self.latest_status_time is not None:
            st = self.latest_status_time.nanoseconds
            if st == self._last_proc_status_ns:
                return
            self._last_proc_status_ns = st

        q = self.latest_angles.copy()
        delta = q - self.prev_step_angles
        self.window.append(self.builder.step_feat(q, delta))
        self.prev_step_angles = q

        # 진행도: 활성화 자세 대비 현재 누적 변위.
        progress_vec = self.builder.scale_progress(q - self.start_angles)
        inp = self.builder.build_input(self.anchor_vec, progress_vec, self.window)
        x = torch.from_numpy(inp).unsqueeze(0).to(self.device)
        with torch.no_grad():
            delta_norm = self.policy(x)[0].cpu().numpy()       # (n_ctrl,)
            p_success = float(torch.sigmoid(self.grip_net(x)[0]).cpu().item())

        delta_deg = delta_norm * self.max_delta_deg * self.delta_scale

        # EMA 평활: 측정 노이즈로 인한 step 간 방향 반전을 누른다.
        if self.delta_smooth_alpha < 1.0:
            if self._delta_filt is None:
                self._delta_filt = delta_deg.copy()
            else:
                self._delta_filt = (
                    self.delta_smooth_alpha * delta_deg
                    + (1.0 - self.delta_smooth_alpha) * self._delta_filt
                )
            delta_deg = self._delta_filt

        # z-floor hard constraint: commanded z가 shelf 아래로 못 가게 delta를 보정한다.
        delta_deg = self._apply_z_floor(q, delta_deg)

        # settle-hold: 목표 근방(평활 delta가 충분히 작음)이면 명령을 보류하고
        # 정지 상태에서 grip 판정만 진행한다(진동으로 인한 맴돌이 차단).
        # 단 z-floor 보정으로 위로 들어올려야 하는 경우는 settle하지 않는다.
        settled = (
            self.settle_delta_deg > 0.0
            and float(np.max(np.abs(delta_deg))) < self.settle_delta_deg
        )

        # 제어 관절(J1~J5)만 delta 적용.
        if settled:
            q_cmd = self.cmd_ref.copy()
        elif self.command_integration:
            # 직전 명령 기준 적분 + leash. 측정값 재기준은 로봇이 못 따라간 만큼
            # 목표가 끌려와 실현율이 낮아지므로, 목표를 연속 전진시키되 측정값에서
            # command_leash_deg 이상 벌어지지 않게 제한한다.
            for k, j in enumerate(self.controlled_joints):
                target = self.cmd_ref[j] + float(delta_deg[k])
                target = float(np.clip(
                    target, q[j] - self.command_leash_deg, q[j] + self.command_leash_deg
                ))
                self.cmd_ref[j] = self.clip_joint(j, target)
            q_cmd = self.cmd_ref.copy()
        else:
            # 기존 방식: 측정값 재기준. 나머지(J6 등)는 현재값 passthrough.
            q_cmd = q.copy()
            for k, j in enumerate(self.controlled_joints):
                q_cmd[j] = self.clip_joint(j, q[j] + float(delta_deg[k]))

        if not settled:
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
            f"{'SETTLED ' if settled else ''}"
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
