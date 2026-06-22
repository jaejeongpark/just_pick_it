#!/usr/bin/env python3

import math
import os
import queue
import shutil
import threading
import tkinter as tk
from datetime import datetime
from enum import Enum
from pathlib import Path
from typing import List, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.serialization import serialize_message

import rosbag2_py

import re

from rclpy.qos import QoSProfile, QoSDurabilityPolicy

from std_msgs.msg import Empty, Float64MultiArray, Int32, String

from just_pick_it_interfaces.msg import HumanInteractionSample, TrackedObjectArray


PHASE_WAITING = 0
PHASE_FREE_DRIVE = 1
PHASE_GRIPPING = 2
PHASE_WAITING_RESULT = 3
PHASE_RESULT = 4
PHASE_READY_TO_RELEASE = 5
PHASE_RELEASING = 6


class InteractionPhase(Enum):
    WAITING = PHASE_WAITING
    # ibvs_done 수신 후 사람이 팔을 잡고 [R]을 누를 때까지 관절을 풀지 않고 대기.
    READY_TO_RELEASE = PHASE_READY_TO_RELEASE
    # [R] 후 서보 해제 + gripper open이 확인될 때까지 기록을 보류하는 단계.
    RELEASING = PHASE_RELEASING
    FREE_DRIVE = PHASE_FREE_DRIVE
    GRIPPING = PHASE_GRIPPING
    WAITING_RESULT = PHASE_WAITING_RESULT
    RESULT = PHASE_RESULT


class HumanInteractionRecorderNode(Node):
    """
    ibvs_done 수신 후 free-drive 구간 데이터를 기록하고 grip 결과를 레이블링한다.

    키 입력은 tkinter GUI(별도 클래스)에서 받아 handle_key()로 전달된다.
    ros2 launch로 띄운 노드는 stdin이 연결되지 않으므로, GUI 창을 통해
    버튼/키보드 입력을 받는다.

    State Machine:
      WAITING -> (ibvs_done) -> READY_TO_RELEASE
        관절은 풀지 않고 대기. 사람이 팔을 잡고 [R]을 누를 때까지.

      READY_TO_RELEASE -> (R) -> RELEASING
        gripper open + release_all_servos() 발행. 아직 기록하지 않는다.

      RELEASING -> (release + gripper open 확인) -> FREE_DRIVE
        서보 해제와 gripper open이 실제로 반영된 뒤, 실제 움직임이 감지되면 기록 시작.
        record_mode=displacement(기본): 마지막 waypoint 대비 J1~J5 최대 변위가
          displacement_threshold_deg 이상일 때마다 저장. 시연 속도와 무관하게
          관절 공간 등간격 waypoint가 만들어진다(정지 구간은 기록 안 됨).
        record_mode=fixed_rate: record_rate_hz 고정 주기 저장(status 갱신 시에만).

      FREE_DRIVE -> (G) -> GRIPPING
        set_gripper([0, 50]), grip_wait_sec 대기.

      GRIPPING -> WAITING_RESULT
        [S]/[F] 입력 대기.

      WAITING_RESULT -> (S or F) -> RESULT
        result 샘플 기록, bag 닫기, episode 디렉토리 이동.
    """

    def __init__(self):
        super().__init__("human_interaction_recorder")

        # ============================================================
        # Parameters
        # ============================================================
        self.declare_parameter("robot_name", "jetcobot1")
        self.declare_parameter("episode_id", "")
        self.declare_parameter("bag_base_dir", str(Path.home() / "rosbags"))
        self.declare_parameter("bag_topic", "/nn_controller/human_sample")
        self.declare_parameter("storage_id", "sqlite3")
        # 고정 주기(Hz) 기록. event-driven은 작은 움직임에도 과도하게 쌓여 replay 시
        # 큐가 밀리므로 폐기하고, NN 추론과 동일한 5Hz 고정 주기로 기록한다.
        self.declare_parameter("record_rate_hz", 5.0)
        self.declare_parameter("grip_wait_sec", 0.5)
        # [G] 순간 정확한 grip 위치를 위해 fresh status를 요청하고 수신 후 기록한다.
        # 이 시간 내 새 status가 안 오면 timeout으로 마지막 값으로 기록.
        self.declare_parameter("grip_capture_timeout_sec", 0.3)
        self.declare_parameter("shutdown_on_done", True)
        # 연속 에피소드 기록. True면 결과(S/F)/ERROR 후 종료하지 않고 다음 episode로 루프한다.
        # 전체 종료는 GUI 창의 X(우상단)로만 한다.
        self.declare_parameter("loop_episodes", True)
        # release(set_arm [0])는 gripper 서보까지 푼다. 이 지연 후 gripper open을 1회
        # 재발행해 gripper 서보만 다시 잡아 100(open)을 유지한다(팔 관절은 풀린 채).
        self.declare_parameter("release_gripper_reopen_delay_sec", 0.6)
        self.declare_parameter("gripper_open_speed", 50)
        # RELEASING: [R] 후 서보 해제 + gripper open이 확인될 때까지 기록을 보류한다.
        # reopen_delay + settle_margin 경과 후, gripper_value >= confirm 이거나
        # timeout이면 FREE_DRIVE로 전환하여 기록을 시작한다.
        self.declare_parameter("release_settle_margin_sec", 0.3)
        self.declare_parameter("gripper_open_confirm_value", 80.0)
        self.declare_parameter("release_confirm_timeout_sec", 2.0)
        # FREE_DRIVE 진입 후 사람이 실제로 움직이기 시작할 때까지 기록을 보류한다.
        # 서보 해제 지연/반응 시간 동안의 정지 프레임이 데이터에 섞이는 것을 방지.
        # 기준 자세 대비 관절 최대 변위가 이 값(deg) 이상이면 움직임 시작으로 판정.
        self.declare_parameter("motion_start_threshold_deg", 1.0)
        # 기록 모드.
        #   displacement: 마지막 저장 waypoint 대비 제어 관절(J1~J5) 최대 변위가
        #     displacement_threshold_deg 이상이 될 때마다 저장(event-interrupt).
        #     사람 시연 속도와 무관하게 관절 공간 등간격 waypoint가 만들어져,
        #     학습 step delta가 임계값 수준으로 균일해진다. 정지 구간은 기록되지 않는다.
        #   fixed_rate: record_rate_hz 고정 주기 저장(기존 방식, status 갱신 시에만).
        self.declare_parameter("record_mode", "displacement")
        self.declare_parameter("displacement_threshold_deg", 2.0)
        # 한 step 변위가 임계값의 이 배수를 넘으면 경고(status 공백/과속 시연 의심).
        self.declare_parameter("displacement_warn_factor", 2.5)
        # Closed-loop NN 재수집: human 정밀보정 구간에 대상 물체 detection을 함께 기록한다.
        # 선택 규칙은 inference(nn_controller_node) / visual_servo_bag_recorder 와 동일한
        # nearest_center(class/confidence 필터 후 화면 중심 최근접).
        self.declare_parameter("detection_topic", "/infer/tracked_objects")
        self.declare_parameter("target_class_label", "watermelon")
        self.declare_parameter("min_confidence", 0.5)
        self.declare_parameter("image_width", 640.0)
        self.declare_parameter("image_height", 480.0)
        # 이 시간보다 오래된 detection은 stale로 보고 det_valid=false(freeze 대상)로 기록.
        self.declare_parameter("detection_timeout_sec", 0.5)

        self.robot_name = str(self.get_parameter("robot_name").value)
        self.episode_id = str(self.get_parameter("episode_id").value)
        self.bag_base_dir = str(self.get_parameter("bag_base_dir").value)
        self.bag_topic = str(self.get_parameter("bag_topic").value)
        self.storage_id = str(self.get_parameter("storage_id").value)
        self.record_rate_hz = max(0.5, float(self.get_parameter("record_rate_hz").value))
        self.grip_wait_sec = float(self.get_parameter("grip_wait_sec").value)
        self.grip_capture_timeout_sec = float(
            self.get_parameter("grip_capture_timeout_sec").value
        )
        self.shutdown_on_done = self._parse_bool(self.get_parameter("shutdown_on_done").value)
        self.loop_episodes = self._parse_bool(self.get_parameter("loop_episodes").value)
        self.release_gripper_reopen_delay_sec = float(
            self.get_parameter("release_gripper_reopen_delay_sec").value
        )
        self.gripper_open_speed = int(self.get_parameter("gripper_open_speed").value)
        self.release_settle_margin_sec = float(
            self.get_parameter("release_settle_margin_sec").value
        )
        self.gripper_open_confirm_value = float(
            self.get_parameter("gripper_open_confirm_value").value
        )
        self.release_confirm_timeout_sec = float(
            self.get_parameter("release_confirm_timeout_sec").value
        )
        self.motion_start_threshold_deg = float(
            self.get_parameter("motion_start_threshold_deg").value
        )
        self.record_mode = str(self.get_parameter("record_mode").value).strip().lower()
        if self.record_mode not in ("displacement", "fixed_rate"):
            self.get_logger().warn(
                f"Unknown record_mode '{self.record_mode}'. Falling back to 'displacement'."
            )
            self.record_mode = "displacement"
        self.displacement_threshold_deg = float(
            self.get_parameter("displacement_threshold_deg").value
        )
        self.displacement_warn_factor = float(
            self.get_parameter("displacement_warn_factor").value
        )
        self.detection_topic = str(self.get_parameter("detection_topic").value)
        self.target_class_label = str(self.get_parameter("target_class_label").value)
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        self.image_width = float(self.get_parameter("image_width").value)
        self.image_height = float(self.get_parameter("image_height").value)
        self.detection_timeout_sec = float(
            self.get_parameter("detection_timeout_sec").value
        )

        self.ns = f"/{self.robot_name}"

        # ============================================================
        # Runtime state
        # ============================================================
        self.phase = InteractionPhase.WAITING

        self.ibvs_done_ros_time: Optional[float] = None
        self.gripping_start_ros_time: Optional[float] = None

        self.latest_angles: Optional[List[float]] = None
        self.latest_gripper_value: float = math.nan
        self.latest_status_time = None

        # 최신 유효 detection (cx_px, cy_px, area_norm) 과 수신 시각(ros sec).
        # det_valid 판정은 detection_timeout_sec 기준 freshness. stale 여도 마지막 값은
        # 유지해 freeze 정보를 보존한다(학습/추론 일관).
        self.latest_det: Optional[tuple] = None
        self.latest_det_time: Optional[float] = None

        # release 후 gripper open 재발행용 one-shot 타이머.
        self._gripper_reopen_timer = None
        self._gripper_reopen_done = False

        # RELEASING: release + gripper open 확인 후에만 기록(FREE_DRIVE)을 시작한다.
        self._release_request_sec = 0.0
        self._release_status_poll_sec = 0.0

        # [G] 시 fresh status 수신 대기 상태.
        self._grip_capture_pending = False
        self._grip_capture_req_status_ns = None
        self._grip_capture_start_sec = 0.0

        self.prev_recorded_angles: Optional[List[float]] = None
        self.prev_recorded_ros_time: Optional[float] = None

        # 중복 기록 방지: 마지막으로 기록한 status 측정 시각(ns).
        # status가 갱신되지 않았으면 같은 측정값을 다시 쓰지 않는다.
        self._last_committed_status_ns: Optional[int] = None

        # 움직임 감지 게이트: FREE_DRIVE 진입 후 사람이 실제로 움직이기 전까지 기록 보류.
        self._motion_started: bool = False
        self._free_drive_baseline_angles: Optional[List[float]] = None

        self.sample_index: int = 0

        self._write_queue: queue.Queue = queue.Queue()
        self._write_thread: Optional[threading.Thread] = None

        # GUI에서 넣은 키를 ROS executor 스레드(sm_timer)에서 단일 처리한다.
        self._key_queue: queue.Queue = queue.Queue()

        # 에피소드 종료 플래그. GUI가 감지하여 창을 닫는다.
        self._finished: bool = False
        self._result_text: str = ""

        # ============================================================
        # Publishers
        # ============================================================
        # snatch-only: request_status는 발행하지 않는다. DONE 이후의 status는
        # ibvs_controller가 저주파로 폴링해 공급하며, 여기서는 구독만 한다.
        self.set_arm_pub = self.create_publisher(
            Float64MultiArray, f"{self.ns}/set_arm", 10
        )
        self.set_gripper_pub = self.create_publisher(
            Float64MultiArray, f"{self.ns}/set_gripper", 10
        )
        # [G] 순간 fresh status를 요청하기 위한 publisher.
        self.request_status_pub = self.create_publisher(
            Empty, f"{self.ns}/request_status", 10
        )
        # 다음 episode_id를 ibvs_controller / visual_servo_recorder에 알린다.
        # transient_local로 늦게 구독해도 마지막 값 수신.
        self.episode_pub = self.create_publisher(
            String,
            f"{self.ns}/nn_episode",
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL),
        )

        # ============================================================
        # Subscribers
        # ============================================================
        self.status_sub = self.create_subscription(
            Float64MultiArray,
            f"{self.ns}/status",
            self.status_callback,
            10,
        )
        self.ibvs_done_sub = self.create_subscription(
            Empty,
            f"{self.ns}/ibvs_done",
            self.ibvs_done_callback,
            1,
        )
        self.ibvs_phase_sub = self.create_subscription(
            Int32,
            f"{self.ns}/ibvs_phase",
            self.ibvs_phase_callback,
            10,
        )
        # Closed-loop NN 입력용 대상 물체 detection 구독.
        self.detection_sub = self.create_subscription(
            TrackedObjectArray,
            self.detection_topic,
            self.detection_callback,
            10,
        )

        # ============================================================
        # Bag writer + write thread
        # ============================================================
        self.writer = None
        self.resolved_bag_uri = self._resolve_bag_uri()
        self._open_bag_writer()

        self._write_thread = threading.Thread(
            target=self._write_worker, daemon=True, name="human_bag_write_worker"
        )
        self._write_thread.start()

        # ============================================================
        # Timers
        # ============================================================
        self.sm_timer = self.create_timer(0.05, self._state_machine_callback)
        # 고정 주기 기록 타이머 (FREE_DRIVE 동안만 기록).
        self.record_timer = self.create_timer(
            1.0 / self.record_rate_hz, self._record_timer_callback
        )

        self.get_logger().info("HumanInteractionRecorderNode started")
        self.get_logger().info(f"robot_name={self.robot_name}, episode_id='{self.episode_id}'")
        self.get_logger().info(f"bag_uri={self.resolved_bag_uri}")
        self.get_logger().info(
            f"record_rate_hz={self.record_rate_hz} (fixed-rate, status from ibvs_controller)"
        )
        self.get_logger().info("Waiting for ibvs_done signal...")

    # ============================================================
    # Helpers
    # ============================================================
    @staticmethod
    def _parse_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("true", "1", "yes")
        return bool(value)

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _time_since_ibvs_done(self) -> float:
        if self.ibvs_done_ros_time is None:
            return 0.0
        return self._now_sec() - self.ibvs_done_ros_time

    def get_action_hint(self) -> str:
        if self.phase == InteractionPhase.WAITING:
            return "Waiting for IBVS DONE (ibvs_done)..."
        if self.phase == InteractionPhase.READY_TO_RELEASE:
            return "Hold the arm FIRMLY, then press [R] to release servos"
        if self.phase == InteractionPhase.RELEASING:
            return "Releasing servos / confirming gripper open... please wait"
        if self.phase == InteractionPhase.FREE_DRIVE:
            return "Fine-tune position by hand (J6 already aligned), then [G] to grip"
        if self.phase == InteractionPhase.GRIPPING:
            return "Gripping..."
        if self.phase == InteractionPhase.WAITING_RESULT:
            return "Grip done. [S] Success   [F] Fail"
        if self.phase == InteractionPhase.RESULT:
            return self._result_text or "Done."
        return ""

    # ============================================================
    # Bag URI / writer
    # ============================================================
    def _resolve_bag_uri(self) -> str:
        if self.episode_id:
            path = (
                Path(os.path.expanduser(self.bag_base_dir))
                / "raw"
                / f"episode_{self.episode_id}"
                / "human"
            )
        else:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = Path(os.path.expanduser(self.bag_base_dir)) / f"human_{ts}"

        path.parent.mkdir(parents=True, exist_ok=True)

        if path.exists():
            ts = datetime.now().strftime("%Y%m%d_%H%M%S")
            path = path.with_name(f"{path.name}_{ts}")

        return str(path)

    def _open_bag_writer(self):
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

        msg_type = "just_pick_it_interfaces/msg/HumanInteractionSample"
        try:
            topic_info = rosbag2_py.TopicMetadata(
                0, self.bag_topic, msg_type, "cdr", [], ""
            )
        except TypeError:
            topic_info = rosbag2_py.TopicMetadata(
                name=self.bag_topic,
                type=msg_type,
                serialization_format="cdr",
            )
        self.writer.create_topic(topic_info)

    def _close_bag_writer(self):
        if self._write_thread is not None and self._write_thread.is_alive():
            self._write_queue.put(None)
            self._write_thread.join(timeout=5.0)
            self._write_thread = None
        if self.writer is not None:
            self.get_logger().info(
                f"Closing human bag. samples={self.sample_index}, "
                f"uri={self.resolved_bag_uri}"
            )
            self.writer = None

    def _write_worker(self):
        while True:
            item = self._write_queue.get()
            if item is None:
                self._write_queue.task_done()
                break
            topic, data, ts_ns = item
            try:
                if self.writer is not None:
                    self.writer.write(topic, data, ts_ns)
            except Exception as exc:
                self.get_logger().error(f"write_worker error: {exc}")
            self._write_queue.task_done()

    # ============================================================
    # ROS callbacks
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

        self.latest_angles = angles
        self.latest_gripper_value = gripper_value
        self.latest_status_time = self.get_clock().now()

    def detection_callback(self, msg: TrackedObjectArray):
        # inference(nn_controller_node) / visual_servo_bag_recorder 와 동일한 nearest_center:
        # class/confidence 필터 후 화면 중심에 가장 가까운 대상.
        best = None
        best_d2 = float("inf")
        for obj in msg.objects:
            if self.target_class_label and str(obj.class_label) != self.target_class_label:
                continue
            if float(obj.confidence) < self.min_confidence:
                continue
            d2 = ((float(obj.bbox_x) - self.image_width * 0.5) ** 2
                  + (float(obj.bbox_y) - self.image_height * 0.5) ** 2)
            if d2 < best_d2:
                best_d2 = d2
                best = obj
        if best is None:
            return
        cx = float(best.bbox_x)
        cy = float(best.bbox_y)
        area_norm = (float(best.bbox_w) * float(best.bbox_h)) / max(
            self.image_width * self.image_height, 1.0
        )
        if not (math.isfinite(cx) and math.isfinite(cy) and math.isfinite(area_norm)) \
                or area_norm <= 0.0:
            return
        self.latest_det = (cx, cy, area_norm)
        self.latest_det_time = self.get_clock().now().nanoseconds * 1e-9

    def ibvs_done_callback(self, _msg: Empty):
        if self.phase != InteractionPhase.WAITING:
            return

        # 관절은 아직 풀지 않는다. 사람이 팔을 잡고 [R]을 누르면 그때 release한다.
        self.get_logger().info(
            "ibvs_done received. Hold the arm FIRMLY, then press [R] to release servos."
        )
        self.ibvs_done_ros_time = self._now_sec()
        self.phase = InteractionPhase.READY_TO_RELEASE

    def ibvs_phase_callback(self, msg: Int32):
        if int(msg.data) == 99 and self.phase in (
            InteractionPhase.WAITING,
            InteractionPhase.READY_TO_RELEASE,
            InteractionPhase.RELEASING,
            InteractionPhase.FREE_DRIVE,
        ):
            self.get_logger().error("ibvs ERROR(99) received. Aborting episode.")
            # 루프 모드면 다음 episode로 진행, 아니면 종료.
            self._abort_episode(quit_session=False)

    # ============================================================
    # Key handling (GUI -> queue -> sm_timer)
    # ============================================================
    def handle_key(self, ch: str):
        # GUI 스레드에서 호출된다. 실제 처리는 sm_timer(ROS 스레드)에서 한다.
        if not ch:
            return
        self._key_queue.put(ch.lower())

    def _state_machine_callback(self):
        self._process_keys()

        if self.phase == InteractionPhase.RELEASING:
            self._update_releasing()
            return

        # [G] 후 fresh status 대기: 새 status 수신 또는 timeout 시 grip 기록.
        if self._grip_capture_pending:
            now_status_ns = (
                self.latest_status_time.nanoseconds
                if self.latest_status_time is not None else None
            )
            fresh = now_status_ns is not None and (
                self._grip_capture_req_status_ns is None
                or now_status_ns > self._grip_capture_req_status_ns
            )
            timed_out = (
                self._now_sec() - self._grip_capture_start_sec
                >= self.grip_capture_timeout_sec
            )
            if fresh or timed_out:
                if timed_out and not fresh:
                    self.get_logger().warn(
                        "Grip capture: fresh status timeout. Using last status."
                    )
                self._do_grip_after_capture()
            return

        if self.phase == InteractionPhase.GRIPPING:
            if self.gripping_start_ros_time is not None:
                elapsed = self._now_sec() - self.gripping_start_ros_time
                if elapsed >= self.grip_wait_sec:
                    self.phase = InteractionPhase.WAITING_RESULT
                    self.get_logger().info("Grip wait done. Waiting for result [S]/[F].")

    def _process_keys(self):
        while not self._key_queue.empty():
            try:
                key = self._key_queue.get_nowait()
            except queue.Empty:
                break

            if key == "r" and self.phase == InteractionPhase.READY_TO_RELEASE:
                self._trigger_release()
            elif key == "g" and self.phase == InteractionPhase.FREE_DRIVE:
                self._trigger_grip()
            elif key == "s" and self.phase == InteractionPhase.WAITING_RESULT:
                self._record_result(success=True)
            elif key == "f" and self.phase == InteractionPhase.WAITING_RESULT:
                self._record_result(success=False)
            elif key == "q":
                self.get_logger().warn("Quit requested (X). Finishing current episode and shutting down.")
                self._abort_episode(quit_session=True)

    # ============================================================
    # Event-driven sample recording
    # ============================================================
    def _record_timer_callback(self):
        # FREE_DRIVE 동안 고정 주기로 기록을 시도하되,
        #   1) status가 갱신되지 않은 프레임(같은 측정값 복사)은 기록하지 않는다.
        #   2) 사람이 실제로 움직이기 시작하기 전(서보 해제 지연 등)에는 기록하지 않는다.
        #   3) displacement 모드면 마지막 waypoint 대비 변위가 임계값 이상일 때만 기록한다.
        if self.phase != InteractionPhase.FREE_DRIVE:
            return
        if self.latest_angles is None or self.latest_status_time is None:
            return

        status_ns = self.latest_status_time.nanoseconds
        if (
            self._last_committed_status_ns is not None
            and status_ns <= self._last_committed_status_ns
        ):
            return

        if not self._motion_started:
            if self._free_drive_baseline_angles is None:
                # FREE_DRIVE 후 첫 측정값을 기준 자세로 잡는다(기록은 보류).
                self._free_drive_baseline_angles = self.latest_angles[:]
                self._last_committed_status_ns = status_ns
                return
            moved = max(
                abs(a - b)
                for a, b in zip(self.latest_angles, self._free_drive_baseline_angles)
            )
            if moved < self.motion_start_threshold_deg:
                self._last_committed_status_ns = status_ns
                return
            self._motion_started = True
            self.get_logger().info(
                f"Motion detected (max|dq|={moved:.2f}deg >= "
                f"{self.motion_start_threshold_deg}deg). Recording starts."
            )

        if self.record_mode == "displacement" and self.prev_recorded_angles is not None:
            # 제어 관절(J1~J5)만 본다. J6는 IBVS가 정렬한 뒤 human이 건드리지 않는
            # 관절이라 학습에서도 제외되며, 노이즈로 waypoint가 생기는 것을 막는다.
            disp = max(
                abs(self.latest_angles[i] - self.prev_recorded_angles[i])
                for i in range(5)
            )
            if disp < self.displacement_threshold_deg:
                self._last_committed_status_ns = status_ns
                return
            if disp >= self.displacement_threshold_deg * self.displacement_warn_factor:
                self.get_logger().warn(
                    f"Oversized waypoint step: max|dq|={disp:.1f}deg "
                    f"(threshold={self.displacement_threshold_deg}deg). "
                    f"Status gap or too-fast motion suspected."
                )

        self._commit_sample(grip_triggered=False, result_recorded=False)
        self._last_committed_status_ns = status_ns

    def _commit_sample(
        self,
        grip_triggered: bool,
        result_recorded: bool,
        grip_success: bool = False,
    ):
        if self.latest_angles is None:
            return

        now = self.get_clock().now()
        angles = self.latest_angles[:]

        delta = [0.0] * 6
        dt = 0.0
        if self.prev_recorded_angles is not None:
            delta = [angles[i] - self.prev_recorded_angles[i] for i in range(6)]
        if self.prev_recorded_ros_time is not None:
            dt = now.nanoseconds * 1e-9 - self.prev_recorded_ros_time

        sample = HumanInteractionSample()
        sample.header.stamp = now.to_msg()
        sample.header.frame_id = ""
        sample.episode_id = str(self.episode_id)
        sample.sample_index = int(self.sample_index)
        sample.phase = int(self.phase.value)
        sample.joint_angles = [float(v) for v in angles]
        sample.delta_angles = [float(v) for v in delta]
        sample.time_since_prev_sample = float(dt)
        sample.gripper_value = float(self.latest_gripper_value)
        sample.gripper_closed = (
            math.isfinite(self.latest_gripper_value)
            and self.latest_gripper_value <= 20.0
        )
        sample.grip_triggered = bool(grip_triggered)
        sample.result_recorded = bool(result_recorded)
        sample.grip_success = bool(grip_success) if result_recorded else False
        sample.time_since_ibvs_done = float(self._time_since_ibvs_done())

        # Closed-loop NN 입력용 detection. fresh(timeout 이내)면 det_valid=true.
        # stale/없음이면 det_valid=false 로 두되 마지막 유효값은 그대로 실어 freeze 정보 보존.
        det_valid = False
        if self.latest_det is not None and self.latest_det_time is not None:
            age = now.nanoseconds * 1e-9 - self.latest_det_time
            det_valid = age <= self.detection_timeout_sec
        if self.latest_det is not None:
            det_cx, det_cy, det_area = self.latest_det
        else:
            det_cx, det_cy, det_area = 0.0, 0.0, 0.0
        sample.det_valid = bool(det_valid)
        sample.det_cx = float(det_cx)
        sample.det_cy = float(det_cy)
        sample.det_area_norm = float(det_area)
        sample.det_image_width = float(self.image_width)
        sample.det_image_height = float(self.image_height)

        self._write_queue.put(
            (self.bag_topic, serialize_message(sample), now.nanoseconds)
        )
        self.sample_index += 1

        if not result_recorded:
            self.prev_recorded_angles = angles
            self.prev_recorded_ros_time = now.nanoseconds * 1e-9

    # ============================================================
    # State transitions
    # ============================================================
    def _publish_gripper_open(self):
        gripper_msg = Float64MultiArray()
        gripper_msg.data = [100.0, float(self.gripper_open_speed)]
        self.set_gripper_pub.publish(gripper_msg)

    def _trigger_release(self):
        # 사람이 팔을 잡은 상태에서 [R]을 눌렀을 때만 호출된다.
        self.get_logger().info(
            "Release triggered by [R]. Opening gripper and releasing servos. "
            "Recording resumes after release + gripper-open is confirmed."
        )
        # 서보가 살아있을 때 gripper를 먼저 100(open)으로 연다.
        self._publish_gripper_open()

        # 관절 release. release_all_servos는 gripper 서보까지 함께 푼다.
        arm_msg = Float64MultiArray()
        arm_msg.data = [0.0]
        self.set_arm_pub.publish(arm_msg)

        # release가 시리얼에서 먼저 처리되도록 짧은 지연 후 gripper open을 1회 재발행한다.
        # 그러면 gripper 서보만 다시 잡혀 100(open)이 유지되고 팔 관절은 풀린 상태로 남는다.
        self._schedule_gripper_reopen()

        # 즉시 FREE_DRIVE로 가지 않는다. 서보 해제와 gripper open이 시리얼에서 실제로
        # 반영되기 전에는 강체 자세/전이 프레임이 기록되므로, RELEASING에서 확인한 뒤
        # FREE_DRIVE로 전환한다.
        self.phase = InteractionPhase.RELEASING
        self._release_request_sec = self._now_sec()
        self._release_status_poll_sec = 0.0

    def _update_releasing(self):
        # RELEASING: gripper open + 서보 해제가 확인될 때까지 기록을 보류한다.
        elapsed = self._now_sec() - self._release_request_sec

        # gripper 상태를 빨리 확인하기 위해 status를 저주파로 폴링한다.
        self._poll_release_status()

        # reopen 발행 + 정착 시간이 지나기 전에는 무조건 대기.
        if elapsed < self.release_gripper_reopen_delay_sec + self.release_settle_margin_sec:
            return

        gripper_ok = (
            math.isfinite(self.latest_gripper_value)
            and self.latest_gripper_value >= self.gripper_open_confirm_value
        )
        timed_out = elapsed >= self.release_confirm_timeout_sec

        if gripper_ok or timed_out:
            if timed_out and not gripper_ok:
                self.get_logger().warn(
                    f"Release confirm timeout ({self.release_confirm_timeout_sec:.1f}s). "
                    f"gripper_value={self.latest_gripper_value}. Starting FREE_DRIVE anyway."
                )
            self._enter_free_drive(elapsed)

    def _poll_release_status(self):
        # RELEASING 동안 gripper open 여부를 빨리 확인하기 위해 status를 0.2s 간격으로 요청.
        now = self._now_sec()
        if now - self._release_status_poll_sec >= 0.2:
            self._release_status_poll_sec = now
            self.request_status_pub.publish(Empty())

    def _enter_free_drive(self, elapsed: float):
        # release + gripper open 확인 후에만 기록을 시작한다.
        # 기록 기준점 초기화: release 직후 첫 샘플의 delta가 과대해지지 않도록.
        self.prev_recorded_angles = None
        self.prev_recorded_ros_time = None
        # 움직임 감지 게이트 리셋: 실제 움직임이 관측될 때까지 기록 보류.
        self._motion_started = False
        self._free_drive_baseline_angles = None
        self._last_committed_status_ns = None
        self.phase = InteractionPhase.FREE_DRIVE
        self.get_logger().info(
            f"Release + gripper-open confirmed after {elapsed:.2f}s "
            f"(gripper_value={self.latest_gripper_value}). "
            f"Recording starts after motion is detected (FREE_DRIVE)."
        )

    def _schedule_gripper_reopen(self):
        # 기존 타이머가 남아있으면 정리한다.
        if self._gripper_reopen_timer is not None:
            self._gripper_reopen_timer.cancel()
            self._gripper_reopen_timer = None
        self._gripper_reopen_done = False
        self._gripper_reopen_timer = self.create_timer(
            self.release_gripper_reopen_delay_sec, self._gripper_reopen_cb
        )

    def _gripper_reopen_cb(self):
        # one-shot: 1회만 발행하고 타이머를 취소한다.
        if self._gripper_reopen_timer is not None:
            self._gripper_reopen_timer.cancel()
            self._gripper_reopen_timer = None
        if self._gripper_reopen_done:
            return
        self._gripper_reopen_done = True
        self._publish_gripper_open()
        self.get_logger().info(
            "Re-sent gripper open(100) after servo release to hold grip open."
        )

    def _trigger_grip(self):
        # grip 순간의 실제 위치를 정확히 기록하기 위해 fresh status를 요청하고
        # 새 status가 도착하면(또는 timeout) 그때 grip 샘플을 기록한다.
        self.get_logger().info("Grip [G]: requesting fresh status before capturing grip pose.")
        self._grip_capture_pending = True
        self._grip_capture_req_status_ns = (
            self.latest_status_time.nanoseconds if self.latest_status_time is not None else None
        )
        self._grip_capture_start_sec = self._now_sec()
        self.request_status_pub.publish(Empty())

    def _do_grip_after_capture(self):
        # fresh status(또는 timeout) 이후 호출. grip 샘플을 phase=FREE_DRIVE로 기록한 뒤
        # gripper를 닫고 GRIPPING으로 전이한다.
        self._grip_capture_pending = False
        self._commit_sample(grip_triggered=True, result_recorded=False)

        gripper_msg = Float64MultiArray()
        gripper_msg.data = [0.0, 50.0]
        self.set_gripper_pub.publish(gripper_msg)

        self.phase = InteractionPhase.GRIPPING
        self.gripping_start_ros_time = self._now_sec()
        self.get_logger().info("Grip pose captured. Closing gripper (GRIPPING).")

    def _record_result(self, success: bool):
        label = "SUCCESS" if success else "FAIL"
        self.get_logger().info(f"Result recorded: {label}")
        self._commit_sample(grip_triggered=False, result_recorded=True, grip_success=success)
        self.phase = InteractionPhase.RESULT
        self._close_bag_writer()
        dest = self._move_episode(success=success)
        self._result_text = f"{label}. Saved to {dest}" if dest else f"{label}."
        if self.loop_episodes:
            self._advance_episode()
        else:
            self._finished = True

    def _abort_episode(self, quit_session: bool = True):
        if self.phase == InteractionPhase.RESULT:
            return
        self.phase = InteractionPhase.RESULT
        self._close_bag_writer()

        episode_dir = self._episode_dir()
        if episode_dir is not None:
            try:
                episode_dir.mkdir(parents=True, exist_ok=True)
                (episode_dir / "ABORTED").touch()
                self.get_logger().info(f"ABORTED marker written at {episode_dir}")
            except Exception as exc:
                self.get_logger().error(f"Failed to write ABORTED marker: {exc}")

        self._result_text = "ABORTED (kept in raw/)."
        # quit_session(=X/Q) 이거나 루프 모드가 아니면 전체 종료. ERROR 루프면 다음으로.
        if quit_session or not self.loop_episodes:
            self._finished = True
        else:
            self._advance_episode()

    # ============================================================
    # Continuous episode loop
    # ============================================================
    @staticmethod
    def _increment_id(episode_id: str) -> str:
        # 끝의 숫자를 +1 한다(자릿수 유지). 숫자가 없으면 _2를 붙인다.
        m = re.search(r"^(.*?)(\d+)$", episode_id)
        if not m:
            return f"{episode_id}_2"
        prefix, num = m.group(1), m.group(2)
        return f"{prefix}{int(num) + 1:0{len(num)}d}"

    def _next_episode_id(self) -> str:
        # raw/success/fail 어디에도 없는 다음 id로 증가(이미 쌓인 데이터 건너뛰기).
        base = Path(os.path.expanduser(self.bag_base_dir))
        cur = self.episode_id or "ep000"
        for _ in range(100000):
            cur = self._increment_id(cur)
            exists = any(
                (base / d / f"episode_{cur}").exists()
                for d in ("raw", "success", "fail")
            )
            if not exists:
                return cur
        return cur

    def _advance_episode(self):
        next_id = self._next_episode_id()
        self.get_logger().info(
            f"Advancing episode: '{self.episode_id}' -> '{next_id}'"
        )
        self.episode_id = next_id

        # 새 human bag 열기 (이전 bag은 이미 닫힘). write thread도 재시작.
        self.resolved_bag_uri = self._resolve_bag_uri()
        self._open_bag_writer()
        self._write_thread = threading.Thread(
            target=self._write_worker, daemon=True, name="human_bag_write_worker"
        )
        self._write_thread.start()

        # 상태 리셋 → 다음 ibvs_done 대기.
        self.sample_index = 0
        self.prev_recorded_angles = None
        self.prev_recorded_ros_time = None
        self._last_committed_status_ns = None
        self._motion_started = False
        self._free_drive_baseline_angles = None
        self.ibvs_done_ros_time = None
        self.gripping_start_ros_time = None
        self._release_request_sec = 0.0
        self._release_status_poll_sec = 0.0
        self._result_text = ""
        self.phase = InteractionPhase.WAITING

        # ibvs_controller(재시작) / visual_servo_recorder(bag 재오픈)에 새 episode 알림.
        self.episode_pub.publish(String(data=next_id))
        self.get_logger().info(
            f"Published new episode '{next_id}'. Waiting for ibvs_done..."
        )

    # ============================================================
    # Episode directory management
    # ============================================================
    def _episode_dir(self) -> Optional[Path]:
        if not self.episode_id:
            return None
        return (
            Path(os.path.expanduser(self.bag_base_dir))
            / "raw"
            / f"episode_{self.episode_id}"
        )

    def _move_episode(self, success: bool) -> Optional[str]:
        episode_dir = self._episode_dir()
        if episode_dir is None:
            self.get_logger().warn("No episode_id set; skipping directory move.")
            return None

        dest_root = (
            Path(os.path.expanduser(self.bag_base_dir))
            / ("success" if success else "fail")
        )
        dest = dest_root / f"episode_{self.episode_id}"

        try:
            dest_root.mkdir(parents=True, exist_ok=True)
            if dest.exists():
                ts = datetime.now().strftime("%Y%m%d_%H%M%S")
                dest = dest_root / f"episode_{self.episode_id}_{ts}"
            shutil.move(str(episode_dir), str(dest))
            self.get_logger().info(f"Episode moved: {episode_dir} -> {dest}")
            return str(dest)
        except Exception as exc:
            self.get_logger().error(f"Failed to move episode directory: {exc}")
            return None


class RecorderGUI:
    """tkinter 기반 키/버튼 입력 및 상태 표시 창."""

    def __init__(self, root: "tk.Tk", node: HumanInteractionRecorderNode):
        self.root = root
        self.node = node
        self._closing = False

        root.title("Human Interaction Recorder")
        # 고정 높이 대신 콘텐츠에 맞춰 자동 확장(버튼 짤림 방지). 최소 폭만 지정.
        root.minsize(540, 0)

        self.phase_var = tk.StringVar()
        self.hint_var = tk.StringVar()
        self.info_var = tk.StringVar()
        self.angles_var = tk.StringVar()

        tk.Label(
            root, textvariable=self.phase_var, font=("TkDefaultFont", 18, "bold")
        ).pack(pady=(12, 4))
        tk.Label(
            root,
            textvariable=self.hint_var,
            font=("TkDefaultFont", 12),
            fg="#0050b0",
            wraplength=480,
        ).pack(pady=4)
        tk.Label(root, textvariable=self.info_var).pack(pady=2)
        tk.Label(
            root,
            textvariable=self.angles_var,
            font=("TkFixedFont", 9),
            wraplength=480,
        ).pack(pady=2)

        btn_frame = tk.Frame(root)
        btn_frame.pack(pady=12)

        self.btn_release = tk.Button(
            btn_frame, text="Release [R]", width=12, height=2,
            command=lambda: self.node.handle_key("r"),
        )
        self.btn_grip = tk.Button(
            btn_frame, text="Grip [G]", width=12, height=2,
            command=lambda: self.node.handle_key("g"),
        )
        self.btn_success = tk.Button(
            btn_frame, text="Success [S]", width=12, height=2,
            command=lambda: self.node.handle_key("s"),
        )
        self.btn_fail = tk.Button(
            btn_frame, text="Fail [F]", width=12, height=2,
            command=lambda: self.node.handle_key("f"),
        )
        self.btn_release.grid(row=0, column=0, padx=4, pady=4)
        self.btn_grip.grid(row=0, column=1, padx=4, pady=4)
        self.btn_success.grid(row=1, column=0, padx=4, pady=4)
        self.btn_fail.grid(row=1, column=1, padx=4, pady=4)

        # 종료 안내 + 창 우상단 X 로만 종료한다(현재 episode 포함 launch 전체 종료).
        tk.Label(
            root,
            text="창을 닫으면(X) 현재 episode를 마무리하고 전체 종료합니다.",
            font=("TkDefaultFont", 9), fg="#888888",
        ).pack(pady=(0, 8))

        root.bind("<Key>", self._on_key)
        root.protocol("WM_DELETE_WINDOW", lambda: self.node.handle_key("q"))

        self.refresh()

    def _on_key(self, event):
        ch = (event.char or "").lower()
        if ch in ("r", "g", "s", "f", "q"):
            self.node.handle_key(ch)

    def _update_buttons(self, phase: InteractionPhase):
        def st(active: bool):
            return tk.NORMAL if active else tk.DISABLED

        self.btn_release.config(state=st(phase == InteractionPhase.READY_TO_RELEASE))
        self.btn_grip.config(state=st(phase == InteractionPhase.FREE_DRIVE))
        self.btn_success.config(state=st(phase == InteractionPhase.WAITING_RESULT))
        self.btn_fail.config(state=st(phase == InteractionPhase.WAITING_RESULT))

    def refresh(self):
        if not rclpy.ok():
            self.root.destroy()
            return

        node = self.node
        phase = node.phase

        self.phase_var.set(f"Phase: {phase.name}")
        self.hint_var.set(node.get_action_hint())

        t = node._time_since_ibvs_done()
        grip = node.latest_gripper_value
        grip_str = f"{grip:.0f}" if math.isfinite(grip) else "N/A"
        self.info_var.set(
            f"t={t:.1f}s    samples={node.sample_index}    gripper={grip_str}"
        )

        if node.latest_angles:
            self.angles_var.set(
                "angles: " + str([round(v, 1) for v in node.latest_angles])
                + f"   J6={node.latest_angles[5]:.1f}"
            )
        else:
            self.angles_var.set("angles: N/A (waiting status)")

        self._update_buttons(phase)

        if getattr(node, "_finished", False) and not self._closing:
            self._closing = True
            if node.shutdown_on_done:
                self.root.after(2000, self.root.destroy)

        self.root.after(100, self.refresh)


def _spin_node(node):
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


def main(args=None):
    rclpy.init(args=args)
    node = HumanInteractionRecorderNode()

    spin_thread = threading.Thread(target=_spin_node, args=(node,), daemon=True)
    spin_thread.start()

    root = tk.Tk()
    RecorderGUI(root, node)

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node._close_bag_writer()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
