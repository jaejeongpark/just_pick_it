#!/usr/bin/env python3

"""
수집한 에피소드(ibvs + human rosbag)를 로봇에서 그대로 재생하는 테스트 노드.

순서:
  1. ibvs bag의 commanded_angles 를 순차 발행 (IBVS align+approach 경로 재현)
  2. human bag의 joint_angles 를 순차 발행 (사람이 손으로 움직인 fine-tune 궤적 재현)
  3. human bag에서 grip_triggered=True 시점에 set_gripper([0, speed]) 로 grip

타이밍은 bag에 기록된 타임스탬프 간격을 재현한다(time_scale로 배속 조절).
모든 구간을 servo on 상태로 target_pose 명령을 보내므로 release_all_servos는 호출하지 않는다.

주의: 첫 waypoint로 이동할 때 현재 로봇 위치와 차이가 클 수 있으므로,
initial_settle_sec 동안 대기한 뒤 본 재생을 시작한다.
"""

import math
import threading
import time
from pathlib import Path
from typing import List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.serialization import deserialize_message

import rosbag2_py

from std_msgs.msg import Float64MultiArray

from just_pick_it_interfaces.msg import HumanInteractionSample, VisualServoSample


CMD_JOINT = 0


class EpisodeReplayTester(Node):
    def __init__(self):
        super().__init__("episode_replay_tester")

        self.declare_parameter("robot_name", "jetcobot1")
        self.declare_parameter("episode_dir", "")
        self.declare_parameter("ibvs_subdir", "ibvs")
        self.declare_parameter("human_subdir", "human")
        self.declare_parameter("storage_id", "sqlite3")

        self.declare_parameter("replay_speed", 20)
        self.declare_parameter("initial_speed", 20)
        self.declare_parameter("gripper_speed", 50)

        self.declare_parameter("time_scale", 1.0)
        self.declare_parameter("max_step_sleep_sec", 5.0)
        self.declare_parameter("initial_settle_sec", 3.0)
        self.declare_parameter("start_delay_sec", 3.0)
        # grip 발행 전후 대기. 직전 joint 명령과 grip 명령이 시리얼에서 겹쳐
        # 로봇이 중간 자세로 튀는 것을 방지한다.
        self.declare_parameter("grip_settle_sec", 1.0)

        # human 구간 재생 시 joint_angles 대신 commanded가 없으므로 실측 궤적을 명령한다.
        self.declare_parameter("replay_human", True)
        # grip 후 result 샘플(grip 직후 위치)까지 재생할지.
        self.declare_parameter("replay_after_grip", True)

        # J6 hold.
        # J6 정렬은 이제 ibvs_controller가 IBVS 수렴 후 수행하며, 그 정렬 command가
        # IBVS bag에 기록된다(마지막 IBVS waypoint의 J6 = 정렬된 값). human bag의
        # joint_angles는 free-drive 중 서보가 풀려 J6가 drooping될 수 있으므로, human
        # 구간 재생 시 J6는 기록된 human 값 대신 IBVS 종단(정렬) 값으로 고정한다.
        self.declare_parameter("hold_j6_from_ibvs", True)
        self.declare_parameter("j6_grip_index", 5)

        self.robot_name = str(self.get_parameter("robot_name").value)
        self.episode_dir = str(self.get_parameter("episode_dir").value)
        self.ibvs_subdir = str(self.get_parameter("ibvs_subdir").value)
        self.human_subdir = str(self.get_parameter("human_subdir").value)
        self.storage_id = str(self.get_parameter("storage_id").value)

        self.replay_speed = int(self.get_parameter("replay_speed").value)
        self.initial_speed = int(self.get_parameter("initial_speed").value)
        self.gripper_speed = int(self.get_parameter("gripper_speed").value)

        # time_scale = 재생 배속 (ros2 bag play --rate 와 동일). 2.0이면 2배 빠르게.
        self.time_scale = max(0.01, float(self.get_parameter("time_scale").value))
        self.max_step_sleep_sec = float(self.get_parameter("max_step_sleep_sec").value)
        self.initial_settle_sec = float(self.get_parameter("initial_settle_sec").value)
        self.start_delay_sec = float(self.get_parameter("start_delay_sec").value)
        self.grip_settle_sec = float(self.get_parameter("grip_settle_sec").value)

        self.replay_human = bool(self.get_parameter("replay_human").value)
        self.replay_after_grip = bool(self.get_parameter("replay_after_grip").value)

        self.hold_j6_from_ibvs = bool(self.get_parameter("hold_j6_from_ibvs").value)
        self.j6_grip_index = int(self.get_parameter("j6_grip_index").value)

        self.ns = f"/{self.robot_name}"

        if not self.episode_dir:
            raise ValueError("episode_dir parameter is required")

        self.target_pose_pub = self.create_publisher(
            Float64MultiArray, f"{self.ns}/target_pose", 10
        )
        self.set_gripper_pub = self.create_publisher(
            Float64MultiArray, f"{self.ns}/set_gripper", 10
        )

        episode_path = Path(self.episode_dir).expanduser()
        self.ibvs_uri = str(episode_path / self.ibvs_subdir)
        self.human_uri = str(episode_path / self.human_subdir)

        self.ibvs_waypoints = self._load_ibvs(self.ibvs_uri)
        self.human_waypoints = self._load_human(self.human_uri)

        # human 구간 J6 고정값 = IBVS 종단(정렬된) waypoint의 J6.
        # hold가 꺼져 있거나 IBVS waypoint가 없으면 None (override 안 함).
        self.j6_target = self._compute_j6_hold()

        self.get_logger().info("EpisodeReplayTester started")
        self.get_logger().info(f"robot_name={self.robot_name}")
        self.get_logger().info(
            f"ibvs waypoints={len(self.ibvs_waypoints)} from {self.ibvs_uri}"
        )
        self.get_logger().info(
            f"human waypoints={len(self.human_waypoints)} from {self.human_uri}"
        )
        self.get_logger().info(
            f"replay_speed={self.replay_speed}, time_scale={self.time_scale}, "
            f"replay_human={self.replay_human}"
        )
        if self.hold_j6_from_ibvs:
            if self.j6_target is not None:
                self.get_logger().info(
                    f"J6 hold during human replay: J6 = {self.j6_target:.1f} deg "
                    f"(from IBVS end)."
                )
            else:
                self.get_logger().warn(
                    "J6 hold enabled but no IBVS waypoints. Using recorded human J6."
                )

        self._stop = False
        self._thread = threading.Thread(target=self._replay, daemon=True, name="replay")
        self._thread.start()

    # ============================================================
    # Bag loading
    # ============================================================
    def _open_reader(self, uri: str) -> rosbag2_py.SequentialReader:
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=uri, storage_id=self.storage_id),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr",
            ),
        )
        return reader

    def _load_ibvs(self, uri: str) -> List[Tuple[float, List[float]]]:
        reader = self._open_reader(uri)
        out: List[Tuple[float, List[float]]] = []
        while reader.has_next():
            _topic, data, t_ns = reader.read_next()
            msg = deserialize_message(data, VisualServoSample)
            if not msg.has_command:
                continue
            q = [float(v) for v in msg.commanded_angles]
            out.append((t_ns * 1e-9, q))
        return out

    def _load_human(
        self, uri: str
    ) -> List[Tuple[float, List[float], bool, bool]]:
        reader = self._open_reader(uri)
        out: List[Tuple[float, List[float], bool, bool]] = []
        while reader.has_next():
            _topic, data, t_ns = reader.read_next()
            msg = deserialize_message(data, HumanInteractionSample)
            q = [float(v) for v in msg.joint_angles]
            out.append((t_ns * 1e-9, q, bool(msg.grip_triggered), bool(msg.result_recorded)))
        return out

    def _compute_j6_hold(self) -> Optional[float]:
        if not self.hold_j6_from_ibvs:
            return None
        if not self.ibvs_waypoints:
            return None
        idx = self.j6_grip_index
        last_ibvs_q = self.ibvs_waypoints[-1][1]
        if idx < 0 or idx >= len(last_ibvs_q):
            self.get_logger().warn(f"Invalid j6_grip_index={idx}. Skipping J6 hold.")
            return None
        # IBVS 종단 waypoint의 J6 = ibvs_controller가 정렬한 값.
        return float(last_ibvs_q[idx])

    # ============================================================
    # Publish helpers
    # ============================================================
    def _publish_joint(self, q: List[float], speed: int):
        msg = Float64MultiArray()
        msg.data = [float(CMD_JOINT)] + [float(v) for v in q] + [float(speed)]
        self.target_pose_pub.publish(msg)

    def _publish_grip_close(self):
        msg = Float64MultiArray()
        msg.data = [0.0, float(self.gripper_speed)]
        self.set_gripper_pub.publish(msg)

    def _publish_grip_open(self):
        msg = Float64MultiArray()
        msg.data = [100.0, float(self.gripper_speed)]
        self.set_gripper_pub.publish(msg)

    def _sleep_dt(self, dt: float):
        # waypoint 간 간격. 배속(time_scale)을 적용하고 상한으로 clamp한다.
        if dt <= 0.0:
            return
        self._sleep_raw(min(dt / self.time_scale, self.max_step_sleep_sec))

    def _sleep_raw(self, sec: float):
        # 실제 시간만큼 대기(배속 미적용). 중단 가능하도록 잘게 나눠 sleep.
        if sec <= 0.0:
            return
        end = time.monotonic() + sec
        while not self._stop:
            remaining = end - time.monotonic()
            if remaining <= 0.0:
                break
            time.sleep(min(0.05, remaining))

    # ============================================================
    # Replay sequence
    # ============================================================
    def _replay(self):
        try:
            if not self.ibvs_waypoints and not self.human_waypoints:
                self.get_logger().error("No waypoints loaded. Abort.")
                return

            if self.start_delay_sec > 0.0:
                self.get_logger().warn(
                    f"Replay starts in {self.start_delay_sec:.1f}s. "
                    f"Ensure the workspace is clear. Ctrl+C to abort."
                )
                self._sleep_raw(self.start_delay_sec)

            # 항상 gripper를 open 상태로 시작한다.
            self.get_logger().info("Opening gripper before replay.")
            self._publish_grip_open()

            # 1. 첫 waypoint로 안전 이동 후 settle.
            first_wp = None
            if self.ibvs_waypoints:
                first_wp = self.ibvs_waypoints[0][1]
            elif self.human_waypoints:
                first_wp = self.human_waypoints[0][1]

            if first_wp is not None:
                self.get_logger().info(
                    f"Moving to first waypoint at speed={self.initial_speed}: "
                    f"{[round(v, 1) for v in first_wp]}"
                )
                self._publish_joint(first_wp, self.initial_speed)
                self._sleep_raw(self.initial_settle_sec)

            # 2. IBVS 구간 재생.
            self._replay_ibvs()
            if self._stop:
                return

            # 3. HUMAN 구간 재생 (+ grip).
            if self.replay_human:
                self._replay_human()

            self.get_logger().info("Replay done.")
            rclpy.shutdown()

        except Exception as exc:
            self.get_logger().error(f"Replay error: {exc}")
            rclpy.shutdown()

    def _replay_ibvs(self):
        if not self.ibvs_waypoints:
            return
        self.get_logger().info("=== IBVS replay ===")
        prev_t = self.ibvs_waypoints[0][0]
        for i, (t, q) in enumerate(self.ibvs_waypoints):
            if self._stop:
                return
            self._sleep_dt(t - prev_t)
            prev_t = t
            self._publish_joint(q, self.replay_speed)
            self.get_logger().info(
                f"IBVS {i + 1}/{len(self.ibvs_waypoints)} "
                f"q={[round(v, 1) for v in q]}"
            )

    def _apply_j6_override(self, q: List[float]) -> List[float]:
        # J6 align이 활성화되어 있으면 q의 J6를 OBB 장축 기반 목표값으로 덮어쓴다.
        if self.j6_target is None:
            return q
        q = list(q)
        q[self.j6_grip_index] = float(self.j6_target)
        return q

    def _replay_human(self):
        if not self.human_waypoints:
            return
        self.get_logger().info("=== HUMAN replay ===")
        # J6는 IBVS 종단(정렬) 값으로 이미 와 있으므로 별도 정렬 이동은 불필요하다.
        # human 구간 내내 J6를 그 값으로 고정(override)해 drooping된 기록값을 무시한다.

        prev_t = self.human_waypoints[0][0]
        gripped = False
        for i, (t, q, grip, result) in enumerate(self.human_waypoints):
            if self._stop:
                return
            if result and not self.replay_after_grip:
                break
            self._sleep_dt(t - prev_t)
            prev_t = t
            # human 구간 내내 J6는 정렬된 목표값으로 고정한다(기록된 J6는 무시).
            q = self._apply_j6_override(q)
            self._publish_joint(q, self.replay_speed)
            self.get_logger().info(
                f"HUMAN {i + 1}/{len(self.human_waypoints)} "
                f"q={[round(v, 1) for v in q]} grip={grip} result={result}"
            )
            if grip and not gripped:
                gripped = True
                # 직전 joint 명령이 도달하도록 대기한 뒤 grip 을 발행한다.
                # joint 명령과 grip 명령이 시리얼에서 겹치면 로봇이 튄다.
                self._sleep_raw(self.grip_settle_sec)
                self.get_logger().warn("grip_triggered: closing gripper.")
                self._publish_grip_close()
                # grip 이 완료되도록 대기한 뒤 다음 명령으로 넘어간다.
                self._sleep_raw(self.grip_settle_sec)

    def stop(self):
        self._stop = True


def main(args=None):
    rclpy.init(args=args)
    node = EpisodeReplayTester()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        node.get_logger().info("Interrupted.")
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
