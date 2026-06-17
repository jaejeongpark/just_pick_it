#!/usr/bin/env python3
"""
DISPLAY_PLACE 배치 agent (local AI 컴퓨터 192.168.1.70 에서 실행).

cobot 호스트의 cobot_controller 가 보내는 배치 요청을 받아, 이 로컬 머신에서
place_servo.launch.py(csrt_place_tracker + IBVS + place release)를 on-demand 로 실행한다.
ibvs/release 가 발행하는 제어 토픽은 cobot 호스트의 jetcobot_joint_subscriber 드라이버가
snatch 해 로봇을 구동한다. 이 agent 는 로봇을 직접 제어하지 않는다.

픽 agent(ibvs_nn_pick_agent)와 구조가 동일하되 완료 신호가 반대다:
  - 픽: nn_controller 의 grip '닫기'(set_gripper <= close_threshold) 관측 = 성공.
  - 배치: place release 의 gripper '열기'(set_gripper >= open_threshold) 관측 = 성공.

CSRT init 용 빈자리 bbox 는 cobot_controller 가 /place/target_bbox(latched)로 발행하므로
이 agent 가 따로 전달하지 않는다. perception 패키지 코드는 일절 수정하지 않는다.

인터페이스(cross-machine, std_msgs 만 사용):
  구독: /display_place/request  (String, "request_id|product_name")
  발행: /display_place/result   (String, "request_id|success(1/0)")
  관측: /{robot_name}/set_gripper (Float64MultiArray)
"""
import os
import signal
import subprocess
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Float64MultiArray, String


def _reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.VOLATILE,
        depth=depth,
    )


class DisplayPlaceAgent(Node):
    """배치 요청을 받아 로컬에서 place_servo.launch.py 를 실행/관측/종료한다."""

    def __init__(self) -> None:
        super().__init__('display_place_agent')

        self.declare_parameter('robot_name', 'jetcobot1')
        self.declare_parameter('launch_pkg', 'just_pick_it_perception')
        self.declare_parameter('launch_file', 'place_servo.launch.py')
        self.declare_parameter('place_timeout_sec', 120.0)
        # gripper open(>= threshold) 관측을 배치 완료로 본다. release 는 GRIPPER_OPEN(100).
        self.declare_parameter('grip_open_threshold', 60.0)
        self.declare_parameter('grip_settle_sec', 1.5)
        self.declare_parameter('request_topic', '/display_place/request')
        self.declare_parameter('result_topic', '/display_place/result')
        self.declare_parameter('extra_launch_args', [''])

        self._robot_name = self.get_parameter('robot_name').value
        self._launch_pkg = self.get_parameter('launch_pkg').value
        self._launch_file = self.get_parameter('launch_file').value
        self._place_timeout = float(self.get_parameter('place_timeout_sec').value)
        self._grip_open_threshold = float(self.get_parameter('grip_open_threshold').value)
        self._grip_settle_sec = float(self.get_parameter('grip_settle_sec').value)

        self._open_event = threading.Event()
        self._lock = threading.Lock()
        self._active_id: str | None = None
        self._last_result: tuple[str, bool] | None = None
        self._active_proc: subprocess.Popen | None = None
        self._preempt = False
        # launch 시작 직후 그리퍼는 상품을 쥔 '닫힘' 상태다. 그 시점의 잔여 open 명령을
        # 완료로 오인하지 않도록, 첫 set_gripper 관측은 무시 윈도우를 둔다.
        self._observe_open = False

        cb_group = ReentrantCallbackGroup()
        self._result_pub = self.create_publisher(
            String, self.get_parameter('result_topic').value, _reliable_qos()
        )
        self.create_subscription(
            String,
            self.get_parameter('request_topic').value,
            self._on_request,
            _reliable_qos(),
            callback_group=cb_group,
        )
        self.create_subscription(
            Float64MultiArray,
            f'/{self._robot_name}/set_gripper',
            self._on_set_gripper,
            10,
            callback_group=cb_group,
        )

        self.get_logger().info(
            f'[DisplayPlaceAgent] 시작 — robot_name={self._robot_name}, '
            f'launch={self._launch_pkg}/{self._launch_file}'
        )

    # ── 요청 처리 ────────────────────────────────────────────────────────

    def _on_request(self, msg: String) -> None:
        parsed = self._parse(msg.data)
        if parsed is None:
            return
        request_id, product_name = parsed

        with self._lock:
            if request_id == self._active_id:
                return
            if self._last_result is not None and self._last_result[0] == request_id:
                self._publish_result(request_id, self._last_result[1])
                return
            if self._active_id is not None:
                self.get_logger().warn(
                    f'[DisplayPlaceAgent] 새 요청 {request_id} 수신 — 기존 배치 '
                    f'{self._active_id} 선점/중단(재전송 시 수락)'
                )
                self._preempt = True
                self._open_event.set()
                return
            self._active_id = request_id

        threading.Thread(
            target=self._run_place, args=(request_id, product_name), daemon=True
        ).start()

    def _run_place(self, request_id: str, product_name: str) -> None:
        self.get_logger().info(
            f'[DisplayPlaceAgent] 배치 시작 — product={product_name}, request_id={request_id}'
        )
        success = False
        preempted = False
        proc = None
        try:
            with self._lock:
                self._preempt = False
            self._open_event.clear()
            # 시작 직후 닫힘 상태의 잔여 명령 오인 방지(짧은 무시 윈도우 후 관측 시작).
            self._observe_open = False
            proc = self._spawn_launch(product_name)
            if proc is not None:
                with self._lock:
                    self._active_proc = proc
                time.sleep(2.0)
                self._observe_open = True
                got = self._open_event.wait(timeout=self._place_timeout)
                with self._lock:
                    preempted = self._preempt
                if preempted:
                    self.get_logger().warn(
                        f'[DisplayPlaceAgent] 배치 {request_id} 선점되어 중단')
                elif got:
                    self.get_logger().info(
                        '[DisplayPlaceAgent] gripper open 관측 — 배치 성공 판정')
                    time.sleep(self._grip_settle_sec)
                    success = True
                else:
                    self.get_logger().error(
                        f'[DisplayPlaceAgent] 배치 타임아웃 ({self._place_timeout}s) — open 미관측')
        finally:
            if proc is not None:
                self._terminate(proc)
            with self._lock:
                self._active_proc = None
                self._active_id = None
                self._preempt = False
                self._observe_open = False
                if not preempted:
                    self._last_result = (request_id, success)
            if not preempted:
                self._publish_result(request_id, success)

    def shutdown(self) -> None:
        with self._lock:
            proc = self._active_proc
            self._active_proc = None
        if proc is not None:
            self.get_logger().info('[DisplayPlaceAgent] 종료 — 실행 중 launch 정리')
            self._terminate(proc)

    # ── set_gripper 관측 ─────────────────────────────────────────────────

    def _on_set_gripper(self, msg: Float64MultiArray) -> None:
        if not msg.data or not self._observe_open:
            return
        # place release 의 gripper 열기([>=open_threshold, speed]) 를 배치 완료로 본다.
        if float(msg.data[0]) >= self._grip_open_threshold:
            self._open_event.set()

    # ── launch 수명 관리 ─────────────────────────────────────────────────

    def _spawn_launch(self, product_name: str) -> subprocess.Popen | None:
        cmd = [
            'ros2', 'launch', self._launch_pkg, self._launch_file,
            f'robot_name:={self._robot_name}',
        ]
        for extra in self.get_parameter('extra_launch_args').value or []:
            if extra:
                cmd.append(extra)
        try:
            return subprocess.Popen(cmd, start_new_session=True)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'[DisplayPlaceAgent] launch 실행 실패: {exc}')
            return None

    def _terminate(self, proc: subprocess.Popen) -> None:
        if proc.poll() is not None:
            return
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGINT)
            try:
                proc.wait(timeout=10.0)
                return
            except subprocess.TimeoutExpired:
                os.killpg(pgid, signal.SIGTERM)
                proc.wait(timeout=5.0)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'[DisplayPlaceAgent] launch 종료 실패: {exc}')

    # ── 유틸 ─────────────────────────────────────────────────────────────

    def _publish_result(self, request_id: str, success: bool) -> None:
        msg = String()
        msg.data = f'{request_id}|{1 if success else 0}'
        self._result_pub.publish(msg)

    @staticmethod
    def _parse(data: str) -> tuple[str, str] | None:
        if '|' not in data:
            return None
        rid, _, product = data.partition('|')
        rid = rid.strip()
        product = product.strip()
        if not rid or not product:
            return None
        return rid, product


def main(args=None) -> None:
    rclpy.init(args=args)
    agent = DisplayPlaceAgent()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(agent)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        agent.shutdown()
        executor.shutdown()
        agent.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
