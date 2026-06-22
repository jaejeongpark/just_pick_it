#!/usr/bin/env python3
"""
DISPLAY_PLACE 배치 agent (local AI 컴퓨터 192.168.1.70 에서 실행).

cobot 호스트의 cobot_controller 가 보내는 배치 요청을 받아, 이 로컬 머신에서
place_nn_servo.launch.py(csrt_place_tracker + IBVS + 픽 nn_controller, weight 공유)를
on-demand 로 실행한다. 제어 토픽은 cobot 호스트의 jetcobot_joint_subscriber 드라이버가
snatch 해 로봇을 구동한다.

release 반전(perception 무수정 핵심):
  - 픽 nn_controller 는 grip 을 [0](close)로 하드코딩한다(perception 코드라 수정하지 않는다).
  - place 는 물건을 놓아야 하므로, nn_controller 의 close 발행을 'NN 정렬 완료 = 놓을 타이밍'
    신호로만 본다. 이 agent 가 그 close 를 관측하면(set_gripper <= close_threshold) 곧바로
    set_gripper [place_open_value](open) 를 발행해 release 하고, 안정 대기 후 완료로 보고한다.
    즉 여는 동작(release)은 perception 이 아니라 이 agent 가 담당한다.

픽 agent(ibvs_nn_pick_agent)와 구조가 동일하되, 픽은 close 관측 자체가 완료인 반면
place 는 close 관측 -> agent 가 open 발행 -> 완료다.

CSRT init 용 빈자리 bbox 는 cobot_controller 가 /place/target_bbox(latched)로 발행하므로
이 agent 가 따로 전달하지 않는다.

우승 스캔 자세(IBVS pregrasp 일치):
  - cobot_controller 가 우승 스캔 자세를 /place/pregrasp_angles(latched)로 발행한다.
  - 이 agent 가 받아 place_nn_servo 기동 시 place_pregrasp_angles:= 로 주입해, IBVS 의
    pregrasp = 우승 스캔 자세 = CSRT init 자세가 되도록 맞춘다. 이렇게 하면 servo 시작 시
    IBVS 가 카메라를 다른 자세로 옮기지 않아 init 한 bbox 가 유효한 채 수렴한다.
    (미수신 시 place_nn_servo 의 placeholder pregrasp 사용 — left/right 우승 시 어긋남 주의)

인터페이스(cross-machine, std_msgs 만 사용):
  구독: /display_place/request  (String, "request_id|product_name")
  구독: /place/pregrasp_angles  (Float64MultiArray, 우승 스캔 자세 6 관절 deg, latched)
  발행: /display_place/result   (String, "request_id|success(1/0)")
  관측: /{robot_name}/set_gripper (Float64MultiArray)  (nn 의 close 관측)
  발행: /{robot_name}/set_gripper (Float64MultiArray)  (release open 발행)
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


def _latched_qos(depth: int = 1) -> QoSProfile:
    # cobot_controller 가 우승 자세를 latched(transient_local)로 발행하므로 동일 durability 로
    # 받아야 늦게 떠도 마지막 값을 수신한다.
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        depth=depth,
    )


class DisplayPlaceAgent(Node):
    """배치 요청을 받아 로컬에서 place_nn_servo.launch.py 를 실행/관측/종료한다."""

    def __init__(self) -> None:
        super().__init__('display_place_agent')

        self.declare_parameter('robot_name', 'jetcobot1')
        self.declare_parameter('launch_pkg', 'picky_cobot_1')
        self.declare_parameter('launch_file', 'place_nn_servo.launch.py')
        self.declare_parameter('place_timeout_sec', 120.0)
        # nn_controller 의 grip close(<= threshold) 관측을 'NN 정렬 완료 = 놓을 타이밍'으로 본다.
        self.declare_parameter('grip_close_threshold', 50.0)
        # release 로 발행할 open 값(70% 개방, 완전 개방 100 대신).
        self.declare_parameter('place_open_value', 70.0)
        self.declare_parameter('gripper_speed', 50)
        self.declare_parameter('grip_settle_sec', 1.5)
        self.declare_parameter('request_topic', '/display_place/request')
        self.declare_parameter('result_topic', '/display_place/result')
        # cobot_controller 가 발행하는 우승 스캔 자세 토픽(IBVS pregrasp 주입용).
        self.declare_parameter('pregrasp_topic', '/place/pregrasp_angles')
        self.declare_parameter('extra_launch_args', [''])

        self._robot_name = self.get_parameter('robot_name').value
        self._launch_pkg = self.get_parameter('launch_pkg').value
        self._launch_file = self.get_parameter('launch_file').value
        self._place_timeout = float(self.get_parameter('place_timeout_sec').value)
        self._grip_close_threshold = float(self.get_parameter('grip_close_threshold').value)
        self._place_open_value = float(self.get_parameter('place_open_value').value)
        self._gripper_speed = int(self.get_parameter('gripper_speed').value)
        self._grip_settle_sec = float(self.get_parameter('grip_settle_sec').value)

        # nn_controller 의 grip close 발행을 기다리는 이벤트(= NN 정렬 완료 신호).
        self._align_event = threading.Event()
        self._lock = threading.Lock()
        self._active_id: str | None = None
        self._last_result: tuple[str, bool] | None = None
        self._active_proc: subprocess.Popen | None = None
        self._preempt = False
        # 시작 직후엔 물건을 쥔 '닫힘' 상태라 잔여 close 명령을 정렬완료로 오인할 수 있다.
        # 짧은 무시 윈도우 후에만 close 를 관측한다.
        self._observe_close = False
        # cobot_controller 가 보낸 우승 스캔 자세(6 관절 deg). place_nn_servo IBVS pregrasp 으로 주입.
        self._pregrasp_angles: list[float] | None = None

        cb_group = ReentrantCallbackGroup()
        self._result_pub = self.create_publisher(
            String, self.get_parameter('result_topic').value, _reliable_qos()
        )
        # release(open) 발행용. cobot 호스트 드라이버가 snatch 해 로봇을 구동한다.
        self._gripper_pub = self.create_publisher(
            Float64MultiArray, f'/{self._robot_name}/set_gripper', 10
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
        # 우승 스캔 자세 수신(latched). place_nn_servo 기동 시 IBVS pregrasp 으로 주입.
        self.create_subscription(
            Float64MultiArray,
            self.get_parameter('pregrasp_topic').value,
            self._on_pregrasp,
            _latched_qos(),
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
                self._align_event.set()
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
            self._align_event.clear()
            # 시작 직후 닫힘 상태의 잔여 명령 오인 방지(짧은 무시 윈도우 후 관측 시작).
            self._observe_close = False
            proc = self._spawn_launch(product_name)
            if proc is not None:
                with self._lock:
                    self._active_proc = proc
                time.sleep(2.0)
                self._observe_close = True
                got = self._align_event.wait(timeout=self._place_timeout)
                with self._lock:
                    preempted = self._preempt
                if preempted:
                    self.get_logger().warn(
                        f'[DisplayPlaceAgent] 배치 {request_id} 선점되어 중단')
                elif got:
                    # NN 정렬 완료(close 관측) -> agent 가 직접 open(release) 발행.
                    self.get_logger().info(
                        '[DisplayPlaceAgent] nn grip close 관측(정렬 완료) — '
                        f'release open({self._place_open_value:.0f}) 발행')
                    self._observe_close = False  # release open 을 다시 관측하지 않도록
                    self._publish_gripper_open()
                    time.sleep(self._grip_settle_sec)
                    success = True
                else:
                    self.get_logger().error(
                        f'[DisplayPlaceAgent] 배치 타임아웃 ({self._place_timeout}s) — '
                        'nn grip close 미관측')
        finally:
            if proc is not None:
                self._terminate(proc)
            with self._lock:
                self._active_proc = None
                self._active_id = None
                self._preempt = False
                self._observe_close = False
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

    def _on_pregrasp(self, msg: Float64MultiArray) -> None:
        # 6 관절 각도(deg). place_nn_servo 의 place_pregrasp_angles 로 주입할 우승 자세.
        if len(msg.data) != 6:
            self.get_logger().warn(
                f'[DisplayPlaceAgent] pregrasp 자세 길이 오류({len(msg.data)}!=6) — 무시')
            return
        with self._lock:
            self._pregrasp_angles = [float(v) for v in msg.data]
        self.get_logger().info(
            f'[DisplayPlaceAgent] 우승 자세 수신 — pregrasp={self._pregrasp_angles}')

    def _on_set_gripper(self, msg: Float64MultiArray) -> None:
        if not msg.data or not self._observe_close:
            return
        # nn_controller 의 grip 닫기([<=close_threshold, speed]) 를 'NN 정렬 완료'로 본다.
        if float(msg.data[0]) <= self._grip_close_threshold:
            self._align_event.set()

    def _publish_gripper_open(self) -> None:
        msg = Float64MultiArray()
        msg.data = [self._place_open_value, float(self._gripper_speed)]
        self._gripper_pub.publish(msg)

    # ── launch 수명 관리 ─────────────────────────────────────────────────

    def _spawn_launch(self, product_name: str) -> subprocess.Popen | None:
        cmd = [
            'ros2', 'launch', self._launch_pkg, self._launch_file,
            f'robot_name:={self._robot_name}',
        ]
        # 우승 스캔 자세를 IBVS pregrasp 으로 주입(우승 자세=CSRT init 자세 일치). 없으면
        # place_nn_servo 의 placeholder 기본값을 쓴다(left/right 우승 시 시야 어긋남 주의).
        with self._lock:
            pregrasp = list(self._pregrasp_angles) if self._pregrasp_angles else None
        if pregrasp is not None:
            angles_str = '[' + ','.join(f'{v:.4f}' for v in pregrasp) + ']'
            cmd.append(f'place_pregrasp_angles:={angles_str}')
        else:
            self.get_logger().warn(
                '[DisplayPlaceAgent] 우승 자세 미수신 — place_nn_servo placeholder pregrasp 사용')
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
