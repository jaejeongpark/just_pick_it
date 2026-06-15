#!/usr/bin/env python3
"""
IBVS+NN 픽 agent (local AI 컴퓨터 192.168.1.70 에서 실행).

cobot 호스트의 IbvsNnPickClient 가 보내는 픽 요청을 받아, 이 로컬 머신에서
nn_inference.launch.py(사용자의 ibvs_controller + nn_controller)를 on-demand 로 실행한다.
ibvs/nn 이 발행하는 제어 토픽은 cobot 호스트의 jetcobot_joint_subscriber 드라이버가
snatch 해서 로봇을 구동한다. 이 agent 는 로봇을 직접 제어하지 않는다.

인터페이스(cross-machine, std_msgs 만 사용):
  구독: /ibvs_nn_pick/request  (String, "request_id|product_name")
  발행: /ibvs_nn_pick/result   (String, "request_id|success(1/0)")
  관측: /{robot_name}/set_gripper (Float64MultiArray)
        nn_controller 가 grip 성공 후 발행하는 닫기 명령을 픽 완료 신호로 본다.

perception 패키지 코드는 일절 수정하지 않는다.
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


class IbvsNnPickAgent(Node):
    """픽 요청을 받아 로컬에서 nn_inference.launch.py 를 실행/관측/종료한다."""

    def __init__(self) -> None:
        super().__init__('ibvs_nn_pick_agent')

        self.declare_parameter('robot_name', 'jetcobot1')
        self.declare_parameter('launch_pkg', 'just_pick_it_perception')
        self.declare_parameter('launch_file', 'nn_inference.launch.py')
        self.declare_parameter('pick_timeout_sec', 120.0)
        self.declare_parameter('grip_close_threshold', 50.0)
        self.declare_parameter('grip_settle_sec', 2.5)
        self.declare_parameter('request_topic', '/ibvs_nn_pick/request')
        self.declare_parameter('result_topic', '/ibvs_nn_pick/result')
        # nn_inference.launch.py 에 그대로 넘길 추가 인자("key:=value" 목록).
        self.declare_parameter('extra_launch_args', [''])

        self._robot_name = self.get_parameter('robot_name').value
        self._launch_pkg = self.get_parameter('launch_pkg').value
        self._launch_file = self.get_parameter('launch_file').value
        self._pick_timeout = float(self.get_parameter('pick_timeout_sec').value)
        self._grip_close_threshold = float(self.get_parameter('grip_close_threshold').value)
        self._grip_settle_sec = float(self.get_parameter('grip_settle_sec').value)

        self._close_event = threading.Event()
        self._lock = threading.Lock()
        self._active_id: str | None = None
        self._last_result: tuple[str, bool] | None = None
        # 실행 중인 nn_inference subprocess. 노드 종료 시 정리해 고아 방지.
        self._active_proc: subprocess.Popen | None = None
        # 새 요청 도착 시 진행 중 픽을 선점(중단)하기 위한 플래그.
        self._preempt = False

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
        # nn_controller 의 grip 닫기 관측.
        self.create_subscription(
            Float64MultiArray,
            f'/{self._robot_name}/set_gripper',
            self._on_set_gripper,
            10,
            callback_group=cb_group,
        )

        self.get_logger().info(
            f'[IbvsNnPickAgent] 시작 — robot_name={self._robot_name}, '
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
                return  # 이미 처리 중인 동일 요청(클라이언트 재전송) 무시
            if self._last_result is not None and self._last_result[0] == request_id:
                # 이미 끝난 요청의 재전송 — 캐시된 결과를 다시 보내 idempotent 하게.
                self._publish_result(request_id, self._last_result[1])
                return
            if self._active_id is not None:
                # 다른(새) request_id 도착 = 기존 픽이 더 이상 유효하지 않음(예: 클라이언트
                # 재시작). 기존 픽을 선점해 중단시킨다. 워커가 정리하고 _active_id 를 비우면
                # 클라이언트 재전송(retry) 시 새 요청이 수락된다.
                self.get_logger().warn(
                    f'[IbvsNnPickAgent] 새 요청 {request_id} 수신 — 기존 픽 '
                    f'{self._active_id} 선점/중단(재전송 시 수락)'
                )
                self._preempt = True
                self._close_event.set()
                return
            self._active_id = request_id

        # 픽은 길게(최대 timeout) 걸리므로 worker 스레드에서 처리(executor 블로킹 방지).
        threading.Thread(
            target=self._run_pick, args=(request_id, product_name), daemon=True
        ).start()

    def _run_pick(self, request_id: str, product_name: str) -> None:
        self.get_logger().info(
            f'[IbvsNnPickAgent] 픽 시작 — product={product_name}, request_id={request_id}'
        )
        success = False
        preempted = False
        proc = None
        try:
            with self._lock:
                self._preempt = False
            self._close_event.clear()
            proc = self._spawn_launch(product_name)
            if proc is not None:
                with self._lock:
                    self._active_proc = proc
                got = self._close_event.wait(timeout=self._pick_timeout)
                with self._lock:
                    preempted = self._preempt
                if preempted:
                    self.get_logger().warn(
                        f'[IbvsNnPickAgent] 픽 {request_id} 선점되어 중단'
                    )
                elif got:
                    self.get_logger().info(
                        '[IbvsNnPickAgent] grip 닫기 관측 — 픽 성공 판정'
                    )
                    time.sleep(self._grip_settle_sec)
                    success = True
                else:
                    self.get_logger().error(
                        f'[IbvsNnPickAgent] 픽 타임아웃 ({self._pick_timeout}s) — grip 미관측'
                    )
        finally:
            if proc is not None:
                self._terminate(proc)
            with self._lock:
                self._active_proc = None
                self._active_id = None
                self._preempt = False
                # 선점된 경우 결과를 캐시/발행하지 않는다(옛 클라이언트는 이미 떠났고,
                # 새 요청은 재전송 시 정상 처리되도록 둔다).
                if not preempted:
                    self._last_result = (request_id, success)
            if not preempted:
                self._publish_result(request_id, success)

    def shutdown(self) -> None:
        """노드 종료 시 실행 중인 nn_inference subprocess 를 정리한다(고아 방지).

        subprocess 는 start_new_session 으로 별도 세션에서 돌기 때문에 agent launch 의
        SIGINT 가 전달되지 않는다. 따라서 종료 시 명시적으로 process group 을 정리한다.
        """
        with self._lock:
            proc = self._active_proc
            self._active_proc = None
        if proc is not None:
            self.get_logger().info('[IbvsNnPickAgent] 종료 — 실행 중 launch 정리')
            self._terminate(proc)

    # ── set_gripper 관측 ─────────────────────────────────────────────────

    def _on_set_gripper(self, msg: Float64MultiArray) -> None:
        if not msg.data:
            return
        # launch 시작 시 gripper open([100.0, ...])은 threshold 위라 무시.
        # nn_controller 의 grip 닫기([0.0, speed])만 픽 성공으로 잡는다.
        if float(msg.data[0]) <= self._grip_close_threshold:
            self._close_event.set()

    # ── launch 수명 관리 ─────────────────────────────────────────────────

    def _spawn_launch(self, product_name: str) -> subprocess.Popen | None:
        cmd = [
            'ros2', 'launch', self._launch_pkg, self._launch_file,
            f'robot_name:={self._robot_name}',
            f'target_class_label:={product_name}',
        ]
        for extra in self.get_parameter('extra_launch_args').value or []:
            if extra:
                cmd.append(extra)
        try:
            # 자식 노드까지 한 번에 정리할 수 있도록 새 프로세스 그룹으로 띄운다.
            return subprocess.Popen(cmd, start_new_session=True)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'[IbvsNnPickAgent] launch 실행 실패: {exc}')
            return None

    def _terminate(self, proc: subprocess.Popen) -> None:
        if proc.poll() is not None:
            return
        try:
            pgid = os.getpgid(proc.pid)
            os.killpg(pgid, signal.SIGINT)  # ros2 launch graceful 정리
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
            self.get_logger().error(f'[IbvsNnPickAgent] launch 종료 실패: {exc}')

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
    agent = IbvsNnPickAgent()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(agent)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        agent.shutdown()  # 실행 중 nn_inference subprocess 정리(고아 방지)
        executor.shutdown()
        agent.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
