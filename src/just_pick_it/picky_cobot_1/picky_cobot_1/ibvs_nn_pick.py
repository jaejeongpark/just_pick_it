#!/usr/bin/env python3
"""
IBVS+NN 픽 트리거 클라이언트 (cobot 호스트 측).

배치:
  - cobot_state_machine / cobot_controller 는 cobot 호스트(192.168.1.99)에서 돈다.
  - IBVS+NN 파이프라인은 local AI 컴퓨터(192.168.1.70)에서 ibvs_nn_pick_agent 가
    on-demand 로 띄운다.

이 클라이언트는 로봇이나 launch 를 직접 다루지 않는다. 대신 cross-machine ROS 토픽으로
agent 에 픽을 요청하고 결과만 기다린다.
  발행: /ibvs_nn_pick/request  (std_msgs/String, "request_id|product_name")
  구독: /ibvs_nn_pick/result   (std_msgs/String, "request_id|success(1/0)")

agent 가 로컬에서 nn_inference.launch.py 를 실행하면, ibvs/nn 이 발행하는 제어 토픽
(/{robot}/target_pose, /{robot}/set_gripper)을 cobot 호스트의 jetcobot_joint_subscriber
드라이버가 snatch 해 로봇을 구동한다. perception 패키지 코드는 수정하지 않는다.
"""
import threading

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String


def _reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.VOLATILE,
        depth=depth,
    )


class IbvsNnPickClient:
    """ibvs_nn_pick_agent(local 컴퓨터)에 픽을 요청하고 결과를 기다리는 토픽 RPC 클라이언트."""

    def __init__(
        self,
        node,
        *,
        request_topic: str = '/ibvs_nn_pick/request',
        result_topic: str = '/ibvs_nn_pick/result',
        pick_timeout_sec: float = 120.0,
        retry_interval_sec: float = 3.0,
    ) -> None:
        self._node = node
        self._pick_timeout = pick_timeout_sec
        self._retry_interval = max(0.5, retry_interval_sec)

        self._lock = threading.Lock()
        self._event = threading.Event()
        self._pending_id: str | None = None
        self._result_success = False

        # request_id 충돌 방지용 세션 prefix + 시퀀스.
        self._session = str(node.get_clock().now().nanoseconds)
        self._seq = 0

        self._req_pub = node.create_publisher(String, request_topic, _reliable_qos())
        # run_sorting 이 pick() 에서 블로킹하는 동안에도 result 콜백이 다른 스레드에서
        # 돌도록 ReentrantCallbackGroup 으로 등록한다(MultiThreadedExecutor 전제).
        self._res_sub = node.create_subscription(
            String,
            result_topic,
            self._on_result,
            _reliable_qos(),
            callback_group=ReentrantCallbackGroup(),
        )

    # ── 공개 API ─────────────────────────────────────────────────────────

    def pick(self, product_name: str, timeout: float | None = None) -> bool:
        """agent 에 product_name 픽을 요청하고 완료될 때까지 기다린다.

        반환값: 픽 성공 여부.
        """
        if not product_name:
            self._log_err('product_name 이 비어 있어 픽을 요청할 수 없다')
            return False

        timeout = timeout if timeout is not None else self._pick_timeout

        with self._lock:
            self._seq += 1
            request_id = f'{self._session}-{self._seq}'
            self._pending_id = request_id
            self._result_success = False
            self._event.clear()

        msg = String()
        msg.data = f'{request_id}|{product_name}'

        self._log(f'픽 요청 — product={product_name}, request_id={request_id}, timeout={timeout}s')

        deadline = self._now() + timeout
        # agent 가 늦게 떠도 받도록 retry 주기로 재발행하며 result 를 기다린다.
        while self._now() < deadline:
            self._req_pub.publish(msg)
            if self._event.wait(timeout=self._retry_interval):
                with self._lock:
                    success = self._result_success
                self._log(f'픽 결과 수신 — request_id={request_id}, success={success}')
                return success

        self._log_err(
            f'픽 타임아웃 ({timeout}s) — agent 응답 없음. '
            f'local 컴퓨터(192.168.1.70)에서 ibvs_nn_pick_agent 가 떠 있는지 확인하라'
        )
        with self._lock:
            self._pending_id = None
        return False

    # ── 내부 구현 ────────────────────────────────────────────────────────

    def _on_result(self, msg: String) -> None:
        parsed = self._parse(msg.data)
        if parsed is None:
            return
        result_id, success = parsed
        with self._lock:
            if result_id != self._pending_id:
                return
            self._result_success = success
            self._pending_id = None
        self._event.set()

    @staticmethod
    def _parse(data: str) -> tuple[str, bool] | None:
        if '|' not in data:
            return None
        rid, _, raw = data.partition('|')
        return rid, raw.strip() in ('1', 'true', 'True', 'success')

    def _now(self) -> float:
        return self._node.get_clock().now().nanoseconds / 1e9

    def _log(self, msg: str) -> None:
        self._node.get_logger().info(f'[IbvsNnPick] {msg}')

    def _log_err(self, msg: str) -> None:
        self._node.get_logger().error(f'[IbvsNnPick] {msg}')
