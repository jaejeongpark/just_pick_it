#!/usr/bin/env python3
"""DISPLAY_PLACE 데이터 수집용 빈자리 자동 seed 글루.

운영(cobot_controller.run_scanning + _place_at_scanned)이 하던 empty_slot_detector
트리거와 /place/target_bbox seed 를, 데이터 수집 단계에서만 대신 수행하는 경량 노드다.

데이터 수집은 팔을 진열대 관측 자세(place_pregrasp) 한 곳에 고정한 채(ibvs_controller
가 search 로 그 자세에 세워 둔다) 진행하므로, 운영처럼 left/center/right 스윕이나 우승
자세 복귀는 하지 않는다. 현재 한 자세에서 detector 를 1회 트리거해 최적 빈자리 1곳을
자동 선정하고, 그 bbox 를 CSRT init 토픽(/place/target_bbox, latched)으로 넘긴다.
그러면 csrt_place_tracker 가 /place/tracked_objects 를 합성하고 ibvs_controller 가
search 를 빠져나와 그 빈자리로 수렴한다.

perception 코드(empty_slot_detector, csrt_place_tracker)는 수정하지 않는다.

트리거 시점:
  - 노드 시작 후 initial_delay_sec (팔이 place_pregrasp 에 도달할 시간) 경과 시 1회.
  - human recorder 의 episode advance(/{ns}/nn_episode) 수신 시마다 재선정. loop_episodes
    로 다음 episode 가 시작되면 선반 상태가 바뀌므로 빈자리를 다시 자동 선정한다.

데이터 수집은 운영처럼 매번 실제 물건을 그리퍼에 물리기 번거로우므로, '물건을 쥔 척'
그리퍼를 hold 값(예: 30)으로 닫은 채 IBVS 가 빈자리로 수렴하게 한다. ibvs_controller 는
episode advance 마다 그리퍼를 100(open)으로 열므로(pick 에서 물려받은 동작), seeder 가
시작 시 1회 + episode advance 직후(open 을 덮도록 짧게 지연)에 hold 값으로 다시 닫는다.

발행:
  /place/reset, /place/capture_view, /place/plan (Empty) : empty_slot_detector 트리거
  /place/target_bbox (Float64MultiArray, latched)        : [cx,cy,w,h,angle] CSRT init
  /{ns}/set_gripper  (Float64MultiArray)                 : [hold_value, speed] 그리퍼 닫기
구독:
  /place/scan_result (Float64MultiArray)                 : detector plan 결과
                                                           [found,cx,cy,w,h,angle,idx,score]
  /{ns}/nn_episode   (String, transient_local)           : episode advance 알림
"""

import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Empty, Float64MultiArray, String


def _latched_qos(depth: int = 1) -> QoSProfile:
    # transient_local: 늦게 뜬 csrt_place_tracker 도 마지막 target_bbox 를 받는다.
    # recorder 의 nn_episode(동일 durability)도 마지막 값 수신용으로 맞춘다.
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        depth=depth,
    )


class PlaceScanSeederNode(Node):

    def __init__(self):
        super().__init__('place_scan_seeder_node')

        self.declare_parameter('robot_name', 'jetcobot1')
        # 시작 후 첫 선정까지 대기(ibvs_controller 가 팔을 place_pregrasp 에 세울 시간).
        self.declare_parameter('initial_delay_sec', 4.0)
        # episode advance 후 재선정까지 대기(ibvs 가 팔을 place_pregrasp 로 복귀시킬 시간).
        self.declare_parameter('episode_delay_sec', 4.0)
        # reset 발행 후 detector 내부 상태 정리 대기.
        self.declare_parameter('reset_settle_sec', 0.3)
        # capture_view 발행 후 detector 다중프레임 샘플링 완료 대기(detector 의 capture_sample_sec
        # 보다 약간 길게).
        self.declare_parameter('capture_sample_sec', 1.5)
        # plan 결과(/place/scan_result) 대기 timeout.
        self.declare_parameter('plan_timeout_sec', 5.0)
        # 빈자리 후보 0개 또는 무응답 시 추가 재시도 횟수.
        self.declare_parameter('max_rescans', 2)
        # '물건 쥔 척' 그리퍼 닫힘값(0=완전 닫힘). place_interaction_recorder 의 동일
        # 파라미터와 맞춰 수렴~free-drive 내내 같은 그리퍼 상태를 유지한다.
        self.declare_parameter('gripper_hold_value', 0.0)
        self.declare_parameter('gripper_speed', 50)
        # episode advance 후 ibvs 의 그리퍼 open(100)을 덮어 다시 닫기까지의 지연(초).
        self.declare_parameter('gripper_close_after_episode_sec', 1.0)

        ns = str(self.get_parameter('robot_name').value).strip('/')
        self.initial_delay_sec = float(self.get_parameter('initial_delay_sec').value)
        self.episode_delay_sec = float(self.get_parameter('episode_delay_sec').value)
        self.reset_settle_sec = float(self.get_parameter('reset_settle_sec').value)
        self.capture_sample_sec = float(self.get_parameter('capture_sample_sec').value)
        self.plan_timeout_sec = float(self.get_parameter('plan_timeout_sec').value)
        self.max_rescans = int(self.get_parameter('max_rescans').value)
        self.gripper_hold_value = float(self.get_parameter('gripper_hold_value').value)
        self.gripper_speed = int(self.get_parameter('gripper_speed').value)
        self.gripper_close_after_episode_sec = float(
            self.get_parameter('gripper_close_after_episode_sec').value)

        cbg = ReentrantCallbackGroup()

        self._reset_pub = self.create_publisher(Empty, '/place/reset', 10)
        self._capture_pub = self.create_publisher(Empty, '/place/capture_view', 10)
        self._plan_pub = self.create_publisher(Empty, '/place/plan', 10)
        self._target_bbox_pub = self.create_publisher(
            Float64MultiArray, '/place/target_bbox', _latched_qos())
        self._gripper_pub = self.create_publisher(
            Float64MultiArray, f'/{ns}/set_gripper', 10)

        self.create_subscription(
            Float64MultiArray, '/place/scan_result', self._scan_result_cb, 10,
            callback_group=cbg)
        self.create_subscription(
            String, f'/{ns}/nn_episode', self._episode_cb, _latched_qos(),
            callback_group=cbg)

        self._scan_lock = threading.Lock()
        self._latest_scan_result = None
        self._scan_event = threading.Event()
        self._episode_event = threading.Event()

        self._worker = threading.Thread(target=self._run_loop, daemon=True)
        self._worker.start()

        self.get_logger().info(
            f'PlaceScanSeeder 준비 — 시작 {self.initial_delay_sec:.1f}s 후 첫 빈자리 자동 '
            f'선정, 이후 /{ns}/nn_episode 마다 재선정.')

    # ── 콜백 ──────────────────────────────────────────────────────────────
    def _scan_result_cb(self, msg: Float64MultiArray):
        if len(msg.data) < 6:
            return
        with self._scan_lock:
            self._latest_scan_result = list(msg.data)
        self._scan_event.set()

    def _episode_cb(self, msg: String):
        # nn_episode 는 recorder 의 _advance_episode 에서만 발행되므로(시작 시 발행 없음)
        # 수신 = 항상 새 episode 전환이다. 재선정을 예약한다.
        self.get_logger().info(f"Episode advance ('{msg.data}') — 빈자리 재선정 예약.")
        self._episode_event.set()

    def _publish_gripper_close(self):
        # '물건 쥔 척' hold 값으로 그리퍼를 닫는다(0=완전 닫힘, 30=30% 개방 등).
        msg = Float64MultiArray()
        msg.data = [self.gripper_hold_value, float(self.gripper_speed)]
        self._gripper_pub.publish(msg)
        self.get_logger().info(
            f'그리퍼 hold({self.gripper_hold_value:.0f}) 발행 — 물건 쥔 채 IBVS 수렴.')

    # ── 워커 루프 ─────────────────────────────────────────────────────────
    def _run_loop(self):
        # 첫 episode: 아무도 그리퍼를 열지 않으므로 시작 시 hold 값으로 닫아 '물건 쥔 척'.
        self._publish_gripper_close()
        self._sleep(self.initial_delay_sec)
        self._seed_once()
        while rclpy.ok():
            if not self._episode_event.wait(timeout=0.5):
                continue
            self._episode_event.clear()
            # ibvs 가 episode advance 에서 그리퍼를 100(open)으로 열므로(놓기 후 재탐색),
            # 그 open 을 덮도록 짧게 지연 후 다시 hold 값으로 닫아 수렴 구간 내내 유지한다.
            self._sleep(self.gripper_close_after_episode_sec)
            self._publish_gripper_close()
            self._sleep(max(0.0, self.episode_delay_sec - self.gripper_close_after_episode_sec))
            self._seed_once()

    def _seed_once(self):
        """detector 를 reset -> capture_view -> plan 으로 1회 트리거하고, found 면 bbox seed."""
        for attempt in range(self.max_rescans + 1):
            if not rclpy.ok():
                return
            self._reset_pub.publish(Empty())
            self._sleep(self.reset_settle_sec)
            self._capture_pub.publish(Empty())
            self._sleep(self.capture_sample_sec)

            with self._scan_lock:
                self._latest_scan_result = None
            self._scan_event.clear()
            self._plan_pub.publish(Empty())
            if not self._scan_event.wait(timeout=self.plan_timeout_sec):
                self.get_logger().warn(
                    f'scan_result 응답 없음(timeout {self.plan_timeout_sec:.1f}s) — '
                    'empty_slot_detector 가 떠 있는지 확인. 재시도.')
                continue

            with self._scan_lock:
                res = list(self._latest_scan_result) if self._latest_scan_result else None
            if not res or res[0] < 0.5:
                self.get_logger().warn(
                    f'빈자리 후보 없음(시도 {attempt + 1}/{self.max_rescans + 1}) — 재시도.')
                continue

            # res = [found, cx, cy, w, h, angle, capture_index, score]
            cx, cy, w, h, angle = res[1], res[2], res[3], res[4], res[5]
            msg = Float64MultiArray()
            msg.data = [float(cx), float(cy), float(w), float(h), float(angle)]
            self._target_bbox_pub.publish(msg)
            self.get_logger().info(
                f'빈자리 자동 선정 완료 — target_bbox center=({cx:.0f},{cy:.0f}), '
                f'size=({w:.0f}x{h:.0f}) 발행(CSRT init).')
            return

        self.get_logger().error(
            '빈자리 자동 선정 실패 — 재시도 한도 초과. 선반에 빈자리가 보이는지, '
            'empty_slot_detector/yolo_seg_infer 영상이 들어오는지 확인.')

    def _sleep(self, sec: float):
        # rclpy 종료에 빠르게 반응하도록 분할 sleep.
        remaining = float(sec)
        while remaining > 0.0 and rclpy.ok():
            dt = min(0.1, remaining)
            time.sleep(dt)
            remaining -= dt


def main(args=None):
    rclpy.init(args=args)
    node = PlaceScanSeederNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
