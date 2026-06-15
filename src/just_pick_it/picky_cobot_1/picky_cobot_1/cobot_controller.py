#!/usr/bin/env python3
import threading
import time

from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import Empty, Float64MultiArray

from just_pick_it_interfaces.msg import TrackedObjectArray

from .ibvs_nn_pick import IbvsNnPickClient


# 드라이버(jetcobot_joint_subscriber) target_pose command_type.
CMD_JOINT = 0
CMD_COORD = 1

GRIPPER_OPEN      = 100
GRIPPER_CLOSED    = 0
GRIPPER_LOAD_OPEN = 70   # 적재 시 부분 개방값(완전 개방 100 대신 살짝만 열어 충돌/이탈 방지)
GRIPPER_SPEED     = 100
GRIPPER_WAIT_SEC  = 2.0  # 그리퍼 동작 완료 대기 시간

DEFAULT_SPEED         = 20    # 관절 이동 속도 (1~100)
MOTION_TIMEOUT_SEC    = 20.0
STATUS_POLL_INTERVAL_SEC = 0.3   # 모션 완료 폴링 주기(드라이버 serial read 부하 고려)
JOINT_CONVERGE_TOL_DEG   = 3.0   # 관절 수렴 판정 허용오차(deg)

# 드라이버 status(Float64MultiArray) 레이아웃:
#   [tool(6), world(6), reference_frame, end_type, angles(6), coords(6), gripper_value]
STATUS_ANGLES_SLICE = slice(14, 20)
STATUS_COORDS_SLICE = slice(20, 26)

# 단위: degree,  순서: [J1, J2, J3, J4, J5, J6]
_HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# 4 SLOT 이 모두 보이는 INSPECTION 관측 자세(관절각, degree).
INSPECTION_POSE = [20.39, -7.29, -28.82, -50.44, 5.36, -110.65]

# picky 적재 슬롯별 2단계 접근 관절각(pre-load 자세). 인덱스 = 적재 순서(슬롯 번호).
# place 위치는 상품 종류가 아니라 '집는 순서'로 결정한다. 첫 번째 적재 item은 slot 0,
# 두 번째는 slot 1 ... 에 적재한다.
# 협소 공간 충돌 방지를 위해 각 슬롯은 2단계로 접근한다.
#   approach (1단계): 슬롯 진입 전 안전 경유점.
#   place    (2단계): 실제 내려놓는 슬롯 위치.
# 적재 동작은 approach -> place -> grip 부분개방(70) -> 다시 approach 순으로 빠져나온다.
# 슬롯 4개 = picky 바구니 용량(공간 협소). 단위: degree, [J1..J6].
LOAD_SLOT_ANGLES = [
    {  # slot 0 (item 1)
        'approach': [7.38, -33.83, -49.92, -7.38, 3.60, -125.59],
        'place':    [6.41, -37.79, -65.47, 14.32, 4.30, -124.45],
    },
    {  # slot 1 (item 2)
        'approach': [11.33, -14.15, -64.59, -14.50, 4.39, -121.46],
        'place':    [9.93, -15.02, -96.32, 18.01, 6.15, -120.76],
    },
    {  # slot 2 (item 3)
        'approach': [26.71, -36.47, -44.20, -11.77, 3.60, -108.10],
        'place':    [24.96, -39.11, -60.55, 7.73, 4.39, -111.35],
    },
    {  # slot 3 (item 4)
        'approach': [29.61, -6.76, -71.98, -13.35, 4.57, -101.25],
        'place':    [29.53, -15.02, -98.70, 21.97, 4.04, -102.56],
    },
]

# IBVS pregrasp(탐색) 자세. center/left/right 고정 관절각. ibvs_controller 가 픽 시작 시
# 이 자세 중 하나에서 물체를 감지한다(탐색 순서 center -> left -> right).
# LOADING 3-step 의 1단계(위로 올리기)는 '이번 픽이 감지를 시작한 pregrasp 자세'로 복귀한 뒤
# 슬롯 approach -> place 로 이동한다. 감지 위치는 토픽으로 노출되지 않으므로 현재 J1(base)에
# 가장 가까운 자세를 추론해 사용한다.
# [중요] ibvs_controller.launch.py 의 center/left/right_pregrasp_angles 와 값을 일치시킬 것.
PREGRASP_ANGLES = {
    'center': [114.78, -5.09, -9.05, -75.49, 9.05, -107.31],
    'left':   [147.48, -8.96, -24.08, -59.85, 4.39, -73.12],
    'right':  [94.39, 1.31, -26.19, -62.84, 3.51, -127.08],
}

# 상품 수령장소(PICKUP SLOT)는 picky 정차 오차로 위치/자세가 매번 조금씩 틀어지므로
# 고정 관절각으로 가지 않는다(약 15x10cm 공간이라 픽보다 수렴은 쉬울 것으로 기대).
# 드롭 파이프라인(perception 측 추후 구현):
#   1. edge detection 으로 놓을 영역을 찾고, CSRT tracker 로 frame 을 추적해 일부 edge 가
#      가려져도 수렴을 유지하면서 IBVS 로 그 영역에 접근한다.
#   2. 수렴(ibvs_done) 후 drop 전용 NN(pick 과 별도 가중치)으로 최종 자세를 보정한다.
#   3. gripper predictor 가 '여는 시점'을 예측(pick 의 닫는 예측과 반대)해 gripper open 으로
#      물건을 내려놓는다. drop 완료 신호 = gripper open.
# 완성되면 pick 과 동일하게 ibvs_nn_pick_agent 에 'drop' 요청으로 연동한다(agent 가 drop
# launch 를 실행하고 set_gripper open 관측을 완료로 보고). 그 전까지 _unload_slot 의
# 드롭 단계는 미연동 상태다.


class CobotController:
    """
    Cobot 동작 제어기(토픽 기반).

    cobot_state_machine.py 에서 생성한다. serial 은 jetcobot_joint_subscriber 드라이버가
    단독 점유하므로, 이 제어기는 드라이버 토픽으로 로봇을 구동한다.
      발행: /{robot}/target_pose  (관절/좌표 명령), /{robot}/set_gripper, /{robot}/request_status
      구독: /{robot}/status       (현재 관절각으로 모션 완료 판정), detection_topic (INSPECTION)
    SORTING 픽은 IBVS+NN(IbvsNnPickClient) 에 위임한다.
    각 phase 메서드는 (success: bool, quantity: int) 를 반환한다.
    dry_run=True 이면 토픽을 발행하지 않고 시뮬레이션(성공 가정)한다.
    """

    def __init__(
        self,
        node,
        robot_name: str = 'jetcobot1',
        dry_run: bool = False,
        default_speed: int = DEFAULT_SPEED,
        motion_timeout_sec: float = MOTION_TIMEOUT_SEC,
        pick_timeout_sec: float = 120.0,
        pick_request_topic: str = '/ibvs_nn_pick/request',
        pick_result_topic: str = '/ibvs_nn_pick/result',
        detection_topic: str = '/infer/tracked_objects',
        inspect_min_confidence: float = 0.5,
        inspect_settle_sec: float = 2.0,
    ) -> None:
        self._node           = node
        self._robot_name     = robot_name
        self._dry_run        = dry_run
        self._default_speed  = int(default_speed)
        self._motion_timeout = float(motion_timeout_sec)
        self._inspect_min_confidence = float(inspect_min_confidence)
        self._inspect_settle_sec     = float(inspect_settle_sec)

        # SORTING 픽은 local AI 컴퓨터의 ibvs_nn_pick_agent 에 토픽으로 요청한다.
        self._pick = IbvsNnPickClient(
            node,
            request_topic=pick_request_topic,
            result_topic=pick_result_topic,
            pick_timeout_sec=pick_timeout_sec,
        )

        # 슬롯 점유 상태. 인덱스 = 슬롯 번호, 값 = 적재된 상품명(빈 슬롯은 None).
        self._slot_occupant: list[str | None] = [None] * len(LOAD_SLOT_ANGLES)
        # 적재 이력(순서대로). {'slot', 'product_name', 'order_id'}.
        self._placements: list[dict] = []

        ns = f'/{robot_name}'
        self._target_pub  = node.create_publisher(Float64MultiArray, f'{ns}/target_pose', 1)
        self._gripper_pub = node.create_publisher(Float64MultiArray, f'{ns}/set_gripper', 10)
        self._req_status_pub = node.create_publisher(Empty, f'{ns}/request_status', 10)

        cb_group = ReentrantCallbackGroup()
        self._status_lock = threading.Lock()
        self._latest_angles: list[float] | None = None
        self._latest_coords: list[float] | None = None
        self._status_seq = 0  # status 수신 카운터(fresh 판정용)
        node.create_subscription(
            Float64MultiArray, f'{ns}/status', self._status_callback, 10,
            callback_group=cb_group,
        )

        self._detect_lock = threading.Lock()
        self._latest_detection: TrackedObjectArray | None = None
        node.create_subscription(
            TrackedObjectArray, detection_topic, self._detection_callback, 10,
            callback_group=cb_group,
        )

        mode = 'dry_run(시뮬레이션)' if dry_run else f'토픽 구동({ns})'
        self._log(f'CobotController 초기화 — {mode}')

    # ── 공개 phase 메서드 ────────────────────────────────────────────────

    def run_sorting(self, product_name: str) -> tuple[bool, int]:
        """IBVS+NN 픽으로 지정 상품 1개를 집어 올린다.

        그리퍼가 1개라 한 번에 1개만 집을 수 있다. quantity 반복은 state machine 이
        '집기1 -> 적재1' 단위로 처리하므로 여기서는 1개만 집는다.
        반환값: (success, 집은 개수 0/1)
        """
        self._log(f'SORTING 시작 — product={product_name}')
        if not self._pick.pick(product_name):
            return False, 0
        return True, 1

    def run_loading(
        self,
        product_name: str,
        order_id: int = 0,
        target_zone_name: str = '',
    ) -> tuple[bool, int]:
        """집어 올린 상품 1개를 picky 의 다음 빈 적재 슬롯에 내려놓는다."""
        return self.load_to_next_slot(product_name, order_id)

    def run_inspecting(self) -> tuple[bool, int]:
        """4 SLOT 이 모두 보이는 자세에서 한 번에 검출해 적재 항목/수량을 검증한다.

        YOLO-seg 검출(detection_topic)을 cobot 자신의 적재 기록(_slot_occupant)과 비교한다.
        일치하면 (True, 검출 총개수), 불일치면 (False, 검출 총개수)를 반환한다.
        """
        expected = self._expected_counts()
        self._log(f'INSPECTING 시작 — 관측 자세 이동, 기대 적재={expected}')

        if not self.move_to_angles(INSPECTION_POSE):
            self._log_err('INSPECTION 관측 자세 이동 실패')
            return False, 0

        detected = self._observe_counts()
        total = sum(detected.values())

        if detected == expected:
            self._log(f'INSPECTION 일치 — detected={detected}')
            return True, total

        self._log_err(f'INSPECTION 불일치 — expected={expected}, detected={detected}')
        return False, total

    def run_unloading(self) -> tuple[bool, int]:
        """적재된 모든 item(최대 4개)을 순서대로 PICKUP SLOT 으로 옮겨 drop 한다.

        각 슬롯에서 2단계 접근으로 재파지 -> PICKUP 으로 이송 -> drop 한다.
        반환값: (success, drop 한 개수)
        """
        self._log('UNLOADING 시작 — 적재 item 을 PICKUP SLOT 으로 이송')
        dropped = 0
        for slot in range(len(LOAD_SLOT_ANGLES)):
            product = self._slot_occupant[slot]
            if product is None:
                continue
            if not self._unload_slot(slot, product):
                self._log_err(f'slot {slot} unloading 실패')
                return False, dropped
            self._slot_occupant[slot] = None
            dropped += 1
        self._log(f'UNLOADING 완료 — drop {dropped}개')
        return True, dropped

    def run_placing(self, product_name: str = '', target_zone_name: str = '') -> tuple[bool, int]:
        """진열(DISPLAY_PLACE) 동작. [구현 필요] 실제 진열 궤적/위치 연동."""
        self._log(f'PLACING 시작 — product={product_name}, zone={target_zone_name}')
        if self._dry_run:
            return True, 1
        # [구현 필요] 진열 위치로 이동 후 내려놓는 실제 동작.
        return True, 1

    def stow_arm(self) -> bool:
        """팔을 안전 복귀 자세(home)로 이동한다."""
        self._log('STOWING_ARM 시작')
        ok = self.move_to_angles(_HOME)
        self.open_gripper()
        return ok

    # ── picky 적재 슬롯 관리 ──────────────────────────────────────────────

    def load_to_next_slot(self, product_name: str, order_id: int = 0) -> tuple[bool, int]:
        """집은 상품 1개를 다음 빈 슬롯의 2단계 접근으로 내려놓고 적재 순서를 기록한다.

        place 위치는 상품 종류가 아니라 '집는 순서'로 정해진다. 협소 공간 충돌 방지를 위해
        approach(1단계) -> place(2단계) -> grip 부분개방(70) -> 다시 approach 로 빠져나온다.
        어떤 상품이 어느 슬롯에 들어갔는지 기억해 두어, 차후 inspection/unloading 에서 쓴다.
        """
        slot = self._next_free_slot()
        if slot is None:
            self._log_err('빈 적재 슬롯 없음 — picky 바구니가 가득 참')
            return False, 0

        approach = LOAD_SLOT_ANGLES[slot]['approach']
        place    = LOAD_SLOT_ANGLES[slot]['place']
        self._log(f'LOADING — product={product_name}, slot={slot}, order_id={order_id}')

        if not self._dry_run:
            # SORTING(픽)에서 닫은 GRIP 을 LOADING 이동 내내 유지한다(이동 중 열지 않음).
            # 핸드오프에서 그리퍼가 확실히 닫혀 있도록 명시적으로 재-grip.
            if not self.close_gripper():
                return False, 0
            # step1: 위로 올리기 — 이번 픽이 감지를 시작한 pregrasp(center/left/right)로 복귀.
            #         (그래스프 자세에서 슬롯으로 곧장 가지 않고 한 번 들어 올려 충돌 방지)
            pregrasp = self._current_pregrasp_name()
            self._log(f'LOADING step1 — pregrasp={pregrasp} 복귀')
            if not self.move_to_angles(PREGRASP_ANGLES[pregrasp]):
                return False, 0
            # step2: 슬롯 approach.
            if not self.move_to_angles(approach):
                return False, 0
            # step3: 슬롯 place — 안정적으로 도착할 때까지 확인.
            if not self.move_to_angles(place):
                return False, 0
            # 안정 도착 후 완전 개방(100) 대신 70 으로만 살짝 열어 내려놓는다.
            if not self._set_gripper(GRIPPER_LOAD_OPEN):
                return False, 0
            # approach 위치로 빠져나와 다음 작업을 준비한다.
            if not self.move_to_angles(approach):
                return False, 0
        else:
            self._log(f'(dry_run) slot {slot} 적재 기록만 — {product_name}')

        # 적재 성공 후 슬롯 점유 + 순서 기록.
        self._slot_occupant[slot] = product_name
        self._placements.append({
            'slot': slot,
            'product_name': product_name,
            'order_id': int(order_id or 0),
        })
        return True, 1

    def slot_angles_for_product(self, product_name: str) -> dict | None:
        """해당 상품이 적재된 슬롯의 2단계 접근 관절각을 반환한다(가장 먼저 적재된 것).

        반환: {'approach': [...], 'place': [...]} (없으면 None).
        차후 inspection/unloading/place 에서 1단계 approach 를 거쳐 2단계 place 로
        접근할 때 쓴다.
        """
        slot = self._slot_for_product(product_name)
        return LOAD_SLOT_ANGLES[slot] if slot is not None else None

    def release_slot_for_product(self, product_name: str) -> int | None:
        """해당 상품이 적재된 슬롯 하나를 비운다(unloading 등으로 꺼낸 뒤 호출).

        비운 슬롯 번호를 반환한다. 해당 상품이 없으면 None.
        """
        slot = self._slot_for_product(product_name)
        if slot is None:
            self._log_err(f'적재된 슬롯 없음 — product={product_name}')
            return None
        self._slot_occupant[slot] = None
        self._log(f'슬롯 {slot} 비움 — product={product_name}')
        return slot

    def _next_free_slot(self) -> int | None:
        for i, occupant in enumerate(self._slot_occupant):
            if occupant is None:
                return i
        return None

    def _slot_for_product(self, product_name: str) -> int | None:
        for i, occupant in enumerate(self._slot_occupant):
            if occupant == product_name:
                return i
        return None

    @property
    def placements(self) -> list[dict]:
        """적재 이력(순서대로): {'slot', 'product_name', 'order_id'}."""
        return list(self._placements)

    @property
    def current_loadout(self) -> dict[int, str]:
        """현재 슬롯별 적재 상품(점유된 슬롯만): {slot: product_name}."""
        return {i: occ for i, occ in enumerate(self._slot_occupant) if occ is not None}

    def flush_loadout(self) -> int:
        """picky 적재 슬롯 점유와 적재 이력을 모두 비운다(수동 리셋용).

        실제 물건을 내리는 게 아니라 내부 상태만 초기화한다. UNLOADING 드롭이 아직
        미연동이라 슬롯이 가득 차 막힐 때 수동 flush 에 쓴다. task 실행 중이 아닐 때
        호출해야 한다. 반환값: 비운(점유돼 있던) 슬롯 개수.
        """
        cleared = sum(1 for occ in self._slot_occupant if occ is not None)
        self._slot_occupant = [None] * len(LOAD_SLOT_ANGLES)
        self._placements.clear()
        self._log(f'적재 상태 flush — {cleared}개 슬롯 비움')
        return cleared

    # ── INSPECTION 검출 비교 ─────────────────────────────────────────────

    def _expected_counts(self) -> dict[str, int]:
        """적재 기록(_slot_occupant) 기준 상품별 기대 수량."""
        counts: dict[str, int] = {}
        for occupant in self._slot_occupant:
            if occupant:
                counts[occupant] = counts.get(occupant, 0) + 1
        return counts

    def _observe_counts(self) -> dict[str, int]:
        """관측 자세에서 YOLO-seg 검출을 상품별 수량으로 집계한다."""
        if self._dry_run:
            # 시뮬레이션: 적재 기록과 동일하게 검출됐다고 가정.
            return self._expected_counts()

        time.sleep(self._inspect_settle_sec)  # 검출 안정화 대기
        with self._detect_lock:
            msg = self._latest_detection

        counts: dict[str, int] = {}
        if msg is None:
            self._log_err('INSPECTION 검출 메시지 없음(detection_topic 확인)')
            return counts

        seen: set[int] = set()
        for obj in msg.objects:
            if obj.confidence < self._inspect_min_confidence:
                continue
            if obj.track_id in seen:  # 같은 객체 중복 제거
                continue
            seen.add(obj.track_id)
            counts[obj.class_label] = counts.get(obj.class_label, 0) + 1
        return counts

    # ── UNLOADING 슬롯별 이송 ────────────────────────────────────────────

    def _unload_slot(self, slot: int, product_name: str) -> bool:
        """슬롯에서 재파지(고정 2단계) 후 IBVS 로 pickup-space 에 접근해 release 한다."""
        approach = LOAD_SLOT_ANGLES[slot]['approach']
        place    = LOAD_SLOT_ANGLES[slot]['place']
        self._log(f'UNLOADING slot {slot} ({product_name}) -> PICKUP(IBVS)')

        if self._dry_run:
            self._log(f'(dry_run) slot {slot} {product_name} 재파지 + PICKUP IBVS 드롭 시뮬레이션')
            return True

        # 1) 슬롯에서 재파지(슬롯은 picky 에 고정 -> 정차 오차 무관, 고정 2단계 접근).
        if not self.move_to_angles(approach):
            return False
        if not self.move_to_angles(place):
            return False
        if not self.close_gripper():
            return False
        if not self.move_to_angles(approach):
            return False
        # 2) pickup-space 로 IBVS 접근 후 release(고정 자세 금지 — picky 정차 오차 보정).
        return self._drop_at_pickup_via_ibvs(product_name)

    def _drop_at_pickup_via_ibvs(self, product_name: str) -> bool:
        """pickup-space 로 IBVS 접근 후 drop NN + gripper predictor 로 release 한다.

        edge detection + CSRT tracker 로 찾은 놓을 영역에 IBVS 로 수렴(ibvs_done)한 뒤,
        drop 전용 NN(pick 과 별도 가중치)으로 자세를 보정하고 gripper predictor 가 여는
        시점을 예측하면 gripper open 으로 내려놓는다(완료 = gripper open).
        [구현 필요] drop 파이프라인(launch)이 준비되면 pick 과 동일하게 ibvs_nn_pick_agent
        에 'drop' 요청으로 연동한다(agent 가 set_gripper open 관측을 완료로 보고). drop
        파이프라인이 직접 release 하므로 cobot 이 별도로 open 하지 않는다.
        """
        self._log_err(
            'PICKUP 드롭 미연동 — drop 파이프라인(edge+CSRT+IBVS+drop NN) 준비 후 agent 연동 필요'
        )
        return False

    # ── 저수준 모션(토픽 기반) ───────────────────────────────────────────

    def move_to_angles(self, angles: list[float], speed: int | None = None) -> bool:
        """관절 각도(degree)를 target_pose 로 발행하고 status 수렴까지 블로킹."""
        speed = self._default_speed if speed is None else speed
        if self._dry_run:
            self._log(f'(dry_run) move_to_angles {angles}')
            return True
        self._publish_joint(angles, speed)
        return self._wait_until_angles(angles)

    def move_to_coords(
        self,
        coords: list[float],
        speed: int | None = None,
        mode: int = 0,
    ) -> bool:
        """Cartesian 좌표 [x,y,z,rx,ry,rz](mm/deg)를 target_pose 로 발행하고 블로킹."""
        speed = self._default_speed if speed is None else speed
        if self._dry_run:
            self._log(f'(dry_run) move_to_coords {coords}')
            return True
        msg = Float64MultiArray()
        msg.data = [float(CMD_COORD)] + [float(c) for c in coords] + [float(speed), float(mode)]
        self._target_pub.publish(msg)
        return self._wait_until_coords(coords)

    def execute_grasp_trajectory(self, trajectory: list[list[float]]) -> bool:
        """관절각 waypoint 목록을 순차 실행(blocking)."""
        self._log(f'궤적 실행 — {len(trajectory)}개 waypoint')
        for i, angles in enumerate(trajectory):
            if not self.move_to_angles(angles):
                self._log_err(f'궤적 {i}번 waypoint 실패')
                return False
        return True

    def _publish_joint(self, angles: list[float], speed: int) -> None:
        msg = Float64MultiArray()
        msg.data = [float(CMD_JOINT)] + [float(a) for a in angles] + [float(speed)]
        self._target_pub.publish(msg)

    def _wait_until_angles(self, target: list[float], timeout: float | None = None) -> bool:
        """target 관절각에 '안정적으로' 도착할 때까지 status 를 폴링한다.

        송신은 send_angles(_async) 라 즉시 리턴하므로, request_status 로 현재 관절각을
        받아 목표 ±JOINT_CONVERGE_TOL_DEG 이내인지 확인한다. 통과 중 순간 일치 오판을
        막기 위해 연속 2회 수렴해야 '안정 도착'으로 본다.
        """
        timeout = self._motion_timeout if timeout is None else timeout
        deadline = time.time() + timeout
        time.sleep(0.3)  # 이동 시작 여유(send_angles 는 비동기)
        stable = 0
        while time.time() < deadline:
            self._req_status_pub.publish(Empty())
            time.sleep(STATUS_POLL_INTERVAL_SEC)
            with self._status_lock:
                angles = list(self._latest_angles) if self._latest_angles is not None else None
            if angles is not None and self._converged(angles, target):
                stable += 1
                if stable >= 2:  # 연속 2회 수렴 = 안정적 도착
                    return True
            else:
                stable = 0
        self._log_err(f'관절 이동 타임아웃({timeout}s) — target={target}')
        return False

    def _wait_until_coords(self, target: list[float], timeout: float | None = None) -> bool:
        # 위치(x,y,z) 수렴만 본다(자세는 생략). 단위 mm.
        timeout = self._motion_timeout if timeout is None else timeout
        deadline = time.time() + timeout
        time.sleep(0.3)
        while time.time() < deadline:
            self._req_status_pub.publish(Empty())
            time.sleep(STATUS_POLL_INTERVAL_SEC)
            with self._status_lock:
                coords = list(self._latest_coords) if self._latest_coords is not None else None
            if coords is not None and all(
                abs(coords[i] - target[i]) <= 8.0 for i in range(3)
            ):
                return True
        self._log_err(f'좌표 이동 타임아웃({timeout}s) — target={target}')
        return False

    @staticmethod
    def _converged(measured: list[float], target: list[float]) -> bool:
        return all(abs(m - t) <= JOINT_CONVERGE_TOL_DEG for m, t in zip(measured, target))

    def _get_fresh_angles(self, timeout: float = 1.0) -> list[float] | None:
        """request_status 로 최신 status 를 받아 현재 관절각을 반환한다(없으면 None)."""
        with self._status_lock:
            start_seq = self._status_seq
        deadline = time.time() + timeout
        while time.time() < deadline:
            self._req_status_pub.publish(Empty())
            time.sleep(0.15)
            with self._status_lock:
                if self._status_seq != start_seq and self._latest_angles is not None:
                    return list(self._latest_angles)
        with self._status_lock:
            return list(self._latest_angles) if self._latest_angles is not None else None

    def _current_pregrasp_name(self) -> str:
        """현재 J1(base 회전각)에 가장 가까운 pregrasp(center/left/right)를 추론한다.

        IBVS 가 감지 위치(center/left/right)를 토픽으로 노출하지 않으므로, 픽 직후 자세의
        J1 으로 추론한다(탐색 자세 J1: center~115, left~147, right~94 로 잘 분리됨).
        status 가 없으면 안전하게 center 를 사용한다.
        """
        angles = self._get_fresh_angles()
        if angles is None:
            self._log_err('status 없음 — pregrasp 기본값 center 사용')
            return 'center'
        j1 = angles[0]
        name = min(PREGRASP_ANGLES, key=lambda k: abs(PREGRASP_ANGLES[k][0] - j1))
        self._log(f'pregrasp 판별 — J1={j1:.1f} -> {name}')
        return name

    # ── 그리퍼(토픽 기반) ────────────────────────────────────────────────

    def open_gripper(self) -> bool:
        return self._set_gripper(GRIPPER_OPEN)

    def close_gripper(self) -> bool:
        return self._set_gripper(GRIPPER_CLOSED)

    def _set_gripper(self, value: int, speed: int = GRIPPER_SPEED) -> bool:
        if self._dry_run:
            self._log(f'(dry_run) set_gripper {value}')
            return True
        msg = Float64MultiArray()
        msg.data = [float(value), float(speed)]
        self._gripper_pub.publish(msg)
        time.sleep(GRIPPER_WAIT_SEC)
        return True

    # ── 비상 정지 ────────────────────────────────────────────────────────

    def emergency_stop(self) -> None:
        if self._dry_run:
            return
        with self._status_lock:
            angles = list(self._latest_angles) if self._latest_angles is not None else None
        if angles is None:
            return
        # 현재 측정 자세를 목표로 발행해 추가 이동을 멈춘다(임시 hold).
        # [구현 필요] 드라이버에 정식 stop 명령이 생기면 교체.
        self._publish_joint(angles, self._default_speed)

    # ── status / detection 콜백 ──────────────────────────────────────────

    def _status_callback(self, msg: Float64MultiArray) -> None:
        data = list(msg.data)
        if len(data) < 26:
            return
        with self._status_lock:
            self._latest_angles = [float(v) for v in data[STATUS_ANGLES_SLICE]]
            self._latest_coords = [float(v) for v in data[STATUS_COORDS_SLICE]]
            self._status_seq += 1

    def _detection_callback(self, msg: TrackedObjectArray) -> None:
        with self._detect_lock:
            self._latest_detection = msg

    # ── 로그 ─────────────────────────────────────────────────────────────

    def _log(self, msg: str) -> None:
        self._node.get_logger().info(f'[CobotController] {msg}')

    def _log_err(self, msg: str) -> None:
        self._node.get_logger().error(f'[CobotController] {msg}')
