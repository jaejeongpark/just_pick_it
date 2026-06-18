#!/usr/bin/env python3
import threading
import time

from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from std_msgs.msg import Empty, Float64MultiArray

from just_pick_it_interfaces.msg import TrackedObjectArray

from .ibvs_nn_pick import IbvsNnPickClient


def _latched_qos(depth: int = 1) -> QoSProfile:
    # transient_local: 늦게 뜬 CSRT tracker(agent on-demand 기동)도 마지막 bbox를 받는다.
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        depth=depth,
    )


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
        'approach': [9.39, -30.94, -48.00, -7.67, 2.98, -131.03],
        'place':    [9.39, -30.94, -56.00, -5.67, 2.98, -131.03],
    },
    {  # slot 1 (item 2)
        'approach': [11.44, -10.01, -75.82, 3.00, 2.81, -134.47],
        'place':    [11.44, -10.01, -84.82, 3.00, 2.81, -134.47],
    },
    {  # slot 2 (item 3)
        'approach': [28.83, -39.76, -25.17, -17.21, -1.65, -108.98],
        'place':    [28.83, -39.76, -36.17, -17.21, -1.65, -108.98],
    },
    {  # slot 3 (item 4)
        'approach': [28.91, -15.38, -62.47, -11.51, 1.14, -109.68],
        'place':    [24.87, -20.19, -67.78, -4.65, 4.57, -110.91],
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

# DISPLAY_SCAN 스윕 자세(관절각, degree, [J1..J6]). 진열대 위를 좌->중->우로 훑어보며
# 각 자세에서 빈자리 후보를 누적한다. empty_slot_detector 가 각 capture 를 자세 1개에
# 대응시키므로, 우승 capture_index 로 이 리스트에서 복귀할 자세를 되찾는다.
# [하드웨어 보정 필요] 실제 진열대(가로 24 x 세로 10 x 높이 12cm)를 내려다보는 관측
# 자세로 실측 보정할 것. 아래는 픽 pregrasp 를 출발점으로 둔 placeholder 다.
DISPLAY_SCAN_ANGLES = [
    [147.48, -8.96, -24.08, -59.85, 4.39, -73.12],   # left
    [114.78, -5.09, -9.05, -75.49, 9.05, -107.31],   # center
    [94.39, 1.31, -26.19, -62.84, 3.51, -127.08],    # right
]

# 재파지 직후 진열대로 이동할 때 거치는 안전 경유(상승) 자세. picky 슬롯 lift 만으로는
# 상승분이 부족해 진열된 상품을 칠 위험이 있어, center pregrasp 로 가기 전에 먼저 거친다.
REGRASP_TRANSIT_ANGLES = [49.39, -17.40, -0.79, -68.99, 5.27, -122.69]

# 스캔 타이밍.
SCAN_SETTLE_SEC   = 0.6    # 스캔 자세 도착 후 영상 안정 대기
SCAN_CAPTURE_SEC  = 1.5    # capture_view 발행 후 detector 다중프레임 샘플링 완료 대기
SCAN_PLAN_TIMEOUT = 5.0    # plan 결과(/place/scan_result) 대기
SCAN_MAX_RESCANS  = 2      # 빈자리 0개 시 재스캔 추가 시도 횟수
# 우승 자세 복귀 후 CSRT init 전 안정 대기. 팔 진동이 가라앉은 안정 프레임에서 추적 시작.
PLACE_SETTLE_SEC  = 2.0

PICKUP_SLOT_APPROACH = [118.74, 11.68, -47.84, -30.76, 1.05, -112.06]
PICKUP_SLOT_PLACE   = [118.74, 11.68, -74.35, -30.76, 1.05, -112.06]


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
        place_request_topic: str = '/display_place/request',
        place_result_topic: str = '/display_place/result',
        place_timeout_sec: float = 120.0,
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

        self._e_stop_event = threading.Event()
        self._e_stop_event.set()

        # SORTING 픽은 local AI 컴퓨터의 ibvs_nn_pick_agent 에 토픽으로 요청한다.
        self._pick = IbvsNnPickClient(
            node,
            request_topic=pick_request_topic,
            result_topic=pick_result_topic,
            pick_timeout_sec=pick_timeout_sec,
        )

        # DISPLAY_PLACE 배치는 local AI 컴퓨터의 display_place_agent 에 요청한다.
        # 픽과 동일한 토픽 RPC(IbvsNnPickClient 재사용, 토픽만 display 용으로). agent 가
        # place_nn_servo(csrt + IBVS + 픽 nn_controller)를 띄우고, nn 의 grip close(=정렬 완료)
        # 관측 후 자기가 open(70) 발행으로 release 한 걸 완료로 본다.
        self._place = IbvsNnPickClient(
            node,
            request_topic=place_request_topic,
            result_topic=place_result_topic,
            pick_timeout_sec=place_timeout_sec,
        )

        # DISPLAY_SCAN 트리거(empty_slot_detector, AI 컴퓨터 상시 가동) 발행.
        self._place_reset_pub   = node.create_publisher(Empty, '/place/reset', 10)
        self._place_capture_pub = node.create_publisher(Empty, '/place/capture_view', 10)
        self._place_plan_pub    = node.create_publisher(Empty, '/place/plan', 10)
        # PLACE 시 CSRT init 용 bbox(center 기준 px) latched 발행.
        self._target_bbox_pub = node.create_publisher(
            Float64MultiArray, '/place/target_bbox', _latched_qos())
        # 우승 스캔 자세(6 관절 deg) latched 발행. display_place_agent 가 받아 place_nn_servo
        # 의 IBVS pregrasp 으로 주입한다(우승 자세=CSRT init 자세=IBVS pregrasp 일치 보장).
        self._pregrasp_pub = node.create_publisher(
            Float64MultiArray, '/place/pregrasp_angles', _latched_qos())

        # 스캔 plan 결과 수신.
        self._scan_lock = threading.Lock()
        self._scan_event = threading.Event()
        self._latest_scan_result: list[float] | None = None
        node.create_subscription(
            Float64MultiArray, '/place/scan_result', self._scan_result_callback, 10,
            callback_group=ReentrantCallbackGroup(),
        )
        # SCAN 우승 자리: {'pose': [J1..J6], 'bbox': [cx,cy,w,h,angle]}. SCAN -> PLACE 전달.
        self._scan_winner: dict | None = None

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
        if self._dry_run:
            self._log(f'(dry_run) SORTING 시뮬레이션 — product={product_name}')
            return True, 1
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

    def run_scanning(self, product_name: str = '', target_zone_name: str = '') -> tuple[bool, int]:
        """DISPLAY_PLACE 내부 스캔 단계: 진열대를 스윕하며 빈자리 후보를 누적하고 최적 1곳을 선정한다.

        empty_slot_detector(AI 컴퓨터 상시 가동)에 reset -> capture_view(자세별) -> plan 을
        트리거한다. plan 결과(/place/scan_result)가 found 면 우승 capture_index 로 복귀할
        스캔 자세와 bbox 를 self._scan_winner 에 저장한다. 스윕 전체에서 후보가 없으면
        재스캔을 SCAN_MAX_RESCANS 회까지 반복하고, 끝내 없으면 실패를 반환한다.
        반환값: (success, 0)
        """
        self._log(f'SCANNING 시작 — product={product_name}, zone={target_zone_name}')
        self._scan_winner = None

        if self._dry_run:
            self._scan_winner = {
                'pose': DISPLAY_SCAN_ANGLES[len(DISPLAY_SCAN_ANGLES) // 2],
                'bbox': [320.0, 240.0, 60.0, 60.0, 0.0],
            }
            self._log('(dry_run) 스캔 우승 자리 더미 저장')
            return True, 0

        for attempt in range(SCAN_MAX_RESCANS + 1):
            self._place_reset_pub.publish(Empty())
            time.sleep(0.2)

            for i, pose in enumerate(DISPLAY_SCAN_ANGLES):
                if not self.move_to_angles(pose):
                    self._log_err(f'스캔 자세 {i} 이동 실패')
                    return False, 0
                time.sleep(SCAN_SETTLE_SEC)
                self._place_capture_pub.publish(Empty())
                time.sleep(SCAN_CAPTURE_SEC)

            with self._scan_lock:
                self._latest_scan_result = None
            self._scan_event.clear()
            self._place_plan_pub.publish(Empty())
            if not self._scan_event.wait(timeout=SCAN_PLAN_TIMEOUT):
                self._log_err(
                    f'스캔 plan 응답 없음(timeout {SCAN_PLAN_TIMEOUT}s) — '
                    'empty_slot_detector 가 AI 컴퓨터에서 떠 있는지 확인. 재시도.')
                continue

            with self._scan_lock:
                res = list(self._latest_scan_result) if self._latest_scan_result else None
            if not res or res[0] < 0.5:
                self._log_err(
                    f'빈자리 후보 없음(시도 {attempt + 1}/{SCAN_MAX_RESCANS + 1}) — 재스캔')
                continue

            # res = [found, cx, cy, w, h, angle, capture_index, score]
            cx, cy, w, h, angle = res[1], res[2], res[3], res[4], res[5]
            cap_idx = int(round(res[6]))
            if not (0 <= cap_idx < len(DISPLAY_SCAN_ANGLES)):
                self._log_err(f'capture_index 범위 밖({cap_idx}) — center 자세로 대체')
                cap_idx = len(DISPLAY_SCAN_ANGLES) // 2
            self._scan_winner = {
                'pose': DISPLAY_SCAN_ANGLES[cap_idx],
                'bbox': [cx, cy, w, h, angle],
            }
            self._log(
                f'SCANNING 완료 — 우승 capture #{cap_idx}, bbox center=({cx:.0f},{cy:.0f}), '
                f'score={res[7]:.3f}')
            return True, 0

        self._log_err('SCANNING 실패 — 재스캔 한도 초과(빈자리 없음). 진열 보류.')
        return False, 0

    def run_placing(self, product_name: str = '', target_zone_name: str = '') -> tuple[bool, int]:
        """DISPLAY_PLACE: picky 에 실린 product 를 모두 진열한다.

        DISPLAY task 발행 시점엔 이미 SORTING_AND_LOAD 로 해당 product 가 picky 에 실려 있고,
        로봇은 그 product 진열대 앞에 있다는 전제다. picky 자체 slot DB(_slot_occupant) 에서
        해당 product 가 실린 모든 슬롯에 대해 [슬롯 재파지(LOAD_SLOT_ANGLES 매핑) -> 빈자리
        스캔 -> IBVS+NN 배치] 를 반복한다. 각 배치가 선반을 바꾸므로 unit 마다 다시 스캔한다.
        반환값: (success, 진열한 개수)
        """
        self._log(f'PLACING 시작 — product={product_name}, zone={target_zone_name}')
        if self._dry_run:
            n = sum(1 for o in self._slot_occupant if o == product_name)
            for i, o in enumerate(self._slot_occupant):
                if o == product_name:
                    self._slot_occupant[i] = None
            self._scan_winner = None
            self._log(f'(dry_run) {n}개 진열 시뮬레이션')
            return (n > 0), n

        if self._slot_for_product(product_name) is None:
            self._log_err(f'picky 에 실린 {product_name} 없음 — 진열할 물건 없음')
            return False, 0

        placed = 0
        while True:
            slot = self._slot_for_product(product_name)
            if slot is None:
                break  # 해당 product 모두 진열 완료

            self._log(f'진열 unit {placed + 1} — slot {slot} 재파지')
            if not self._regrasp_from_slot(slot):
                self._log_err(f'slot {slot} 재파지 실패 — 진열 중단')
                return False, placed

            # 재파지 직후 진열품 충돌 회피: 안전 경유 자세 -> center pregrasp 를 거쳐 스캔으로.
            if not self.move_to_angles(REGRASP_TRANSIT_ANGLES):
                self._log_err('재파지 후 안전 경유 자세 이동 실패 — 진열 중단')
                return False, placed
            if not self.move_to_angles(PREGRASP_ANGLES['center']):
                self._log_err('center pregrasp 이동 실패 — 진열 중단')
                return False, placed

            # 선반이 바뀌었으므로 unit 마다 재스캔(빈자리 재선정).
            ok, _ = self.run_scanning(product_name, target_zone_name)
            if not ok:
                self._log_err('빈자리 없음(재스캔 실패) — 물건을 쥔 채 진열 중단')
                return False, placed

            if not self._place_at_scanned(product_name):
                self._log_err('배치 실패 — 진열 중단')
                return False, placed

            self._slot_occupant[slot] = None  # 진열 성공 -> 슬롯 비움
            placed += 1
            self._log(f'진열 unit {placed} 완료 — slot {slot} 비움')

            # 배치(release) 후 항상 center pregrasp 로 상승 복귀(다음 unit/task 전 진열품 충돌 회피).
            if not self.move_to_angles(PREGRASP_ANGLES['center']):
                self._log_err('배치 후 center pregrasp 복귀 실패 — 진열 중단')
                return False, placed

        self._log(f'PLACING 완료 — 총 {placed}개 진열')
        return (placed > 0), placed

    def _regrasp_from_slot(self, slot: int) -> bool:
        """picky 슬롯에서 적재된 물건을 재파지한다.

        approach -> gripper 70% 선개방 -> pick(place 각도로 하강) -> close -> approach(들어올림).
        슬롯은 picky 에 고정이라 정차 오차와 무관하게 고정 2단계 접근을 쓴다.
        """
        approach = LOAD_SLOT_ANGLES[slot]['approach']
        place    = LOAD_SLOT_ANGLES[slot]['place']
        if not self.move_to_angles(approach):
            return False
        if not self._set_gripper(GRIPPER_LOAD_OPEN):  # pick 전 70% 선개방
            return False
        if not self.move_to_angles(place):
            return False
        if not self.close_gripper():                  # 집기
            return False
        if not self.move_to_angles(approach):         # 들어올림
            return False
        return True

    def _place_at_scanned(self, product_name: str) -> bool:
        """run_scanning 이 선정한 빈자리로 IBVS+NN 배치를 1회 수행한다.

        우승 스캔 자세로 복귀(스캔 시점과 동일 시야) -> bbox latched 발행(CSRT init) ->
        display_place_agent 요청(agent 가 place_nn_servo 기동, nn grip close 관측 후 open(70)
        release 발행을 완료로 보고).
        """
        if self._scan_winner is None:
            self._log_err('스캔 우승 자리 없음 — 배치 불가')
            return False
        pose = self._scan_winner.get('pose')
        bbox = self._scan_winner.get('bbox')
        if pose is not None and not self.move_to_angles(pose):
            self._log_err('스캔 우승 자세 복귀 실패')
            return False

        # 우승 자세를 IBVS pregrasp 으로 먼저 알린다(agent 가 place_nn_servo 기동 전에 latched
        # 수신하도록 bbox 보다 먼저 발행). 우승 자세=현재 복귀 자세=CSRT init 자세로 일치시켜
        # IBVS 가 servo 시작 시 카메라를 다른 곳으로 옮기지 않게 한다.
        if pose is not None:
            pose_msg = Float64MultiArray()
            pose_msg.data = [float(v) for v in pose]
            self._pregrasp_pub.publish(pose_msg)

        # 복귀 직후 팔 진동이 가라앉도록 안정 대기 후 bbox(CSRT init 타깃) 발행. 흔들리는
        # 프레임에서 CSRT 가 init 되면 잘못된 패치를 잡으므로 안정 프레임에서 넘긴다.
        time.sleep(PLACE_SETTLE_SEC)

        msg = Float64MultiArray()
        msg.data = [float(v) for v in bbox]
        self._target_bbox_pub.publish(msg)
        time.sleep(0.3)

        if not self._place.pick(product_name):
            self._log_err('display_place_agent 미응답 또는 배치 실패')
            return False
        self._scan_winner = None
        return True

    def _scan_result_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 8:
            return
        with self._scan_lock:
            self._latest_scan_result = list(msg.data)
        self._scan_event.set()

    def stow_arm(self) -> bool:
        """팔을 안전 복귀 자세(home)로 이동한다."""
        self._log('STOWING_ARM 시작')
        ok = self.move_to_angles(_HOME)
        self.open_gripper()
        return ok

    def go_to_center(self) -> bool:
        """다음 픽 준비를 위해 center pregrasp 자세로 복귀한다.

        SORTING_AND_LOAD 에서 아직 집을 물건이 남았을 때(STOWING 대신) 호출한다.
        """
        self._log('center pregrasp 복귀 — 다음 픽 준비')
        return self.move_to_angles(PREGRASP_ANGLES['center'])

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

    def seed_loadout(self, products: list[str]) -> int:
        """[디버그] 실제 픽 없이 적재 DB(_slot_occupant/_placements)를 가상으로 채운다.

        products 를 적재 순서대로 빈 슬롯 0번부터 채운다(용량 초과분은 버림). 기존 적재는
        먼저 비운다. SORTING_AND_LOAD 를 거치지 않고 INSPECTION/UNLOAD/DISPLAY_PLACE 를
        단독 테스트할 때 쓴다. 내부 상태만 바꿀 뿐 실제 로봇은 움직이지 않는다.
        반환값: 채운 슬롯 개수.
        """
        self._slot_occupant = [None] * len(LOAD_SLOT_ANGLES)
        self._placements.clear()
        seeded = 0
        for raw in products:
            product = raw.strip()
            if not product:
                continue
            slot = self._next_free_slot()
            if slot is None:
                self._log_err(
                    f'seed: 슬롯 용량({len(LOAD_SLOT_ANGLES)}) 초과 — 나머지 무시')
                break
            self._slot_occupant[slot] = product
            self._placements.append({'slot': slot, 'product_name': product, 'order_id': 0})
            seeded += 1
        self._log(f'[디버그] 가상 적재 — {seeded}개: {self.current_loadout}')
        return seeded

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
        # 2) PICKUP_SLOT approach -> place 2단계 접근 후 그리퍼 열어 드롭, INSPECTION_POSE 복귀.
        if not self.move_to_angles(PICKUP_SLOT_APPROACH):
            return False
        if not self.move_to_angles(PICKUP_SLOT_PLACE):
            return False
        if not self.open_gripper():
            return False
        if not self.move_to_angles(INSPECTION_POSE):
            return False
        return True

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
        time.sleep(0.3)
        stable = 0
        while time.time() < deadline:
            if not self._e_stop_event.is_set():
                self._log('비상정지 감지 — 이동 일시정지')
                self._e_stop_event.wait()
                self._log('비상정지 해제 — 이동 재개')
                self._publish_joint(target, self._default_speed)
                deadline = time.time() + timeout
                stable = 0
                time.sleep(0.3)
                continue
            self._req_status_pub.publish(Empty())
            time.sleep(STATUS_POLL_INTERVAL_SEC)
            with self._status_lock:
                angles = list(self._latest_angles) if self._latest_angles is not None else None
            if angles is not None and self._converged(angles, target):
                stable += 1
                if stable >= 2:
                    return True
            else:
                stable = 0
        self._log_err(f'관절 이동 타임아웃({timeout}s) — target={target}')
        return False

    def _wait_until_coords(self, target: list[float], timeout: float | None = None) -> bool:
        timeout = self._motion_timeout if timeout is None else timeout
        deadline = time.time() + timeout
        time.sleep(0.3)
        while time.time() < deadline:
            if not self._e_stop_event.is_set():
                self._log('비상정지 감지 — 이동 일시정지')
                self._e_stop_event.wait()
                self._log('비상정지 해제 — 이동 재개')
                deadline = time.time() + timeout
                time.sleep(0.3)
                continue
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
        self._e_stop_event.clear()
        if self._dry_run:
            self._log('(dry_run) emergency_stop')
            return
        with self._status_lock:
            angles = list(self._latest_angles) if self._latest_angles is not None else None
        if angles is None:
            return
        msg = Float64MultiArray()
        msg.data = [float(CMD_JOINT)] + [float(a) for a in angles] + [float(self._default_speed)]
        self._target_pub.publish(msg)
        self._log('비상정지 — 현재 자세 hold')

    def emergency_resume(self) -> None:
        if self._dry_run:
            self._log('(dry_run) emergency_resume')
        else:
            self._log('비상정지 해제 — 동작 재개 준비')
        self._e_stop_event.set()

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
