#!/usr/bin/env python3
import time

from pymycobot.mycobot280 import MyCobot280

from .ibvs_nn_pick import IbvsNnPickClient


GRIPPER_OPEN   = 100
GRIPPER_CLOSED = 0
GRIPPER_SPEED  = 100
GRIPPER_WAIT_SEC = 2.0  # 그리퍼 동작 완료 대기 시간

DEFAULT_SPEED        = 20   # 관절 이동 속도 (1~100)
STREAM_SPEED         = 50   # 센터링 실시간 추종 속도
MOTION_TIMEOUT_SEC   = 15.0
POLL_INTERVAL_SEC    = 0.05

# 단위: degree,  순서: [J1, J2, J3, J4, J5, J6]
_HOME = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# 상품별 place 좌표(고정 매핑). 단위: mm/deg, 순서: [x, y, z, rx, ry, rz].
# [구현 필요] 실제 picky 적재 위치에 맞춰 좌표 보정. 현재는 shell 더미값.
PLACE_COORDS = {
    'fanta':       [180.0, -60.0, 200.0, -90.0, 0.0, 0.0],
    'water':       [180.0, -20.0, 200.0, -90.0, 0.0, 0.0],
    'watermelon':  [180.0,  20.0, 200.0, -90.0, 0.0, 0.0],
    'bread':       [180.0,  60.0, 200.0, -90.0, 0.0, 0.0],
    'cream_bread': [220.0, -40.0, 200.0, -90.0, 0.0, 0.0],
    'choco_pie':   [220.0,  40.0, 200.0, -90.0, 0.0, 0.0],
}


class CobotController:
    """
    MyCobot280 하드웨어 직접 제어 래퍼.

    cobot_state_machine.py 에서 생성하며, 상태 전환 없이 순수 동작만 담당한다.
    각 phase 메서드는 (success: bool, detected_quantity: int) 를 반환한다.
    """

    def __init__(
        self,
        node,
        port: str = '/dev/ttyJETCOBOT',
        baudrate: int = 1_000_000,
        dry_run: bool = False,
        pick_timeout_sec: float = 120.0,
        pick_request_topic: str = '/ibvs_nn_pick/request',
        pick_result_topic: str = '/ibvs_nn_pick/result',
    ) -> None:
        self._node    = node
        self._dry_run = dry_run

        # SORTING 픽은 local AI 컴퓨터의 ibvs_nn_pick_agent 에 토픽으로 요청한다.
        # agent 가 IBVS+NN 을 띄우면 cobot 호스트의 드라이버가 제어 토픽을 snatch 해 구동한다.
        # serial 직접 제어(dry_run 여부)와 무관하게 동작한다.
        self._pick = IbvsNnPickClient(
            node,
            request_topic=pick_request_topic,
            result_topic=pick_result_topic,
            pick_timeout_sec=pick_timeout_sec,
        )
        # 어떤 상품을 어디(좌표/zone)에 놓았는지 기록. 차후 picky 에서 재집기용.
        self._placements: list[dict] = []

        if dry_run:
            self._mc = None
            self._log('CobotController dry_run 모드 — serial 직접 제어 생략(픽은 IBVS+NN)')
            return
        self._mc = MyCobot280(port, baudrate, thread_lock=True)
        time.sleep(0.4)
        self._mc.set_fresh_mode(1)
        time.sleep(0.4)
        self._log(f'CobotController 초기화 완료 — port={port}')

    # ── 공개 phase 메서드 ────────────────────────────────────────────────

    def run_sorting(self, product_name: str, quantity: int = 1) -> tuple[bool, int]:
        """IBVS+NN 픽으로 지정 상품을 quantity 개 집어 올린다.

        한 정차 위치에는 한 종류 상품만 진열되어 있으므로 product_name 한 종류를
        quantity 번 반복해 집는다. 각 픽은 nn_inference.launch.py 1회 실행에 대응한다.
        반환값: (success, 집어 올린 개수)
        """
        self._log(f'SORTING 시작 — product={product_name}, quantity={quantity}')
        target_qty = max(1, int(quantity or 1))
        picked = 0
        for i in range(target_qty):
            self._log(f'SORTING {i + 1}/{target_qty} 픽 시도')
            if not self._pick.pick(product_name):
                self._log_err(f'{i + 1}번째 픽 실패')
                return False, picked
            picked += 1
        return True, picked

    def run_loading(
        self,
        product_name: str,
        target_zone_name: str = '',
    ) -> tuple[bool, int]:
        """집어 올린 상품을 picky 적재 위치(place)로 내려놓는다(shell)."""
        return self.place_product(product_name, target_zone_name)

    # ── place(내려놓기) shell ─────────────────────────────────────────────

    def place_product(self, product_name: str, target_zone_name: str = '') -> tuple[bool, int]:
        """상품을 고정 place 좌표로 내려놓고, 무엇을 어디 놓았는지 기록한다.

        실제 place 동작은 PLACE_COORDS 의 고정 좌표로 send_coords 하는 shell 이다.
        '어떤 상품을 어디에 놓았는지' 는 차후 picky 에서 재집기/재진열할 때 쓰도록
        self._placements 에 저장한다.
        """
        self._log(f'PLACE 시작 — product={product_name}, zone={target_zone_name}')
        coords = PLACE_COORDS.get(product_name)
        if coords is None:
            self._log_err(f'place 좌표 미정의 상품: {product_name}')
            return False, 0

        # 어디에 놓았는지 먼저 기록(좌표 매핑은 cobot 내부 보관).
        self._placements.append({
            'product_name': product_name,
            'coords': list(coords),
            'zone': target_zone_name,
        })

        if self._dry_run:
            self._log(f'(dry_run) place 좌표 기록만 — {product_name} -> {coords}')
            return True, 1

        # [구현 필요] 집은 물체를 들고 이 좌표로 이동 후 내려놓는 실제 동작 보강.
        if not self.move_to_coords(coords):
            return False, 0
        if not self.open_gripper():
            return False, 0
        return True, 1

    @property
    def placements(self) -> list[dict]:
        """차후 재집기를 위한 place 기록(상품명/좌표/zone)."""
        return list(self._placements)

    def run_inspecting(self, inspect_trajectory: list[list[float]]) -> tuple[bool, int]:
        self._log('INSPECTING 시작')
        if self._dry_run:
            return True, 0  # [구현 필요] inspect_trajectory 기반 실제 검사 동작으로 교체
        ok = self.execute_grasp_trajectory(inspect_trajectory)
        if not ok:
            return False, 0
        time.sleep(1.0)
        return True, 0

    def run_unloading(
        self,
        pick_trajectory: list[list[float]],
        place_trajectory: list[list[float]],
    ) -> tuple[bool, int]:
        self._log('UNLOADING 시작')
        if self._dry_run:
            return True, 1  # [구현 필요] pick_trajectory/place_trajectory 기반 실제 하역 동작으로 교체
        self.open_gripper()
        ok = self._execute_pick_and_place(pick_trajectory, place_trajectory)
        return ok, (1 if ok else 0)

    def run_placing(
        self,
        pick_trajectory: list[list[float]],
        place_trajectory: list[list[float]],
    ) -> tuple[bool, int]:
        self._log('PLACING 시작')
        if self._dry_run:
            return True, 1  # [구현 필요] pick_trajectory/place_trajectory 기반 실제 진열 동작으로 교체
        self.open_gripper()
        ok = self._execute_pick_and_place(pick_trajectory, place_trajectory)
        return ok, (1 if ok else 0)

    def stow_arm(self) -> bool:
        """팔을 안전 복귀 자세(home)로 이동한다."""
        self._log('STOWING_ARM 시작')
        if self._dry_run:
            return True  # [구현 필요] _HOME 자세로 실제 복귀 동작으로 교체
        ok = self.move_to_angles(_HOME, speed=DEFAULT_SPEED)
        self.open_gripper()
        return ok

    # ── 서버 스트리밍 제어 메서드 ────────────────────────────────────────

    def stream_joint_angles(self, angles: list[float]) -> bool:
        """서버 스트리밍 관절각을 즉시 반영 (non-blocking). 센터링 추종 루프에서 사용."""
        try:
            self._mc.send_angles(angles, STREAM_SPEED)
            return True
        except Exception as e:
            self._log_err(f'스트리밍 관절 제어 실패: {e}')
            return False

    def execute_grasp_trajectory(self, trajectory: list[list[float]]) -> bool:
        """학습 알고리즘 기반 파지 궤적을 순차 실행 (blocking). 각 waypoint는 6축 관절각."""
        self._log(f'파지 궤적 실행 — {len(trajectory)}개 waypoint')
        for i, angles in enumerate(trajectory):
            if not self.move_to_angles(angles):
                self._log_err(f'파지 궤적 {i}번 waypoint 실패')
                return False
        return True

    # ── 저수준 이동 메서드 ───────────────────────────────────────────────

    def move_to_angles(self, angles: list[float], speed: int = DEFAULT_SPEED) -> bool:
        """관절 각도(degree)로 이동하고 완료될 때까지 블로킹."""
        try:
            self._mc.send_angles(angles, speed)
        except Exception as e:
            self._log_err(f'send_angles 실패: {e}')
            return False
        return self._wait_for_stop()

    def move_to_coords(
        self,
        coords: list[float],
        speed: int = DEFAULT_SPEED,
        mode: int = 0,
    ) -> bool:
        """Cartesian 좌표 [x, y, z, rx, ry, rz](mm/deg)로 이동하고 블로킹."""
        try:
            self._mc.send_coords(coords, speed, mode=mode)
        except Exception as e:
            self._log_err(f'send_coords 실패: {e}')
            return False
        return self._wait_for_stop()

    # ── 그리퍼 메서드 ────────────────────────────────────────────────────

    def open_gripper(self) -> bool:
        return self._set_gripper(GRIPPER_OPEN)

    def close_gripper(self) -> bool:
        return self._set_gripper(GRIPPER_CLOSED)

    def _set_gripper(self, value: int) -> bool:
        try:
            self._mc.set_gripper_value(value, GRIPPER_SPEED)
            time.sleep(GRIPPER_WAIT_SEC)
            return True
        except Exception as e:
            self._log_err(f'그리퍼 제어 실패: {e}')
            return False

    # ── 비상 정지 ────────────────────────────────────────────────────────

    def emergency_stop(self) -> None:
        if self._dry_run:
            return
        try:
            self._mc.stop()
        except Exception as e:
            self._log_err(f'emergency_stop 실패: {e}')

    # ── 내부 유틸 ────────────────────────────────────────────────────────

    def _execute_pick_and_place(
        self,
        pick_trajectory: list[list[float]],
        place_trajectory: list[list[float]],
    ) -> bool:
        """pick 궤적 실행 후 파지하고, place 궤적 실행 후 내려놓는다."""
        if not self.execute_grasp_trajectory(pick_trajectory):
            self._log_err('pick 궤적 실행 실패')
            return False
        if not self.close_gripper():
            self._log_err('그리퍼 닫기 실패')
            return False
        if not self.execute_grasp_trajectory(place_trajectory):
            self._log_err('place 궤적 실행 실패')
            return False
        if not self.open_gripper():
            self._log_err('그리퍼 열기 실패')
            return False
        return True

    def _wait_for_stop(self, timeout: float = MOTION_TIMEOUT_SEC) -> bool:
        # send_angles/send_coords 모두 non-blocking이라 이동 시작 전 is_moving()==0 타이밍이 존재.
        # 충분히 대기 후 이동 시작을 확인한 뒤 정지를 기다린다.
        time.sleep(0.5)

        # 이동 시작 확인 (최대 2초)
        start_wait = time.time()
        while time.time() - start_wait < 2.0:
            if self._mc.is_moving() == 1:
                break
            time.sleep(0.05)

        # 정지 확인
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self._mc.is_moving() == 0:
                return True
            time.sleep(0.1)

        self._log_err(f'동작 타임아웃 ({timeout}s)')
        return False

    def _log(self, msg: str) -> None:
        self._node.get_logger().info(f'[CobotController] {msg}')

    def _log_err(self, msg: str) -> None:
        self._node.get_logger().error(f'[CobotController] {msg}')
