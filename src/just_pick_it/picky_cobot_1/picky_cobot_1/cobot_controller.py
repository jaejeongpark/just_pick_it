#!/usr/bin/env python3
import time

from pymycobot.mycobot280 import MyCobot280


GRIPPER_OPEN   = 100
GRIPPER_CLOSED = 0
GRIPPER_SPEED  = 100
GRIPPER_WAIT_SEC = 2.0  # 그리퍼 동작 완료 대기 시간

DEFAULT_SPEED        = 20   # 관절 이동 속도 (1~100)
MOTION_TIMEOUT_SEC   = 15.0
POLL_INTERVAL_SEC    = 0.05

# ── 임시 테스트용 고정 자세 (vision 서버 연동 후 실제 좌표로 교체 필요) ─────────
# 단위: degree,  순서: [J1, J2, J3, J4, J5, J6]
_HOME          = [  0.0,   0.0,   0.0,   0.0,   0.0,   0.0]

# SORTING: joint angles [J1, J2, J3, J4, J5, J6] (deg) — 서버 연동 후 실제값으로 교체
_SORT_PICK  = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
_SORT_PLACE = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

_LOAD_PICK     = [  0.0, -20.0,  90.0, -70.0,   0.0,   0.0]
_LOAD_PLACE    = [-90.0, -20.0,  90.0, -70.0,   0.0,   0.0]

_INSPECT       = [  0.0,  30.0,  60.0, -90.0,   0.0,   0.0]

_UNLOAD_PICK   = [-90.0, -20.0,  90.0, -70.0,   0.0,   0.0]
_UNLOAD_PLACE  = [  0.0, -20.0,  90.0, -70.0,   0.0,   0.0]

_PLACE_TEMP    = [ 45.0, -20.0,  90.0, -70.0,   0.0,   0.0]


class CobotController:
    """
    MyCobot280 하드웨어 직접 제어 래퍼.

    cobot_state_machine.py 에서 생성하며, 상태 전환 없이 순수 동작만 담당한다.
    각 phase 메서드는 (success: bool, detected_quantity: int) 를 반환한다.
    """

    def __init__(self, node, port: str = '/dev/ttyJETCOBOT', baudrate: int = 1_000_000) -> None:
        self._node = node
        self._mc   = MyCobot280(port, baudrate, thread_lock=True)
        time.sleep(0.4)
        self._mc.set_fresh_mode(1)
        time.sleep(0.4)
        self._log(f'CobotController 초기화 완료 — port={port}')

    # ── 공개 phase 메서드 ────────────────────────────────────────────────

    def run_sorting(self, request) -> tuple[bool, int]:
        self._log('SORTING 시작')
        if not self.move_to_angles(_SORT_PICK):
            return False, 0
        if not self.close_gripper():
            return False, 0
        if not self.move_to_angles(_SORT_PLACE):
            return False, 0
        if not self.open_gripper():
            return False, 0
        return True, 1

    def run_loading(self, request) -> tuple[bool, int]:
        self._log('LOADING 시작')
        self.open_gripper()
        ok = self._pick_and_place(_LOAD_PICK, _LOAD_PLACE)
        return ok, (1 if ok else 0)

    def run_inspecting(self, request) -> tuple[bool, int]:
        self._log('INSPECTING 시작')
        ok = self.move_to_angles(_INSPECT)
        if not ok:
            return False, 0
        time.sleep(1.0)
        return True, 0

    def run_unloading(self, request) -> tuple[bool, int]:
        self._log('UNLOADING 시작')
        self.open_gripper()
        ok = self._pick_and_place(_UNLOAD_PICK, _UNLOAD_PLACE)
        return ok, (1 if ok else 0)

    def run_placing(self, scan_result, request) -> tuple[bool, int]:
        """scan_result 가 None 이면 임시 고정 좌표(_PLACE_TEMP)로 진열한다."""
        if scan_result is None:
            self._log('PLACING: scan_result 없음 — 임시 고정 좌표 사용')
            place_angles = _PLACE_TEMP
        else:
            # [구현 필요] scan_result 에서 실제 각도/좌표 추출
            place_angles = _PLACE_TEMP

        self._log('PLACING 시작')
        self.open_gripper()
        ok = self._pick_and_place(_LOAD_PICK, place_angles)
        return ok, (1 if ok else 0)

    def stow_arm(self) -> bool:
        """팔을 안전 복귀 자세(home)로 이동한다."""
        ok = self.move_to_angles(_HOME, speed=DEFAULT_SPEED)
        self.open_gripper()
        return ok

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
        try:
            self._mc.stop()
        except Exception as e:
            self._log_err(f'emergency_stop 실패: {e}')

    # ── 내부 유틸 ────────────────────────────────────────────────────────

    def _pick_and_place(
        self,
        pick_angles: list[float],
        place_angles: list[float],
        speed: int = DEFAULT_SPEED,
    ) -> bool:
        """pick 위치로 이동 후 파지하고, place 위치로 이동 후 내려놓는다."""
        if not self.move_to_angles(pick_angles, speed):
            self._log_err('pick 이동 실패')
            return False
        if not self.close_gripper():
            self._log_err('그리퍼 닫기 실패')
            return False
        if not self.move_to_angles(place_angles, speed):
            self._log_err('place 이동 실패')
            return False
        if not self.open_gripper():
            self._log_err('그리퍼 열기 실패')
            return False
        return True

    def _wait_for_stop(self, timeout: float = MOTION_TIMEOUT_SEC) -> bool:
        # send_coords는 non-blocking이라 이동 시작 전 is_moving()==0 타이밍이 존재.
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
