#!/usr/bin/env python3
import time

from pymycobot.mycobot280 import MyCobot280


GRIPPER_OPEN   = 100
GRIPPER_CLOSED = 0
GRIPPER_SPEED  = 100

DEFAULT_SPEED        = 20   # 관절 이동 속도 (1~100)
MOTION_TIMEOUT_SEC   = 15.0
POLL_INTERVAL_SEC    = 0.05


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
        # [구현 필요] 분류 동작 수행
        # 예시 흐름:
        #   1. 분류 위치로 이동
        #   2. 그리퍼로 물체 집기
        #   3. 대상 위치로 이동 후 내려놓기
        #   4. 처리 수량 반환
        return True, 0

    def run_loading(self, request) -> tuple[bool, int]:
        # [구현 필요] 적재 동작 수행
        return True, 0

    def run_inspecting(self, request) -> tuple[bool, int]:
        # [구현 필요] 검수 동작 수행
        return True, 0

    def run_unloading(self, request) -> tuple[bool, int]:
        # [구현 필요] 하역 동작 수행
        return True, 0

    def run_placing(self, scan_result, request) -> tuple[bool, int]:
        # [구현 필요] scan_result 좌표로 진열 동작 수행
        # scan_result 는 vision_client 가 반환한 빈 슬롯 좌표
        if scan_result is None:
            self._log_err('PLACING 실패 — scan_result 없음')
            return False, 0
        return True, 0

    def stow_arm(self) -> bool:
        """팔을 안전 복귀 자세(home)로 이동한다."""
        # [구현 필요] 실제 home 각도로 교체
        home_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        return self.move_to_angles(home_angles, speed=DEFAULT_SPEED)

    # ── 저수준 이동 메서드 ───────────────────────────────────────────────

    def move_to_angles(self, angles: list[float], speed: int = DEFAULT_SPEED) -> bool:
        """관절 각도(degree)로 이동하고 완료될 때까지 블로킹."""
        try:
            self._mc.send_angles(angles, speed, _async=True)
        except Exception as e:
            self._log_err(f'send_angles 실패: {e}')
            return False
        return self._wait_for_stop()

    def move_to_coords(
        self,
        coords: list[float],
        speed: int = DEFAULT_SPEED,
        mode: int = 1,
    ) -> bool:
        """Cartesian 좌표 [x, y, z, rx, ry, rz](mm/deg)로 이동하고 블로킹."""
        try:
            self._mc.send_coords(coords, speed, mode=mode, _async=True)
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
        for attempt in range(5):
            try:
                self._mc.set_gripper_value(value, GRIPPER_SPEED)
                result = self._mc.get_gripper_value()
                if result != -1:
                    return True
            except Exception as e:
                self._log_err(f'그리퍼 제어 실패 (시도 {attempt + 1}/5): {e}')
        self._log_err('그리퍼 제어 5회 재시도 후 실패')
        return False

    # ── 비상 정지 ────────────────────────────────────────────────────────

    def emergency_stop(self) -> None:
        try:
            self._mc.stop()
        except Exception as e:
            self._log_err(f'emergency_stop 실패: {e}')

    # ── 내부 유틸 ────────────────────────────────────────────────────────

    def _wait_for_stop(self, timeout: float = MOTION_TIMEOUT_SEC) -> bool:
        """로봇이 정지할 때까지 블로킹. 타임아웃 시 False 반환."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            try:
                moving = self._mc.is_moving()
            except Exception as e:
                self._log_err(f'is_moving 실패: {e}')
                return False

            if moving == -1:
                self._log_err('로봇 이동 상태 오류 (is_moving == -1)')
                return False
            if moving == 0:
                return True

            time.sleep(POLL_INTERVAL_SEC)

        self._log_err(f'동작 타임아웃 ({timeout}s)')
        return False

    def _log(self, msg: str) -> None:
        self._node.get_logger().info(f'[CobotController] {msg}')

    def _log_err(self, msg: str) -> None:
        self._node.get_logger().error(f'[CobotController] {msg}')
