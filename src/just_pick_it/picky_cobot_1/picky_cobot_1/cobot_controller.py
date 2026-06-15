#!/usr/bin/env python3
import time

from pymycobot.mycobot280 import MyCobot280


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


class CobotController:
    """
    MyCobot280 하드웨어 직접 제어 래퍼.

    cobot_state_machine.py 에서 생성하며, 상태 전환 없이 순수 동작만 담당한다.
    각 phase 메서드는 (success: bool, detected_quantity: int) 를 반환한다.
    """

    def __init__(self, node, port: str = '/dev/ttyJETCOBOT', baudrate: int = 1_000_000, dry_run: bool = False) -> None:
        self._node    = node
        self._dry_run = dry_run
        if dry_run:
            self._mc = None
            self._log('CobotController dry_run 모드 — 하드웨어 연결 생략')
            return
        self._mc = MyCobot280(port, baudrate, thread_lock=True)
        time.sleep(0.4)
        self._mc.set_fresh_mode(1)
        time.sleep(0.4)
        self._log(f'CobotController 초기화 완료 — port={port}')

    # ── 공개 phase 메서드 ────────────────────────────────────────────────

    def run_sorting(self, grasp_trajectory: list[list[float]]) -> tuple[bool, int]:
        """서버에서 받은 학습 파지 궤적으로 물체를 집는다."""
        self._log('SORTING 시작')
        if self._dry_run:
            return True, 1  # [구현 필요] grasp_trajectory 기반 실제 파지 동작으로 교체
        if not self.execute_grasp_trajectory(grasp_trajectory):
            return False, 0
        if not self.close_gripper():
            return False, 0
        return True, 1

    def run_loading(
        self,
        pick_trajectory: list[list[float]],
        place_trajectory: list[list[float]],
    ) -> tuple[bool, int]:
        self._log('LOADING 시작')
        if self._dry_run:
            return True, 1  # [구현 필요] pick_trajectory/place_trajectory 기반 실제 적재 동작으로 교체
        self.open_gripper()
        ok = self._execute_pick_and_place(pick_trajectory, place_trajectory)
        return ok, (1 if ok else 0)

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
