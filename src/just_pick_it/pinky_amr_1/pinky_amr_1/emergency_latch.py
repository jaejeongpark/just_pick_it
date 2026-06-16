#!/usr/bin/env python3
"""PICKY1 비상 정지(emergency stop) 공유 래치(latch).

StateManager(서비스 콜백)와 MoveToGoal / ReverseDocking(주행 루프)이 같은
인스턴스를 공유한다. 서비스 콜백 스레드와 주행 루프 스레드가 동시에 접근하므로
threading 으로 보호한다.

E-stop 은 한 번 걸리면 명시적 해제(resume) 전까지 유지되는 래치 동작이라
이름을 latch 로 둔다. 정책은 'pause-continue' 다: 비상 정지는 동작을 중단(abort)
하지 않고 제자리에 멈춰두며(action goal 은 살아있음), 재개 시 같은 동작을 이어서
계속한다. 이는 Fleet Manager 의 PAUSED -> RUNNING 재개 정책과 일치한다.
"""

import threading


class EmergencyLatch:
    """비상 정지 플래그와 사유를 스레드 안전하게 공유하는 래치."""

    def __init__(self) -> None:
        # set 이면 비상 정지(멈춤) 상태.
        self._stopped = threading.Event()
        self._lock = threading.Lock()
        self._reason = ""

    @property
    def reason(self) -> str:
        with self._lock:
            return self._reason

    def stop(self, reason: str) -> None:
        """비상 정지 진입(래치 걸기). 사유를 기록하고 플래그를 set 한다."""
        with self._lock:
            self._reason = reason or "UNKNOWN"
        self._stopped.set()

    def resume(self) -> None:
        """재개(래치 풀기). 플래그를 clear 하고 사유를 비운다."""
        self._stopped.clear()
        with self._lock:
            self._reason = ""

    def is_stopped(self) -> bool:
        return self._stopped.is_set()

    def should_reject_goal(self) -> bool:
        """비상 정지 중이면 새 goal 을 받지 않는다."""
        return self._stopped.is_set()
