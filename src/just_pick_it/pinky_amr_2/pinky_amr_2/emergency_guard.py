"""Emergency state holder for the PICKY2 communication skeleton."""


class Amr2EmergencyGuard:
    """Keep emergency stop state shared by services and action callbacks."""

    def __init__(self) -> None:
        self._stopped = False
        self._reason = ""

    @property
    def reason(self) -> str:
        return self._reason

    def stop(self, reason: str) -> None:
        self._stopped = True
        self._reason = reason or "UNKNOWN"

    def resume(self) -> None:
        self._stopped = False
        self._reason = ""

    def is_stopped(self) -> bool:
        return self._stopped

    def should_reject_goal(self) -> bool:
        return self._stopped
