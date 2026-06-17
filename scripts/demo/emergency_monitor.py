#!/usr/bin/env python3
"""Emergency latch 테스트용 라이브 모니터.

CLI(ros2 topic echo)는 Discovery Server 너머에서 토픽 타입 조회가 자주 실패하므로,
타입을 명시 구독하는 실제 노드로 확실하게 받는다.

- /picky1/cmd_vel (Twist): 정지(0) 전환·재개를 표시한다.
- /rosout (Log): emergency / resume / StateManager 관련 로그만 골라 찍는다.

stdout 과 /tmp/emergency_monitor.log 에 동시에 남긴다. 끝낼 땐 Ctrl+C.
"""

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Twist
from rcl_interfaces.msg import Log

NS = sys.argv[1] if len(sys.argv) > 1 else "picky1"
LOGPATH = "/tmp/emergency_monitor.log"
KEYS = ("emergency", "resume", "estop", "e-stop", "statemanager", "paused", "latch")


def ts() -> str:
    return time.strftime("%H:%M:%S") + f".{int((time.time() % 1) * 1000):03d}"


class Mon(Node):
    def __init__(self) -> None:
        super().__init__("emergency_monitor")
        self._f = open(LOGPATH, "a", buffering=1)
        self._last_zero = None

        self.create_subscription(Twist, f"/{NS}/cmd_vel", self._on_cmd, 10)

        # /rosout 은 TRANSIENT_LOCAL+RELIABLE 로 발행된다. 맞춰야 받는다.
        rosout_qos = QoSProfile(
            depth=50,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Log, "/rosout", self._on_log, rosout_qos)
        self._emit(f"=== monitor 시작 (ns={NS}) — cmd_vel + /rosout 감시 ===")

    def _emit(self, line: str) -> None:
        out = f"[{ts()}] {line}"
        print(out, flush=True)
        self._f.write(out + "\n")

    def _on_cmd(self, m: Twist) -> None:
        lx, az = m.linear.x, m.angular.z
        is_zero = abs(lx) < 1e-4 and abs(az) < 1e-4
        # 상태 전환(0<->비0)일 때만 찍어 노이즈를 줄인다.
        if is_zero != self._last_zero:
            tag = ">>> cmd_vel 0 (정지)" if is_zero else ">>> cmd_vel 움직임"
            self._emit(f"{tag}  lin.x={lx:+.3f} ang.z={az:+.3f}")
            self._last_zero = is_zero

    def _on_log(self, m: Log) -> None:
        text = m.msg.lower()
        name = m.name.lower()
        if any(k in text or k in name for k in KEYS):
            lvl = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}.get(m.level, str(m.level))
            self._emit(f"[rosout {lvl}] ({m.name}) {m.msg}")


def main() -> None:
    rclpy.init()
    node = Mon()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
