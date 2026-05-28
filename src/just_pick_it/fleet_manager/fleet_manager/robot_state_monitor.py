from __future__ import annotations

import math
import threading
from collections.abc import Callable
from typing import Any

from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import Float32, String


StateCallback = Callable[[str, str], None]
BatteryCallback = Callable[[str, int], None]


def _quat_to_yaw(q: Any) -> float:
    """quaternion 을 2D yaw(theta) 로 변환한다."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class RobotStateMonitor:
    """PICKY 텔레메트리(picky_state / battery / pose)를 구독해 DB와 TrafficManager에 반영한다.

    System Architecture 기준 Fleet Manager <-> AMR Controller 통신은 ROS2이므로,
    로봇 상태는 HTTP가 아니라 ROS2 토픽으로만 들어온다.

    구독 토픽(각 robot namespace 기준):
    - `/pickyX/picky_state` (std_msgs/String): 상태 머신 출력. State Manager가 발행.
    - `/pickyX/battery/percent` (std_msgs/Float32): 이미 % 값. pinky_bringup이 발행.
    - `/pickyX/amcl_pose` (geometry_msgs/PoseWithCovarianceStamped): map frame 위치.

    반영 정책:
    - picky_state는 경로/도크 자동 해제가 지연되면 안 되므로 수신 즉시 `on_state_change`
      (TrafficManager.notify_state)로 전달한다.
    - battery/pose는 고빈도라 최신값만 캐시하고, db_flush_period_sec 주기로 변경분만
      한 번에 DB에 반영한다(coalesce). picky_state도 같은 주기에 함께 반영한다.
    - **robot_status는 절대 기록하지 않는다.** robot_status는 task 전이(workflow_service)
      전용이며, 텔레메트리는 picky_state / battery_level / pos_* 만 갱신한다.
    - battery_level이 바뀌면 `on_battery_update`(TaskManager.handle_battery_update)를 호출해
      충전 완료를 즉시 반영할 수 있게 한다.
    """

    def __init__(
        self,
        node: Node,
        robot_ids: list[str],
        fleet_repo: Any,
        on_state_change: StateCallback,
        on_battery_update: BatteryCallback | None = None,
        db_flush_period_sec: float = 1.0,
    ) -> None:
        self._node = node
        self._repo = fleet_repo
        self._on_state_change = on_state_change
        self._on_battery_update = on_battery_update
        self._lock = threading.Lock()

        # robot_id -> 최신 수신 캐시 {picky_state, battery_level, pos_x, pos_y, pos_theta}
        self._latest: dict[str, dict[str, Any]] = {rid: {} for rid in robot_ids}
        # robot_id -> 마지막으로 DB에 기록한 값. 변경분만 쓰기 위함.
        self._last_written: dict[str, dict[str, Any]] = {rid: {} for rid in robot_ids}

        for robot_id in robot_ids:
            ns = robot_id.lower()
            node.create_subscription(
                String,
                f'/{ns}/picky_state',
                lambda msg, rid=robot_id: self._on_picky_state(rid, msg),
                10,
            )
            node.create_subscription(
                Float32,
                f'/{ns}/battery/percent',
                lambda msg, rid=robot_id: self._on_battery(rid, msg),
                10,
            )
            node.create_subscription(
                PoseWithCovarianceStamped,
                f'/{ns}/amcl_pose',
                lambda msg, rid=robot_id: self._on_pose(rid, msg),
                10,
            )

        self._flush_timer = node.create_timer(db_flush_period_sec, self._flush_to_db)

        node.get_logger().info(
            f'[RobotStateMonitor] 텔레메트리 구독 시작 — {robot_ids} '
            f'(picky_state/battery/amcl_pose), DB flush {db_flush_period_sec:.1f}s'
        )

    # ==================================================================
    # 토픽 콜백 (executor thread)
    # ==================================================================

    def _on_picky_state(self, robot_id: str, msg: String) -> None:
        state = msg.data
        with self._lock:
            self._latest[robot_id]['picky_state'] = state
        # 경로/도크 자동 해제는 지연되면 안 되므로 즉시 전달한다.
        self._on_state_change(robot_id, state)

    def _on_battery(self, robot_id: str, msg: Float32) -> None:
        level = max(0, min(100, int(round(float(msg.data)))))
        with self._lock:
            self._latest[robot_id]['battery_level'] = level

    def _on_pose(self, robot_id: str, msg: PoseWithCovarianceStamped) -> None:
        pose = msg.pose.pose
        with self._lock:
            cache = self._latest[robot_id]
            cache['pos_x'] = float(pose.position.x)
            cache['pos_y'] = float(pose.position.y)
            cache['pos_theta'] = _quat_to_yaw(pose.orientation)

    # ==================================================================
    # 주기 DB 반영 (timer thread)
    # ==================================================================

    def _flush_to_db(self) -> None:
        """캐시된 최신값 중 변경분만 robot 별로 DB에 한 번에 반영한다."""
        for robot_id in list(self._latest):
            with self._lock:
                latest = dict(self._latest[robot_id])
                written = self._last_written[robot_id]
                changed = {key: value for key, value in latest.items() if written.get(key) != value}

            if not changed:
                continue

            updated = self._repo.update_robot_state(robot_id, **changed)
            if updated is None:
                # not-found 등으로 실패하면 다음 주기에 다시 시도한다(written 갱신 안 함).
                continue

            with self._lock:
                self._last_written[robot_id].update(changed)

            if self._on_battery_update is not None and 'battery_level' in changed:
                self._on_battery_update(robot_id, int(changed['battery_level']))
