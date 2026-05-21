from collections import deque
from dataclasses import dataclass
import threading

from rclpy.node import Node


@dataclass(frozen=True)
class PathResult:
    """경로 탐색/예약 결과.

    Attributes:
        ok: 경로 탐색 또는 예약 성공 여부.
        waypoints: 시작 zone 부터 목적지 zone 까지의 zone_name 튜플.
                   ok=False 이면 빈 튜플.
        cost: 경로 비용 (현재 구현은 hop 수). ok=False 이면 None.
        reason: 실패 사유. ok=True 이면 None.
    """
    ok: bool
    waypoints: tuple[str, ...] = ()
    cost: float | None = None
    reason: str | None = None


# zone 이름 기반 인접 그래프.
# 맵 상의 실제 노드와 엣지를 그대로 반영한다.
#   - 상단 복도 T1~T3, 하단 복도 B1~B3 단차선 양방향
#   - 우측 수직 복도 R1~R4 단차선 양방향
#   - 각 열 내부 수직 통로: TRAFFIC_T(i) ↔ PZ_i ↔ PZ_(i+3) ↔ TRAFFIC_B(i)
#   - STANDBY_ZONE_1 은 STANDBY_ZONE_2 를 거쳐야만 외부로 진출 (안쪽 도크 안전)
#   - STOCK_ZONE 은 TRAFFIC_T1 으로만 진출
#   - STANDBY_ZONE_2 는 PZ_1, PZ_4 로 직접 진입 가능
ZONE_GRAPH: dict[str, list[str]] = {
    # 좌측 충전 구역
    'CHARGING_DOCK_1': ['STANDBY_ZONE_1'],
    'CHARGING_DOCK_2': ['STANDBY_ZONE_2'],

    # 좌측 대기 구역
    'STANDBY_ZONE_1':  ['CHARGING_DOCK_1', 'STANDBY_ZONE_2'],
    'STANDBY_ZONE_2':  ['CHARGING_DOCK_2', 'STANDBY_ZONE_1',
                        'PRODUCT_ZONE_1', 'PRODUCT_ZONE_4'],

    # 좌측 재고 구역
    'STOCK_ZONE':      ['TRAFFIC_T1'],

    # 상단 복도 (단차선 양방향)
    'TRAFFIC_T1': ['STOCK_ZONE', 'PRODUCT_ZONE_1', 'TRAFFIC_T2'],
    'TRAFFIC_T2': ['TRAFFIC_T1', 'PRODUCT_ZONE_2', 'TRAFFIC_T3'],
    'TRAFFIC_T3': ['TRAFFIC_T2', 'PRODUCT_ZONE_3', 'TRAFFIC_R1'],

    # 하단 복도 (단차선 양방향)
    'TRAFFIC_B1': ['PRODUCT_ZONE_4', 'TRAFFIC_B2'],
    'TRAFFIC_B2': ['TRAFFIC_B1', 'PRODUCT_ZONE_5', 'TRAFFIC_B3'],
    'TRAFFIC_B3': ['TRAFFIC_B2', 'PRODUCT_ZONE_6', 'TRAFFIC_R4'],

    # 상품 구역 (열 내부 수직 단차선 + 좌측 1열은 STANDBY_2 진입)
    'PRODUCT_ZONE_1': ['TRAFFIC_T1', 'PRODUCT_ZONE_4', 'STANDBY_ZONE_2'],
    'PRODUCT_ZONE_2': ['TRAFFIC_T2', 'PRODUCT_ZONE_5'],
    'PRODUCT_ZONE_3': ['TRAFFIC_T3', 'PRODUCT_ZONE_6'],
    'PRODUCT_ZONE_4': ['TRAFFIC_B1', 'PRODUCT_ZONE_1', 'STANDBY_ZONE_2'],
    'PRODUCT_ZONE_5': ['TRAFFIC_B2', 'PRODUCT_ZONE_2'],
    'PRODUCT_ZONE_6': ['TRAFFIC_B3', 'PRODUCT_ZONE_3'],

    # 우측 수직 복도 (단차선 양방향)
    'TRAFFIC_R1': ['TRAFFIC_T3', 'TRAFFIC_R2', 'PICKUP_ZONE_1'],
    'TRAFFIC_R2': ['TRAFFIC_R1', 'TRAFFIC_R3', 'PICKUP_ZONE_2'],
    'TRAFFIC_R3': ['TRAFFIC_R2', 'TRAFFIC_R4', 'PICKUP_ZONE_3'],
    'TRAFFIC_R4': ['TRAFFIC_R3', 'TRAFFIC_B3', 'PICKUP_ZONE_4'],

    # 우측 픽업 구역
    'PICKUP_ZONE_1': ['TRAFFIC_R1'],
    'PICKUP_ZONE_2': ['TRAFFIC_R2'],
    'PICKUP_ZONE_3': ['TRAFFIC_R3'],
    'PICKUP_ZONE_4': ['TRAFFIC_R4'],
}

# 이동 중인 상태: 해당 로봇의 경로 노드 + 엣지 모두 차단
MOVING_STATES = frozenset({
    'MOVING_TO_PRODUCT',
    'MOVING_TO_PICKUP',
    'MOVING_TO_STOCK',
    'MOVING_TO_STORAGE',
    'RETURNING',
    'DOCKING',
})

# 특정 노드에 머무는 상태: 목적지 노드만 차단
OCCUPYING_STATES = frozenset({
    'WAITING_FOR_COBOT',
})

# 안쪽 도크 우선 순서: (충전 도크 이름, 도킹 시작점 STANDBY_ZONE 이름)
DOCK_PRIORITY = [
    ('CHARGING_DOCK_1', 'STANDBY_ZONE_1'),  # 안쪽
    ('CHARGING_DOCK_2', 'STANDBY_ZONE_2'),  # 바깥쪽
]

# SLAM 완료 전 임시 좌표. Traffic_node.pdf 의 노드 배치를 2m x 1m 맵에 추정 배치.
# 원점은 좌하단 (x: 0 → 2.0 오른쪽, y: 0 → 1.0 위쪽).
# 좌측 구역  : x ≈ 0.00 ~ 0.40
# 중앙 구역  : x ≈ 0.40 ~ 1.55
# 우측 구역  : x ≈ 1.55 ~ 2.00
# Control Server 의 /api/fleet/zones 응답이 있으면 그 값으로 덮어쓴다.
DEFAULT_ZONE_COORDS: dict[str, tuple[float, float]] = {
    # 좌측 충전 구역 (도킹 위치)
    'CHARGING_DOCK_1':  (0.13, 0.12),
    'CHARGING_DOCK_2':  (0.30, 0.12),

    # 좌측 대기 구역 (도킹 진입 위치)
    'STANDBY_ZONE_1':   (0.13, 0.38),
    'STANDBY_ZONE_2':   (0.30, 0.38),

    # 좌측 재고 구역
    'STOCK_ZONE':       (0.20, 0.85),

    # 상단 복도 (y ≈ 0.85)
    'TRAFFIC_T1':       (0.60, 0.85),
    'TRAFFIC_T2':       (0.95, 0.85),
    'TRAFFIC_T3':       (1.30, 0.85),

    # 하단 복도 (y ≈ 0.15)
    'TRAFFIC_B1':       (0.60, 0.15),
    'TRAFFIC_B2':       (0.95, 0.15),
    'TRAFFIC_B3':       (1.30, 0.15),

    # 상품 구역 (열별로 위/아래 2개)
    'PRODUCT_ZONE_1':   (0.60, 0.62),
    'PRODUCT_ZONE_2':   (0.95, 0.62),
    'PRODUCT_ZONE_3':   (1.30, 0.62),
    'PRODUCT_ZONE_4':   (0.60, 0.38),
    'PRODUCT_ZONE_5':   (0.95, 0.38),
    'PRODUCT_ZONE_6':   (1.30, 0.38),

    # 우측 수직 복도 (x ≈ 1.60)
    'TRAFFIC_R1':       (1.60, 0.85),
    'TRAFFIC_R2':       (1.60, 0.62),
    'TRAFFIC_R3':       (1.60, 0.38),
    'TRAFFIC_R4':       (1.60, 0.15),

    # 우측 픽업 구역 (x ≈ 1.85)
    'PICKUP_ZONE_1':    (1.85, 0.85),
    'PICKUP_ZONE_2':    (1.85, 0.62),
    'PICKUP_ZONE_3':    (1.85, 0.38),
    'PICKUP_ZONE_4':    (1.85, 0.15),
}


class TrafficManager:
    """
    BFS 기반 경로 계획 및 다중 AMR 충돌 회피 모듈.

    외부 I/O 는 없다.
      - 로봇 상태는 RobotStateMonitor 가 받아서 notify_state() 로 전달한다.
      - zone 좌표는 ControlServerClient 가 받은 값을 생성자에서 주입한다.
    """

    def __init__(
        self,
        node: Node,
        robot_ids: list[str],
        zone_coords: dict[str, tuple[float, float]] | None = None,
    ) -> None:
        self._node = node
        self._lock = threading.Lock()

        # robot_id -> picky_state 문자열
        self._robot_states: dict[str, str] = {rid: 'STANDBY' for rid in robot_ids}
        # robot_id -> 배정된 waypoint 경로 (zone 이름 리스트)
        self._robot_paths: dict[str, list[str]] = {rid: [] for rid in robot_ids}
        # robot_id -> 현재 경로를 예약한 task_id (없으면 None)
        self._robot_reservations: dict[str, int | None] = {rid: None for rid in robot_ids}
        # robot_id -> 예약/점유 중인 충전 도크 이름 (없으면 None)
        self._robot_dock: dict[str, str | None] = {rid: None for rid in robot_ids}
        # zone_name -> (pos_x, pos_y). 외부 주입값으로 DEFAULT 를 덮어쓴다.
        self._zone_coords: dict[str, tuple[float, float]] = dict(DEFAULT_ZONE_COORDS)
        if zone_coords:
            self._zone_coords.update(zone_coords)

        node.get_logger().info(
            f'[TrafficManager] 초기화 완료 — 관리 로봇: {robot_ids}, '
            f'zone 좌표 {len(self._zone_coords)}개'
        )

    # ── 외부 인터페이스 (TaskManager 가 호출) ──────────────────────────
    #
    # 표준 흐름:
    #   1. estimate_path() 로 후보 zone 들의 가능 여부/cost 를 미리 평가
    #   2. reserve_path() 또는 reserve_return_home_path() 로 실제 경로 예약
    #   3. update_path_progress() 로 waypoint 통과 시점마다 점유 해제
    #   4. task 종료(SUCCESS/FAILED/CANCELLED/timeout) 시 release_path()
    #
    # 한 robot 은 한 시점에 최대 1개의 reserve 만 보유한다.
    # 도크 점유는 notify_state(CHARGING -> 타상태) 에서 자동 해제된다.

    def estimate_path(
        self,
        robot_id: str,
        source_zone: str,
        target_zone: str,
    ) -> PathResult:
        """예약 없이 현재 traffic 기준 경로 가능 여부와 cost 만 반환한다.

        상품 후보 선정 등 평가 단계에서 사용한다.
        다음 순간 다른 로봇이 점유할 수 있으므로 100% 보장이 아니며,
        실제 실행 직전에 reserve_path() 로 확정해야 한다.
        """
        with self._lock:
            blocked_nodes, blocked_edges = self._build_blocked_sets(robot_id)
            path = self._bfs(source_zone, target_zone, blocked_nodes, blocked_edges)

        if path is None:
            return PathResult(
                ok=False,
                reason=f'no path: {source_zone} -> {target_zone}',
            )
        return PathResult(
            ok=True,
            waypoints=tuple(path),
            cost=float(len(path) - 1),
        )

    def reserve_path(
        self,
        robot_id: str,
        task_id: int,
        source_zone: str,
        target_zone: str,
    ) -> PathResult:
        """경로를 계산하고 예약한다. 성공 시 다른 로봇의 평가에서 차단된다.

        BFS 계산과 예약 등록을 단일 lock 안에서 원자적으로 수행하여,
        두 로봇이 동시에 같은 노드/엣지를 점유하는 race 를 차단한다.

        한 로봇이 이미 다른 task 의 예약을 보유 중이면 그 예약을 덮어쓴다.
        호출자는 일반적으로 release_path() 후 호출해야 한다.
        """
        with self._lock:
            blocked_nodes, blocked_edges = self._build_blocked_sets(robot_id)
            path = self._bfs(source_zone, target_zone, blocked_nodes, blocked_edges)
            if path is not None:
                self._robot_paths[robot_id] = path
                self._robot_reservations[robot_id] = task_id

        if path is None:
            self._node.get_logger().warn(
                f'[TrafficManager] {robot_id} task={task_id} 예약 실패: '
                f'{source_zone} -> {target_zone}'
            )
            return PathResult(
                ok=False,
                reason=f'no path: {source_zone} -> {target_zone}',
            )

        self._node.get_logger().info(
            f'[TrafficManager] {robot_id} task={task_id} 예약: '
            f'{" -> ".join(path)}'
        )
        return PathResult(
            ok=True,
            waypoints=tuple(path),
            cost=float(len(path) - 1),
        )

    def reserve_nearest_from(
        self,
        robot_id: str,
        task_id: int,
        source_zone: str,
        candidates: list[str],
    ) -> PathResult:
        """candidates 중 reserve 가능하고 cost 가 가장 낮은 zone 을 atomic 하게 예약한다.

        TaskManager 가 남은 상품 zone 리스트를 그대로 넘기면 한 번의 호출로
        평가 + 선정 + 예약이 끝난다. 선택된 zone 은 PathResult.waypoints[-1].
        호출자는 zone -> 도메인 객체(상품 등) 매핑을 자체적으로 보유해야 한다.

        평가와 예약을 단일 lock 안에서 원자적으로 수행하여, 평가 결과를 보고
        예약하는 사이에 다른 로봇이 점유하는 race 를 차단한다.
        """
        best_path: list[str] | None = None
        best_zone: str | None = None
        best_cost = float('inf')

        with self._lock:
            blocked_nodes, blocked_edges = self._build_blocked_sets(robot_id)

            for zone_name in candidates:
                path = self._bfs(source_zone, zone_name, blocked_nodes, blocked_edges)
                if path is None:
                    continue
                cost = float(len(path) - 1)
                if cost < best_cost:
                    best_cost = cost
                    best_zone = zone_name
                    best_path = path

            if best_path is not None:
                self._robot_paths[robot_id] = best_path
                self._robot_reservations[robot_id] = task_id

        if best_path is None:
            self._node.get_logger().warn(
                f'[TrafficManager] {robot_id} task={task_id} reserve_nearest_from '
                f'실패: 후보 모두 차단 (candidates={candidates})'
            )
            return PathResult(ok=False, reason='all candidates blocked')

        self._node.get_logger().info(
            f'[TrafficManager] {robot_id} task={task_id} 최근접 예약: {best_zone} '
            f'(cost={best_cost}), 경로: {" -> ".join(best_path)}'
        )
        return PathResult(
            ok=True,
            waypoints=tuple(best_path),
            cost=best_cost,
        )

    def reserve_return_home_path(
        self,
        robot_id: str,
        task_id: int,
        source_zone: str,
    ) -> PathResult:
        """RETURN_HOME 전용. 비어있는 충전 도크를 안쪽 우선으로 선택해 예약한다.

        도크 선정과 경로/도크 예약을 단일 lock 안에서 원자적으로 수행하여
        두 로봇이 같은 도크를 동시에 비어있다고 판단하는 race 를 차단한다.

        반환되는 PathResult.waypoints 의 마지막 zone 이 도착 STANDBY_ZONE 이다.
        도킹 자체는 호출자(State Manager)가 별도로 수행한다.
        """
        chosen_path: list[str] | None = None
        chosen_dock: str | None = None

        with self._lock:
            occupied = {dock for dock in self._robot_dock.values() if dock is not None}
            blocked_nodes, blocked_edges = self._build_blocked_sets(robot_id)

            for dock_name, standby_zone in DOCK_PRIORITY:
                if dock_name in occupied:
                    continue
                path = self._bfs(source_zone, standby_zone, blocked_nodes, blocked_edges)
                if path is None:
                    continue
                self._robot_paths[robot_id] = path
                self._robot_reservations[robot_id] = task_id
                self._robot_dock[robot_id] = dock_name
                chosen_path = path
                chosen_dock = dock_name
                break

        if chosen_path is None:
            self._node.get_logger().warn(
                f'[TrafficManager] {robot_id} task={task_id} 귀환 도크 없음'
            )
            return PathResult(ok=False, reason='no available charging dock')

        self._node.get_logger().info(
            f'[TrafficManager] {robot_id} task={task_id} 귀환 예약: '
            f'dock={chosen_dock}, 경로: {" -> ".join(chosen_path)}'
        )
        return PathResult(
            ok=True,
            waypoints=tuple(chosen_path),
            cost=float(len(chosen_path) - 1),
        )

    def update_path_progress(
        self,
        robot_id: str,
        task_id: int,
        current_waypoint_index: int,
    ) -> None:
        """로봇이 waypoint 를 통과할 때마다 호출한다.

        지나온 구간을 _robot_paths 에서 제거하여 다른 로봇이 그 노드/엣지를
        사용할 수 있게 한다. task_id 가 현재 예약과 다르면 stale 호출로
        간주하고 무시한다.

        Task Manager 가 Action 피드백(current_waypoint_index) 을 받을 때 호출.
        """
        with self._lock:
            current_task = self._robot_reservations.get(robot_id)
            if current_task != task_id:
                self._node.get_logger().warn(
                    f'[TrafficManager] {robot_id} update_path_progress(task={task_id}) '
                    f'무시: 현재 예약 task={current_task}'
                )
                return
            path = self._robot_paths.get(robot_id, [])
            if 0 < current_waypoint_index < len(path):
                self._robot_paths[robot_id] = path[current_waypoint_index:]

    def release_path(self, robot_id: str, task_id: int) -> None:
        """task 종료(SUCCESS/FAILED/CANCELLED/timeout) 시 경로 예약을 해제한다.

        task_id 가 현재 예약과 일치하지 않으면 stale release 로 간주하고
        무시한다. 도크 점유는 별도이며, picky_state 의 CHARGING 이탈 시점에
        notify_state() 가 자동 해제한다.
        """
        with self._lock:
            current_task = self._robot_reservations.get(robot_id)
            if current_task is None:
                return
            if current_task != task_id:
                self._node.get_logger().warn(
                    f'[TrafficManager] {robot_id} release_path(task={task_id}) '
                    f'무시: 현재 예약 task={current_task}'
                )
                return
            self._robot_paths[robot_id] = []
            self._robot_reservations[robot_id] = None

        self._node.get_logger().info(
            f'[TrafficManager] {robot_id} task={task_id} 경로 해제'
        )

    def get_robot_state(self, robot_id: str) -> str | None:
        with self._lock:
            return self._robot_states.get(robot_id)

    def get_all_states(self) -> dict[str, str]:
        with self._lock:
            return dict(self._robot_states)

    # ── 외부에서 주입되는 상태 갱신 ────────────────────────────────────

    def notify_state(self, robot_id: str, state: str) -> None:
        """RobotStateMonitor 가 picky_state 변경 시 호출한다.

        명시적인 release_path() 가 누락된 경우의 안전망으로,
        이동/점유 상태가 모두 아닐 때 path 와 reservation 을 함께 해제한다.
        도크 점유는 CHARGING 이탈 시점에 해제한다.
        """
        released_dock: str | None = None
        with self._lock:
            prev = self._robot_states.get(robot_id)
            self._robot_states[robot_id] = state

            # 이동/점유 상태가 모두 아닐 때만 경로/예약 해제.
            # OCCUPYING_STATES(WAITING_FOR_COBOT 등)는 마지막 노드 차단이 필요하므로 path 유지.
            if state not in MOVING_STATES and state not in OCCUPYING_STATES:
                self._robot_paths[robot_id] = []
                self._robot_reservations[robot_id] = None

            # CHARGING 에서 벗어나면 도크 점유 해제
            # (State Manager 가 도크 이탈 후 picky_state 를 변경하면 자동 트리거)
            if prev == 'CHARGING' and state != 'CHARGING':
                released_dock = self._robot_dock.get(robot_id)
                if released_dock is not None:
                    self._robot_dock[robot_id] = None

        if released_dock is not None:
            self._node.get_logger().info(
                f'[TrafficManager] {robot_id} 도크 해제: {released_dock}'
            )
        if prev != state:
            self._node.get_logger().debug(
                f'[TrafficManager] {robot_id} 상태 변경: {prev} -> {state}'
            )

    # ── BFS ────────────────────────────────────────────────────────────

    def _bfs(
        self,
        source: str,
        target: str,
        blocked_nodes: set[str],
        blocked_edges: set[tuple[str, str]],
    ) -> list[str] | None:
        if source == target:
            return [source]

        queue: deque[list[str]] = deque([[source]])
        visited: set[str] = {source}

        while queue:
            path = queue.popleft()
            current = path[-1]

            for neighbor in ZONE_GRAPH.get(current, []):
                if neighbor in visited:
                    continue
                # 차단된 노드는 목적지여도 도달 불가.
                # 같은 노드에 두 로봇이 동시에 머무를 수 있는 공간이 없으므로
                # WAITING_FOR_COBOT 등 점유 노드는 접근 금지가 도메인 제약이다.
                if neighbor in blocked_nodes:
                    continue
                if (
                    (current, neighbor) in blocked_edges
                    or (neighbor, current) in blocked_edges
                ):
                    continue

                new_path = path + [neighbor]

                if neighbor == target:
                    return new_path

                visited.add(neighbor)
                queue.append(new_path)

        return None

    # ── 차단 집합 생성 ─────────────────────────────────────────────────

    def _build_blocked_sets(
        self,
        exclude_robot_id: str,
    ) -> tuple[set[str], set[tuple[str, str]]]:
        """
        exclude_robot_id 를 제외한 다른 로봇의 상태와 경로를 기반으로
        차단 노드 집합과 차단 엣지 집합을 생성한다.
        """
        blocked_nodes: set[str] = set()
        blocked_edges: set[tuple[str, str]] = set()

        for robot_id, state in self._robot_states.items():
            if robot_id == exclude_robot_id:
                continue

            path = self._robot_paths.get(robot_id, [])

            if state in MOVING_STATES and path:
                # 이동 중: 경로 전체 노드 + 엣지 차단
                blocked_nodes.update(path)
                for i in range(len(path) - 1):
                    blocked_edges.add((path[i], path[i + 1]))

            elif state in OCCUPYING_STATES and path:
                # 목적지에서 작업 중: 해당 노드만 차단
                blocked_nodes.add(path[-1])

        return blocked_nodes, blocked_edges
