import math
from collections import deque
import threading

import requests
from rclpy.node import Node
from std_msgs.msg import String


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

    로봇 상태는 각 AMR의 /{ns}/picky_state 토픽을 구독해 실시간으로 유지하고,
    배정된 경로는 내부 딕셔너리로 관리한다.
    zone 좌표는 시작 시 Control Server에서 받아 내부에 캐싱한다.
    """

    def __init__(self, node: Node, robot_ids: list[str], server_url: str) -> None:
        self._node = node
        self._server_url = server_url
        self._lock = threading.Lock()

        # robot_id -> picky_state 문자열
        self._robot_states: dict[str, str] = {rid: 'STANDBY' for rid in robot_ids}
        # robot_id -> 배정된 waypoint 경로 (zone 이름 리스트)
        self._robot_paths: dict[str, list[str]] = {rid: [] for rid in robot_ids}
        # robot_id -> 예약/점유 중인 충전 도크 이름 (없으면 None)
        self._robot_dock: dict[str, str | None] = {rid: None for rid in robot_ids}
        # zone_name -> (pos_x, pos_y). SLAM 전 임시 좌표로 초기화한다.
        self._zone_coords: dict[str, tuple[float, float]] = dict(DEFAULT_ZONE_COORDS)

        for robot_id in robot_ids:
            ns = robot_id.lower()
            node.create_subscription(
                String,
                f'/{ns}/picky_state',
                lambda msg, rid=robot_id: self._on_state(rid, msg.data),
                10,
            )

        self._fetch_zone_coords()
        node.get_logger().info(
            f'[TrafficManager] 초기화 완료 — 관리 로봇: {robot_ids}'
        )

    # ── 외부 인터페이스 ────────────────────────────────────────────────

    def find_path(
        self,
        source_zone: str,
        target_zone: str,
        robot_id: str,
    ) -> list[str] | None:
        """
        source_zone 에서 target_zone 까지 충돌 없는 waypoint 경로를 반환한다.
        경로가 존재하면 내부 경로 레지스트리에 자동 등록된다.

        BFS 계산과 경로 등록을 단일 lock 안에서 원자적으로 수행하여,
        두 로봇이 동시에 find_path 를 호출했을 때 서로의 경로를 보지 못한 채
        같은 노드를 점유하는 race condition 을 차단한다.
        (블락 단위는 복도가 아닌 다른 로봇의 남은 경로상 노드/엣지)
        """
        with self._lock:
            blocked_nodes, blocked_edges = self._build_blocked_sets(robot_id)
            path = self._bfs(source_zone, target_zone, blocked_nodes, blocked_edges)
            if path is not None:
                self._robot_paths[robot_id] = path

        if path is not None:
            self._node.get_logger().info(
                f'[TrafficManager] {robot_id} 경로 생성: {" -> ".join(path)}'
            )
        else:
            self._node.get_logger().warn(
                f'[TrafficManager] {robot_id} 경로 없음: {source_zone} -> {target_zone}'
            )

        return path

    def find_nearest_product_path(
        self,
        robot_id: str,
        source_zone: str,
        remaining_zones: list[str],
    ) -> tuple[str, list[str]] | None:
        """
        remaining_zones (아직 상차되지 않은 PRODUCT_ZONE 목록) 중에서
        이동 가능한 경로가 있는 zone 중 source_zone 에서 유클리드 거리가
        가장 가까운 zone 을 선정하고 (zone_name, waypoints) 를 반환한다.

        모든 candidate 가 차단되어 있으면 None 을 반환한다.

        후보 평가와 경로 등록을 단일 lock 안에서 원자적으로 수행하여,
        평가 도중 다른 로봇의 경로가 갱신되어 결과가 일관되지 않게 되는
        race condition 을 차단한다.
        """
        src_x, src_y = self._zone_coords.get(source_zone, (0.0, 0.0))

        best_zone: str | None = None
        best_path: list[str] | None = None
        best_dist = float('inf')

        with self._lock:
            blocked_nodes, blocked_edges = self._build_blocked_sets(robot_id)

            for zone_name in remaining_zones:
                path = self._bfs(source_zone, zone_name, blocked_nodes, blocked_edges)
                if path is None:
                    continue

                tgt_x, tgt_y = self._zone_coords.get(zone_name, (0.0, 0.0))
                dist = math.hypot(tgt_x - src_x, tgt_y - src_y)

                if dist < best_dist:
                    best_dist = dist
                    best_zone = zone_name
                    best_path = path

            if best_zone is not None:
                self._robot_paths[robot_id] = best_path

        if best_zone is not None:
            self._node.get_logger().info(
                f'[TrafficManager] {robot_id} 최근접 상품 선정: {best_zone} '
                f'(거리: {best_dist:.2f}m), 경로: {" -> ".join(best_path)}'
            )
        else:
            self._node.get_logger().warn(
                f'[TrafficManager] {robot_id} 이동 가능한 상품 zone 없음 '
                f'(candidates: {remaining_zones})'
            )

        return (best_zone, best_path) if best_zone is not None else None

    def find_return_home_path(
        self,
        robot_id: str,
        source_zone: str,
    ) -> tuple[str, list[str]] | None:
        """
        RETURN_HOME 시 비어있는 충전 도크 중 안쪽(CHARGING_DOCK_1)을 우선 선택하고
        해당 STANDBY_ZONE 까지의 경로를 반환한다.
        도크 예약은 이 메서드 호출 시점에 기록된다.

        반환: (선택된 standby_zone_name, waypoints) 또는 None (모든 도크 점유 시)

        도크 점유 조회, BFS 계산, 경로/도크 예약을 단일 lock 안에서 원자적으로
        수행하여 두 로봇이 동시에 같은 도크를 비어있다고 판단해 둘 다 예약하는
        race condition 을 차단한다.
        """
        chosen: tuple[str, list[str]] | None = None

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
                self._robot_dock[robot_id] = dock_name
                chosen = (dock_name, standby_zone, path)
                break

        if chosen is not None:
            dock_name, standby_zone, path = chosen
            self._node.get_logger().info(
                f'[TrafficManager] {robot_id} 귀환 도크 선정: {dock_name} '
                f'경로: {" -> ".join(path)}'
            )
            return standby_zone, path

        self._node.get_logger().warn(
            f'[TrafficManager] {robot_id} 귀환 가능한 도크 없음'
        )
        return None

    def update_path_progress(self, robot_id: str, current_waypoint_index: int) -> None:
        """
        로봇이 waypoint 를 통과할 때마다 호출한다.
        이미 지나온 구간을 _robot_paths 에서 제거하여
        다른 로봇이 해당 노드/엣지를 사용할 수 있게 한다.

        Task Manager 가 Action 피드백(current_waypoint_index)을 받을 때 호출.
        """
        with self._lock:
            path = self._robot_paths.get(robot_id, [])
            if current_waypoint_index < len(path):
                self._robot_paths[robot_id] = path[current_waypoint_index:]

    def clear_path(self, robot_id: str) -> None:
        """로봇이 목적지에 도착하면 경로를 해제한다."""
        with self._lock:
            self._robot_paths[robot_id] = []

    def get_robot_state(self, robot_id: str) -> str | None:
        with self._lock:
            return self._robot_states.get(robot_id)

    def get_all_states(self) -> dict[str, str]:
        with self._lock:
            return dict(self._robot_states)

    # ── zone 좌표 캐시 ─────────────────────────────────────────────────

    def _fetch_zone_coords(self) -> None:
        """Control Server에서 zone 좌표를 받아 캐시를 덮어쓴다.

        서버 응답이 없으면 DEFAULT_ZONE_COORDS 가 그대로 유지된다.
        SLAM 완료 후 서버 DB 의 zone 좌표를 채워두면 자동으로 실측치로 대체된다.
        """
        try:
            resp = requests.get(
                f'{self._server_url}/api/fleet/zones', timeout=5.0
            )
            if resp.status_code == 200:
                overridden = 0
                for zone in resp.json():
                    self._zone_coords[zone['zone_name']] = (
                        float(zone['pos_x']),
                        float(zone['pos_y']),
                    )
                    overridden += 1
                self._node.get_logger().info(
                    f'[TrafficManager] zone 좌표 서버 동기화 완료: '
                    f'{overridden}개 갱신 (전체 캐시 {len(self._zone_coords)}개)'
                )
            else:
                self._node.get_logger().warn(
                    f'[TrafficManager] zone 좌표 서버 동기화 실패: HTTP {resp.status_code} '
                    f'— 기본 좌표 사용 ({len(self._zone_coords)}개)'
                )
        except Exception as e:
            self._node.get_logger().warn(
                f'[TrafficManager] zone 좌표 서버 동기화 오류: {e} '
                f'— 기본 좌표 사용 ({len(self._zone_coords)}개)'
            )

    # ── 상태 토픽 콜백 ─────────────────────────────────────────────────

    def _on_state(self, robot_id: str, state: str) -> None:
        with self._lock:
            prev = self._robot_states.get(robot_id)
            self._robot_states[robot_id] = state

            # 이동 완료 후 정지 상태 전환 시 경로 해제
            if state not in MOVING_STATES:
                self._robot_paths[robot_id] = []

            # CHARGING 에서 벗어나면 도크 점유 해제
            # (State Manager가 도크 이탈 후 picky_state를 변경하면 자동 트리거)
            if prev == 'CHARGING' and state != 'CHARGING':
                released = self._robot_dock.pop(robot_id, None)
                self._robot_dock[robot_id] = None
                if released:
                    self._node.get_logger().info(
                        f'[TrafficManager] {robot_id} 도크 해제: {released}'
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
                # 목적지 자체는 차단 무시 — 도달은 허용
                if neighbor in blocked_nodes and neighbor != target:
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
