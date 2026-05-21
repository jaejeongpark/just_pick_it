from collections import deque
import threading

from rclpy.node import Node
from std_msgs.msg import String


# zone 이름 기반 인접 그래프.
# TRAFFIC_ZONE_1 이 허브 역할을 한다.
# 실제 맵 변경 시 여기만 수정하면 된다.
ZONE_GRAPH: dict[str, list[str]] = {
    'CHARGING_DOCK_1': ['STANDBY_ZONE_1'],
    'CHARGING_DOCK_2': ['STANDBY_ZONE_2'],
    'STANDBY_ZONE_1':  ['CHARGING_DOCK_1', 'TRAFFIC_ZONE_1'],
    'STANDBY_ZONE_2':  ['CHARGING_DOCK_2', 'TRAFFIC_ZONE_1'],
    'TRAFFIC_ZONE_1':  [
        'STANDBY_ZONE_1', 'STANDBY_ZONE_2',
        'PRODUCT_ZONE_1', 'PRODUCT_ZONE_2', 'PRODUCT_ZONE_3',
        'PRODUCT_ZONE_4', 'PRODUCT_ZONE_5', 'PRODUCT_ZONE_6',
        'PICKUP_ZONE', 'STOCK_ZONE',
    ],
    'PRODUCT_ZONE_1':  ['TRAFFIC_ZONE_1'],
    'PRODUCT_ZONE_2':  ['TRAFFIC_ZONE_1'],
    'PRODUCT_ZONE_3':  ['TRAFFIC_ZONE_1'],
    'PRODUCT_ZONE_4':  ['TRAFFIC_ZONE_1'],
    'PRODUCT_ZONE_5':  ['TRAFFIC_ZONE_1'],
    'PRODUCT_ZONE_6':  ['TRAFFIC_ZONE_1'],
    'PICKUP_ZONE':     ['TRAFFIC_ZONE_1'],
    'STOCK_ZONE':      ['TRAFFIC_ZONE_1'],
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


class TrafficManager:
    """
    BFS 기반 경로 계획 및 다중 AMR 충돌 회피 모듈.

    로봇 상태는 각 AMR의 /{ns}/picky_state 토픽을 구독해 실시간으로 유지하고,
    배정된 경로는 내부 딕셔너리로 관리한다.
    """

    def __init__(self, node: Node, robot_ids: list[str]) -> None:
        self._node = node
        self._lock = threading.Lock()

        # robot_id -> picky_state 문자열
        self._robot_states: dict[str, str] = {rid: 'STANDBY' for rid in robot_ids}
        # robot_id -> 배정된 waypoint 경로 (zone 이름 리스트)
        self._robot_paths: dict[str, list[str]] = {rid: [] for rid in robot_ids}

        for robot_id in robot_ids:
            ns = robot_id.lower()
            node.create_subscription(
                String,
                f'/{ns}/picky_state',
                lambda msg, rid=robot_id: self._on_state(rid, msg.data),
                10,
            )

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
        경로를 찾지 못하면 None 을 반환한다.
        """
        with self._lock:
            blocked_nodes, blocked_edges = self._build_blocked_sets(robot_id)

        path = self._bfs(source_zone, target_zone, blocked_nodes, blocked_edges)

        if path is not None:
            with self._lock:
                self._robot_paths[robot_id] = path
            self._node.get_logger().info(
                f'[TrafficManager] {robot_id} 경로 생성: {" -> ".join(path)}'
            )
        else:
            self._node.get_logger().warn(
                f'[TrafficManager] {robot_id} 경로 없음: {source_zone} -> {target_zone}'
            )

        return path

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

    # ── 상태 토픽 콜백 ─────────────────────────────────────────────────

    def _on_state(self, robot_id: str, state: str) -> None:
        with self._lock:
            prev = self._robot_states.get(robot_id)
            self._robot_states[robot_id] = state

            # 이동 완료 후 정지 상태 전환 시 경로 해제
            if state not in MOVING_STATES:
                self._robot_paths[robot_id] = []

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
