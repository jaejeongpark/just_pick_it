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


# zone 이름 기반 인접 그래프. docs/Traffic_node_2.1.jpg 의 노드 배치를 반영한다.
#   - 상단 복도 T1, T2, T3 과 하단 복도 B1, B2, B3 은 단차선 양방향
#   - 좌측 수직 복도(TRAFFIC_L 열)는 노드 간격이 너무 촘촘해 제거했다(2.1).
#     좌측 구역과 복도 사이는 상품 열(PRODUCT_ZONE_1/4)을 통해 상하로 오간다.
#   - 각 열 내부 수직 통로는 TRAFFIC_T(i) 와 PRODUCT_ZONE_i, PRODUCT_ZONE_(i+3),
#     TRAFFIC_B(i) 로 이어진다
#   - STANDBY_ZONE_1 은 STANDBY_ZONE_2 를 거쳐야만 외부로 진출 (안쪽 도크 안전)
#   - STOCK_ZONE 은 TRAFFIC_T1 으로만 진출
#   - STANDBY_ZONE_2 는 PRODUCT_ZONE_1, PRODUCT_ZONE_4 로 직접 진입
#   - PICKUP_ZONE_1 은 TRAFFIC_T3, PICKUP_ZONE_2 는 TRAFFIC_B3 와 직접 인접
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
    'TRAFFIC_T1': ['STOCK_ZONE', 'TRAFFIC_T2', 'PRODUCT_ZONE_1'],
    'TRAFFIC_T2': ['TRAFFIC_T1', 'TRAFFIC_T3', 'PRODUCT_ZONE_2'],
    'TRAFFIC_T3': ['TRAFFIC_T2', 'PRODUCT_ZONE_3', 'PICKUP_ZONE_1'],

    # 하단 복도 (단차선 양방향)
    'TRAFFIC_B1': ['TRAFFIC_B2', 'PRODUCT_ZONE_4'],
    'TRAFFIC_B2': ['TRAFFIC_B1', 'TRAFFIC_B3', 'PRODUCT_ZONE_5'],
    'TRAFFIC_B3': ['TRAFFIC_B2', 'PRODUCT_ZONE_6', 'PICKUP_ZONE_2'],

    # 상품 구역 (열 내부 수직 단차선, 좌측 1열은 STANDBY_ZONE_2 와도 인접)
    'PRODUCT_ZONE_1': ['TRAFFIC_T1', 'PRODUCT_ZONE_4', 'STANDBY_ZONE_2'],
    'PRODUCT_ZONE_2': ['TRAFFIC_T2', 'PRODUCT_ZONE_5'],
    'PRODUCT_ZONE_3': ['TRAFFIC_T3', 'PRODUCT_ZONE_6'],
    'PRODUCT_ZONE_4': ['TRAFFIC_B1', 'PRODUCT_ZONE_1', 'STANDBY_ZONE_2'],
    'PRODUCT_ZONE_5': ['TRAFFIC_B2', 'PRODUCT_ZONE_2'],
    'PRODUCT_ZONE_6': ['TRAFFIC_B3', 'PRODUCT_ZONE_3'],

    # 우측 픽업 구역 (각 복도 끝에 1개씩)
    'PICKUP_ZONE_1': ['TRAFFIC_T3'],
    'PICKUP_ZONE_2': ['TRAFFIC_B3'],
}

# 이동 중인 상태: 해당 로봇의 경로 노드 + 엣지 모두 차단
MOVING_STATES = frozenset({
    'MOVING_TO_PRODUCT',
    'MOVING_TO_PICKUP',
    'MOVING_TO_STOCK',
    'MOVING_TO_DISPLAY',
    'RETURNING',
    'DOCKING',
})

# 특정 노드에 머무는 상태: 목적지 노드만 차단
OCCUPYING_STATES = frozenset({
    'WAITING_FOR_COBOT',
})

# 로봇이 도크를 실제로 빠져나가는(언도크) 이동 상태. 도크 점유는 이 상태로
# 진입할 때만 해제한다. DOCKING 은 도크로 들어오는 중이라 제외한다.
LEAVING_DOCK_STATES = MOVING_STATES - frozenset({'DOCKING'})

# 안쪽 도크 우선 순서: (충전 도크 이름, 도킹 시작점 STANDBY_ZONE 이름)
DOCK_PRIORITY = [
    ('CHARGING_DOCK_1', 'STANDBY_ZONE_1'),  # 안쪽
    ('CHARGING_DOCK_2', 'STANDBY_ZONE_2'),  # 바깥쪽
]

# RETURN_HOME 의 목적지 후보. BFS 비용 최소 zone 을 선택한다.
STANDBY_ZONES: list[str] = ['STANDBY_ZONE_1', 'STANDBY_ZONE_2']

# SLAM 완료 전 임시 좌표. docs/Traffic_node_2.1.jpg 의 노드 배치를 2m x 1m 맵에 추정 배치.
# 원점은 좌하단 (x: 0 이 좌측, 2.0 이 우측. y: 0 이 하단, 1.0 이 상단).
# 좌측 구역  : x 는 약 0.00 부터 0.45
# 중앙 구역  : x 는 약 0.55 부터 1.50
# 우측 구역  : x 는 약 1.70 부터 2.00
# Fleet API 의 /api/fleet/zones 응답이 있으면 그 값으로 덮어쓴다.
DEFAULT_ZONE_COORDS: dict[str, tuple[float, float]] = {
    # 좌측 충전 구역 (도킹 위치)
    'CHARGING_DOCK_1':  (0.11, 0.07),
    'CHARGING_DOCK_2':  (0.28, 0.07),

    # 좌측 대기 구역 (도킹 진입 위치, 각 도크 채널 바로 위)
    'STANDBY_ZONE_1':   (0.11, 0.40),
    'STANDBY_ZONE_2':   (0.28, 0.40),

    # 좌측 재고 구역
    'STOCK_ZONE':       (0.18, 0.85),

    # 상단 복도 (y 약 0.85). x 는 같은 열 PRODUCT_ZONE 과 수직 정렬(0.64/1.06/1.48).
    'TRAFFIC_T1':       (0.64, 0.85),
    'TRAFFIC_T2':       (1.05, 0.85),
    'TRAFFIC_T3':       (1.48, 0.85),

    # 하단 복도 (y 약 0.15). x 는 같은 열 PRODUCT_ZONE 과 수직 정렬(0.64/1.06/1.48).
    'TRAFFIC_B1':       (0.64, 0.15),
    'TRAFFIC_B2':       (1.05, 0.15),
    'TRAFFIC_B3':       (1.48, 0.15),

    # 상품 구역 (열별로 위/아래 2개)
    'PRODUCT_ZONE_1':   (0.70, 0.60),
    'PRODUCT_ZONE_2':   (1.05, 0.60),
    'PRODUCT_ZONE_3':   (1.40, 0.60),
    'PRODUCT_ZONE_4':   (0.70, 0.35),
    'PRODUCT_ZONE_5':   (1.05, 0.35),
    'PRODUCT_ZONE_6':   (1.40, 0.35),

    # 우측 픽업 구역
    'PICKUP_ZONE_1':    (1.80, 0.82),
    'PICKUP_ZONE_2':    (1.80, 0.18),
}


class TrafficManager:
    """
    BFS 기반 경로 계획 및 다중 AMR 충돌 회피 모듈.

    외부 I/O 는 없다.
      - 로봇 상태는 RobotStateMonitor 가 받아서 notify_state() 로 전달한다.
      - zone 좌표는 FleetRepository 가 받은 값을 생성자에서 주입한다.
    """

    def __init__(
        self,
        node: Node,
        robot_ids: list[str],
        zone_coords: dict[str, tuple[float, float]] | None = None,
        assume_docked_at_start: bool = False,
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
        # 콜드 스타트: 로봇은 자기 충전 도크에서 부팅한다(시연 기준, seed robot pos=도크).
        # 점유로 초기화하지 않으면 다른 로봇 복귀 시 그 도크를 비었다고 보고 이미 점유된
        # 도크로 도킹해 충돌한다. robot_ids 순서 == DOCK_PRIORITY 순서 전제로 매핑하고,
        # undock(이동 상태 진입) 시 notify_state 가 자동 해제한다.
        if assume_docked_at_start:
            for rid, (dock, _standby) in zip(robot_ids, DOCK_PRIORITY):
                self._robot_dock[rid] = dock
        # zone_name -> (pos_x, pos_y). 외부 주입값으로 DEFAULT 를 덮어쓴다.
        self._zone_coords: dict[str, tuple[float, float]] = dict(DEFAULT_ZONE_COORDS)
        if zone_coords:
            self._zone_coords.update(zone_coords)

        node.get_logger().info(
            f'[TrafficManager] 초기화 완료 — 관리 로봇: {robot_ids}, '
            f'zone 좌표 {len(self._zone_coords)}개, '
            f'시작 도크 점유: {self._robot_dock}'
        )

    # ── 외부 인터페이스 (TaskManager 가 호출) ──────────────────────────
    #
    # 표준 흐름:
    #   1. reserve_path() / reserve_nearest_from() / reserve_return_home_path()
    #      중 하나로 경로를 예약한다. 평가와 예약은 단일 lock 안에서 atomic.
    #   2. update_path_progress() 로 waypoint 통과 시점마다 점유 해제
    #   3. task 종료(SUCCESS/FAILED/CANCELLED/timeout) 시 release_path()
    #
    # 후보 상품 중 가장 가까운 zone 선정도 TrafficManager 책임이며
    # reserve_nearest_from() 이 평가와 예약을 한 번에 처리한다.
    #
    # 한 robot 은 한 시점에 최대 1개의 reserve 만 보유한다.
    # 도크 점유는 notify_state(CHARGING -> 타상태) 에서 자동 해제된다.

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
        task_id: int | None,
        source_zone: str,
        candidates: dict[str, int],
    ) -> PathResult:
        """candidates 중 reserve 가능하고 cost 가 가장 낮은 zone 을 atomic 하게 예약한다.

        candidates 는 {zone_name: 상품 수량} 매핑이다. TaskManager 가 남은
        상품 목록을 zone 단위로 집계한 dict 을 그대로 넘기면 한 번의 호출로
        평가 + 선정 + 예약이 끝난다. 선택된 zone 은 PathResult.waypoints[-1].
        호출자는 zone -> 도메인 객체(상품) 매핑을 자체적으로 보유해야 한다.

        같은 zone 에 상품이 여러 개여도 픽업 자체는 한 번이므로
        TrafficManager 는 zone 이름만 사용하고 수량 값은 참고하지 않는다.
        수량을 받는 이유는 TaskManager 측 자료구조를 변환 없이 그대로
        넘길 수 있도록 하기 위한 것뿐이다.

        task_id 는 None 으로 호출할 수 있다. MOVE_TO_PRODUCT 의 첫 호출처럼
        path 가 먼저 결정돼야 task INSERT 가 가능 (= task_id 도 그때 발행)
        인 흐름에서 None 으로 호출해 path 만 받고, INSERT 후 발행된
        task_id 를 attach_task_id() 로 사후 연결한다.

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

    def attach_task_id(self, robot_id: str, task_id: int) -> bool:
        """task INSERT 후 발행된 task_id 를 현재 임시 예약에 사후 연결한다.

        호출 전제: reserve_nearest_from(task_id=None) 으로 path 가 등록돼 있고
        _robot_reservations[robot_id] 가 None 인 상태.

        성공 시 True. 이미 다른 task_id 가 연결돼 있거나 임시 예약 path 자체가
        없으면 False 와 함께 warn 로그.
        """
        with self._lock:
            current = self._robot_reservations.get(robot_id)
            if current is not None:
                self._node.get_logger().warn(
                    f'[TrafficManager] {robot_id} attach_task_id({task_id}) '
                    f'무시: 이미 task={current} 가 연결되어 있음'
                )
                return False
            if not self._robot_paths.get(robot_id):
                self._node.get_logger().warn(
                    f'[TrafficManager] {robot_id} attach_task_id({task_id}) '
                    f'무시: 임시 예약된 path 가 없음'
                )
                return False
            self._robot_reservations[robot_id] = task_id

        self._node.get_logger().info(
            f'[TrafficManager] {robot_id} task_id 연결: {task_id}'
        )
        return True

    def reserve_return_home_path(
        self,
        robot_id: str,
        task_id: int,
        source_zone: str,
    ) -> PathResult:
        """RETURN_HOME 전용. 빈 충전 도크 우선순위(안쪽 1번 먼저)에 해당하는
        STANDBY_ZONE 으로 경로를 예약한다.

        도크 수가 로봇 수와 같아 귀환 시 빈 도크가 항상 하나 이상 있다. 둘 다
        비어 있으면 CHARGING_DOCK_1 의 STANDBY_ZONE_1 로 귀환해, 이어지는 DOCK_IN
        이 안쪽 도크부터 채우도록 한다(reserve_dock_path 의 DOCK_PRIORITY 와 일치).
        도크 점유 판정이 정확하려면 STANDBY(도크 내 대기) 상태에서도 도크 점유가
        유지돼야 한다(notify_state 의 도크 해제 조건 참고).

        우선 도크의 STANDBY_ZONE 이 일시적으로 막혀 있으면 도달 가능한 다른
        STANDBY_ZONE 으로 귀환한다(로봇 고립 방지). 도크는 예약하지 않으며,
        도크 예약은 이후 DOCK_IN task 의 reserve_dock_path() 가 수행한다.

        반환되는 PathResult.waypoints 의 마지막 zone 이 목적지 STANDBY_ZONE 이다.
        """
        best_path: list[str] | None = None
        best_zone: str | None = None

        with self._lock:
            occupied = {
                dock for rid, dock in self._robot_dock.items()
                if dock is not None and rid != robot_id
            }
            blocked_nodes, blocked_edges = self._build_blocked_sets(robot_id)

            # 1순위: 빈 도크를 우선순위대로 보고 그 도크의 STANDBY_ZONE 을 목적지로.
            for dock_name, standby_zone in DOCK_PRIORITY:
                if dock_name in occupied:
                    continue
                path = self._bfs(source_zone, standby_zone, blocked_nodes, blocked_edges)
                if path is not None:
                    best_path = path
                    best_zone = standby_zone
                    break

            # 2순위(안전망): 우선 도크의 STANDBY 가 막혀 있으면 도달 가능한
            #               아무 STANDBY_ZONE 으로라도 귀환한다.
            if best_path is None:
                best_cost = float('inf')
                for zone in STANDBY_ZONES:
                    path = self._bfs(source_zone, zone, blocked_nodes, blocked_edges)
                    if path is None:
                        continue
                    cost = float(len(path) - 1)
                    if cost < best_cost:
                        best_cost = cost
                        best_zone = zone
                        best_path = path

            if best_path is not None:
                self._robot_paths[robot_id] = best_path
                self._robot_reservations[robot_id] = task_id

        if best_path is None:
            self._node.get_logger().warn(
                f'[TrafficManager] {robot_id} task={task_id} 귀환 경로 없음'
            )
            return PathResult(ok=False, reason='no path to standby zone')

        self._node.get_logger().info(
            f'[TrafficManager] {robot_id} task={task_id} 귀환 예약: '
            f'{best_zone}, 경로: {" -> ".join(best_path)}'
        )
        return PathResult(
            ok=True,
            waypoints=tuple(best_path),
            cost=float(len(best_path) - 1),
        )

    def reserve_dock_path(
        self,
        robot_id: str,
        task_id: int,
        source_zone: str,
    ) -> PathResult:
        """DOCK_IN 전용. 빈 충전 도크를 안쪽 우선으로 선택해 경로 + 도크를 예약한다.

        도크 선정과 경로/도크 예약을 단일 lock 안에서 원자적으로 수행하여
        두 로봇이 같은 도크를 동시에 비어있다고 판단하는 race 를 차단한다.

        반환되는 PathResult.waypoints 의 마지막 zone 이 목적지 CHARGING_DOCK 이다.
        """
        chosen_path: list[str] | None = None
        chosen_dock: str | None = None

        with self._lock:
            occupied = {dock for dock in self._robot_dock.values() if dock is not None}
            blocked_nodes, blocked_edges = self._build_blocked_sets(robot_id)

            for dock_name, _standby in DOCK_PRIORITY:
                if dock_name in occupied:
                    continue
                path = self._bfs(source_zone, dock_name, blocked_nodes, blocked_edges)
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
                f'[TrafficManager] {robot_id} task={task_id} 도크 예약 실패 — '
                f'모두 점유 중이거나 경로 없음'
            )
            return PathResult(ok=False, reason='no available charging dock')

        self._node.get_logger().info(
            f'[TrafficManager] {robot_id} task={task_id} 도크 예약: '
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

    def release_path(self, robot_id: str, task_id: int | None) -> None:
        """task 종료(SUCCESS/FAILED/CANCELLED/timeout) 시 경로 예약을 해제한다.

        task_id 의 의미:
          - 정상 케이스: 현재 예약의 task_id 와 일치해야 해제. 다르면 stale
            release 로 간주하고 warn 로그 후 무시.
          - None: 현재 예약이 task_id 미배정 상태(reserve_nearest_from(task_id=None)
            직후, attach_task_id 호출 전) 일 때만 임시 예약 path 를 해제.
            이미 task_id 가 연결된 상태라면 warn 후 무시한다.

        도크 점유는 별도이며, 로봇이 도크를 떠나는 이동 상태로 전환될 때
        notify_state() 가 자동 해제한다.
        """
        with self._lock:
            current_task = self._robot_reservations.get(robot_id)
            if task_id is None:
                # 임시 예약 해제: 현재 task_id 미배정 상태일 때만 동작
                if current_task is not None:
                    self._node.get_logger().warn(
                        f'[TrafficManager] {robot_id} release_path(None) 무시: '
                        f'task={current_task} 가 연결되어 있음'
                    )
                    return
                if not self._robot_paths.get(robot_id):
                    return
                self._robot_paths[robot_id] = []
            else:
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

    # ── 재시작 복구 지원 (R1 / A'') ────────────────────────────────────
    #
    # Fleet Manager 재시작 시 in-memory 점유가 비므로, 로봇의 현재 위치를 기준으로
    # 점유를 다시 세워야 한다. MOVE 점유는 기존 reserve_path(source=현재 zone, target)를
    # 그대로 재사용하고, 도크 점유만 아래 rebuild_dock 으로 복원한다.

    def nearest_zone(self, x: float, y: float) -> str | None:
        """map 좌표 (x, y) 에 가장 가까운 zone_name 을 반환한다.

        재시작 복구에서 로봇 pose 를 그래프 노드로 매핑하는 데 쓴다.
        좌표가 없거나 zone 좌표 테이블이 비어 있으면 None.
        (_zone_coords 는 생성자 이후 변하지 않으므로 lock 없이 읽는다.)
        """
        if x is None or y is None or not self._zone_coords:
            return None

        best_zone: str | None = None
        best_dist_sq = float('inf')
        for zone_name, (zx, zy) in self._zone_coords.items():
            dist_sq = (zx - x) ** 2 + (zy - y) ** 2
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_zone = zone_name
        return best_zone

    def nearest_dock(self, x: float, y: float) -> str | None:
        """map 좌표에 가장 가까운 충전 도크 이름을 반환한다(재시작 도크 추론용)."""
        if x is None or y is None:
            return None

        best_dock: str | None = None
        best_dist_sq = float('inf')
        for dock_name, _standby in DOCK_PRIORITY:
            coord = self._zone_coords.get(dock_name)
            if coord is None:
                continue
            dx, dy = coord
            dist_sq = (dx - x) ** 2 + (dy - y) ** 2
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_dock = dock_name
        return best_dock

    def rebuild_dock(self, robot_id: str, dock_name: str) -> None:
        """재시작 복구 시 CHARGING/DOCKING 로봇의 도크 점유를 다시 표시한다.

        다른 로봇이 같은 도크를 비어있다고 보고 reserve_dock_path 로 예약하는 것을 막는다.
        """
        with self._lock:
            self._robot_dock[robot_id] = dock_name
        self._node.get_logger().info(
            f'[TrafficManager] {robot_id} 도크 점유 복원: {dock_name}'
        )

    # ── 외부에서 주입되는 상태 갱신 ────────────────────────────────────

    def notify_state(self, robot_id: str, state: str) -> None:
        """RobotStateMonitor 가 picky_state 변경 시 호출한다.

        명시적인 release_path() 가 누락된 경우의 안전망으로,
        이동/점유 상태가 모두 아닐 때 path 와 reservation 을 함께 해제한다.
        도크 점유는 로봇이 실제로 도크를 떠날 때(언도크)만 해제한다.
        """
        released_dock: str | None = None
        with self._lock:
            prev = self._robot_states.get(robot_id)
            self._robot_states[robot_id] = state

            # 이동/점유에서 '빠져나올 때만' 경로/예약을 해제한다(release_path 누락 안전망).
            # prev 조건이 없으면, 갓 예약한 직후 로봇이 아직 STANDBY 텔레메트리를 보내는
            # 사이 그 신선한 예약까지 지워져(예약 task=None) 이후 흐름이 꼬인다.
            # OCCUPYING_STATES(WAITING_FOR_COBOT 등)는 마지막 노드 차단이 필요하므로 path 유지.
            idle_now = state not in MOVING_STATES and state not in OCCUPYING_STATES
            was_active = prev in MOVING_STATES or prev in OCCUPYING_STATES
            if idle_now and was_active:
                self._robot_paths[robot_id] = []
                self._robot_reservations[robot_id] = None

            # 도크 점유는 로봇이 실제로 도크를 빠져나갈 때만 해제한다.
            # State Manager 는 충전 후 배터리가 임계를 넘으면 picky_state 를
            # CHARGING -> STANDBY 로 바꾸지만 로봇은 도크 안에 그대로 머문다. 따라서
            # CHARGING 이탈만으로 해제하면 도크에 로봇이 있는데도 빈 도크로 오인된다.
            # 실제 move task 로 도크를 떠나는 이동 상태(RETURNING / MOVING_TO_*)로
            # 진입할 때만 해제한다(DOCKING 은 도크로 들어오는 중이라 제외).
            if state in LEAVING_DOCK_STATES:
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
        exclude_robot_id 를 제외한 다른 로봇의 path 와 상태를 기반으로
        차단 노드 집합과 차단 엣지 집합을 생성한다.

        path 가 등록되어 있으면 (picky_state 무관) 그 path 의 노드와 엣지를
        차단한다. reserve_* 성공 자체가 "이 로봇이 곧 그 path 를 점유" 라는
        약속이므로, picky_state 가 MOVING 으로 갱신되기 전 race window 도
        자연히 닫힌다.

        예외: state 가 OCCUPYING_STATES (WAITING_FOR_COBOT 등) 면 이미 도착해
        작업 중이므로 path[-1] (현재 머무는 노드) 만 차단하고 경유 노드는
        풀어준다.
        """
        blocked_nodes: set[str] = set()
        blocked_edges: set[tuple[str, str]] = set()

        for robot_id, state in self._robot_states.items():
            if robot_id == exclude_robot_id:
                continue

            path = self._robot_paths.get(robot_id, [])
            if not path:
                continue

            if state in OCCUPYING_STATES:
                blocked_nodes.add(path[-1])
            else:
                blocked_nodes.update(path)
                for i in range(len(path) - 1):
                    blocked_edges.add((path[i], path[i + 1]))

        return blocked_nodes, blocked_edges
