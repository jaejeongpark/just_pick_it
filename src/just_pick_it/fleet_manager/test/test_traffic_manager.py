"""TrafficManager 단위 테스트.

ROS Node 의존성은 MagicMock 으로 모킹하여 ROS 환경 없이 실행 가능하다.
실행: pytest src/just_pick_it/fleet_manager/test/test_traffic_manager.py -v
"""
import threading
from unittest.mock import MagicMock

import pytest

from fleet_manager.traffic_manager import (
    DOCK_PRIORITY,
    PathResult,
    TrafficManager,
    ZONE_GRAPH,
)


# ──────────────────────────────────────────────────────────────────────
# Fixtures
# ──────────────────────────────────────────────────────────────────────


@pytest.fixture
def mock_node():
    return MagicMock()


@pytest.fixture
def traffic(mock_node):
    """관리 로봇 두 대 (PICKY1, PICKY2). zone 좌표는 DEFAULT 사용."""
    return TrafficManager(
        node=mock_node,
        robot_ids=['PICKY1', 'PICKY2'],
    )


def _set_moving(traffic_mgr, robot_id: str, path: list[str]) -> None:
    """다른 로봇이 path 를 차지하고 이동 중인 상태로 직접 세팅."""
    traffic_mgr._robot_states[robot_id] = 'MOVING_TO_PRODUCT'
    traffic_mgr._robot_paths[robot_id] = path


def _set_waiting(traffic_mgr, robot_id: str, target_zone: str) -> None:
    """다른 로봇이 target_zone 에서 작업 중인 상태로 세팅."""
    traffic_mgr._robot_states[robot_id] = 'WAITING_FOR_COBOT'
    traffic_mgr._robot_paths[robot_id] = [target_zone]


# ──────────────────────────────────────────────────────────────────────
# PathResult
# ──────────────────────────────────────────────────────────────────────


class TestPathResult:
    def test_default_failure(self):
        r = PathResult(ok=False, reason='nope')
        assert r.ok is False
        assert r.waypoints == ()
        assert r.cost is None
        assert r.reason == 'nope'

    def test_is_immutable(self):
        r = PathResult(ok=True, waypoints=('A', 'B'), cost=1.0)
        with pytest.raises(Exception):
            r.ok = False


# ──────────────────────────────────────────────────────────────────────
# estimate_path
# ──────────────────────────────────────────────────────────────────────


class TestEstimatePath:
    def test_returns_path_for_reachable_zones(self, traffic):
        r = traffic.estimate_path('PICKY1', 'TRAFFIC_T1', 'TRAFFIC_T3')
        assert r.ok
        assert r.waypoints == ('TRAFFIC_T1', 'TRAFFIC_T2', 'TRAFFIC_T3')
        assert r.cost == 2.0

    def test_same_source_and_target(self, traffic):
        r = traffic.estimate_path('PICKY1', 'TRAFFIC_T1', 'TRAFFIC_T1')
        assert r.ok
        assert r.waypoints == ('TRAFFIC_T1',)
        assert r.cost == 0.0

    def test_does_not_register_reservation(self, traffic):
        traffic.estimate_path('PICKY1', 'TRAFFIC_T1', 'TRAFFIC_T3')
        assert traffic._robot_paths['PICKY1'] == []
        assert traffic._robot_reservations['PICKY1'] is None

    def test_unreachable_target_returns_failure(self, traffic):
        r = traffic.estimate_path('PICKY1', 'TRAFFIC_T1', 'NO_SUCH_ZONE')
        assert not r.ok
        assert r.reason is not None
        assert r.waypoints == ()
        assert r.cost is None


# ──────────────────────────────────────────────────────────────────────
# reserve_path
# ──────────────────────────────────────────────────────────────────────


class TestReservePath:
    def test_registers_path_and_task_id(self, traffic):
        r = traffic.reserve_path('PICKY1', 42, 'TRAFFIC_T1', 'TRAFFIC_T3')
        assert r.ok
        assert traffic._robot_reservations['PICKY1'] == 42
        assert traffic._robot_paths['PICKY1'] == [
            'TRAFFIC_T1', 'TRAFFIC_T2', 'TRAFFIC_T3',
        ]

    def test_overwrites_existing_reservation(self, traffic):
        traffic.reserve_path('PICKY1', 1, 'TRAFFIC_T1', 'TRAFFIC_T2')
        traffic.reserve_path('PICKY1', 2, 'TRAFFIC_T2', 'TRAFFIC_T3')
        assert traffic._robot_reservations['PICKY1'] == 2
        assert traffic._robot_paths['PICKY1'][-1] == 'TRAFFIC_T3'

    def test_failure_does_not_register(self, traffic):
        r = traffic.reserve_path('PICKY1', 1, 'TRAFFIC_T1', 'NO_SUCH_ZONE')
        assert not r.ok
        assert traffic._robot_reservations['PICKY1'] is None
        assert traffic._robot_paths['PICKY1'] == []

    def test_other_robot_moving_forces_detour(self, traffic):
        # PICKY2 가 TRAFFIC_T2 통과 중
        _set_moving(traffic, 'PICKY2', ['TRAFFIC_T2', 'PRODUCT_ZONE_2'])

        r = traffic.reserve_path('PICKY1', 1, 'TRAFFIC_T1', 'TRAFFIC_T3')
        assert r.ok
        assert 'TRAFFIC_T2' not in r.waypoints
        # 직접 경로(hop 2)보다 우회가 길어야 함
        assert r.cost > 2

    def test_target_node_occupied_is_unreachable(self, traffic):
        """다른 로봇이 WAITING_FOR_COBOT 상태로 점유 중인 노드는 도달 불가.

        도메인 제약: 같은 노드에 두 로봇이 동시에 머무를 공간이 없다.
        """
        _set_waiting(traffic, 'PICKY2', 'PRODUCT_ZONE_3')

        r = traffic.estimate_path('PICKY1', 'TRAFFIC_T3', 'PRODUCT_ZONE_3')
        assert not r.ok
        assert r.reason is not None

    def test_target_node_in_moving_path_is_unreachable(self, traffic):
        """다른 로봇이 이동 중에 통과할 노드도 도달 불가."""
        _set_moving(traffic, 'PICKY2', ['PRODUCT_ZONE_3', 'PRODUCT_ZONE_6'])

        r = traffic.estimate_path('PICKY1', 'TRAFFIC_T3', 'PRODUCT_ZONE_3')
        assert not r.ok


# ──────────────────────────────────────────────────────────────────────
# reserve_nearest_from
# ──────────────────────────────────────────────────────────────────────


class TestReserveNearestFrom:
    def test_picks_lowest_cost_candidate(self, traffic):
        # TRAFFIC_T1 기준
        # PRODUCT_ZONE_1: hop 1
        # PRODUCT_ZONE_3: hop 3 (T1 -> T2 -> T3 -> PZ3)
        candidates = ['PRODUCT_ZONE_3', 'PRODUCT_ZONE_1']
        r = traffic.reserve_nearest_from('PICKY1', 1, 'TRAFFIC_T1', candidates)
        assert r.ok
        assert r.waypoints[-1] == 'PRODUCT_ZONE_1'
        assert r.cost == 1.0

    def test_registers_chosen_path(self, traffic):
        r = traffic.reserve_nearest_from(
            'PICKY1', 7, 'TRAFFIC_T1', ['PRODUCT_ZONE_1', 'PRODUCT_ZONE_2'],
        )
        assert r.ok
        assert traffic._robot_reservations['PICKY1'] == 7
        assert traffic._robot_paths['PICKY1'][-1] == r.waypoints[-1]

    def test_traffic_changes_cost_ranking(self, traffic):
        """다른 로봇이 어떤 후보의 경유 노드를 차지하면, 그 후보는 우회 cost 가 늘어
        같은 후보 집합 중 원래는 동등하던 다른 후보가 선택될 수 있다.

        후보가 BFS 의 목적지(target)인 경우 그 노드 자체의 차단은 무시되지만
        (도달은 허용 정책), 경유 노드가 차단되면 우회 비용이 발생한다.
        """
        # 우회 발생 시 cost 가 늘어남을 보기 위해
        # 후보를 같은 hop 거리(2)의 두 zone 으로 잡고 한쪽 경로만 차단한다.
        # TRAFFIC_T1 -> PRODUCT_ZONE_4: hop 2 경로 (T1 -> PZ1 -> PZ4)
        # TRAFFIC_T1 -> PRODUCT_ZONE_2: hop 2 경로 (T1 -> T2 -> PZ2)
        # PICKY2 가 PRODUCT_ZONE_1 을 통과 중이면 PRODUCT_ZONE_4 로 가는 길이 우회됨.
        _set_moving(traffic, 'PICKY2', ['PRODUCT_ZONE_1', 'PRODUCT_ZONE_4'])

        r = traffic.reserve_nearest_from(
            'PICKY1', 1, 'TRAFFIC_T1',
            ['PRODUCT_ZONE_4', 'PRODUCT_ZONE_2'],
        )
        assert r.ok
        # PRODUCT_ZONE_4 는 차단된 PRODUCT_ZONE_1 을 거치려다 우회 필요 → cost 증가
        # PRODUCT_ZONE_2 는 영향 없음 (hop 2 유지)
        assert r.waypoints[-1] == 'PRODUCT_ZONE_2'

    def test_all_blocked_returns_failure(self, traffic):
        r = traffic.reserve_nearest_from(
            'PICKY1', 1, 'TRAFFIC_T1', ['NO_SUCH', 'ALSO_NO_SUCH'],
        )
        assert not r.ok
        assert r.reason == 'all candidates blocked'
        assert traffic._robot_reservations['PICKY1'] is None

    def test_empty_candidates(self, traffic):
        r = traffic.reserve_nearest_from('PICKY1', 1, 'TRAFFIC_T1', [])
        assert not r.ok


# ──────────────────────────────────────────────────────────────────────
# reserve_return_home_path
# ──────────────────────────────────────────────────────────────────────


class TestReserveReturnHomePath:
    def test_picks_inner_dock_first(self, traffic):
        r = traffic.reserve_return_home_path('PICKY1', 1, 'TRAFFIC_T1')
        assert r.ok
        # DOCK_PRIORITY[0] = (CHARGING_DOCK_1, STANDBY_ZONE_1)
        assert r.waypoints[-1] == DOCK_PRIORITY[0][1]
        assert traffic._robot_dock['PICKY1'] == DOCK_PRIORITY[0][0]

    def test_skips_occupied_dock(self, traffic):
        # PICKY1 이 안쪽 도크 예약
        traffic.reserve_return_home_path('PICKY1', 1, 'TRAFFIC_T1')
        # PICKY2 가 같은 시점에 귀환 시도
        r = traffic.reserve_return_home_path('PICKY2', 2, 'TRAFFIC_B1')
        assert r.ok
        assert r.waypoints[-1] == DOCK_PRIORITY[1][1]
        assert traffic._robot_dock['PICKY2'] == DOCK_PRIORITY[1][0]

    def test_all_docks_occupied_returns_failure(self, traffic):
        # 두 도크 모두 점유
        traffic.reserve_return_home_path('PICKY1', 1, 'TRAFFIC_T1')
        traffic.reserve_return_home_path('PICKY2', 2, 'TRAFFIC_B1')

        # 임의의 세 번째 로봇 ID 시뮬레이션
        traffic._robot_states['PICKY3'] = 'STANDBY'
        traffic._robot_paths['PICKY3'] = []
        traffic._robot_reservations['PICKY3'] = None
        traffic._robot_dock['PICKY3'] = None

        r = traffic.reserve_return_home_path('PICKY3', 3, 'TRAFFIC_T1')
        assert not r.ok
        assert r.reason == 'no available charging dock'


# ──────────────────────────────────────────────────────────────────────
# update_path_progress
# ──────────────────────────────────────────────────────────────────────


class TestUpdatePathProgress:
    def test_trims_traversed_waypoints(self, traffic):
        traffic.reserve_path('PICKY1', 1, 'TRAFFIC_T1', 'TRAFFIC_T3')
        traffic.update_path_progress('PICKY1', 1, 1)
        assert traffic._robot_paths['PICKY1'] == ['TRAFFIC_T2', 'TRAFFIC_T3']

    def test_index_zero_keeps_full_path(self, traffic):
        traffic.reserve_path('PICKY1', 1, 'TRAFFIC_T1', 'TRAFFIC_T3')
        traffic.update_path_progress('PICKY1', 1, 0)
        assert traffic._robot_paths['PICKY1'] == [
            'TRAFFIC_T1', 'TRAFFIC_T2', 'TRAFFIC_T3',
        ]

    def test_ignores_mismatched_task_id(self, traffic):
        traffic.reserve_path('PICKY1', 1, 'TRAFFIC_T1', 'TRAFFIC_T3')
        traffic.update_path_progress('PICKY1', 999, 2)
        # 변경되지 않아야 함
        assert traffic._robot_paths['PICKY1'] == [
            'TRAFFIC_T1', 'TRAFFIC_T2', 'TRAFFIC_T3',
        ]

    def test_index_beyond_path_length_safe(self, traffic):
        traffic.reserve_path('PICKY1', 1, 'TRAFFIC_T1', 'TRAFFIC_T2')
        traffic.update_path_progress('PICKY1', 1, 99)
        # 비정상 인덱스에도 path 가 망가지지 않음 (조건문이 막음)
        assert len(traffic._robot_paths['PICKY1']) >= 1


# ──────────────────────────────────────────────────────────────────────
# release_path
# ──────────────────────────────────────────────────────────────────────


class TestReleasePath:
    def test_releases_with_matching_task_id(self, traffic):
        traffic.reserve_path('PICKY1', 42, 'TRAFFIC_T1', 'TRAFFIC_T3')
        traffic.release_path('PICKY1', 42)
        assert traffic._robot_paths['PICKY1'] == []
        assert traffic._robot_reservations['PICKY1'] is None

    def test_ignores_mismatched_task_id(self, traffic):
        traffic.reserve_path('PICKY1', 42, 'TRAFFIC_T1', 'TRAFFIC_T3')
        traffic.release_path('PICKY1', 999)
        assert traffic._robot_reservations['PICKY1'] == 42

    def test_no_op_when_no_reservation(self, traffic):
        # 아무 예약 없는 상태에서 호출
        traffic.release_path('PICKY1', 1)
        assert traffic._robot_paths['PICKY1'] == []


# ──────────────────────────────────────────────────────────────────────
# notify_state (안전망 동작)
# ──────────────────────────────────────────────────────────────────────


class TestNotifyState:
    def test_idle_state_clears_path_and_reservation(self, traffic):
        traffic.reserve_path('PICKY1', 1, 'TRAFFIC_T1', 'TRAFFIC_T3')
        traffic.notify_state('PICKY1', 'STANDBY')
        assert traffic._robot_paths['PICKY1'] == []
        assert traffic._robot_reservations['PICKY1'] is None

    def test_moving_state_keeps_reservation(self, traffic):
        traffic.reserve_path('PICKY1', 1, 'TRAFFIC_T1', 'TRAFFIC_T3')
        traffic.notify_state('PICKY1', 'MOVING_TO_PRODUCT')
        assert traffic._robot_reservations['PICKY1'] == 1

    def test_waiting_state_keeps_reservation(self, traffic):
        traffic.reserve_path('PICKY1', 1, 'TRAFFIC_T1', 'TRAFFIC_T3')
        traffic.notify_state('PICKY1', 'WAITING_FOR_COBOT')
        assert traffic._robot_reservations['PICKY1'] == 1

    def test_charging_leaves_dock_on_state_change(self, traffic):
        # 사전: CHARGING + 도크 점유
        traffic._robot_states['PICKY1'] = 'CHARGING'
        traffic._robot_dock['PICKY1'] = 'CHARGING_DOCK_1'

        traffic.notify_state('PICKY1', 'MOVING_TO_PRODUCT')
        assert traffic._robot_dock['PICKY1'] is None

    def test_charging_to_charging_keeps_dock(self, traffic):
        traffic._robot_states['PICKY1'] = 'CHARGING'
        traffic._robot_dock['PICKY1'] = 'CHARGING_DOCK_1'

        traffic.notify_state('PICKY1', 'CHARGING')
        assert traffic._robot_dock['PICKY1'] == 'CHARGING_DOCK_1'


# ──────────────────────────────────────────────────────────────────────
# 동시성
# ──────────────────────────────────────────────────────────────────────


class TestConcurrency:
    def test_concurrent_non_overlapping_reserves_both_succeed(self, traffic):
        """경로가 겹치지 않으면 두 로봇 모두 동시 예약 성공."""
        results = []

        def worker(rid, task_id, src, tgt):
            results.append((rid, traffic.reserve_path(rid, task_id, src, tgt)))

        t1 = threading.Thread(target=worker,
                              args=('PICKY1', 1, 'TRAFFIC_T1', 'TRAFFIC_T3'))
        t2 = threading.Thread(target=worker,
                              args=('PICKY2', 2, 'TRAFFIC_B1', 'TRAFFIC_B3'))
        t1.start()
        t2.start()
        t1.join()
        t2.join()

        assert all(r.ok for _, r in results)
        # 경로 겹치지 않음 확인
        p1 = set(traffic._robot_paths['PICKY1'])
        p2 = set(traffic._robot_paths['PICKY2'])
        assert p1.isdisjoint(p2)

    def test_concurrent_dock_reservation_serializes(self, traffic):
        """두 로봇이 동시에 reserve_return_home_path 호출 시 서로 다른 도크 할당."""
        results = []

        def worker(rid, task_id, src):
            results.append((rid, traffic.reserve_return_home_path(rid, task_id, src)))

        t1 = threading.Thread(target=worker, args=('PICKY1', 1, 'TRAFFIC_T1'))
        t2 = threading.Thread(target=worker, args=('PICKY2', 2, 'TRAFFIC_B1'))
        t1.start()
        t2.start()
        t1.join()
        t2.join()

        # 둘 다 성공
        assert all(r.ok for _, r in results)
        docks = {traffic._robot_dock['PICKY1'], traffic._robot_dock['PICKY2']}
        # 서로 다른 도크
        assert docks == {DOCK_PRIORITY[0][0], DOCK_PRIORITY[1][0]}


# ──────────────────────────────────────────────────────────────────────
# zone_coords 주입
# ──────────────────────────────────────────────────────────────────────


class TestZoneCoordsInjection:
    def test_override_default_coords(self, mock_node):
        custom = {'TRAFFIC_T1': (9.99, 8.88)}
        tm = TrafficManager(
            node=mock_node,
            robot_ids=['PICKY1'],
            zone_coords=custom,
        )
        assert tm._zone_coords['TRAFFIC_T1'] == (9.99, 8.88)

    def test_keeps_default_for_non_overridden(self, mock_node):
        custom = {'TRAFFIC_T1': (9.99, 8.88)}
        tm = TrafficManager(
            node=mock_node,
            robot_ids=['PICKY1'],
            zone_coords=custom,
        )
        # 다른 zone 은 DEFAULT 유지
        assert 'PRODUCT_ZONE_1' in tm._zone_coords


# ──────────────────────────────────────────────────────────────────────
# ZONE_GRAPH 무결성
# ──────────────────────────────────────────────────────────────────────


class TestZoneGraphIntegrity:
    def test_graph_is_bidirectional(self):
        """모든 엣지가 양방향으로 정의되어 있는지 (단차선 양방향)."""
        for src, neighbors in ZONE_GRAPH.items():
            for nb in neighbors:
                assert nb in ZONE_GRAPH, f'{nb} (from {src}) 가 그래프에 없음'
                assert src in ZONE_GRAPH[nb], (
                    f'엣지 비대칭: {src} -> {nb} 는 있지만 {nb} -> {src} 가 없음'
                )

    def test_all_dock_priority_entries_in_graph(self):
        for dock, standby in DOCK_PRIORITY:
            assert dock in ZONE_GRAPH
            assert standby in ZONE_GRAPH
            assert dock in ZONE_GRAPH[standby], (
                f'도크 {dock} 가 standby {standby} 의 인접 노드가 아님'
            )
