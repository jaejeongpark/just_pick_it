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
    STANDBY_ZONES,
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

        r = traffic.reserve_path('PICKY1', 1, 'TRAFFIC_T3', 'PRODUCT_ZONE_3')
        assert not r.ok
        assert r.reason is not None
        assert traffic._robot_reservations['PICKY1'] is None

    def test_target_node_in_moving_path_is_unreachable(self, traffic):
        """다른 로봇이 이동 중에 통과할 노드도 도달 불가."""
        _set_moving(traffic, 'PICKY2', ['PRODUCT_ZONE_3', 'PRODUCT_ZONE_6'])

        r = traffic.reserve_path('PICKY1', 1, 'TRAFFIC_T3', 'PRODUCT_ZONE_3')
        assert not r.ok


# ──────────────────────────────────────────────────────────────────────
# reserve_nearest_from
# ──────────────────────────────────────────────────────────────────────


class TestReserveNearestFrom:
    def test_picks_lowest_cost_candidate(self, traffic):
        # TRAFFIC_T1 기준
        # PRODUCT_ZONE_1: hop 1
        # PRODUCT_ZONE_3: hop 3 (T1 -> T2 -> T3 -> PZ3)
        candidates = {'PRODUCT_ZONE_3': 1, 'PRODUCT_ZONE_1': 2}
        r = traffic.reserve_nearest_from('PICKY1', 1, 'TRAFFIC_T1', candidates)
        assert r.ok
        assert r.waypoints[-1] == 'PRODUCT_ZONE_1'
        assert r.cost == 1.0

    def test_registers_chosen_path(self, traffic):
        r = traffic.reserve_nearest_from(
            'PICKY1', 7, 'TRAFFIC_T1',
            {'PRODUCT_ZONE_1': 1, 'PRODUCT_ZONE_2': 1},
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
            {'PRODUCT_ZONE_4': 1, 'PRODUCT_ZONE_2': 1},
        )
        assert r.ok
        # PRODUCT_ZONE_4 는 차단된 PRODUCT_ZONE_1 을 거치려다 우회 필요 → cost 증가
        # PRODUCT_ZONE_2 는 영향 없음 (hop 2 유지)
        assert r.waypoints[-1] == 'PRODUCT_ZONE_2'

    def test_all_blocked_returns_failure(self, traffic):
        r = traffic.reserve_nearest_from(
            'PICKY1', 1, 'TRAFFIC_T1', {'NO_SUCH': 1, 'ALSO_NO_SUCH': 1},
        )
        assert not r.ok
        assert r.reason == 'all candidates blocked'
        assert traffic._robot_reservations['PICKY1'] is None

    def test_empty_candidates(self, traffic):
        r = traffic.reserve_nearest_from('PICKY1', 1, 'TRAFFIC_T1', {})
        assert not r.ok


# ──────────────────────────────────────────────────────────────────────
# reserve_return_home_path
# ──────────────────────────────────────────────────────────────────────


class TestReserveReturnHomePath:
    def test_both_docks_empty_targets_dock1_standby(self, traffic):
        # 둘 다 비어 있으면 최근접(SB2)이 아니라 1번 도크 우선 -> STANDBY_ZONE_1.
        r = traffic.reserve_return_home_path('PICKY1', 1, 'TRAFFIC_T1')
        assert r.ok
        assert r.waypoints[-1] == 'STANDBY_ZONE_1'

    def test_dock1_occupied_targets_dock2_standby(self, traffic):
        # 1번 도크를 PICKY2 가 점유(충전/도크 내 대기) 중이면 PICKY1 은
        # 2번 도크의 STANDBY_ZONE_2 로 귀환한다.
        traffic._robot_dock['PICKY2'] = DOCK_PRIORITY[0][0]  # CHARGING_DOCK_1
        r = traffic.reserve_return_home_path('PICKY1', 1, 'TRAFFIC_T1')
        assert r.ok
        assert r.waypoints[-1] == 'STANDBY_ZONE_2'

    def test_does_not_reserve_dock(self, traffic):
        traffic.reserve_return_home_path('PICKY1', 1, 'TRAFFIC_T1')
        assert traffic._robot_dock['PICKY1'] is None

    def test_falls_back_when_priority_standby_blocked(self, traffic):
        # 도크는 둘 다 비었지만 1번 도크의 STANDBY_ZONE_1 을 PICKY2 가 점유 중이면,
        # 다음 빈 도크의 STANDBY_ZONE_2 로 귀환한다(우선 도크 STANDBY 막힘 처리).
        _set_waiting(traffic, 'PICKY2', 'STANDBY_ZONE_1')
        r = traffic.reserve_return_home_path('PICKY1', 1, 'TRAFFIC_T1')
        assert r.ok
        assert r.waypoints[-1] == 'STANDBY_ZONE_2'

    def test_all_standby_zones_blocked_returns_failure(self, traffic):
        # 두 standby zone 을 각각 다른 로봇이 점유
        _set_waiting(traffic, 'PICKY2', 'STANDBY_ZONE_2')
        traffic._robot_states['PICKY3'] = 'WAITING_FOR_COBOT'
        traffic._robot_paths['PICKY3'] = ['STANDBY_ZONE_1']
        traffic._robot_reservations['PICKY3'] = None
        traffic._robot_dock['PICKY3'] = None

        r = traffic.reserve_return_home_path('PICKY1', 1, 'TRAFFIC_T1')
        assert not r.ok
        assert r.reason == 'no path to standby zone'


# ──────────────────────────────────────────────────────────────────────
# reserve_dock_path
# ──────────────────────────────────────────────────────────────────────


class TestReserveDockPath:
    def test_picks_inner_dock_first(self, traffic):
        r = traffic.reserve_dock_path('PICKY1', 1, 'STANDBY_ZONE_1')
        assert r.ok
        assert r.waypoints[-1] == DOCK_PRIORITY[0][0]  # CHARGING_DOCK_1
        assert traffic._robot_dock['PICKY1'] == DOCK_PRIORITY[0][0]

    def test_skips_occupied_inner_dock(self, traffic):
        # 안쪽 도크가 이미 점유 중이면 바깥쪽 도크를 선택한다.
        traffic._robot_dock['PICKY2'] = DOCK_PRIORITY[0][0]  # CHARGING_DOCK_1 점유
        r = traffic.reserve_dock_path('PICKY1', 1, 'STANDBY_ZONE_2')
        assert r.ok
        assert r.waypoints[-1] == DOCK_PRIORITY[1][0]  # CHARGING_DOCK_2
        assert traffic._robot_dock['PICKY1'] == DOCK_PRIORITY[1][0]

    def test_all_docks_occupied_returns_failure(self, traffic):
        traffic._robot_dock['PICKY1'] = DOCK_PRIORITY[0][0]
        traffic._robot_dock['PICKY2'] = DOCK_PRIORITY[1][0]

        traffic._robot_states['PICKY3'] = 'STANDBY'
        traffic._robot_paths['PICKY3'] = []
        traffic._robot_reservations['PICKY3'] = None
        traffic._robot_dock['PICKY3'] = None

        r = traffic.reserve_dock_path('PICKY3', 3, 'STANDBY_ZONE_2')
        assert not r.ok
        assert r.reason == 'no available charging dock'

    def test_registers_path_and_task_id(self, traffic):
        r = traffic.reserve_dock_path('PICKY1', 5, 'STANDBY_ZONE_1')
        assert r.ok
        assert traffic._robot_reservations['PICKY1'] == 5
        assert traffic._robot_dock['PICKY1'] is not None
        assert traffic._robot_paths['PICKY1'][-1] == traffic._robot_dock['PICKY1']

    def test_concurrent_dock_reservation_no_double_booking(self, traffic):
        """동시 DOCK_IN 은 lock 으로 직렬화되어 같은 도크가 중복 예약되지 않는다."""
        results = []

        def worker(rid, task_id, src):
            results.append((rid, traffic.reserve_dock_path(rid, task_id, src)))

        t1 = threading.Thread(target=worker, args=('PICKY1', 1, 'STANDBY_ZONE_1'))
        t2 = threading.Thread(target=worker, args=('PICKY2', 2, 'STANDBY_ZONE_2'))
        t1.start()
        t2.start()
        t1.join()
        t2.join()

        occupied = [d for d in traffic._robot_dock.values() if d is not None]
        assert len(occupied) == len(set(occupied))


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
# Optional task_id + attach_task_id (MOVE_TO_PRODUCT 흐름)
# ──────────────────────────────────────────────────────────────────────


class TestOptionalTaskId:
    """task_id 가 task INSERT 후에 발행되는 흐름. reserve 시점에 None 허용."""

    def test_reserve_nearest_from_accepts_none_task_id(self, traffic):
        r = traffic.reserve_nearest_from(
            'PICKY1', None, 'TRAFFIC_T1', {'PRODUCT_ZONE_1': 1},
        )
        assert r.ok
        assert traffic._robot_paths['PICKY1'] == ['TRAFFIC_T1', 'PRODUCT_ZONE_1']
        # task_id 미배정 상태로 기록
        assert traffic._robot_reservations['PICKY1'] is None

    def test_attach_task_id_links_after_temp_reserve(self, traffic):
        traffic.reserve_nearest_from(
            'PICKY1', None, 'TRAFFIC_T1', {'PRODUCT_ZONE_1': 1},
        )
        ok = traffic.attach_task_id('PICKY1', 42)
        assert ok is True
        assert traffic._robot_reservations['PICKY1'] == 42

    def test_attach_task_id_fails_without_temp_reserve(self, traffic):
        # 임시 예약 path 자체가 없는 상태에서 호출
        ok = traffic.attach_task_id('PICKY1', 42)
        assert ok is False
        assert traffic._robot_reservations['PICKY1'] is None

    def test_attach_task_id_fails_when_already_linked(self, traffic):
        # 이미 task_id 가 연결된 정상 예약 위에는 덮어쓰지 않는다
        traffic.reserve_path('PICKY1', 7, 'TRAFFIC_T1', 'TRAFFIC_T2')
        ok = traffic.attach_task_id('PICKY1', 42)
        assert ok is False
        assert traffic._robot_reservations['PICKY1'] == 7

    def test_release_path_none_releases_temp_reservation(self, traffic):
        traffic.reserve_nearest_from(
            'PICKY1', None, 'TRAFFIC_T1', {'PRODUCT_ZONE_1': 1},
        )
        traffic.release_path('PICKY1', None)
        assert traffic._robot_paths['PICKY1'] == []
        assert traffic._robot_reservations['PICKY1'] is None

    def test_release_path_none_no_op_when_task_id_attached(self, traffic):
        # 정상 reserve 로 task_id 가 연결된 상태에선 None release 가 무시된다
        traffic.reserve_path('PICKY1', 42, 'TRAFFIC_T1', 'TRAFFIC_T2')
        traffic.release_path('PICKY1', None)
        assert traffic._robot_reservations['PICKY1'] == 42
        assert traffic._robot_paths['PICKY1'] != []

    def test_full_move_to_product_flow(self, traffic):
        """MOVE_TO_PRODUCT 의 전형적 흐름: None reserve -> attach -> release."""
        r = traffic.reserve_nearest_from(
            'PICKY1', None, 'TRAFFIC_T1',
            {'PRODUCT_ZONE_1': 2, 'PRODUCT_ZONE_2': 1},
        )
        assert r.ok
        # 가장 가까운 PRODUCT_ZONE_1 선정
        assert r.waypoints[-1] == 'PRODUCT_ZONE_1'

        # task INSERT 결과 task_id=99 발행
        assert traffic.attach_task_id('PICKY1', 99) is True

        # 정상 release (matching task_id)
        traffic.release_path('PICKY1', 99)
        assert traffic._robot_paths['PICKY1'] == []
        assert traffic._robot_reservations['PICKY1'] is None


# ──────────────────────────────────────────────────────────────────────
# race window 닫힘 (path 가 등록되면 state 무관하게 차단)
# ──────────────────────────────────────────────────────────────────────


class TestRaceWindowClosed:
    """reserve 직후 picky_state 가 MOVING 으로 갱신되기 전에도 path 가 차단되는지."""

    def test_reserved_path_blocks_others_in_standby_state(self, traffic):
        # PICKY1 reserve 직후, state 는 default 'STANDBY' (notify_state 아직 안 옴)
        traffic.reserve_path('PICKY1', 1, 'TRAFFIC_T1', 'TRAFFIC_T3')
        assert traffic._robot_states['PICKY1'] == 'STANDBY'

        # PICKY2 가 T2 를 통과하려 시도. T2 는 PICKY1 path 의 경유 노드이므로
        # 차단되어야 함 (state 가 MOVING 으로 갱신되지 않아도).
        r = traffic.reserve_path('PICKY2', 2, 'PRODUCT_ZONE_2', 'TRAFFIC_T2')
        assert not r.ok

    def test_temp_reservation_also_blocks_others(self, traffic):
        # task_id None 임시 예약도 동일하게 path 차단 효과를 가진다.
        traffic.reserve_nearest_from(
            'PICKY1', None, 'TRAFFIC_T1', {'PRODUCT_ZONE_1': 1},
        )
        # PICKY2 가 PRODUCT_ZONE_1 로 가려 시도 → 차단
        r = traffic.reserve_path('PICKY2', 2, 'TRAFFIC_T2', 'PRODUCT_ZONE_1')
        assert not r.ok


# ──────────────────────────────────────────────────────────────────────
# notify_state (안전망 동작)
# ──────────────────────────────────────────────────────────────────────


class TestNotifyState:
    def test_idle_after_active_clears_path_and_reservation(self, traffic):
        # 이동/점유에서 idle 로 빠져나올 때만 안전망으로 경로/예약을 해제한다.
        traffic.reserve_path('PICKY1', 1, 'TRAFFIC_T1', 'TRAFFIC_T3')
        traffic.notify_state('PICKY1', 'MOVING_TO_PRODUCT')   # active
        traffic.notify_state('PICKY1', 'STANDBY')             # active -> idle
        assert traffic._robot_paths['PICKY1'] == []
        assert traffic._robot_reservations['PICKY1'] is None

    def test_fresh_reservation_kept_during_idle_telemetry(self, traffic):
        # 갓 예약한 직후 로봇이 아직 idle(STANDBY) 텔레메트리를 보내도 예약은 유지돼야 한다.
        # prev 도 idle 이라 안전망이 신선한 예약을 지우면 안 됨('예약 task=None' 레이스 방지).
        traffic.reserve_path('PICKY1', 1, 'TRAFFIC_T1', 'TRAFFIC_T3')
        traffic.notify_state('PICKY1', 'STANDBY')
        assert traffic._robot_reservations['PICKY1'] == 1

    def test_moving_state_keeps_reservation(self, traffic):
        traffic.reserve_path('PICKY1', 1, 'TRAFFIC_T1', 'TRAFFIC_T3')
        traffic.notify_state('PICKY1', 'MOVING_TO_PRODUCT')
        assert traffic._robot_reservations['PICKY1'] == 1

    def test_waiting_state_keeps_reservation(self, traffic):
        traffic.reserve_path('PICKY1', 1, 'TRAFFIC_T1', 'TRAFFIC_T3')
        traffic.notify_state('PICKY1', 'WAITING_FOR_COBOT')
        assert traffic._robot_reservations['PICKY1'] == 1

    def test_charging_to_standby_keeps_dock(self, traffic):
        # 배터리 임계 초과로 CHARGING -> STANDBY 가 돼도 로봇은 도크 안에 있으므로
        # 도크 점유를 유지해야 한다(빈 도크로 오인 금지).
        traffic._robot_states['PICKY1'] = 'CHARGING'
        traffic._robot_dock['PICKY1'] = 'CHARGING_DOCK_1'

        traffic.notify_state('PICKY1', 'STANDBY')
        assert traffic._robot_dock['PICKY1'] == 'CHARGING_DOCK_1'

    def test_charging_to_charging_keeps_dock(self, traffic):
        traffic._robot_states['PICKY1'] = 'CHARGING'
        traffic._robot_dock['PICKY1'] = 'CHARGING_DOCK_1'

        traffic.notify_state('PICKY1', 'CHARGING')
        assert traffic._robot_dock['PICKY1'] == 'CHARGING_DOCK_1'

    def test_docking_keeps_dock(self, traffic):
        # DOCKING 은 도크로 들어오는 중이라 점유를 유지한다.
        traffic._robot_dock['PICKY1'] = 'CHARGING_DOCK_1'
        traffic.notify_state('PICKY1', 'DOCKING')
        assert traffic._robot_dock['PICKY1'] == 'CHARGING_DOCK_1'

    def test_undock_via_move_releases_dock(self, traffic):
        # 충전 후 STANDBY(도크 내 대기)에서 move task 로 도크를 빠져나갈 때
        # (MOVING_TO_PRODUCT 등) 비로소 도크 점유를 해제한다.
        traffic._robot_states['PICKY1'] = 'CHARGING'
        traffic._robot_dock['PICKY1'] = 'CHARGING_DOCK_1'
        traffic.notify_state('PICKY1', 'STANDBY')             # 충전 완료, 도크 유지
        assert traffic._robot_dock['PICKY1'] == 'CHARGING_DOCK_1'
        traffic.notify_state('PICKY1', 'MOVING_TO_PRODUCT')   # 언도크
        assert traffic._robot_dock['PICKY1'] is None

    def test_returning_releases_dock(self, traffic):
        # RETURNING 도 도크를 떠나는 이동 상태라 점유 해제 대상이다.
        traffic._robot_dock['PICKY1'] = 'CHARGING_DOCK_1'
        traffic.notify_state('PICKY1', 'RETURNING')
        assert traffic._robot_dock['PICKY1'] is None


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

    def test_concurrent_return_home_serializes(self, traffic):
        """동시 RETURN_HOME 은 STANDBY_ZONE 공유로 인해 한 쪽만 성공한다. 도크 예약 없음.

        새 맵(2.1)에서 TRAFFIC_T1(상단), TRAFFIC_B1(하단) 양쪽의 최근접 귀환 목적지가
        모두 STANDBY_ZONE_2 라, 동시 호출 시 한 쪽의 path 예약이 다른 쪽을 차단한다.
        """
        results = []

        def worker(rid, task_id, src):
            results.append((rid, traffic.reserve_return_home_path(rid, task_id, src)))

        t1 = threading.Thread(target=worker, args=('PICKY1', 1, 'TRAFFIC_T1'))
        t2 = threading.Thread(target=worker, args=('PICKY2', 2, 'TRAFFIC_B1'))
        t1.start()
        t2.start()
        t1.join()
        t2.join()

        ok_count = sum(1 for _, r in results if r.ok)
        assert ok_count == 1
        # 도크 예약 없음
        assert all(d is None for d in traffic._robot_dock.values())


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


# ──────────────────────────────────────────────────────────────────────
# 새 맵 토폴로지 (docs/Traffic_node_2.1.jpg)
# ──────────────────────────────────────────────────────────────────────


class TestNewMapTopology:
    """좌측 수직 복도(TRAFFIC_L) 제거 같은 2.1 맵 구조의 회귀 방지."""

    def test_stock_zone_exits_through_top_corridor(self, traffic):
        # 2.1: STOCK_ZONE 은 TRAFFIC_T1 과 직접 인접(좌측 복도 L1 제거).
        r = traffic.reserve_path('PICKY1', 1, 'STOCK_ZONE', 'TRAFFIC_T1')
        assert r.ok
        assert r.waypoints == ('STOCK_ZONE', 'TRAFFIC_T1')

    def test_standby_to_product_direct(self, traffic):
        # 2.1: STANDBY_ZONE_2 와 PRODUCT_ZONE_1 이 직접 인접(hop 1, L2 제거).
        r = traffic.reserve_path('PICKY1', 1, 'STANDBY_ZONE_2', 'PRODUCT_ZONE_1')
        assert r.ok
        assert r.waypoints == ('STANDBY_ZONE_2', 'PRODUCT_ZONE_1')

    def test_pickup_zone_adjacent_to_corridor_end(self, traffic):
        # PICKUP_ZONE_1 은 TRAFFIC_T3, PICKUP_ZONE_2 는 TRAFFIC_B3 와 직접 인접.
        r1 = traffic.reserve_path('PICKY1', 1, 'TRAFFIC_T3', 'PICKUP_ZONE_1')
        assert r1.ok and r1.waypoints == ('TRAFFIC_T3', 'PICKUP_ZONE_1')

        r2 = traffic.reserve_path('PICKY2', 2, 'TRAFFIC_B3', 'PICKUP_ZONE_2')
        assert r2.ok and r2.waypoints == ('TRAFFIC_B3', 'PICKUP_ZONE_2')

    def test_removed_nodes_are_unreachable(self, traffic):
        # 옛 우측 복도(TRAFFIC_R1~R4), PICKUP_ZONE_3/_4, 그리고 2.1 에서 제거한
        # 좌측 수직 복도(TRAFFIC_L1~L3) 는 새 그래프에 존재하지 않는다.
        for removed in (
            'TRAFFIC_R1', 'TRAFFIC_R2', 'TRAFFIC_R3', 'TRAFFIC_R4',
            'PICKUP_ZONE_3', 'PICKUP_ZONE_4',
            'TRAFFIC_L1', 'TRAFFIC_L2', 'TRAFFIC_L3',
        ):
            assert removed not in ZONE_GRAPH, f'{removed} 가 새 그래프에 남아 있음'

    def test_stock_to_bottom_via_product_column(self, traffic):
        # 2.1: 좌측 복도가 없으므로 STOCK_ZONE 에서 하단 복도(TB1)로 가려면
        # 상단 복도 -> 상품 1열(PD1 -> PD4) 을 통해 내려간다.
        r = traffic.reserve_path('PICKY1', 1, 'STOCK_ZONE', 'TRAFFIC_B1')
        assert r.ok
        assert r.waypoints == (
            'STOCK_ZONE', 'TRAFFIC_T1', 'PRODUCT_ZONE_1',
            'PRODUCT_ZONE_4', 'TRAFFIC_B1',
        )


# ──────────────────────────────────────────────────────────────────────
# 콜드 스타트 도크 점유 (assume_docked_at_start)
# ──────────────────────────────────────────────────────────────────────


class TestColdStartDockOccupancy:
    def test_default_no_dock_occupancy(self, mock_node):
        # 기본값(False): 도크 점유 없이 시작 (기존 동작 보존)
        tm = TrafficManager(node=mock_node, robot_ids=['PICKY1', 'PICKY2'])
        assert all(d is None for d in tm._robot_dock.values())

    def test_assume_docked_marks_each_robot_dock(self, mock_node):
        # 콜드 스타트(True): robot_ids 순서대로 DOCK_PRIORITY 도크를 점유로 초기화
        tm = TrafficManager(
            node=mock_node, robot_ids=['PICKY1', 'PICKY2'],
            assume_docked_at_start=True,
        )
        assert tm._robot_dock['PICKY1'] == DOCK_PRIORITY[0][0]   # CHARGING_DOCK_1
        assert tm._robot_dock['PICKY2'] == DOCK_PRIORITY[1][0]   # CHARGING_DOCK_2

    def test_returning_robot_avoids_other_robots_dock(self, mock_node):
        # 시나리오: 둘 다 도크에서 시작 → PICKY2 가 작업 나갔다(도크 해제) 복귀.
        # PICKY1 이 dock1 에 그대로 있으므로 PICKY2 는 dock2(STANDBY_ZONE_2)로 귀환해야
        # 한다(dock1 충돌 방지). 콜드 스타트 점유가 없으면 dock1 을 비었다고 오인한다.
        tm = TrafficManager(
            node=mock_node, robot_ids=['PICKY1', 'PICKY2'],
            assume_docked_at_start=True,
        )
        tm.notify_state('PICKY2', 'MOVING_TO_PRODUCT')   # PICKY2 undock → dock2 해제
        assert tm._robot_dock['PICKY2'] is None
        assert tm._robot_dock['PICKY1'] == DOCK_PRIORITY[0][0]   # PICKY1 은 dock1 유지
        r = tm.reserve_return_home_path('PICKY2', 1, 'TRAFFIC_T1')
        assert r.ok
        assert r.waypoints[-1] == 'STANDBY_ZONE_2'       # dock1 점유 → dock2 로 귀환
