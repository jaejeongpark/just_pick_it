"""재시작 복구(R1 / A'') 단위 테스트.

TrafficManager는 실제 객체(BFS/예약 포함), Repo/Node/Gateway는 fake로 대체한다.
실행 예: pytest src/just_pick_it/fleet_manager/test/test_recovery.py -v
"""
from time import monotonic
from unittest.mock import MagicMock

from fleet_manager.task_manager import TaskManager
from fleet_manager.traffic_manager import TrafficManager


def make_node():
    node = MagicMock()
    node.get_logger.return_value = MagicMock()
    return node


class FakeRecoveryRepo:
    """reconcile/resync가 호출하는 최소 Repo fake."""

    def __init__(self, *, recovery=None, emergency=False):
        self._recovery = list(recovery or [])
        self._emergency = emergency

    def list_recovery_tasks(self):
        return [dict(task) for task in self._recovery]

    def has_emergency_robots(self):
        return self._emergency

    def list_tasks(self, status=None, robot_name=None, task_type=None):
        return []  # dispatch 단계에 ASSIGNED 없음

    def list_robots(self):
        return []

    def update_task_target_zone(self, task_id, *, target_zone_name):
        return {"task_id": task_id, "target_zone_name": target_zone_name}


# ----------------------------------------------------------------------
# TrafficManager 복구 헬퍼
# ----------------------------------------------------------------------

def test_nearest_zone_maps_pose_to_graph_node():
    traffic = TrafficManager(make_node(), robot_ids=["PICKY1"])
    assert traffic.nearest_zone(0.11, 0.39) == "STANDBY_ZONE_1"
    assert traffic.nearest_zone(1.41, 0.61) == "PRODUCT_ZONE_3"
    assert traffic.nearest_zone(None, 0.0) is None


def test_nearest_dock_picks_closest_charging_dock():
    traffic = TrafficManager(make_node(), robot_ids=["PICKY1"])
    assert traffic.nearest_dock(0.10, 0.10) == "CHARGING_DOCK_1"
    assert traffic.nearest_dock(0.27, 0.10) == "CHARGING_DOCK_2"
    assert traffic.nearest_dock(None, None) is None


def test_rebuild_dock_blocks_other_robot_from_same_dock():
    traffic = TrafficManager(make_node(), robot_ids=["PICKY1", "PICKY2"])
    traffic.rebuild_dock("PICKY1", "CHARGING_DOCK_1")

    result = traffic.reserve_dock_path("PICKY2", task_id=99, source_zone="STANDBY_ZONE_2")

    assert result.ok
    assert result.waypoints[-1] == "CHARGING_DOCK_2"


# ----------------------------------------------------------------------
# reconcile_on_startup
# ----------------------------------------------------------------------

def test_arm_reconcile_gates_dispatch_until_reconciled():
    traffic = TrafficManager(make_node(), robot_ids=["PICKY1"])
    repo = FakeRecoveryRepo()
    tm = TaskManager(node=make_node(), fleet_repo=repo, traffic_manager=traffic, robot_gateway=MagicMock())

    tm.arm_reconcile()
    assert tm._reconcile_pending is True

    tm.reconcile_on_startup()
    assert tm._reconcile_pending is False


def test_reconcile_rebuilds_move_occupancy_from_current_position():
    traffic = TrafficManager(make_node(), robot_ids=["PICKY1"])
    repo = FakeRecoveryRepo(
        recovery=[
            {
                "task_id": 5,
                "task_type": "MOVE_TO_PRODUCT",
                "robot_name": "PICKY1",
                "target_zone_name": "PICKUP_ZONE_1",
                "source_zone_name": "STANDBY_ZONE_1",
                "pos_x": 1.40,
                "pos_y": 0.60,  # PRODUCT_ZONE_3 근처(이미 source 떠남)
                "picky_state": "MOVING_TO_PICKUP",
            }
        ]
    )
    tm = TaskManager(node=make_node(), fleet_repo=repo, traffic_manager=traffic, robot_gateway=MagicMock())
    tm.arm_reconcile()
    tm.reconcile_on_startup()

    waypoints = tm._move_waypoints_by_task[5]
    assert waypoints[0] == "PRODUCT_ZONE_3"   # stale source가 아닌 현재 위치 기준
    assert waypoints[-1] == "PICKUP_ZONE_1"
    assert 5 in tm._recovering


def test_reconcile_emergency_keeps_gate_and_pauses():
    traffic = TrafficManager(make_node(), robot_ids=["PICKY1"])
    repo = FakeRecoveryRepo(emergency=True)
    tm = TaskManager(node=make_node(), fleet_repo=repo, traffic_manager=traffic, robot_gateway=MagicMock())

    tm.arm_reconcile()
    tm.reconcile_on_startup()

    assert tm._fleet_paused is True
    assert tm._reconcile_pending is False
    assert tm._recovering == {}


def test_reconcile_charge_restores_dock_only():
    traffic = TrafficManager(make_node(), robot_ids=["PICKY1", "PICKY2"])
    repo = FakeRecoveryRepo(
        recovery=[
            {
                "task_id": 9,
                "task_type": "CHARGE",
                "robot_name": "PICKY2",
                "pos_x": 0.27,
                "pos_y": 0.10,  # CHARGING_DOCK_2 근처
                "picky_state": "CHARGING",
            }
        ]
    )
    tm = TaskManager(node=make_node(), fleet_repo=repo, traffic_manager=traffic, robot_gateway=MagicMock())
    tm.arm_reconcile()
    tm.reconcile_on_startup()

    assert traffic._robot_dock["PICKY2"] == "CHARGING_DOCK_2"
    assert 9 not in tm._recovering  # CHARGE는 battery/poll이 완료


# ----------------------------------------------------------------------
# _resync_recovering_tasks (완료 재동기 / 타임아웃)
# ----------------------------------------------------------------------

def _resync_manager(recovery):
    traffic = MagicMock()
    traffic.nearest_zone.return_value = "TRAFFIC_T1"  # target과 불일치 -> 위치 보조판정 False
    repo = FakeRecoveryRepo(recovery=recovery)
    tm = TaskManager(node=make_node(), fleet_repo=repo, traffic_manager=traffic, robot_gateway=MagicMock())
    return tm


def test_resync_marks_arrived_move_success_and_times_out_stalled():
    recovery = [
        {"task_id": 1, "task_type": "MOVE_TO_PRODUCT", "robot_name": "PICKY1",
         "picky_state": "WAITING_FOR_COBOT", "target_zone_name": "PRODUCT_ZONE_3", "pos_x": 1.4, "pos_y": 0.6},
        {"task_id": 2, "task_type": "MOVE_TO_PRODUCT", "robot_name": "PICKY1",
         "picky_state": "MOVING_TO_PRODUCT", "target_zone_name": "PRODUCT_ZONE_3", "pos_x": 0.7, "pos_y": 0.85},
        {"task_id": 3, "task_type": "MOVE_TO_PRODUCT", "robot_name": "PICKY2",
         "picky_state": "MOVING_TO_PRODUCT", "target_zone_name": "PRODUCT_ZONE_3", "pos_x": 0.7, "pos_y": 0.85},
    ]
    tm = _resync_manager(recovery)
    now = monotonic()
    tm._recovering = {1: now + 100, 2: now - 1, 3: now + 100}

    calls = []
    tm._complete_recovered_task = lambda rec, *, success, message: calls.append((rec["task_id"], success))

    tm._resync_recovering_tasks()

    assert calls == [(1, True), (2, False)]   # 도착=SUCCESS, 타임아웃=FAILED
    assert set(tm._recovering) == {3}          # 이동중+미타임아웃은 유지


def test_resync_drops_task_that_is_no_longer_running():
    tm = _resync_manager(recovery=[])  # list_recovery_tasks() == []
    tm._recovering = {42: monotonic() + 100}

    tm._resync_recovering_tasks()

    assert tm._recovering == {}
