"""TaskManager 단위 테스트.

ROS Node/DB/Action server는 fake 객체로 대체해 TaskManager 정책만 검증한다.
실행 예: pytest src/just_pick_it/fleet_manager/test/test_task_manager.py -v
"""
import sys
import types
from unittest.mock import MagicMock

import pytest


sys.modules.setdefault("rclpy", types.ModuleType("rclpy"))
sys.modules.setdefault("rclpy.node", types.SimpleNamespace(Node=object))

from fleet_manager.task_manager import CHARGE_BATTERY_THRESHOLD, TaskManager


class FakeRepo:
    """TaskManager 테스트에 필요한 FleetRepository 최소 fake."""

    def __init__(self, *, tasks=None, orders=None, display_items=None):
        self.tasks = list(tasks or [])
        self.orders = list(orders or [])
        self.display_items = list(display_items or [])
        self.updated_tasks = []
        self.created_tasks = []
        self.exceptions = []

    def list_waiting_orders(self):
        return list(self.orders)

    def list_requested_display_items(self):
        return list(self.display_items)

    def list_order_tasks(self, order_id: int):
        return [task for task in self.tasks if task.get("order_id") == order_id]

    def list_tasks(self, status=None, robot_name=None, task_type=None):
        tasks = self.tasks
        if status is not None:
            tasks = [task for task in tasks if task.get("status") == status]
        if robot_name is not None:
            tasks = [task for task in tasks if task.get("assigned_robot_name") == robot_name]
        if task_type is not None:
            tasks = [task for task in tasks if task.get("task_type") == task_type]
        return list(tasks)

    def update_task_status(self, task_id: int, **kwargs):
        self.updated_tasks.append({"task_id": task_id, **kwargs})
        for task in self.tasks:
            if int(task.get("task_id") or 0) != task_id:
                continue
            current_status = kwargs.get("current_status")
            if current_status is not None and task.get("status") != current_status:
                return None
            if "status" in kwargs:
                task["status"] = kwargs["status"]
            if "assigned_robot_name" in kwargs:
                task["assigned_robot_name"] = kwargs["assigned_robot_name"]
            if "result_message" in kwargs:
                task["result_message"] = kwargs["result_message"]
            return dict(task)
        return None

    def create_exception(self, **kwargs):
        self.exceptions.append(kwargs)
        return kwargs

    def get_zone_map(self):
        names = [
            "STANDBY_ZONE_1",
            "STOCK_ZONE",
            "STOCK_SLOT",
            "PRODUCT_ZONE_1",
            "PRODUCT_SLOT_1",
        ]
        return {name: {"zone_id": index + 1, "zone_name": name} for index, name in enumerate(names)}

    def create_tasks_bulk(self, tasks):
        base_id = len(self.tasks) + 1
        task_ids = []
        for index, task in enumerate(tasks):
            new_task = {"task_id": base_id + index, **task}
            self.created_tasks.append(new_task)
            self.tasks.append(new_task)
            task_ids.append(new_task["task_id"])
        return {"status": "ok", "task_ids": task_ids}


class FakeGateway:
    def __init__(self, *, cobot_send_result=False):
        self.cobot_send_result = cobot_send_result
        self.cobot_calls = []

    def send_cobot_task(
        self,
        *,
        robot_name,
        task,
        feedback_callback=None,
        result_callback=None,
    ):
        self.cobot_calls.append(
            {
                "robot_name": robot_name,
                "task_id": task.get("task_id"),
                "task_type": task.get("task_type"),
                "feedback_callback": feedback_callback,
                "result_callback": result_callback,
            }
        )
        return self.cobot_send_result


@pytest.fixture
def mock_node():
    node = MagicMock()
    node.get_logger.return_value = MagicMock()
    return node


def make_manager(mock_node, repo, *, gateway=None, traffic=None):
    return TaskManager(
        node=mock_node,
        fleet_repo=repo,
        traffic_manager=traffic or MagicMock(),
        robot_gateway=gateway or FakeGateway(),
    )


def test_collect_waiting_work_prioritizes_display_before_default_orders(mock_node):
    repo = FakeRepo(
        orders=[
            {"order_id": 20},
            {"order_id": 30, "priority": 3},
        ],
        display_items=[
            {"display_item_id": 10},
            {"display_item_id": 40, "priority": 5},
        ],
    )
    manager = make_manager(mock_node, repo)

    requests = manager._collect_waiting_work()

    assert [(item.kind, item.work_id, item.priority) for item in requests] == [
        ("DISPLAY", 10, 1),
        ("ORDER", 20, 2),
        ("ORDER", 30, 3),
        ("DISPLAY", 40, 5),
    ]


def test_create_display_tasks_uses_display_scenario_sequence(mock_node):
    repo = FakeRepo()
    manager = make_manager(mock_node, repo)

    task_ids = manager.create_display_tasks_for_item(
        {
            "display_item_id": 7,
            "picky_name": "PICKY1",
            "cobot_name": "COBOT1",
            "source_zone_name": "STANDBY_ZONE_1",
            "stock_zone_name": "STOCK_ZONE",
            "stock_slot_name": "STOCK_SLOT",
            "product_zone_name": "PRODUCT_ZONE_1",
            "product_slot_name": "PRODUCT_SLOT_1",
            "priority": 1,
        }
    )

    assert task_ids == [1, 2, 3, 4, 5]
    assert [task["task_type"] for task in repo.created_tasks] == [
        "MOVE_TO_STOCK",
        "SORTING_AND_LOAD",
        "MOVE_TO_DISPLAY",
        "DISPLAY_SCAN",
        "DISPLAY_PLACE",
    ]
    assert all(task["display_item_id"] == 7 for task in repo.created_tasks)


def test_dispatch_cobot_task_retries_when_gateway_is_not_ready(mock_node):
    task = {
        "task_id": 11,
        "order_id": 1,
        "sequence_no": 1,
        "task_type": "SORTING_AND_LOAD",
        "status": "ASSIGNED",
        "assigned_robot_name": "COBOT1",
    }
    repo = FakeRepo(tasks=[task])
    gateway = FakeGateway(cobot_send_result=False)
    manager = make_manager(mock_node, repo, gateway=gateway)

    manager._dispatch_ready_tasks()
    manager._dispatch_ready_tasks()

    assert [call["task_id"] for call in gateway.cobot_calls] == [11, 11]
    assert repo.updated_tasks == []
    assert task["status"] == "ASSIGNED"


def test_cobot_stowing_feedback_triggers_preplan(mock_node):
    repo = FakeRepo()
    manager = make_manager(mock_node, repo)
    manager.preplan_after_cobot_stowing = MagicMock(return_value=True)

    manager.handle_cobot_feedback({"task_id": 11, "status": "RUNNING"})
    manager.preplan_after_cobot_stowing.assert_not_called()

    manager.handle_cobot_feedback({"task_id": 11, "status": "STOWING_ARM"})
    manager.preplan_after_cobot_stowing.assert_called_once_with(11)


def test_charge_task_completes_only_above_threshold(mock_node):
    task = {
        "task_id": 21,
        "task_type": "CHARGE",
        "status": "RUNNING",
        "assigned_robot_name": "PICKY1",
    }
    repo = FakeRepo(tasks=[task])
    manager = make_manager(mock_node, repo)

    assert not manager._complete_charge_tasks_for_robot("PICKY1", CHARGE_BATTERY_THRESHOLD)
    assert repo.updated_tasks == []

    assert manager._complete_charge_tasks_for_robot("PICKY1", CHARGE_BATTERY_THRESHOLD + 1)
    assert repo.updated_tasks[-1]["status"] == "SUCCESS"
    assert task["status"] == "SUCCESS"


def test_cleanup_finished_flow_memory_waits_until_all_tasks_are_final(mock_node):
    tasks = [
        {"task_id": 1, "order_id": 100, "task_type": "MOVE_TO_PRODUCT", "status": "SUCCESS"},
        {"task_id": 2, "order_id": 100, "task_type": "SORTING_AND_LOAD", "status": "RUNNING"},
        {"task_id": 3, "order_id": 100, "task_type": "MOVE_TO_PICKUP", "status": "ASSIGNED"},
        {"task_id": 99, "order_id": 200, "task_type": "MOVE_TO_PRODUCT", "status": "SUCCESS"},
    ]
    repo = FakeRepo(tasks=tasks)
    manager = make_manager(mock_node, repo)
    manager._move_waypoints_by_task.update({1: ("A", "B"), 2: ("B", "C"), 99: ("X", "Y")})
    manager._completed_move_target_by_task.update({1: "B", 99: "Y"})
    manager._housekeeping_stopped_flows.update({("order", 100), ("order", 200)})
    manager._preplanned_created_tasks_by_trigger[2] = {3}
    manager._preplanned_move_tasks_by_trigger[2] = {3}

    manager._cleanup_finished_flow_memory(tasks[0])

    assert 1 in manager._completed_move_target_by_task
    assert ("order", 100) in manager._housekeeping_stopped_flows
    assert 2 in manager._preplanned_created_tasks_by_trigger

    tasks[1]["status"] = "FAILED"
    tasks[2]["status"] = "CANCELLED"
    manager._cleanup_finished_flow_memory(tasks[0])

    assert 1 not in manager._move_waypoints_by_task
    assert 2 not in manager._move_waypoints_by_task
    assert 1 not in manager._completed_move_target_by_task
    assert ("order", 100) not in manager._housekeeping_stopped_flows
    assert manager._preplanned_created_tasks_by_trigger == {}
    assert manager._preplanned_move_tasks_by_trigger == {}

    assert 99 in manager._move_waypoints_by_task
    assert manager._completed_move_target_by_task[99] == "Y"
    assert ("order", 200) in manager._housekeeping_stopped_flows
