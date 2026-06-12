from unittest.mock import MagicMock

from fleet_manager.task_manager import (
    DEFAULT_DISPLAY_PRIORITY,
    DEFAULT_ORDER_PRIORITY,
    TaskManager,
)


def make_node():
    node = MagicMock()
    node.get_logger.return_value = MagicMock()
    return node


class FakeDisplayAppendRepo:
    def __init__(self, tasks):
        self._tasks = list(tasks)

    def list_tasks(self):
        return [dict(task) for task in self._tasks]


def _manager(tasks) -> TaskManager:
    return TaskManager(
        node=make_node(),
        fleet_repo=FakeDisplayAppendRepo(tasks),
        traffic_manager=MagicMock(),
        robot_gateway=MagicMock(),
    )


def _display_task(sequence_no: int, status: str, task_type: str = "MOVE_TO_DISPLAY"):
    return {
        "task_id": sequence_no,
        "display_batch_id": 10,
        "task_type": task_type,
        "status": status,
        "sequence_no": sequence_no,
    }


def test_display_priority_stays_higher_than_order_priority() -> None:
    assert DEFAULT_DISPLAY_PRIORITY < DEFAULT_ORDER_PRIORITY


def test_appendable_display_batch_allows_not_started_tasks() -> None:
    manager = _manager([
        _display_task(2, "ASSIGNED", "DISPLAY_SCAN"),
        _display_task(1, "ASSIGNED", "MOVE_TO_DISPLAY"),
    ])

    tasks = manager._find_appendable_display_batch_tasks()

    assert [task["sequence_no"] for task in tasks] == [1, 2]


def test_appendable_display_batch_skips_running_batch() -> None:
    manager = _manager([
        _display_task(1, "RUNNING", "MOVE_TO_DISPLAY"),
        _display_task(2, "ASSIGNED", "DISPLAY_SCAN"),
    ])

    assert manager._find_appendable_display_batch_tasks() == []


def test_appendable_display_batch_skips_already_started_batch() -> None:
    manager = _manager([
        _display_task(1, "SUCCESS", "MOVE_TO_DISPLAY"),
        _display_task(2, "ASSIGNED", "DISPLAY_SCAN"),
    ])

    assert manager._find_appendable_display_batch_tasks() == []


def test_appendable_display_batch_skips_housekeeping_batch() -> None:
    manager = _manager([
        _display_task(1, "ASSIGNED", "MOVE_TO_DISPLAY"),
        _display_task(2, "ASSIGNED", "RETURN_HOME"),
    ])

    assert manager._find_appendable_display_batch_tasks() == []
