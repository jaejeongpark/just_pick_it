from __future__ import annotations

from dataclasses import dataclass
import threading
from typing import Any

from rclpy.node import Node

from fleet_manager.fleet_repository import FleetRepository
from fleet_manager.traffic_manager import TrafficManager


MOVE_TASK_TYPES = {
    "MOVE_TO_PRODUCT",
    "MOVE_TO_PICKUP",
    "MOVE_TO_STOCK",
    "MOVE_TO_STORAGE",
    "RETURN_HOME",
}

DOCK_TASK_TYPES = {"DOCK_IN"}
PATH_RESERVED_TASK_TYPES = MOVE_TASK_TYPES | DOCK_TASK_TYPES

HOUSEKEEPING_TASK_TYPES = {
    "RETURN_HOME",
    "DOCK_IN",
    "CHARGE",
}

HOUSEKEEPING_REASON_PARKING = "PARKING"
HOUSEKEEPING_REASON_LOW_BATTERY = "LOW_BATTERY"

COBOT_TASK_TYPES = {
    "SORTING_AND_LOAD",
    "INSPECTION",
    "UNLOAD",
    "STOCKING_PICK",
    "STOCKING_PLACE",
}

FINAL_TASK_STATUSES = {"SUCCESS", "FAILED", "CANCELLED"}
CHARGE_BATTERY_THRESHOLD = 40
DEFAULT_ORDER_PRIORITY = 2
DEFAULT_STOCKING_PRIORITY = 1


@dataclass(frozen=True)
class WorkRequest:
    """TaskManager가 처리할 주문/입고 대기 작업 1건."""

    kind: str
    work_id: int
    priority: int
    payload: dict[str, Any]


class TaskManager:
    """Fleet Manager 내부 task 생성/상태 전이 담당 클래스.

    역할:
    - DB를 polling해서 ORDER_WAIT 주문과 REQUESTED 입고 요청을 찾는다.
    - 사용 가능한 robot unit을 배정한다.
    - 주문/입고 데이터를 task payload로 변환한다.
    - TrafficManager와 협업해 PICKY 이동 경로를 예약한다.
    - task 상태 변경과 실패를 DB에 보고한다.

    주의:
    - 이 클래스는 ROS2 Node가 아니다. FleetManagerNode를 주입받아 logger만 사용한다.
    - 경로 탐색 자체는 TrafficManager 책임이다.
    - DB 접근 세부사항은 FleetRepository 책임이다.
    - ROS2 Action 송신은 RobotCommandGateway 책임이다.
    """

    def __init__(
        self,
        node: Node,
        fleet_repo: FleetRepository,
        traffic_manager: TrafficManager,
        robot_gateway: Any | None = None,
    ) -> None:
        """TaskManager를 초기화한다.

        Args:
            node: 로그 출력을 위해 공유받는 FleetManagerNode.
            fleet_repo: DB 접근 Repository.
            traffic_manager: 경로 탐색/예약 담당 객체.
            robot_gateway: ROS2 Action 송신 담당 객체. 초기 task 생성 단계에서는 None 가능.
        """
        self._node = node
        self._repo = fleet_repo
        self._traffic = traffic_manager
        self._robot_gateway = robot_gateway
        self._scheduler_lock = threading.RLock()
        self._fleet_paused = False

        # task_id -> TrafficManager가 예약한 zone 목록.
        # 상세 경로는 충돌 회피/예약용이며, MoveCommand에는 최종 목적지 zone만 보낸다.
        self._move_waypoints_by_task: dict[int, tuple[str, ...]] = {}
        self._completed_move_target_by_task: dict[int, str] = {}
        self._unsupported_task_warned: set[int] = set()
        self._housekeeping_stopped_flows: set[tuple[str, int]] = set()

        # cobot_task_id -> STOWING_ARM 중 미리 생성한 다음 task id 목록.
        # predecessor COBOT task가 실패하면 이 task들은 CANCELLED 처리한다.
        self._preplanned_created_tasks_by_trigger: dict[int, set[int]] = {}

        # cobot_task_id -> STOWING_ARM 중 미리 예약한 다음 MOVE task id 목록.
        # 기존에 이미 DB에 있던 MOVE task를 pre-reserve한 경우도 포함한다.
        self._preplanned_move_tasks_by_trigger: dict[int, set[int]] = {}

    # ==================================================================
    # 신규 주문/입고 확인 진입점
    # ==================================================================

    def has_idle_picky_for_waiting_work(self) -> bool:
        """대기 작업 polling을 열어도 되는 PICKY가 있는지 확인한다.

        호출 주기는 FleetManagerNode timer가 관리하지만, 실제 polling window는
        PICKY가 새 작업을 받을 수 있는 IDLE/STANDBY 상태일 때만 열린다.
        COBOT 가능 여부는 실제 배정 단계에서 `_select_available_unit()`이 다시 확인한다.
        """
        if self._fleet_paused:
            return False

        for robot in self._repo.list_robots():
            if robot.get("robot_type") != "PICKY":
                continue
            if not self._picky_idle_for_waiting_work(robot):
                continue
            return True

        return False

    def check_waiting_work(self) -> None:
        """대기 중인 작업을 확인하고 받을 수 있으면 바로 시작한다.

        여기서 말하는 대기 작업은 두 종류다.
        - 아직 task가 없는 신규 `ORDER_WAIT` 주문 / `REQUESTED` 입고
        - 경로 차단 등으로 다음 task를 못 만들고 멈춰 있는 기존 주문 / 입고 flow

        처리 순서:
        0. 충전 완료 조건을 만족한 CHARGE task 정리
        1. 이미 시작된 주문/입고 flow의 다음 task 생성 또는 막힌 flow 재시도
        2. 새 작업을 받을 수 있는 unit이 있으면 ORDER_WAIT/REQUESTED를 priority queue로 처리
        3. 새로 생성됐거나 이미 ASSIGNED 상태인 실행 가능 task를 dispatch

        새 작업 priority:
        - 숫자가 낮을수록 먼저 처리한다.
        - 입고는 기본 priority=1, 주문은 기본 priority=2로 둔다.
        - 이미 시작된 flow는 검수/하차 또는 입고 place까지 끊지 않고 이어간다.

        정상 task 연결은 handle_task_result()에서 즉시 처리한다. 이 함수는 기존 task를
        진행시키는 메인 루프가 아니라, 새 작업과 막혀 있던 작업을 다시 확인하는 polling 진입점이다.

        재진입 방지:
        - HTTP 요청이 길어져 이전 확인 작업이 끝나기 전에 다음 호출이 들어올 수 있다.
        - `_scheduler_lock`으로 중복 진입을 막는다.
        """
        if not self._scheduler_lock.acquire(blocking=False):
            self._node.get_logger().debug("[TaskManager] previous waiting-work check is still running")
            return

        try:
            self._complete_ready_charge_tasks()
            self._advance_existing_orders()
            self._advance_existing_stocking_items()
            self._process_waiting_work_if_unit_available()
            self._dispatch_ready_tasks()
        finally:
            self._scheduler_lock.release()

    def handle_emergency_stop(self) -> None:
        """Fleet emergency stop 수신 시 신규 dispatch를 막는다.

        Fleet API가 DB의 robot/task 상태 전이는 담당한다. TaskManager는
        emergency 상태 동안 polling이나 task result 후속 처리에서 새 task를
        로봇으로 보내지 않도록 내부 gate만 닫는다.
        """
        with self._scheduler_lock:
            self._fleet_paused = True

    def handle_resume(self) -> None:
        """Fleet resume 수신 시 실행 가능한 task 흐름을 즉시 재개한다.

        Fleet API가 PAUSED task를 ASSIGNED로 되돌리는 정책이면 여기서 바로
        dispatch된다. 이미 RUNNING으로 복구되는 정책이면 로봇 emergency service
        해제 후 기존 action이 이어지고, 이 함수는 대기 작업/누락 보정만 수행한다.
        """
        with self._scheduler_lock:
            self._fleet_paused = False
            self._complete_ready_charge_tasks()
            self._advance_existing_orders()
            self._advance_existing_stocking_items()
            self._process_waiting_work_if_unit_available()
            self._dispatch_ready_tasks()

    # ==================================================================
    # COBOT STOWING_ARM lookahead planning
    # ==================================================================

    def preplan_after_cobot_stowing(self, cobot_task_id: int) -> bool:
        """COBOT이 STOWING_ARM에 들어간 시점에 다음 이동 task를 미리 준비한다.

        실제 로봇 기준:
        - COBOT 작업이 끝나도 로봇팔이 기본 자세로 복귀하기 전까지 PICKY는 움직이면 안 된다.
        - 하지만 STOWING_ARM 동안 다음 PICKY 경로 예약과 task 생성은 미리 해둘 수 있다.
        - 다음 MOVE task는 DB에 ASSIGNED로 만들어두되, sequence gate 때문에 현재 COBOT
          task가 SUCCESS 되기 전에는 dispatch되지 않는다.

        실패 보상:
        - trigger COBOT task가 FAILED/CANCELLED 되면 handle_task_result()가 미리 만든
          task를 CANCELLED 처리하고 TrafficManager 예약을 해제한다.
        """
        if not self._scheduler_lock.acquire(blocking=False):
            self._node.get_logger().debug(
                f"[TaskManager] task_id={cobot_task_id} preplan skip: scheduler/preplan 진행 중"
            )
            return False

        try:
            return self._preplan_after_cobot_stowing_locked(cobot_task_id)
        finally:
            self._scheduler_lock.release()

    def _preplan_after_cobot_stowing_locked(self, cobot_task_id: int) -> bool:
        """lock을 잡은 상태에서 COBOT STOWING_ARM preplan을 수행한다."""
        if cobot_task_id in self._preplanned_created_tasks_by_trigger:
            return False
        if cobot_task_id in self._preplanned_move_tasks_by_trigger:
            return False

        task = self._find_task_by_id(cobot_task_id)
        if task is None:
            self._node.get_logger().warn(
                f"[TaskManager] preplan 실패: task_id={cobot_task_id} 조회 불가"
            )
            return False

        task_type = task.get("task_type")
        if task_type not in COBOT_TASK_TYPES:
            return False
        if task.get("status") != "RUNNING":
            self._node.get_logger().debug(
                f"[TaskManager] preplan skip: task_id={cobot_task_id} status={task.get('status')}"
            )
            return False

        if task_type == "SORTING_AND_LOAD":
            return self._preplan_after_sorting_and_load(task)

        if task_type == "STOCKING_PICK":
            return self._pre_reserve_next_existing_move_task(task)

        if task_type in ("INSPECTION", "UNLOAD", "STOCKING_PLACE"):
            self._node.get_logger().debug(
                f"[TaskManager] task_id={cobot_task_id} {task_type} 이후 preplan 대상 이동 없음"
            )
            return False

        return False

    def _preplan_after_sorting_and_load(self, task: dict[str, Any]) -> bool:
        """SORTING_AND_LOAD의 STOWING_ARM 중 다음 상품 또는 pickup task를 만든다."""
        order_id = task.get("order_id")
        if order_id is None:
            return False

        order_id = int(order_id)
        tasks = self._repo.list_order_tasks(order_id)
        if self._has_task_after_sequence(task, tasks):
            return self._pre_reserve_next_existing_move_task(task)

        order_work = self._repo.get_order_work(order_id)
        if order_work is None:
            return False

        if not order_work.get("picky_name") or not order_work.get("cobot_name"):
            self._node.get_logger().warn(
                f"[TaskManager] order_id={order_id} preplan 실패: assigned robot 이름 없음"
            )
            return False

        current_zone = task.get("target_zone_name") or self._last_picky_target_zone(tasks)
        if current_zone is None:
            current_zone = self._default_source_zone(int(order_work.get("assigned_unit_id") or 1))

        existing_order_item_ids = {
            int(item["order_item_id"])
            for item in tasks
            if item.get("order_item_id") is not None
        }
        remaining_items = [
            item for item in order_work["items"]
            if item.get("status") in (None, "WAITING")
            and item.get("order_item_id") is not None
            and int(item["order_item_id"]) not in existing_order_item_ids
        ]

        before_task_ids = {int(item["task_id"]) for item in tasks if item.get("task_id") is not None}
        trigger_task_id = int(task["task_id"])

        if remaining_items:
            order_work["items"] = remaining_items
            next_sequence_no = max(int(item.get("sequence_no") or 0) for item in tasks) + 1
            created_ids = self._create_next_product_tasks(
                order_work,
                current_zone=str(current_zone),
                base_sequence_no=next_sequence_no,
            )
        elif not self._has_pickup_tasks(tasks):
            created_ids = self._create_pickup_tasks(
                order_work,
                current_zone=str(current_zone),
                existing_tasks=tasks,
            )
        else:
            return False

        created_set = set(created_ids) - before_task_ids
        if not created_set:
            return False

        self._preplanned_created_tasks_by_trigger[trigger_task_id] = created_set
        self._preplanned_move_tasks_by_trigger[trigger_task_id] = {
            task_id for task_id in created_set if task_id in self._move_waypoints_by_task
        }
        self._node.get_logger().info(
            f"[TaskManager] task_id={trigger_task_id} STOWING_ARM preplan 완료: {sorted(created_set)}"
        )
        return True

    def _pre_reserve_next_existing_move_task(self, task: dict[str, Any]) -> bool:
        """이미 생성된 다음 MOVE task의 TrafficManager 경로를 미리 예약한다."""
        next_task = self._find_next_task(task)
        if next_task is None:
            return False
        if next_task.get("task_type") not in MOVE_TASK_TYPES:
            return False
        if next_task.get("status") != "ASSIGNED":
            return False

        next_task_id = int(next_task["task_id"])
        if next_task_id in self._move_waypoints_by_task:
            return False
        if not self._reserve_move_path_for_task(next_task):
            return False

        trigger_task_id = int(task["task_id"])
        self._preplanned_move_tasks_by_trigger.setdefault(trigger_task_id, set()).add(next_task_id)
        self._node.get_logger().info(
            f"[TaskManager] task_id={trigger_task_id} STOWING_ARM 다음 MOVE 경로 선예약: {next_task_id}"
        )
        return True

    def _has_task_after_sequence(
        self,
        task: dict[str, Any],
        tasks: list[dict[str, Any]],
    ) -> bool:
        """같은 주문/입고 흐름에 현재 task 이후 task가 이미 있는지 확인한다."""
        sequence_no = int(task.get("sequence_no") or 0)
        return any(int(item.get("sequence_no") or 0) > sequence_no for item in tasks)

    def _find_next_task(self, task: dict[str, Any]) -> dict[str, Any] | None:
        """같은 주문/입고 흐름에서 현재 task 다음 task를 찾는다."""
        sequence_no = int(task.get("sequence_no") or 0)
        order_id = task.get("order_id")
        stocking_item_id = task.get("stocking_item_id")

        if order_id is not None:
            tasks = self._repo.list_order_tasks(int(order_id))
        elif stocking_item_id is not None:
            tasks = [
                item for item in self._repo.list_tasks()
                if item.get("stocking_item_id") == stocking_item_id
            ]
        else:
            return None

        later_tasks = [
            item for item in tasks
            if int(item.get("sequence_no") or 0) > sequence_no
        ]
        if not later_tasks:
            return None

        later_tasks.sort(key=lambda item: (int(item.get("sequence_no") or 0), int(item.get("task_id") or 0)))
        return later_tasks[0]

    def _find_task_by_id(self, task_id: int) -> dict[str, Any] | None:
        """Fleet API task 목록에서 task_id 하나를 찾는다."""
        for task in self._repo.list_tasks():
            if int(task.get("task_id") or 0) == task_id:
                return task
        return None

    def _cancel_preplanned_after_cobot_failure(self, cobot_task_id: int) -> None:
        """COBOT task 실패 시 STOWING_ARM 중 선계획한 task와 경로 예약을 정리한다."""
        move_task_ids = self._preplanned_move_tasks_by_trigger.pop(cobot_task_id, set())
        created_task_ids = self._preplanned_created_tasks_by_trigger.pop(cobot_task_id, set())

        if not move_task_ids and not created_task_ids:
            return

        tasks_by_id = {
            int(task["task_id"]): task
            for task in self._repo.list_tasks()
            if task.get("task_id") is not None
        }

        for move_task_id in move_task_ids:
            task = tasks_by_id.get(move_task_id)
            robot_name = task.get("assigned_robot_name") if task else None
            if robot_name:
                self._traffic.release_path(str(robot_name), move_task_id)
            self._move_waypoints_by_task.pop(move_task_id, None)

        for created_task_id in created_task_ids:
            task = tasks_by_id.get(created_task_id)
            if not task:
                continue
            if task.get("status") in FINAL_TASK_STATUSES:
                continue
            self._repo.update_task_status(
                created_task_id,
                status="CANCELLED",
                current_status=str(task.get("status") or "ASSIGNED"),
                assigned_robot_name=task.get("assigned_robot_name"),
                result_message="Cancelled because predecessor COBOT task failed during STOWING_ARM",
            )

    # ==================================================================
    # 주문 polling / 중복 방지
    # ==================================================================

    def _process_waiting_work_if_unit_available(self) -> None:
        """작업 가능한 unit이 있을 때만 신규 주문/입고 polling을 수행한다.

        scheduler cycle 자체는 주기적으로 호출될 수 있지만, 모든 PICKY/COBOT unit이 BUSY이거나
        배터리/housekeeping 조건 때문에 새 작업을 받을 수 없으면 Fleet API의
        ORDER_WAIT/REQUESTED 목록을 조회하지 않는다.

        단, 기존 task advance/dispatch/charge fallback 정리는 cycle 뒤쪽에서 수행할 수 있으므로
        cycle 전체를 return하지 않고 신규 작업 polling만 skip한다.
        """
        if self._fleet_paused:
            self._node.get_logger().debug(
                "[TaskManager] 신규 주문/입고 polling skip: fleet emergency paused"
            )
            return

        if not self._has_available_unit_for_new_work():
            self._node.get_logger().debug(
                "[TaskManager] 신규 주문/입고 polling skip: 작업 가능한 robot unit 없음"
            )
            return

        self._process_waiting_work()

    def _picky_idle_for_waiting_work(self, robot: dict[str, Any]) -> bool:
        """PICKY가 대기 작업 polling을 시작할 수 있는 IDLE 상태인지 확인한다."""
        if robot.get("robot_status") != "IDLE":
            return False
        if robot.get("picky_state") not in (None, "STANDBY"):
            return False
        if robot.get("current_task_id") is not None:
            return False

        robot_name = robot.get("robot_name")
        if not robot_name:
            return False
        if self._robot_has_open_task(str(robot_name)):
            return False
        if self._robot_has_pending_low_battery_housekeeping(str(robot_name)):
            return False
        if not self._picky_has_work_battery(robot):
            return False

        return True

    def _has_available_unit_for_new_work(self) -> bool:
        """신규 주문/입고를 받을 수 있는 robot unit이 하나라도 있는지 확인한다.

        `_select_available_unit()`은 실제 배정 직전에 PARKING RETURN_HOME을 취소하는
        side effect가 있다. scheduler cycle 초반 guard에서는 순수 확인만 필요하므로 별도 helper로 둔다.
        """
        robots = self._repo.list_robots()
        units: dict[int, dict[str, Any]] = {}

        for robot in robots:
            unit_id = robot.get("unit_id")
            robot_type = robot.get("robot_type")
            if unit_id is None or robot_type not in ("PICKY", "COBOT"):
                continue

            unit = units.setdefault(int(unit_id), {"unit_id": int(unit_id)})
            if robot_type == "PICKY":
                unit["picky"] = robot
            else:
                unit["cobot"] = robot

        for unit in units.values():
            picky = unit.get("picky")
            cobot = unit.get("cobot")
            if not self._robot_available(picky) or not self._robot_available(cobot):
                continue
            if self._robot_has_open_task(picky["robot_name"]) or self._robot_has_open_task(cobot["robot_name"]):
                continue
            return True

        return False

    def _process_waiting_work(self) -> None:
        """ORDER_WAIT 주문과 REQUESTED 입고 요청을 priority queue 순서로 처리한다."""
        for request in self._collect_waiting_work():
            if request.kind == "ORDER":
                self._process_new_order(request.payload)
            elif request.kind == "STOCKING":
                self._process_new_stocking_item(request.payload)

    def _collect_waiting_work(self) -> list[WorkRequest]:
        """아직 task가 없는 주문/입고 요청을 priority 기준 대기열로 모은다.

        현재 DB schema에서는 주문만 priority 컬럼을 가진다. 입고는 운영 정책상
        주문보다 높은 우선순위로 보고 기본 priority=1로 둔다.
        """
        requests: list[WorkRequest] = []

        for order in self._repo.list_waiting_orders():
            order_id = order.get("order_id")
            if order_id is None:
                self._node.get_logger().warn(f"[TaskManager] order_id 없는 주문 skip: {order}")
                continue
            if self._repo.list_order_tasks(int(order_id)):
                self._node.get_logger().debug(
                    f"[TaskManager] order_id={order_id} 기존 task 존재, 생성 skip"
                )
                continue

            requests.append(
                WorkRequest(
                    kind="ORDER",
                    work_id=int(order_id),
                    priority=int(order.get("priority") or DEFAULT_ORDER_PRIORITY),
                    payload=order,
                )
            )

        for stocking_item in self._repo.list_requested_stocking_items():
            stocking_item_id = stocking_item.get("stocking_item_id")
            if stocking_item_id is None:
                continue
            if self._stocking_item_has_tasks(int(stocking_item_id)):
                self._node.get_logger().debug(
                    f"[TaskManager] stocking_item_id={stocking_item_id} 기존 task 존재, 생성 skip"
                )
                continue

            requests.append(
                WorkRequest(
                    kind="STOCKING",
                    work_id=int(stocking_item_id),
                    priority=int(stocking_item.get("priority") or DEFAULT_STOCKING_PRIORITY),
                    payload=stocking_item,
                )
            )

        requests.sort(key=lambda item: (item.priority, item.work_id, item.kind))
        return requests

    def _process_waiting_orders(self) -> None:
        """호환용 wrapper. 신규 구현은 _process_waiting_work()를 사용한다."""
        for request in self._collect_waiting_work():
            if request.kind == "ORDER":
                self._process_new_order(request.payload)

    def _process_new_order(self, order: dict[str, Any]) -> None:
        """주문 1건을 robot unit에 배정하고 첫 상품 task를 생성한다."""
        order_id = int(order["order_id"])
        unit = self._select_available_unit()

        if unit is None:
            self._node.get_logger().info(
                f"[TaskManager] order_id={order_id} 배정 가능한 robot unit 없음"
            )
            return

        assigned = self._repo.update_order_status(
            order_id,
            assigned_unit_id=unit["unit_id"],
        )
        if assigned is None:
            self._node.get_logger().warn(
                f"[TaskManager] order_id={order_id} robot unit 배정 기록 실패"
            )
            return

        order_work = self._repo.get_order_work(order_id)
        if order_work is None:
            self._node.get_logger().warn(
                f"[TaskManager] order_id={order_id} order_work 정규화 실패"
            )
            return

        order_work["assigned_unit_id"] = unit["unit_id"]
        order_work["picky_name"] = unit["picky_name"]
        order_work["cobot_name"] = unit["cobot_name"]

        self._create_next_product_tasks(order_work, current_zone=unit["source_zone"])

    # ==================================================================
    # Robot unit 배정
    # ==================================================================

    def _select_available_unit(self) -> dict[str, Any] | None:
        """작업 가능한 PICKY/COBOT pair를 선택한다.

        초기 정책:
        - 같은 unit_id를 가진 PICKY와 COBOT을 한 작업 단위로 본다.
        - 둘 다 robot_status=IDLE이고 current_task_id가 없어야 배정 가능하다.
        - 후보가 여러 개면 PICKY battery_level이 높은 unit을 우선한다.
        """
        robots = self._repo.list_robots()
        units: dict[int, dict[str, Any]] = {}

        for robot in robots:
            unit_id = robot.get("unit_id")
            robot_type = robot.get("robot_type")
            if unit_id is None or robot_type not in ("PICKY", "COBOT"):
                continue

            unit = units.setdefault(int(unit_id), {"unit_id": int(unit_id)})
            if robot_type == "PICKY":
                unit["picky"] = robot
            else:
                unit["cobot"] = robot

        candidates: list[dict[str, Any]] = []
        for unit in units.values():
            picky = unit.get("picky")
            cobot = unit.get("cobot")
            if not self._robot_available(picky) or not self._robot_available(cobot):
                continue
            if self._robot_has_open_task(picky["robot_name"]) or self._robot_has_open_task(cobot["robot_name"]):
                continue

            candidates.append(
                {
                    "unit_id": unit["unit_id"],
                    "picky_name": picky["robot_name"],
                    "cobot_name": cobot["robot_name"],
                    "battery_level": picky.get("battery_level") or 0,
                    "source_zone": self._last_robot_target_zone(picky["robot_name"])
                    or self._default_source_zone(unit["unit_id"]),
                }
            )

        if not candidates:
            return None

        candidates.sort(key=lambda item: (-item["battery_level"], item["unit_id"]))
        selected = candidates[0]
        self._cancel_preemptible_return_home(selected["picky_name"])
        return selected

    def _robot_available(self, robot: dict[str, Any] | None) -> bool:
        """로봇이 신규 작업을 받을 수 있는지 확인한다."""
        if robot is None:
            return False
        current_task = self._robot_current_task(robot)
        if robot.get("robot_status") != "IDLE" and not self._is_preemptible_return_home(current_task):
            return False

        if current_task is not None and not self._is_preemptible_return_home(current_task):
            return False
        if robot.get("robot_type") == "PICKY":
            robot_name = robot.get("robot_name")
            if robot_name and self._robot_has_pending_low_battery_housekeeping(str(robot_name)):
                return False
            if not self._picky_has_work_battery(robot):
                return False
        return True

    def _picky_has_work_battery(self, robot: dict[str, Any]) -> bool:
        """PICKY가 신규 주문/입고 작업을 받을 만큼 배터리가 있는지 확인한다.

        정책:
        - battery_level이 없으면 아직 상태 연동 전으로 보고 배정을 허용한다.
        - battery_level이 40 이하이면 신규 작업을 받지 않고 복귀/충전 대상으로 본다.
        """
        battery_level = robot.get("battery_level")
        if battery_level is None:
            return True
        return int(battery_level) > CHARGE_BATTERY_THRESHOLD

    def _robot_has_open_task(self, robot_name: str) -> bool:
        """해당 로봇에 아직 끝나지 않은 task가 있는지 확인한다.

        robot_status가 아직 IDLE로 보이더라도 ASSIGNED task가 이미 있으면
        같은 polling cycle에서 중복 배정하지 않는다.
        단, RETURN_HOME은 새 주문/입고가 들어오면 선점 취소 가능한 housekeeping task로 본다.
        """
        tasks = self._repo.list_tasks(robot_name=robot_name)
        return any(
            task.get("status") not in FINAL_TASK_STATUSES
            and not self._is_preemptible_return_home(task)
            for task in tasks
        )

    def _unit_has_other_open_task(
        self,
        *,
        picky_name: str,
        cobot_name: str,
        order_id: int | None = None,
        stocking_item_id: int | None = None,
    ) -> bool:
        """같은 unit에 현재 flow가 아닌 미완료 task가 있는지 확인한다.

        같은 scheduler cycle 안에서 `ORDER_WAIT` 신규 배정과 기존 주문 advance가 같이 돌면
        DB robot_status가 아직 IDLE처럼 보여 같은 PICKY에 다른 주문의 path 예약이
        겹칠 수 있다. task 묶음 생성 직전에 한 번 더 막는다.
        """
        for robot_name in (picky_name, cobot_name):
            for task in self._repo.list_tasks(robot_name=robot_name):
                if task.get("status") in FINAL_TASK_STATUSES:
                    continue
                if self._is_preemptible_return_home(task):
                    continue
                if order_id is not None and task.get("order_id") == order_id:
                    continue
                if stocking_item_id is not None and task.get("stocking_item_id") == stocking_item_id:
                    continue
                return True

        return False

    def _robot_has_pending_low_battery_housekeeping(self, robot_name: str) -> bool:
        """LOW_BATTERY 복귀 체인이 CHARGE SUCCESS 전인지 확인한다.

        LOW_BATTERY 사유로 RETURN_HOME을 시작한 뒤에는 DOCK_IN/CHARGE까지
        이어져야 한다. 중간에 DB robot_status가 IDLE로 보이더라도 신규 작업을
        배정하지 않기 위한 보호 장치다.
        """
        tasks_by_flow: dict[tuple[str, int], list[dict[str, Any]]] = {}

        for task in self._repo.list_tasks(robot_name=robot_name):
            if task.get("task_type") not in HOUSEKEEPING_TASK_TYPES:
                continue
            if self._housekeeping_reason(task) != HOUSEKEEPING_REASON_LOW_BATTERY:
                continue

            flow_key = self._flow_key_for_task(task)
            if flow_key is None:
                continue
            tasks_by_flow.setdefault(flow_key, []).append(task)

        for tasks in tasks_by_flow.values():
            last_task = self._last_housekeeping_task(tasks)
            if last_task is None:
                continue
            if last_task.get("task_type") == "CHARGE" and last_task.get("status") == "SUCCESS":
                continue
            return True

        return False

    def _robot_current_task(self, robot: dict[str, Any]) -> dict[str, Any] | None:
        """robot.current_task_id에 해당하는 task summary를 찾는다."""
        current_task_id = robot.get("current_task_id")
        if current_task_id is None:
            return None
        return self._find_task_by_id(int(current_task_id))

    def _is_preemptible_return_home(self, task: dict[str, Any] | None) -> bool:
        """새 작업 배정 시 취소할 수 있는 RETURN_HOME task인지 확인한다."""
        if task is None:
            return False
        return (
            task.get("task_type") == "RETURN_HOME"
            and task.get("status") in {"ASSIGNED", "RUNNING"}
            and self._housekeeping_reason(task) == HOUSEKEEPING_REASON_PARKING
        )

    def _cancel_preemptible_return_home(self, robot_name: str) -> None:
        """새 주문/입고 배정 직전 진행 중인 RETURN_HOME을 취소한다.

        PARKING 사유의 RETURN_HOME만 취소한다.
        LOW_BATTERY 사유의 RETURN_HOME은 충전을 우선해야 하므로 선점하지 않는다.
        """
        for task in self._repo.list_tasks(robot_name=robot_name):
            if not self._is_preemptible_return_home(task):
                continue

            task_id = int(task["task_id"])
            current_status = str(task.get("status") or "ASSIGNED")
            if current_status == "RUNNING" and self._robot_gateway is not None:
                self._robot_gateway.cancel_task(robot_name, task_id)

            self._repo.update_task_status(
                task_id,
                status="CANCELLED",
                current_status=current_status,
                assigned_robot_name=robot_name,
                result_message=self._with_housekeeping_reason(
                    "RETURN_HOME preempted by new work",
                    task,
                ),
            )
            self._traffic.release_path(robot_name, task_id)
            self._move_waypoints_by_task.pop(task_id, None)
            self._mark_housekeeping_stopped_for_task(task)

    def _last_robot_target_zone(self, robot_name: str) -> str | None:
        """해당 PICKY가 마지막으로 성공한 이동 task의 target zone을 반환한다."""
        tasks = [
            task for task in self._repo.list_tasks(robot_name=robot_name)
            if task.get("task_type") in MOVE_TASK_TYPES
            and task.get("status") == "SUCCESS"
            and task.get("target_zone_name")
        ]
        if not tasks:
            return None

        tasks.sort(key=lambda item: int(item.get("task_id") or 0), reverse=True)
        return str(tasks[0]["target_zone_name"])

    def _default_source_zone(self, unit_id: int) -> str:
        """robot unit의 초기 출발 zone을 반환한다.

        실제 현재 zone 추적이 붙기 전까지는 seed 기준 standby zone을 사용한다.
        """
        if unit_id == 1:
            return "STANDBY_ZONE_1"
        if unit_id == 2:
            return "STANDBY_ZONE_2"
        return "STANDBY_ZONE_1"

    # ==================================================================
    # 주문 상품 task 생성
    # ==================================================================

    def _create_next_product_tasks(
        self,
        order_work: dict[str, Any],
        *,
        current_zone: str,
        base_sequence_no: int = 1,
    ) -> list[int]:
        """남은 상품 후보 중 TrafficManager가 고른 상품 task 2개를 생성한다."""
        order_id = int(order_work["order_id"])
        picky_name = order_work["picky_name"]
        cobot_name = order_work["cobot_name"]

        if self._unit_has_other_open_task(
            picky_name=picky_name,
            cobot_name=cobot_name,
            order_id=order_id,
        ):
            self._node.get_logger().debug(
                f"[TaskManager] order_id={order_id} 다음 상품 task 생성 보류: unit이 다른 작업 수행 중"
            )
            return []

        self._cancel_preemptible_return_home(picky_name)

        zone_to_items = self._group_items_by_product_zone(order_work["items"])
        candidates = {
            zone_name: sum(int(item.get("quantity") or 0) for item in items)
            for zone_name, items in zone_to_items.items()
        }

        if not candidates:
            self._node.get_logger().warn(
                f"[TaskManager] order_id={order_work.get('order_id')} 상품 후보 없음"
            )
            return []

        result = self._traffic.reserve_nearest_from(
            robot_id=picky_name,
            task_id=None,
            source_zone=current_zone,
            candidates=candidates,
        )

        if not result.ok:
            self._node.get_logger().info(
                f"[TaskManager] order_id={order_work.get('order_id')} 상품 경로 예약 실패: {result.reason}"
            )
            return []

        selected_zone = result.waypoints[-1]
        selected_item = self._select_item_for_zone(zone_to_items, selected_zone)
        if selected_item is None:
            self._traffic.release_path(picky_name, None)
            self._node.get_logger().warn(
                f"[TaskManager] selected_zone={selected_zone}에 매칭되는 item 없음"
            )
            return []

        tasks = self._build_order_product_task_payloads(
            order_work,
            selected_item,
            current_zone_name=current_zone,
            target_zone_name=selected_zone,
            base_sequence_no=base_sequence_no,
        )
        result_data = self._repo.create_tasks_bulk(tasks)
        if result_data is None:
            self._traffic.release_path(picky_name, None)
            return []

        task_ids = [int(task_id) for task_id in result_data.get("task_ids", [])]
        if len(task_ids) < 2:
            self._traffic.release_path(picky_name, None)
            self._node.get_logger().warn(
                f"[TaskManager] order_id={order_work.get('order_id')} task_ids 부족: {task_ids}"
            )
            return task_ids

        move_task_id = task_ids[0]
        if not self._traffic.attach_task_id(picky_name, move_task_id):
            self._node.get_logger().warn(
                f"[TaskManager] task_id={move_task_id} Traffic 예약 연결 실패"
            )
            return task_ids

        self._move_waypoints_by_task[move_task_id] = tuple(result.waypoints)
        self._node.get_logger().info(
            f"[TaskManager] order_id={order_work.get('order_id')} "
            f"{selected_zone} task 생성 완료: {task_ids}"
        )
        return task_ids

    def _group_items_by_product_zone(
        self,
        items: list[dict[str, Any]],
    ) -> dict[str, list[dict[str, Any]]]:
        """order item 목록을 PRODUCT_ZONE 기준으로 묶는다."""
        grouped: dict[str, list[dict[str, Any]]] = {}

        for item in items:
            if item.get("status") not in (None, "WAITING"):
                continue

            zone_name = item.get("product_zone_name")
            if not zone_name:
                continue

            grouped.setdefault(zone_name, []).append(item)

        return grouped

    def _select_item_for_zone(
        self,
        zone_to_items: dict[str, list[dict[str, Any]]],
        selected_zone: str,
    ) -> dict[str, Any] | None:
        """TrafficManager가 선택한 zone에서 이번에 처리할 item 하나를 고른다."""
        items = zone_to_items.get(selected_zone) or []
        if not items:
            return None
        return items[0]

    def _build_order_product_task_payloads(
        self,
        order_work: dict[str, Any],
        item: dict[str, Any],
        *,
        current_zone_name: str,
        target_zone_name: str,
        base_sequence_no: int,
    ) -> list[dict[str, Any]]:
        """선택된 상품 1개에 대한 주문 task payload 2개를 만든다."""
        priority = int(order_work.get("priority") or 2)
        sequence_no = base_sequence_no

        move_task = self._build_task_payload(
            sequence_no=sequence_no,
            task_type="MOVE_TO_PRODUCT",
            assigned_robot_name=order_work["picky_name"],
            order_id=order_work["order_id"],
            order_item_id=item["order_item_id"],
            source_zone_name=current_zone_name,
            target_zone_name=target_zone_name,
            priority=priority,
        )
        sorting_task = self._build_task_payload(
            sequence_no=sequence_no + 1,
            task_type="SORTING_AND_LOAD",
            assigned_robot_name=order_work["cobot_name"],
            order_id=order_work["order_id"],
            order_item_id=item["order_item_id"],
            source_zone_name=item["product_slot_name"],
            target_zone_name=target_zone_name,
            priority=priority,
        )
        return [move_task, sorting_task]

    # ==================================================================
    # 입고 task 생성
    # ==================================================================

    def _process_requested_stocking_items(self) -> None:
        """REQUESTED stocking_item을 조회하고 task가 없는 항목을 처리한다."""
        stocking_items = self._repo.list_requested_stocking_items()

        for stocking_item in stocking_items:
            stocking_item_id = stocking_item.get("stocking_item_id")
            if stocking_item_id is None:
                continue
            if self._stocking_item_has_tasks(int(stocking_item_id)):
                self._node.get_logger().debug(
                    f"[TaskManager] stocking_item_id={stocking_item_id} 기존 task 존재, 생성 skip"
                )
                continue

            self._process_new_stocking_item(stocking_item)

    def _stocking_item_has_tasks(self, stocking_item_id: int) -> bool:
        """stocking_item에 이미 task가 생성되어 있는지 확인한다."""
        tasks = self._repo.list_tasks()
        return any(task.get("stocking_item_id") == stocking_item_id for task in tasks)

    def _process_new_stocking_item(self, stocking_item: dict[str, Any]) -> list[int]:
        """stocking_item 1건을 입고 task 4개로 변환한다."""
        unit = self._select_available_unit()
        if unit is None:
            self._node.get_logger().info("[TaskManager] 입고 배정 가능한 robot unit 없음")
            return []

        stocking_item_id = int(stocking_item["stocking_item_id"])
        stocking_work = self._repo.get_stocking_work(
            {
                **stocking_item,
                "assigned_unit_id": unit["unit_id"],
            }
        )
        if stocking_work is None:
            return []

        stocking_work["picky_name"] = unit["picky_name"]
        stocking_work["cobot_name"] = unit["cobot_name"]
        stocking_work["source_zone_name"] = unit["source_zone"]

        task_ids = self.create_stocking_tasks_for_item(stocking_work)
        if not task_ids:
            return []

        updated = self._repo.update_stocking_item(
            stocking_item_id,
            status="ASSIGNED",
            assigned_unit_id=unit["unit_id"],
        )
        if updated is None:
            self._node.get_logger().warn(
                f"[TaskManager] stocking_item_id={stocking_item_id} 배정 기록 실패"
            )

        return task_ids

    def create_stocking_tasks_for_item(self, stocking_work: dict[str, Any]) -> list[int]:
        """입고 요청 1건에 대한 task 4개를 생성한다."""
        priority = int(stocking_work.get("priority") or 2)
        stocking_item_id = int(stocking_work["stocking_item_id"])

        tasks = [
            self._build_task_payload(
                sequence_no=1,
                task_type="MOVE_TO_STOCK",
                assigned_robot_name=stocking_work["picky_name"],
                stocking_item_id=stocking_item_id,
                source_zone_name=stocking_work.get("source_zone_name"),
                target_zone_name=stocking_work["stock_zone_name"],
                priority=priority,
            ),
            self._build_task_payload(
                sequence_no=2,
                task_type="STOCKING_PICK",
                assigned_robot_name=stocking_work["cobot_name"],
                stocking_item_id=stocking_item_id,
                source_zone_name=stocking_work["stock_slot_name"],
                target_zone_name=stocking_work["stock_slot_name"],
                priority=priority,
            ),
            self._build_task_payload(
                sequence_no=3,
                task_type="MOVE_TO_STORAGE",
                assigned_robot_name=stocking_work["picky_name"],
                stocking_item_id=stocking_item_id,
                source_zone_name=stocking_work["stock_zone_name"],
                target_zone_name=stocking_work["product_zone_name"],
                priority=priority,
            ),
            self._build_task_payload(
                sequence_no=4,
                task_type="STOCKING_PLACE",
                assigned_robot_name=stocking_work["cobot_name"],
                stocking_item_id=stocking_item_id,
                source_zone_name=stocking_work["product_slot_name"],
                target_zone_name=stocking_work["product_slot_name"],
                priority=priority,
            ),
        ]

        result_data = self._repo.create_tasks_bulk(tasks)
        if result_data is None:
            return []

        task_ids = [int(task_id) for task_id in result_data.get("task_ids", [])]
        self._node.get_logger().info(
            f"[TaskManager] stocking_item_id={stocking_item_id} 입고 task 생성 완료: {task_ids}"
        )
        return task_ids

    # ==================================================================
    # Task payload / zone 변환
    # ==================================================================

    def _build_task_payload(
        self,
        *,
        sequence_no: int,
        task_type: str,
        assigned_robot_name: str,
        order_id: int | None = None,
        order_item_id: int | None = None,
        stocking_item_id: int | None = None,
        source_zone_name: str | None = None,
        target_zone_name: str | None = None,
        priority: int = 2,
        status: str = "ASSIGNED",
        result_message: str | None = None,
    ) -> dict[str, Any]:
        """Fleet API의 `/api/fleet/tasks/bulk` payload 1개를 만든다."""
        zone_map = self._repo.get_zone_map()
        source_zone = zone_map.get(source_zone_name or "")
        target_zone = zone_map.get(target_zone_name or "")

        return {
            "order_id": order_id,
            "order_item_id": order_item_id,
            "stocking_item_id": stocking_item_id,
            "sequence_no": sequence_no,
            "assigned_robot_name": assigned_robot_name,
            "task_type": task_type,
            "status": status,
            "priority": priority,
            "source_zone_id": source_zone.get("zone_id") if source_zone else None,
            "target_zone_id": target_zone.get("zone_id") if target_zone else None,
            "result_message": result_message,
        }

    def _move_command_waypoints_for_task(self, task: dict[str, Any]) -> tuple[str, ...]:
        """MoveCommand에는 실제 로봇이 가야 할 목적지 waypoint만 넘긴다.

        TrafficManager의 상세 graph 경로는 충돌 회피/예약용 내부 데이터다.
        로봇 State Machine은 목표 zone pose를 받아 로컬 navigation을 수행하고,
        Fleet은 task 결과 시점에 TrafficManager 예약을 해제한다.
        """
        target_zone = task.get("target_zone_name")
        if target_zone:
            return (str(target_zone),)

        task_id = int(task["task_id"])
        reserved_waypoints = self._move_waypoints_by_task.get(task_id) or ()
        if reserved_waypoints:
            return (str(reserved_waypoints[-1]),)

        source_zone = task.get("source_zone_name")
        return (str(source_zone),) if source_zone else ()

    # ==================================================================
    # 기존 주문 진행 / 다음 task 생성
    # ==================================================================

    def _advance_existing_orders(self) -> None:
        """이미 task가 생성된 주문을 보고 다음 상품 또는 pickup task를 만든다.

        원칙:
        - 기존 task 중 RUNNING/ASSIGNED/QUEUED/PAUSED가 있으면 새 task를 만들지 않는다.
        - 기존 task가 모두 SUCCESS면 order_item 상태를 다시 조회한다.
        - WAITING item이 남아 있으면 다음 상품 1개 task를 만든다.
        - WAITING item이 없고 pickup task가 없으면 pickup task 3개를 만든다.
        """
        for order in self._repo.list_orders(include_completed=False):
            order_id = order.get("order_id")
            if order_id is None:
                continue

            if order.get("status") == "ORDER_WAIT":
                continue

            tasks = self._repo.list_order_tasks(int(order_id))
            if not tasks:
                continue

            self._advance_order_if_ready(order, tasks)

    def _advance_order_by_id_if_ready(self, order_id: int) -> None:
        """task result 직후 해당 주문만 다음 단계로 즉시 진행한다."""
        tasks = self._repo.list_order_tasks(order_id)
        if not tasks:
            return

        self._advance_order_if_ready({"order_id": order_id}, tasks)

    def _advance_order_if_ready(
        self,
        order: dict[str, Any],
        tasks: list[dict[str, Any]],
    ) -> None:
        """주문 1건의 현재 task들이 끝났으면 다음 task 묶음을 생성한다."""
        order_id = int(order["order_id"])

        if not self._all_existing_tasks_success(tasks):
            return

        order_work = self._repo.get_order_work(order_id)
        if order_work is None:
            return

        if not order_work.get("picky_name") or not order_work.get("cobot_name"):
            self._node.get_logger().warn(
                f"[TaskManager] order_id={order_id} assigned robot 이름 없음"
            )
            return

        current_zone = self._last_picky_target_zone(tasks)
        if current_zone is None:
            current_zone = self._default_source_zone(int(order_work.get("assigned_unit_id") or 1))

        remaining_items = [
            item for item in order_work["items"]
            if item.get("status") in (None, "WAITING")
        ]

        if remaining_items:
            order_work["items"] = remaining_items
            next_sequence_no = max(int(task.get("sequence_no") or 0) for task in tasks) + 1
            self._create_next_product_tasks(
                order_work,
                current_zone=current_zone,
                base_sequence_no=next_sequence_no,
            )
            return

        if not self._has_pickup_tasks(tasks):
            self._create_pickup_tasks(
                order_work,
                current_zone=current_zone,
                existing_tasks=tasks,
            )
            return

        self._create_next_housekeeping_task(
            tasks=tasks,
            flow_kind="order",
            flow_id=order_id,
            unit_id=int(order_work.get("assigned_unit_id") or 1),
            picky_name=str(order_work["picky_name"]),
            priority=int(order_work.get("priority") or 2),
        )

    def _all_existing_tasks_success(self, tasks: list[dict[str, Any]]) -> bool:
        """현재 주문 task가 모두 SUCCESS인지 확인한다."""
        if not tasks:
            return False

        for task in tasks:
            if task.get("status") != "SUCCESS":
                return False
        return True

    def _last_picky_target_zone(self, tasks: list[dict[str, Any]]) -> str | None:
        """완료된 PICKY 이동 task 중 마지막 target zone을 반환한다."""
        sorted_tasks = sorted(
            tasks,
            key=lambda item: int(item.get("sequence_no") or 0),
            reverse=True,
        )

        for task in sorted_tasks:
            if task.get("task_type") not in MOVE_TASK_TYPES:
                continue

            task_id = task.get("task_id")
            if task_id is not None and int(task_id) in self._completed_move_target_by_task:
                return self._completed_move_target_by_task[int(task_id)]

            if task.get("target_zone_name"):
                return str(task["target_zone_name"])
        return None

    def _has_pickup_tasks(self, tasks: list[dict[str, Any]]) -> bool:
        """주문에 pickup 마무리 task가 이미 생성되었는지 확인한다."""
        pickup_types = {"MOVE_TO_PICKUP", "INSPECTION", "UNLOAD"}
        return any(task.get("task_type") in pickup_types for task in tasks)

    def _create_pickup_tasks(
        self,
        order_work: dict[str, Any],
        *,
        current_zone: str,
        existing_tasks: list[dict[str, Any]],
    ) -> list[int]:
        """모든 상품 상차 후 pickup zone/slot을 선택하고 마무리 task 3개를 생성한다."""
        order_id = int(order_work["order_id"])
        picky_name = order_work["picky_name"]
        cobot_name = order_work["cobot_name"]

        if self._unit_has_other_open_task(
            picky_name=picky_name,
            cobot_name=cobot_name,
            order_id=order_id,
        ):
            self._node.get_logger().debug(
                f"[TaskManager] order_id={order_id} pickup task 생성 보류: unit이 다른 작업 수행 중"
            )
            return []

        self._cancel_preemptible_return_home(picky_name)

        empty_slots = self._repo.list_pickup_slots(status="EMPTY")
        slot_by_zone = self._pickup_slots_by_zone(empty_slots)

        if not slot_by_zone:
            self._node.get_logger().info(
                f"[TaskManager] order_id={order_work.get('order_id')} EMPTY pickup slot 없음"
            )
            return []

        result = self._traffic.reserve_nearest_from(
            robot_id=picky_name,
            task_id=None,
            source_zone=current_zone,
            candidates={zone_name: 1 for zone_name in slot_by_zone},
        )

        if not result.ok:
            self._node.get_logger().info(
                f"[TaskManager] order_id={order_work.get('order_id')} pickup 경로 예약 실패: {result.reason}"
            )
            return []

        selected_zone = result.waypoints[-1]
        selected_slot = slot_by_zone.get(selected_zone)
        if selected_slot is None:
            self._traffic.release_path(picky_name, None)
            self._node.get_logger().warn(
                f"[TaskManager] selected pickup zone={selected_zone}에 대응 slot 없음"
            )
            return []

        slot_id = int(selected_slot["slot_id"])
        assigned = self._repo.update_order_status(order_id, pickup_slot_id=slot_id)
        if assigned is None:
            self._traffic.release_path(picky_name, None)
            return []

        base_sequence = max(int(task.get("sequence_no") or 0) for task in existing_tasks) + 1
        priority = int(order_work.get("priority") or 2)
        slot_name = selected_slot.get("slot_name")

        tasks = [
            self._build_task_payload(
                sequence_no=base_sequence,
                task_type="MOVE_TO_PICKUP",
                assigned_robot_name=picky_name,
                order_id=order_id,
                source_zone_name=current_zone,
                target_zone_name=selected_zone,
                priority=priority,
            ),
            self._build_task_payload(
                sequence_no=base_sequence + 1,
                task_type="INSPECTION",
                assigned_robot_name=order_work["cobot_name"],
                order_id=order_id,
                source_zone_name=slot_name,
                target_zone_name=slot_name,
                priority=priority,
            ),
            self._build_task_payload(
                sequence_no=base_sequence + 2,
                task_type="UNLOAD",
                assigned_robot_name=order_work["cobot_name"],
                order_id=order_id,
                source_zone_name=slot_name,
                target_zone_name=slot_name,
                priority=priority,
            ),
        ]

        result_data = self._repo.create_tasks_bulk(tasks)
        if result_data is None:
            self._traffic.release_path(picky_name, None)
            return []

        task_ids = [int(task_id) for task_id in result_data.get("task_ids", [])]
        if not task_ids:
            self._traffic.release_path(picky_name, None)
            return []

        move_task_id = task_ids[0]
        if not self._traffic.attach_task_id(picky_name, move_task_id):
            self._node.get_logger().warn(
                f"[TaskManager] pickup task_id={move_task_id} Traffic 예약 연결 실패"
            )
            return task_ids

        self._move_waypoints_by_task[move_task_id] = tuple(result.waypoints)
        self._node.get_logger().info(
            f"[TaskManager] order_id={order_id} pickup task 생성 완료: {task_ids}"
        )
        return task_ids

    def _pickup_slots_by_zone(
        self,
        slots: list[dict[str, Any]],
    ) -> dict[str, dict[str, Any]]:
        """EMPTY pickup slot 목록을 PICKUP_ZONE 기준 dict로 변환한다."""
        result: dict[str, dict[str, Any]] = {}

        for slot in slots:
            slot_name = slot.get("slot_name")
            zone_name = self._pickup_slot_to_zone_name(slot_name)
            if zone_name is None:
                continue
            result[zone_name] = slot

        return result

    def _pickup_slot_to_zone_name(self, slot_name: str | None) -> str | None:
        """PICKUP_SLOT_n 이름을 PICKUP_ZONE_n 이름으로 변환한다."""
        if slot_name is None:
            return None
        if slot_name.startswith("PICKUP_SLOT_"):
            return slot_name.replace("PICKUP_SLOT_", "PICKUP_ZONE_", 1)
        if slot_name.startswith("PICKUP_ZONE_"):
            return slot_name
        return None

    # ==================================================================
    # 완료 후 복귀 / 도킹 / 충전 task 생성
    # ==================================================================

    def _advance_existing_stocking_items(self) -> None:
        """입고 흐름이 끝난 뒤 필요한 housekeeping task를 이어서 만든다."""
        tasks_by_item: dict[int, list[dict[str, Any]]] = {}

        for task in self._repo.list_tasks():
            stocking_item_id = task.get("stocking_item_id")
            if stocking_item_id is None:
                continue
            tasks_by_item.setdefault(int(stocking_item_id), []).append(task)

        for stocking_item_id, tasks in tasks_by_item.items():
            self._advance_stocking_item_if_ready(stocking_item_id, tasks)

    def _advance_stocking_item_by_id_if_ready(self, stocking_item_id: int) -> None:
        """task result 직후 해당 입고 흐름만 다음 단계로 즉시 진행한다."""
        tasks = [
            task for task in self._repo.list_tasks()
            if task.get("stocking_item_id") is not None
            and int(task["stocking_item_id"]) == stocking_item_id
        ]
        self._advance_stocking_item_if_ready(stocking_item_id, tasks)

    def _advance_stocking_item_if_ready(
        self,
        stocking_item_id: int,
        tasks: list[dict[str, Any]],
    ) -> None:
        """입고 task가 모두 끝났으면 housekeeping task를 이어서 만든다."""
        if not tasks or not self._all_existing_tasks_success(tasks):
            return

        picky_name = self._picky_name_from_tasks(tasks)
        if picky_name is None:
            return

        self._create_next_housekeeping_task(
            tasks=tasks,
            flow_kind="stocking",
            flow_id=stocking_item_id,
            unit_id=self._unit_id_from_robot_name(picky_name),
            picky_name=picky_name,
            priority=max(int(task.get("priority") or 2) for task in tasks),
        )

    def _create_next_housekeeping_task(
        self,
        *,
        tasks: list[dict[str, Any]],
        flow_kind: str,
        flow_id: int,
        unit_id: int,
        picky_name: str,
        priority: int,
    ) -> list[int]:
        """완료된 주문/입고 흐름 뒤에 필요한 다음 housekeeping task 하나를 만든다.

        정책:
        - 다음 주문/입고가 있고 PICKY 배터리가 40% 초과면 복귀 체인을 만들지 않는다.
        - 다음 주문/입고가 없으면 PARKING 사유로 RETURN_HOME을 만든다.
        - 배터리가 40% 이하이면 LOW_BATTERY 사유로 RETURN_HOME을 만든다.
        - RETURN_HOME 성공 후에도 같은 판단 함수를 다시 호출한다.
          PARKING 중 새 작업이 생기면 DOCK_IN/CHARGE로 이어가지 않는다.
        """
        flow_key = (flow_kind, flow_id)
        if flow_key in self._housekeeping_stopped_flows:
            return []

        housekeeping_tasks = [
            task for task in tasks
            if task.get("task_type") in HOUSEKEEPING_TASK_TYPES
        ]
        if any(task.get("status") not in FINAL_TASK_STATUSES for task in housekeeping_tasks):
            return []

        last_housekeeping = self._last_housekeeping_task(housekeeping_tasks)

        if last_housekeeping is None:
            decision = self._evaluate_housekeeping_decision(picky_name)
            if not decision["should_return_home"]:
                self._housekeeping_stopped_flows.add(flow_key)
                return []

            return self._create_housekeeping_task(
                tasks=tasks,
                task_type="RETURN_HOME",
                assigned_robot_name=picky_name,
                flow_kind=flow_kind,
                flow_id=flow_id,
                source_zone_name=self._last_picky_target_zone(tasks)
                or self._default_source_zone(unit_id),
                target_zone_name=None,
                priority=priority,
                reason=str(decision["reason"]),
            )

        last_type = last_housekeeping.get("task_type")
        reason = self._housekeeping_reason(last_housekeeping)

        if last_type == "RETURN_HOME":
            decision = self._evaluate_housekeeping_decision(picky_name)
            if reason == HOUSEKEEPING_REASON_PARKING and not decision["should_return_home"]:
                self._housekeeping_stopped_flows.add(flow_key)
                return []

            return self._create_housekeeping_task(
                tasks=tasks,
                task_type="DOCK_IN",
                assigned_robot_name=picky_name,
                flow_kind=flow_kind,
                flow_id=flow_id,
                source_zone_name=self._last_picky_target_zone(tasks)
                or self._default_source_zone(unit_id),
                target_zone_name=None,
                priority=priority,
                reason=reason,
            )

        if last_type == "DOCK_IN":
            return self._create_housekeeping_task(
                tasks=tasks,
                task_type="CHARGE",
                assigned_robot_name=picky_name,
                flow_kind=flow_kind,
                flow_id=flow_id,
                source_zone_name=self._last_picky_target_zone(tasks),
                target_zone_name=self._last_picky_target_zone(tasks),
                priority=priority,
                reason=reason,
            )

        return []

    def _evaluate_housekeeping_decision(self, picky_name: str) -> dict[str, Any]:
        """완료된 흐름 뒤 RETURN_HOME이 필요한지 판단한다.

        이 함수 하나만 호출하면 같은 정책을 완료 시점, INSPECTION 시작 시점,
        STOWING_ARM lookahead 등 여러 위치에서 재사용할 수 있다.
        """
        robot = self._robot_by_name(picky_name)
        battery_low = robot is not None and not self._picky_has_work_battery(robot)

        if battery_low:
            return {
                "should_return_home": True,
                "reason": HOUSEKEEPING_REASON_LOW_BATTERY,
            }

        if self._has_assignable_waiting_work() or self._has_open_non_housekeeping_task(picky_name):
            return {
                "should_return_home": False,
                "reason": "NEXT_WORK_AVAILABLE",
            }

        return {
            "should_return_home": True,
            "reason": HOUSEKEEPING_REASON_PARKING,
        }

    def _create_housekeeping_task(
        self,
        *,
        tasks: list[dict[str, Any]],
        task_type: str,
        assigned_robot_name: str,
        flow_kind: str,
        flow_id: int,
        source_zone_name: str | None,
        target_zone_name: str | None,
        priority: int,
        reason: str,
    ) -> list[int]:
        """RETURN_HOME / DOCK_IN / CHARGE task 하나를 생성한다."""
        payload = self._build_task_payload(
            sequence_no=max(int(task.get("sequence_no") or 0) for task in tasks) + 1,
            task_type=task_type,
            assigned_robot_name=assigned_robot_name,
            order_id=flow_id if flow_kind == "order" else None,
            stocking_item_id=flow_id if flow_kind == "stocking" else None,
            source_zone_name=source_zone_name,
            target_zone_name=target_zone_name,
            priority=priority,
            result_message=f"HOUSEKEEPING_REASON={reason}",
        )
        result_data = self._repo.create_tasks_bulk([payload])
        if result_data is None:
            return []

        task_ids = [int(task_id) for task_id in result_data.get("task_ids", [])]
        self._node.get_logger().info(
            f"[TaskManager] {flow_kind}_id={flow_id} {task_type} task 생성 완료: {task_ids}, reason={reason}"
        )
        return task_ids

    def _complete_ready_charge_tasks(self) -> None:
        """배터리가 기준치를 넘은 CHARGE task를 SUCCESS로 정리한다."""
        for task in self._repo.list_tasks(status="RUNNING", task_type="CHARGE"):
            robot_name = task.get("assigned_robot_name")
            if not robot_name:
                continue

            robot = self._robot_by_name(str(robot_name))
            battery_level = robot.get("battery_level") if robot else None
            if battery_level is None or int(battery_level) <= CHARGE_BATTERY_THRESHOLD:
                continue

            self._complete_charge_task(task, int(battery_level))

    def handle_battery_update(self, robot_name: str, battery_level: int) -> None:
        """RobotStateMonitor가 battery update를 받을 때 호출하는 hook.

        CHARGE는 별도 Action result가 없으므로 배터리 이벤트가 완료 트리거다.
        battery_level이 기준치를 넘으면 해당 PICKY의 RUNNING CHARGE task를 즉시
        SUCCESS 처리하고, 새 작업이 대기 중이면 다음 polling 주기를 기다리지 않고 바로 배정/dispatch한다.

        RobotStateMonitor는 별도로 Fleet API에 battery_level을 보고하고,
        이 함수에는 같은 값만 전달하면 된다.
        """
        with self._scheduler_lock:
            completed = self._complete_charge_tasks_for_robot(robot_name, battery_level)
            if not completed:
                return

            self._process_waiting_work_if_unit_available()
            self._dispatch_ready_tasks()

    def _complete_charge_tasks_for_robot(self, robot_name: str, battery_level: int) -> bool:
        """특정 로봇의 RUNNING CHARGE task를 배터리 기준으로 완료 처리한다."""
        if int(battery_level) <= CHARGE_BATTERY_THRESHOLD:
            return False

        completed = False
        for task in self._repo.list_tasks(status="RUNNING", task_type="CHARGE"):
            if task.get("assigned_robot_name") != robot_name:
                continue
            completed = self._complete_charge_task(task, int(battery_level)) or completed

        return completed

    def _complete_charge_task(self, task: dict[str, Any], battery_level: int) -> bool:
        """CHARGE task 1건을 SUCCESS로 전환한다."""
        robot_name = str(task.get("assigned_robot_name") or "")
        if not robot_name:
            return False

        updated = self._repo.update_task_status(
            int(task["task_id"]),
            status="SUCCESS",
            current_status="RUNNING",
            assigned_robot_name=robot_name,
            result_message=self._with_housekeeping_reason(
                f"battery charged above {CHARGE_BATTERY_THRESHOLD}%: {battery_level}%",
                task,
            ),
        )
        return updated is not None

    def _dispatch_charge_task(self, task: dict[str, Any]) -> bool:
        """CHARGE task를 RUNNING으로 전환한다.

        실제 충전은 도크에 들어간 로봇의 물리 상태이며 별도 Action goal이 아니다.
        battery_level이 기준치를 넘으면 _complete_ready_charge_tasks()가 SUCCESS 처리한다.
        """
        robot_name = str(task.get("assigned_robot_name") or "")
        if not robot_name:
            return False

        task_id = int(task["task_id"])
        updated = self._repo.update_task_status(
            task_id,
            status="RUNNING",
            current_status="ASSIGNED",
            assigned_robot_name=robot_name,
            result_message=self._with_housekeeping_reason("CHARGE started", task),
        )
        return updated is not None

    def _last_housekeeping_task(self, tasks: list[dict[str, Any]]) -> dict[str, Any] | None:
        """마지막 housekeeping task를 반환한다."""
        if not tasks:
            return None
        return max(
            tasks,
            key=lambda item: (int(item.get("sequence_no") or 0), int(item.get("task_id") or 0)),
        )

    def _housekeeping_reason(self, task: dict[str, Any]) -> str | None:
        """task.result_message에서 housekeeping reason을 읽는다."""
        message = str(task.get("result_message") or "")
        marker = "HOUSEKEEPING_REASON="
        if marker not in message:
            return None
        return message.split(marker, 1)[1].split()[0].strip()

    def _with_housekeeping_reason(self, message: str, task: dict[str, Any]) -> str:
        """상태 변경 메시지에 housekeeping reason marker를 보존한다."""
        reason = self._housekeeping_reason(task)
        if reason is None:
            return message
        return f"{message} HOUSEKEEPING_REASON={reason}"

    def _has_assignable_waiting_work(self) -> bool:
        """아직 task가 없는 ORDER_WAIT 주문이나 REQUESTED 입고 요청이 있는지 확인한다."""
        for order in self._repo.list_waiting_orders():
            order_id = order.get("order_id")
            if order_id is not None and not self._repo.list_order_tasks(int(order_id)):
                return True

        for item in self._repo.list_requested_stocking_items():
            stocking_item_id = item.get("stocking_item_id")
            if stocking_item_id is not None and not self._stocking_item_has_tasks(int(stocking_item_id)):
                return True

        return False

    def _has_open_non_housekeeping_task(self, robot_name: str) -> bool:
        """해당 로봇에 housekeeping이 아닌 미완료 task가 있는지 확인한다."""
        return any(
            task.get("status") not in FINAL_TASK_STATUSES
            and task.get("task_type") not in HOUSEKEEPING_TASK_TYPES
            for task in self._repo.list_tasks(robot_name=robot_name)
        )

    def _robot_by_name(self, robot_name: str) -> dict[str, Any] | None:
        """snapshot robot 목록에서 robot_name 하나를 찾는다."""
        for robot in self._repo.list_robots():
            if robot.get("robot_name") == robot_name:
                return robot
        return None

    def _picky_name_from_tasks(self, tasks: list[dict[str, Any]]) -> str | None:
        """task 목록에서 PICKY 담당 robot_name을 찾는다."""
        for task in tasks:
            robot_name = task.get("assigned_robot_name")
            if not robot_name:
                continue
            robot = self._robot_by_name(str(robot_name))
            if robot and robot.get("robot_type") == "PICKY":
                return str(robot_name)
        return None

    def _unit_id_from_robot_name(self, robot_name: str) -> int:
        """robot_name에서 unit_id를 얻고, 실패 시 seed 기본값을 반환한다."""
        robot = self._robot_by_name(robot_name)
        unit_id = robot.get("unit_id") if robot else None
        return int(unit_id or 1)

    def _mark_housekeeping_stopped_for_task(self, task: dict[str, Any]) -> None:
        """해당 flow에서 더 이상 DOCK_IN/CHARGE를 이어 만들지 않도록 표시한다."""
        flow_key = self._flow_key_for_task(task)
        if flow_key is not None:
            self._housekeeping_stopped_flows.add(flow_key)

    def _flow_key_for_task(self, task: dict[str, Any]) -> tuple[str, int] | None:
        """task가 속한 주문/입고 흐름 key를 반환한다."""
        order_id = task.get("order_id")
        if order_id is not None:
            return ("order", int(order_id))

        stocking_item_id = task.get("stocking_item_id")
        if stocking_item_id is not None:
            return ("stocking", int(stocking_item_id))

        return None

    # ==================================================================
    # Task 실행 dispatch
    # ==================================================================

    def _dispatch_ready_tasks(self) -> None:
        """ASSIGNED task 중 순서상 실행 가능한 task를 로봇으로 보낸다."""
        if self._fleet_paused:
            self._node.get_logger().debug(
                "[TaskManager] dispatch skip: fleet emergency paused"
            )
            return

        assigned_tasks = self._repo.list_tasks(status="ASSIGNED")
        if not assigned_tasks:
            return

        all_tasks = self._repo.list_tasks()

        for task in assigned_tasks:
            if not self._task_ready_by_sequence(task, all_tasks):
                continue

            robot_name = task.get("assigned_robot_name")
            if not robot_name:
                continue

            if self._robot_has_running_task(str(robot_name)):
                continue

            self._dispatch_task(task)

    def _task_ready_by_sequence(
        self,
        task: dict[str, Any],
        all_tasks: list[dict[str, Any]],
    ) -> bool:
        """같은 주문/입고 묶음의 이전 sequence task가 모두 SUCCESS인지 확인한다."""
        sequence_no = int(task.get("sequence_no") or 0)
        order_id = task.get("order_id")
        stocking_item_id = task.get("stocking_item_id")

        if order_id is not None:
            related_tasks = self._repo.list_order_tasks(int(order_id))
        elif stocking_item_id is not None:
            related_tasks = [
                item for item in all_tasks
                if item.get("stocking_item_id") == stocking_item_id
            ]
        else:
            related_tasks = all_tasks

        for prev_task in related_tasks:
            if int(prev_task.get("sequence_no") or 0) >= sequence_no:
                continue
            if prev_task.get("status") != "SUCCESS":
                return False

        return True

    def _robot_has_running_task(self, robot_name: str) -> bool:
        """해당 로봇에 이미 RUNNING task가 있는지 확인한다."""
        running = self._repo.list_tasks(status="RUNNING", robot_name=robot_name)
        return bool(running)

    def _dispatch_task(self, task: dict[str, Any]) -> bool:
        """task_type에 따라 PICKY/COBOT dispatch로 분기한다."""
        task_type = task.get("task_type")

        if task_type in MOVE_TASK_TYPES:
            return self._dispatch_move_task(task)

        if task_type in DOCK_TASK_TYPES:
            return self._dispatch_dock_task(task)

        if task_type in COBOT_TASK_TYPES:
            return self._dispatch_cobot_task(task)

        if task_type == "CHARGE":
            return self._dispatch_charge_task(task)

        self._node.get_logger().warn(
            f"[TaskManager] 지원하지 않는 task_type: task_id={task.get('task_id')}, task_type={task_type}"
        )
        return False

    def _dispatch_move_task(self, task: dict[str, Any]) -> bool:
        """PICKY 이동 task를 RUNNING으로 전환하고 RobotCommandGateway로 보낸다."""
        if self._robot_gateway is None:
            self._node.get_logger().warn("[TaskManager] RobotCommandGateway 없음")
            return False

        task_id = int(task["task_id"])
        robot_name = str(task["assigned_robot_name"])
        task_type = str(task["task_type"])

        if not self._reserve_move_path_for_task(task):
            return False

        updated = self._repo.update_task_status(
            task_id,
            status="RUNNING",
            current_status="ASSIGNED",
            assigned_robot_name=robot_name,
            result_message=self._with_housekeeping_reason(f"{task_type} started", task),
        )
        if updated is None:
            self._traffic.release_path(robot_name, task_id)
            self._move_waypoints_by_task.pop(task_id, None)
            return False

        sent = self._robot_gateway.send_move_task(
            robot_name=robot_name,
            task_id=task_id,
            task_type=task_type,
            waypoints=self._move_command_waypoints_for_task(task),
            zone_map=self._repo.get_zone_map(),
            feedback_callback=self.handle_move_feedback,
            result_callback=self.handle_task_result,
        )

        if not sent:
            self._mark_task_failed_before_dispatch(
                task,
                message="MoveCommand action server unavailable or waypoint conversion failed",
            )
            return False

        return True

    def _reserve_move_path_for_task(self, task: dict[str, Any]) -> bool:
        """DB task에 대응되는 TrafficManager 경로 예약을 보장한다."""
        task_id = int(task["task_id"])
        if task_id in self._move_waypoints_by_task:
            return True

        robot_name = str(task.get("assigned_robot_name") or "")
        task_type = str(task.get("task_type") or "")
        source_zone = task.get("source_zone_name")
        target_zone = task.get("target_zone_name")

        if not robot_name or not source_zone:
            self._node.get_logger().warn(
                f"[TaskManager] task_id={task_id} 경로 예약 실패: robot/source 없음"
            )
            return False

        if task_type == "RETURN_HOME":
            result = self._traffic.reserve_return_home_path(
                robot_id=robot_name,
                task_id=task_id,
                source_zone=str(source_zone),
            )
        else:
            if not target_zone:
                self._node.get_logger().warn(
                    f"[TaskManager] task_id={task_id} 경로 예약 실패: target 없음"
                )
                return False
            result = self._traffic.reserve_path(
                robot_id=robot_name,
                task_id=task_id,
                source_zone=str(source_zone),
                target_zone=str(target_zone),
            )

        if not result.ok:
            self._node.get_logger().info(
                f"[TaskManager] task_id={task_id} 경로 예약 실패: {result.reason}"
            )
            return False

        self._move_waypoints_by_task[task_id] = tuple(result.waypoints)
        return True

    def _dispatch_dock_task(self, task: dict[str, Any]) -> bool:
        """DOCK_IN task를 RUNNING으로 전환하고 DockCommand로 보낸다."""
        if self._robot_gateway is None:
            self._node.get_logger().warn("[TaskManager] RobotCommandGateway 없음")
            return False

        task_id = int(task["task_id"])
        robot_name = str(task["assigned_robot_name"])

        dock_target = self._reserve_dock_for_task(task)
        if dock_target is None:
            return False
        dock_name, start_zone_name = dock_target

        updated = self._repo.update_task_status(
            task_id,
            status="RUNNING",
            current_status="ASSIGNED",
            assigned_robot_name=robot_name,
            result_message=self._with_housekeeping_reason("DOCK_IN started", task),
        )
        if updated is None:
            self._traffic.release_path(robot_name, task_id)
            self._move_waypoints_by_task.pop(task_id, None)
            return False

        sent = self._robot_gateway.send_dock_task(
            robot_name=robot_name,
            task_id=task_id,
            dock_name=dock_name,
            start_zone_name=start_zone_name,
            result_callback=self.handle_task_result,
        )

        if not sent:
            self._mark_task_failed_before_dispatch(
                task,
                message="DockCommand action server unavailable",
            )
            return False

        return True

    def _reserve_dock_for_task(self, task: dict[str, Any]) -> tuple[str, str] | None:
        """DOCK_IN task의 논리 도크 점유와 시작 STANDBY zone을 보장한다."""
        task_id = int(task["task_id"])
        if task_id in self._move_waypoints_by_task:
            return self._dock_target_from_waypoints(task, self._move_waypoints_by_task[task_id])

        robot_name = str(task.get("assigned_robot_name") or "")
        source_zone = task.get("source_zone_name")
        if not robot_name or not source_zone:
            self._node.get_logger().warn(
                f"[TaskManager] task_id={task_id} 도크 예약 실패: robot/source 없음"
            )
            return None

        result = self._traffic.reserve_dock_path(
            robot_id=robot_name,
            task_id=task_id,
            source_zone=str(source_zone),
        )
        if not result.ok:
            self._node.get_logger().info(
                f"[TaskManager] task_id={task_id} 도크 예약 실패: {result.reason}"
            )
            return None

        self._move_waypoints_by_task[task_id] = tuple(result.waypoints)
        return self._dock_target_from_waypoints(task, tuple(result.waypoints))

    def _dock_target_from_waypoints(
        self,
        task: dict[str, Any],
        waypoints: tuple[str, ...],
    ) -> tuple[str, str] | None:
        """TrafficManager DOCK_IN 예약 결과를 DockCommand goal 값으로 변환한다.

        현재 TrafficManager는 마지막 waypoint로 논리 도크 이름을 반환한다.
        실제 주행 제어는 그 직전 STANDBY_ZONE부터 PICKY State Manager가 맡는다.
        """
        if not waypoints:
            return None

        dock_name = waypoints[-1]
        start_zone_name = waypoints[-2] if len(waypoints) >= 2 else task.get("source_zone_name")
        if not start_zone_name:
            self._node.get_logger().warn(
                f"[TaskManager] task_id={task.get('task_id')} DockCommand 시작 zone 없음"
            )
            return None

        return str(dock_name), str(start_zone_name)

    def _dispatch_cobot_task(self, task: dict[str, Any]) -> bool:
        """COBOT task를 RobotCommandGateway로 보낸다.

        현재 저장소에는 ExecuteTask.action이 없으므로 Gateway가 False를 반환한다.
        이 경우 task 상태는 ASSIGNED로 유지해서 인터페이스 확정 후 재시도 가능하게 둔다.
        """
        if self._robot_gateway is None:
            return False

        task_id = int(task["task_id"])
        robot_name = str(task.get("assigned_robot_name") or "")
        if task_id in self._unsupported_task_warned:
            return False

        sent = self._robot_gateway.send_cobot_task(
            robot_name=robot_name,
            task=task,
            result_callback=self.handle_task_result,
        )
        if not sent and task_id not in self._unsupported_task_warned:
            self._unsupported_task_warned.add(task_id)
            self._node.get_logger().warn(
                f"[TaskManager] task_id={task_id} COBOT 실행 인터페이스 대기 중, 상태는 ASSIGNED 유지"
            )
            return False

        if sent:
            self._repo.update_task_status(
                task_id,
                status="RUNNING",
                current_status="ASSIGNED",
                assigned_robot_name=robot_name,
                result_message=self._with_housekeeping_reason(
                    f"{task.get('task_type')} started",
                    task,
                ),
            )
            return True

        return False

    def _mark_task_failed_before_dispatch(
        self,
        task: dict[str, Any],
        *,
        message: str,
    ) -> None:
        """로봇으로 보내기 전에 실패한 task를 FAILED로 정리한다."""
        task_id = int(task["task_id"])
        robot_name = task.get("assigned_robot_name")
        task_type = task.get("task_type")

        self._repo.update_task_status(
            task_id,
            status="FAILED",
            current_status="RUNNING",
            assigned_robot_name=str(robot_name) if robot_name else None,
            result_message=self._with_housekeeping_reason(message, task),
        )

        if task_type in PATH_RESERVED_TASK_TYPES and robot_name:
            self._traffic.release_path(str(robot_name), task_id)
            self._move_waypoints_by_task.pop(task_id, None)

        self._repo.create_exception(
            exception_type="NAVIGATION_FAILED" if task_type in PATH_RESERVED_TASK_TYPES else "SYSTEM_ERROR",
            robot_name=str(robot_name) if robot_name else None,
            task_id=task_id,
            order_id=task.get("order_id"),
            detail=message,
        )

    # ==================================================================
    # RobotCommandGateway callback
    # ==================================================================

    def handle_move_feedback(
        self,
        robot_name: str,
        task_id: int,
        current_waypoint_index: int,
    ) -> None:
        """PICKY MoveCommand feedback을 TrafficManager에 전달한다."""
        self._traffic.update_path_progress(
            robot_name,
            task_id,
            current_waypoint_index,
        )

    def handle_task_result(self, result: dict[str, Any]) -> None:
        """RobotCommandGateway가 전달한 task result를 처리한다."""
        self._scheduler_lock.acquire()
        try:
            self._handle_task_result_locked(result)
        finally:
            self._scheduler_lock.release()

    def _handle_task_result_locked(self, result: dict[str, Any]) -> None:
        """lock을 잡은 상태에서 task result를 반영하고 다음 단계를 즉시 진행한다."""
        task_id = int(result["task_id"])
        robot_name = result.get("robot_name")
        task_type = result.get("task_type")
        success = bool(result.get("success"))
        message = result.get("message") or ""
        task = self._find_task_by_id(task_id) or {}
        current_task_status = task.get("status")

        if current_task_status in FINAL_TASK_STATUSES:
            self._node.get_logger().debug(
                f"[TaskManager] task_id={task_id} stale result ignored: status={current_task_status}"
            )
            return

        next_status = "SUCCESS" if success else "FAILED"
        self._repo.update_task_status(
            task_id,
            status=next_status,
            current_status="RUNNING",
            assigned_robot_name=robot_name,
            result_message=self._with_housekeeping_reason(message, task),
        )

        if task_type in PATH_RESERVED_TASK_TYPES and robot_name:
            waypoints = self._move_waypoints_by_task.get(task_id)
            if success and waypoints and task_type in MOVE_TASK_TYPES:
                self._completed_move_target_by_task[task_id] = waypoints[-1]
            self._traffic.release_path(robot_name, task_id)
            self._move_waypoints_by_task.pop(task_id, None)

        if task_type in COBOT_TASK_TYPES:
            if success:
                self._preplanned_created_tasks_by_trigger.pop(task_id, None)
                self._preplanned_move_tasks_by_trigger.pop(task_id, None)
            else:
                self._cancel_preplanned_after_cobot_failure(task_id)

        if not success:
            self._repo.create_exception(
                exception_type="NAVIGATION_FAILED" if task_type in PATH_RESERVED_TASK_TYPES else "SYSTEM_ERROR",
                robot_name=robot_name,
                task_id=task_id,
                detail=message,
            )
            return

        if self._fleet_paused:
            return

        self._advance_flow_after_task_success(task)
        self._dispatch_ready_tasks()

    def _advance_flow_after_task_success(self, task: dict[str, Any]) -> None:
        """성공한 task가 속한 주문/입고 흐름만 즉시 다음 단계로 넘긴다."""
        order_id = task.get("order_id")
        if order_id is not None:
            self._advance_order_by_id_if_ready(int(order_id))
            return

        stocking_item_id = task.get("stocking_item_id")
        if stocking_item_id is not None:
            self._advance_stocking_item_by_id_if_ready(int(stocking_item_id))
