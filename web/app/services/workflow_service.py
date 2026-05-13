from datetime import UTC, datetime

from sqlalchemy.orm import Session

from app.models import Order, OrderItem, PickupSlot, Robot, Task, TaskEvent, Zone
from app.services.robot_runtime_policy import (
    FINAL_TASK_STATUSES,
    MIN_AMR_BATTERY_LEVEL,
    UNAVAILABLE_ROBOT_STATUSES,
)


# 주문 workflow helper다.
# 주문 접수 시 Control Server는 task 목록을 만들고 Cobot 담당 task를 고정 배정한다.
# AMR 담당 task는 AMR이 대기 상태를 보고할 때 배정한다.
ORDER_PRIORITY = 2
PATROL_PRIORITY = 1
AMR_ORDER_TASK_TYPES = ("STANDBY_LOAD", "STANDBY_UNLOAD")
ACTIVE_RUNTIME_TASK_STATUSES = ("ASSIGNED", "RUNNING", "PAUSED")
ORDER_WORKFLOW_INACTIVE_STATUSES = ("PICKUP_READY", "COMPLETED", "ERROR")
AMR_ASSIGNMENT_READY_STATUSES = ("IDLE", "STANDBY")

ORDER_TASK_TYPES = (
    "STANDBY_LOAD",
    "SORTING",
    "LOAD",
    "STANDBY_UNLOAD",
    "INSPECTION",
    "UNLOAD",
)

FIXED_TASK_ROBOT_ID = {
    "SORTING": "SORTING_COBOT",
    "LOAD": "SORTING_COBOT",
    "INSPECTION": "INSPECTION_COBOT",
    "UNLOAD": "INSPECTION_COBOT",
}

TASK_ZONE_NAMES = {
    "STANDBY_LOAD": (None, "STANDBY_LOADING_ZONE"),
    "SORTING": ("PRODUCT_ZONE", "LOADING_ZONE"),
    "LOAD": ("PRODUCT_ZONE", "LOADING_ZONE"),
    "STANDBY_UNLOAD": ("LOADING_ZONE", "STANDBY_UNLOADING_ZONE"),
    "INSPECTION": ("UNLOADING_ZONE", "UNLOADING_ZONE"),
    "UNLOAD": ("UNLOADING_ZONE", None),
}

ORDER_STATUS_BY_TASK = {
    "STANDBY_LOAD": "ORDER_WAIT",
    "SORTING": "SORTING",
    "LOAD": "SORTING",
    "STANDBY_UNLOAD": "DELIVERING",
    "INSPECTION": "INSPECTING",
    "UNLOAD": "DELIVERING",
}

ROBOT_STATUS_BY_TASK = {
    "STANDBY_LOAD": "STANDBY",
    "SORTING": "SORTING",
    "LOAD": "LOADING",
    "STANDBY_UNLOAD": "MOVING",
    "INSPECTION": "INSPECTING",
    "UNLOAD": "UNLOADING",
    "PATROL": "PATROLLING",
    "CHARGE": "CHARGING",
    "RETURN_HOME": "RETURNING",
}


def create_order_workflow(db: Session, order: Order) -> None:
    existing_task = (
        db.query(Task)
        .filter(Task.order_id == order.order_id)
        .first()
    )

    if existing_task:
        return

    order.status = "ORDER_WAIT"
    order.priority = ORDER_PRIORITY
    zone_ids_by_name = build_zone_ids_by_name(db)

    for task_type in ORDER_TASK_TYPES:
        source_zone_id, target_zone_id = resolve_task_zone_ids(zone_ids_by_name, task_type)
        task = Task(
            order_id=order.order_id,
            assigned_robot_id=resolve_order_task_robot_id(task_type),
            task_type=task_type,
            status="QUEUED",
            priority=order.priority,
            source_zone_id=source_zone_id,
            target_zone_id=target_zone_id,
        )
        db.add(task)

    db.flush()
    assign_ready_tasks(db)


def complete_order_workflow(db: Session, order: Order) -> None:
    order.status = "COMPLETED"

    if order.pickup_slot_id:
        pickup_slot = db.get(PickupSlot, order.pickup_slot_id)
        if pickup_slot:
            pickup_slot.status = "EMPTY"

    for task in order_tasks(db, order):
        if task.status not in FINAL_TASK_STATUSES:
            record_task_event(db, task, "SUCCESS", "ORDER_COMPLETED")
            task.status = "SUCCESS"
        clear_robot_task(db, task)

    db.flush()
    assign_ready_tasks(db)


def resolve_order_task_robot_id(task_type: str) -> str | None:
    if task_type in AMR_ORDER_TASK_TYPES:
        return None

    return FIXED_TASK_ROBOT_ID[task_type]


def assign_ready_tasks(db: Session, preferred_robot_id: str | None = None) -> int:
    db.flush()

    assigned_count = 0
    preferred_robot = db.get(Robot, preferred_robot_id) if preferred_robot_id else None
    tasks = (
        db.query(Task)
        .filter(Task.status == "QUEUED")
        .order_by(Task.priority.desc(), Task.task_id)
        .all()
    )

    for task in tasks:
        if not task_is_ready_to_assign(db, task):
            continue

        robot = resolve_task_assignment_robot(db, task, preferred_robot)
        if not robot:
            continue

        if not robot_is_assignable(db, robot, task):
            continue

        order = db.get(Order, task.order_id) if task.order_id else None
        if order and task.task_type == "INSPECTION" and not reserve_pickup_slot(db, order):
            continue

        if task.assigned_robot_id is None:
            task.assigned_robot_id = robot.robot_id
            reserve_related_amr_tasks(db, task, robot.robot_id)

        mark_task_assigned(db, task, "Ready task assigned by Control Server.")
        assigned_count += 1

    return assigned_count


def task_is_ready_to_assign(db: Session, task: Task) -> bool:
    if task.order_id is None:
        return True

    order = db.get(Order, task.order_id)
    if not order or order.status in ORDER_WORKFLOW_INACTIVE_STATUSES:
        return False

    if order_has_active_runtime_task(db, order):
        return False

    next_task = next_pending_task(db, order)
    return bool(next_task and next_task.task_id == task.task_id)


def resolve_task_assignment_robot(
    db: Session,
    task: Task,
    preferred_robot: Robot | None,
) -> Robot | None:
    if task.assigned_robot_id is not None:
        return db.get(Robot, task.assigned_robot_id)

    if task.task_type not in AMR_ORDER_TASK_TYPES and task.task_type != "PATROL":
        return None

    if not preferred_robot or not preferred_robot.robot_id.startswith("AMR_"):
        return None

    return preferred_robot


def reserve_related_amr_tasks(db: Session, task: Task, robot_id: str) -> None:
    if task.order_id is None or task.task_type not in AMR_ORDER_TASK_TYPES:
        return

    tasks = (
        db.query(Task)
        .filter(
            Task.order_id == task.order_id,
            Task.task_type.in_(AMR_ORDER_TASK_TYPES),
            Task.status.notin_(FINAL_TASK_STATUSES),
        )
        .all()
    )

    for order_task in tasks:
        if order_task.assigned_robot_id is None:
            order_task.assigned_robot_id = robot_id


def order_has_active_runtime_task(db: Session, order: Order) -> bool:
    return (
        db.query(Task)
        .filter(
            Task.order_id == order.order_id,
            Task.status.in_(ACTIVE_RUNTIME_TASK_STATUSES),
        )
        .first()
        is not None
    )


def build_zone_ids_by_name(db: Session) -> dict[str, int]:
    return {
        zone.zone_name: zone.zone_id
        for zone in db.query(Zone).all()
    }


def resolve_task_zone_ids(
    zone_ids_by_name: dict[str, int],
    task_type: str,
) -> tuple[int | None, int | None]:
    source_zone_name, target_zone_name = TASK_ZONE_NAMES.get(task_type, (None, None))
    source_zone_id = zone_ids_by_name.get(source_zone_name) if source_zone_name else None
    target_zone_id = zone_ids_by_name.get(target_zone_name) if target_zone_name else None

    return source_zone_id, target_zone_id


def robot_battery_can_take_order(robot: Robot) -> bool:
    if robot.battery_level is None:
        return True

    return robot.battery_level >= MIN_AMR_BATTERY_LEVEL


def robot_is_assignable(db: Session, robot: Robot | None, task: Task) -> bool:
    if not robot or robot.status in UNAVAILABLE_ROBOT_STATUSES:
        return False

    if robot.robot_id.startswith("AMR_"):
        if robot.status not in AMR_ASSIGNMENT_READY_STATUSES:
            return False

        if not robot_battery_can_take_order(robot):
            return False

    if robot.current_task_id:
        current_task = db.get(Task, robot.current_task_id)
        if current_task and current_task.task_id != task.task_id:
            return False

    other_task = (
        db.query(Task)
        .filter(
            Task.assigned_robot_id == robot.robot_id,
            Task.task_id != task.task_id,
            Task.status.in_(ACTIVE_RUNTIME_TASK_STATUSES),
        )
        .first()
    )

    return other_task is None


def apply_task_runtime_state(db: Session, task: Task) -> None:
    if task.status == "RUNNING":
        start_runtime_task(db, task)
    elif task.status == "SUCCESS":
        finish_runtime_task(db, task)
        assign_ready_tasks(db)
    elif task.status in ("FAILED", "CANCELLED"):
        clear_robot_task(db, task)
        if task.status == "FAILED" and task.order_id:
            order = db.get(Order, task.order_id)
            if order:
                order.status = "ERROR"
        assign_ready_tasks(db)


def start_runtime_task(db: Session, task: Task) -> None:
    if not task.assigned_robot_id:
        return

    robot = db.get(Robot, task.assigned_robot_id)
    order = db.get(Order, task.order_id) if task.order_id else None

    if robot:
        robot.current_task_id = task.task_id
        robot.status = ROBOT_STATUS_BY_TASK.get(task.task_type, robot.status)

    if order:
        if task.task_type == "INSPECTION":
            reserve_pickup_slot(db, order)

        order.status = ORDER_STATUS_BY_TASK.get(task.task_type, order.status)


def finish_runtime_task(db: Session, task: Task) -> None:
    order = db.get(Order, task.order_id) if task.order_id else None
    clear_robot_task(db, task)

    if not order:
        return

    if task.task_type == "LOAD":
        update_order_items(db, order, "SORTED")
    elif task.task_type == "INSPECTION":
        update_order_items(db, order, "INSPECTED")
    elif task.task_type == "UNLOAD":
        if order.pickup_slot_id:
            pickup_slot = db.get(PickupSlot, order.pickup_slot_id)
            if pickup_slot and pickup_slot.status != "BLOCKED":
                pickup_slot.status = "OCCUPIED"
        order.status = "PICKUP_READY"


def clear_robot_task(db: Session, task: Task) -> None:
    if not task.assigned_robot_id:
        return

    robot = db.get(Robot, task.assigned_robot_id)

    if not robot or robot.current_task_id != task.task_id:
        return

    robot.current_task_id = None

    if robot.status not in UNAVAILABLE_ROBOT_STATUSES:
        robot.status = "IDLE"


def reserve_pickup_slot(db: Session, order: Order) -> bool:
    if order.pickup_slot_id:
        pickup_slot = db.get(PickupSlot, order.pickup_slot_id)
        if pickup_slot and pickup_slot.status == "EMPTY":
            pickup_slot.status = "RESERVED"
        return bool(pickup_slot and pickup_slot.status != "BLOCKED")

    pickup_slot = (
        db.query(PickupSlot)
        .filter(PickupSlot.status == "EMPTY")
        .order_by(PickupSlot.slot_id)
        .with_for_update()
        .first()
    )

    if not pickup_slot:
        return False

    pickup_slot.status = "RESERVED"
    order.pickup_slot_id = pickup_slot.slot_id
    return True


def update_order_items(db: Session, order: Order, status: str) -> None:
    items = db.query(OrderItem).filter(OrderItem.order_id == order.order_id).all()

    for item in items:
        item.status = status


def record_task_event(
    db: Session,
    task: Task,
    to_status: str,
    event_name: str,
    reason: str | None = None,
) -> None:
    db.add(
        TaskEvent(
            task_id=task.task_id,
            robot_id=task.assigned_robot_id,
            from_status=task.status,
            to_status=to_status,
            event_name=event_name,
            reason=reason,
            created_at=datetime.now(UTC),
        )
    )


def mark_task_assigned(
    db: Session,
    task: Task,
    reason: str | None = None,
) -> None:
    if task.status != "ASSIGNED":
        record_task_event(db, task, "ASSIGNED", "TASK_ASSIGNED", reason)

    task.status = "ASSIGNED"


def next_pending_task(db: Session, order: Order) -> Task | None:
    db.flush()

    return (
        db.query(Task)
        .filter(
            Task.order_id == order.order_id,
            Task.status.notin_(FINAL_TASK_STATUSES),
        )
        .order_by(Task.task_id)
        .first()
    )


def order_tasks(db: Session, order: Order) -> list[Task]:
    db.flush()

    return (
        db.query(Task)
        .filter(Task.order_id == order.order_id)
        .order_by(Task.task_id)
        .all()
    )
