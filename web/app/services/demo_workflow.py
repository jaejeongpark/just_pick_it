from datetime import UTC, datetime

from sqlalchemy.orm import Session

from app.models import ExceptionLog, Order, OrderItem, PickupSlot, Robot, Task


# Fleet Manager가 붙기 전 UI 시연을 위한 demo helper다.
# 실제 운영에서는 Fleet Manager가 상태를 판단하고 Control Server는 그 상태를 DB에 반영한다.
TASK_ROBOT_ID = {
    "SORTING": "COBOT1",
    "DELIVERY": "AMR1",
    "INSPECTION": "COBOT2",
    "UNLOAD": "AMR2",
}

TASK_ROBOT_PREFIX = {
    "SORTING": "COBOT",
    "DELIVERY": "AMR",
    "INSPECTION": "COBOT",
    "UNLOAD": "AMR",
}

ORDER_STATUS_BY_TASK = {
    "SORTING": "SORTING",
    "DELIVERY": "DELIVERING",
    "INSPECTION": "INSPECTING",
    "UNLOAD": "DELIVERING",
}

ROBOT_STATUS_BY_TASK = {
    "SORTING": "SORTING",
    "DELIVERY": "DELIVERING",
    "INSPECTION": "INSPECTING",
    "UNLOAD": "UNLOADING",
}

FINAL_TASK_STATUSES = ("SUCCESS", "FAILED", "CANCELLED")


def create_order_workflow(db: Session, order: Order) -> None:
    existing_task = (
        db.query(Task)
        .filter(Task.order_id == order.order_id)
        .first()
    )

    if existing_task:
        start_next_task_for_order(db, order)
        return

    for task_type in ("SORTING", "DELIVERY", "INSPECTION", "UNLOAD"):
        db.add(
            Task(
                order_id=order.order_id,
                assigned_robot_id=find_robot_id(db, task_type),
                task_type=task_type,
                status="QUEUED",
            )
        )

    db.flush()
    start_next_task_for_order(db, order)


def advance_order_workflow(db: Session, order: Order) -> None:
    if order.status in ("PICKUP_READY", "COMPLETED"):
        return

    if not order_tasks(db, order):
        create_order_workflow(db, order)
        return

    running_task = current_order_task(db, order)

    if not running_task:
        start_next_task_for_order(db, order)
        return

    finish_task(db, order, running_task)

    if running_task.task_type != "UNLOAD":
        start_next_task_for_order(db, order)

    start_waiting_orders(db)


def complete_order_workflow(db: Session, order: Order) -> None:
    order.status = "COMPLETED"

    if order.pickup_slot_id:
        pickup_slot = db.get(PickupSlot, order.pickup_slot_id)
        if pickup_slot:
            pickup_slot.status = "EMPTY"

    for task in order_tasks(db, order):
        if task.status not in FINAL_TASK_STATUSES:
            task.status = "SUCCESS"
        clear_robot_task(db, task)

    start_waiting_orders(db)


def fail_order_workflow(
    db: Session,
    order: Order,
    task_id: int | None = None,
    robot_id: str | None = None,
    exception_type: str = "SYSTEM_ERROR",
    detail: str | None = None,
) -> None:
    task = db.get(Task, task_id) if task_id else current_order_task(db, order)

    if task:
        task.status = "FAILED"
        clear_robot_task(db, task)

    robot = db.get(Robot, robot_id) if robot_id else None

    if robot and robot.status not in ("EMERGENCY_STOP", "OFFLINE"):
        robot.status = "ERROR"

    order.status = "ERROR"
    failure_robot_id = robot_id or (task.assigned_robot_id if task else None)
    db.add(
        ExceptionLog(
            robot_id=failure_robot_id,
            task_id=task.task_id if task else task_id,
            order_id=order.order_id,
            exception_type=exception_type,
            detail=detail or "Fleet manager reported an order workflow failure.",
            is_resolved=False,
            created_at=datetime.now(UTC),
        )
    )


def apply_fleet_order_event(
    db: Session,
    order: Order,
    event: str,
    task_id: int | None = None,
    robot_id: str | None = None,
    exception_type: str = "SYSTEM_ERROR",
    detail: str | None = None,
) -> None:
    if event == "ORDER_ACCEPTED":
        create_order_workflow(db, order)
    elif event == "TASK_DONE":
        advance_order_workflow(db, order)
    elif event in ("TASK_FAILED", "EXCEPTION_RAISED"):
        fail_order_workflow(db, order, task_id, robot_id, exception_type, detail)


def start_waiting_orders(db: Session) -> None:
    db.flush()

    orders = (
        db.query(Order)
        .filter(Order.status.notin_(("PICKUP_READY", "COMPLETED", "ERROR")))
        .order_by(Order.priority.desc(), Order.order_id)
        .all()
    )

    for order in orders:
        if current_order_task(db, order):
            continue

        start_next_task_for_order(db, order)


def start_next_task_for_order(db: Session, order: Order) -> None:
    db.flush()

    next_task = next_pending_task(db, order)

    if not next_task:
        order.status = "PICKUP_READY"
        return

    if next_task.task_type not in ORDER_STATUS_BY_TASK:
        next_task.status = "ASSIGNED"
        order.status = "ORDER_WAIT"
        return

    robot = db.get(Robot, next_task.assigned_robot_id) if next_task.assigned_robot_id else None

    if not robot or not robot_is_available(db, robot):
        next_task.status = "ASSIGNED"
        order.status = "ORDER_WAIT"
        return

    if next_task.task_type == "UNLOAD":
        reserve_pickup_slot(db, order)

    next_task.status = "RUNNING"
    robot.current_task_id = next_task.task_id
    robot.status = ROBOT_STATUS_BY_TASK[next_task.task_type]
    order.status = ORDER_STATUS_BY_TASK[next_task.task_type]


def finish_task(db: Session, order: Order, task: Task) -> None:
    task.status = "SUCCESS"
    clear_robot_task(db, task)

    if task.task_type == "SORTING":
        update_order_items(db, order, "SORTED")
    elif task.task_type == "INSPECTION":
        update_order_items(db, order, "INSPECTED")
    elif task.task_type == "UNLOAD":
        if order.pickup_slot_id:
            pickup_slot = db.get(PickupSlot, order.pickup_slot_id)
            if pickup_slot:
                pickup_slot.status = "OCCUPIED"
        order.status = "PICKUP_READY"


def clear_robot_task(db: Session, task: Task) -> None:
    if not task.assigned_robot_id:
        return

    robot = db.get(Robot, task.assigned_robot_id)

    if not robot or robot.current_task_id != task.task_id:
        return

    robot.current_task_id = None

    if robot.status not in ("EMERGENCY_STOP", "ERROR", "OFFLINE"):
        robot.status = "IDLE"


def reserve_pickup_slot(db: Session, order: Order) -> None:
    if order.pickup_slot_id:
        pickup_slot = db.get(PickupSlot, order.pickup_slot_id)
        if pickup_slot and pickup_slot.status == "EMPTY":
            pickup_slot.status = "RESERVED"
        return

    pickup_slot = (
        db.query(PickupSlot)
        .filter(PickupSlot.status == "EMPTY")
        .order_by(PickupSlot.slot_id)
        .with_for_update()
        .first()
    )

    if not pickup_slot:
        return

    pickup_slot.status = "RESERVED"
    order.pickup_slot_id = pickup_slot.slot_id


def update_order_items(db: Session, order: Order, status: str) -> None:
    items = db.query(OrderItem).filter(OrderItem.order_id == order.order_id).all()

    for item in items:
        item.status = status


def current_order_task(db: Session, order: Order) -> Task | None:
    db.flush()

    return (
        db.query(Task)
        .filter(Task.order_id == order.order_id, Task.status == "RUNNING")
        .order_by(Task.task_id)
        .first()
    )


def next_pending_task(db: Session, order: Order) -> Task | None:
    db.flush()

    return (
        db.query(Task)
        .filter(Task.order_id == order.order_id, Task.status.notin_(FINAL_TASK_STATUSES))
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


def find_robot_id(db: Session, task_type: str) -> str | None:
    preferred_robot = db.get(Robot, TASK_ROBOT_ID[task_type])

    if preferred_robot:
        return preferred_robot.robot_id

    fallback_robot = (
        db.query(Robot)
        .filter(Robot.robot_id.like(f"{TASK_ROBOT_PREFIX[task_type]}%"))
        .order_by(Robot.robot_id)
        .first()
    )

    return fallback_robot.robot_id if fallback_robot else None


def robot_is_available(db: Session, robot: Robot) -> bool:
    if robot.status in ("EMERGENCY_STOP", "ERROR", "OFFLINE"):
        return False

    if not robot.current_task_id:
        return True

    current_task = db.get(Task, robot.current_task_id)
    return not current_task or current_task.status != "RUNNING"
