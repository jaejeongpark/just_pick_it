from sqlalchemy.orm import Session

from app.models import Order, OrderItem, PickupSlot, Product, Robot, StockingItem, Task
from app.services.robot_runtime_policy import FINAL_TASK_STATUSES, UNAVAILABLE_ROBOT_STATUSES
from app.services.stocking_service import FINAL_STOCKING_ITEM_STATUSES, resolve_stock_delta


ORDER_PRIORITY = 2

ORDER_STATUS_BY_RUNNING_TASK = {
    "MOVE_TO_PRODUCT": "SORTING",
    "SORTING_AND_LOAD": "SORTING",
    "MOVE_TO_PICKUP": "DELIVERING",
    "INSPECTION": "INSPECTING",
    "UNLOAD": "INSPECTING",
}

PICKY_STATE_BY_TASK = {
    "MOVE_TO_PRODUCT": "MOVING_TO_PRODUCT",
    "MOVE_TO_PICKUP": "MOVING_TO_PICKUP",
    "MOVE_TO_STOCK": "MOVING_TO_STOCK",
    "MOVE_TO_STORAGE": "MOVING_TO_STORAGE",
    "RETURN_HOME": "RETURNING",
    "CHARGE": "CHARGING",
}

COBOT_STATE_BY_TASK = {
    "SORTING_AND_LOAD": "SORTING",
    "INSPECTION": "INSPECTING",
    "UNLOAD": "UNLOADING",
    "STOCKING_PICK": "STOCKING_SORTING",
    "STOCKING_PLACE": "STOCKING_PLACING",
}


def create_order_workflow(_db: Session, order: Order) -> None:
    """Move a newly created order into the Fleet Manager waiting queue.

    Task creation and robot assignment are owned by Fleet Manager. The Control
    Server only stores the order and order_item rows, then exposes them through
    snapshot/event APIs.
    """

    order.status = "ORDER_WAIT"
    order.priority = ORDER_PRIORITY


def complete_order_workflow(db: Session, order: Order) -> None:
    order.status = "COMPLETED"

    if order.pickup_slot_id:
        pickup_slot = db.get(PickupSlot, order.pickup_slot_id)
        if pickup_slot and pickup_slot.status != "BLOCKED":
            pickup_slot.status = "EMPTY"

    db.flush()


def apply_task_runtime_state(db: Session, task: Task, previous_status: str | None = None) -> None:
    if task.status == "RUNNING":
        start_runtime_task(db, task)
    elif task.status == "SUCCESS":
        finish_runtime_task(db, task, previous_status=previous_status)
    elif task.status in ("FAILED", "CANCELLED"):
        update_stocking_item_status(db, task, "FAILED" if task.status == "FAILED" else "CANCELLED")
        clear_robot_task(db, task)
        if task.status == "FAILED" and task.order_id:
            order = db.get(Order, task.order_id)
            if order:
                order.status = "ERROR"


def start_runtime_task(db: Session, task: Task) -> None:
    robot = db.get(Robot, task.assigned_robot_id) if task.assigned_robot_id else None
    order = db.get(Order, task.order_id) if task.order_id else None

    if robot:
        robot.current_task_id = task.task_id
        robot.robot_status = "CHARGING" if task.task_type == "CHARGE" else "BUSY"
        apply_robot_state_for_task(robot, task.task_type)

    update_stocking_item_status(db, task, "IN_PROGRESS")

    if order:
        if task.task_type == "INSPECTION":
            reserve_pickup_slot(db, order)

        order.status = ORDER_STATUS_BY_RUNNING_TASK.get(task.task_type, order.status)


def finish_runtime_task(
    db: Session,
    task: Task,
    previous_status: str | None = None,
) -> None:
    order = db.get(Order, task.order_id) if task.order_id else None
    clear_robot_task(db, task)

    if task.task_type == "STOCKING_PLACE":
        apply_stocking_success(db, task, previous_status)

    if not order:
        return

    if task.task_type == "SORTING_AND_LOAD":
        update_order_item(db, task.order_item_id, "SORTED")
    elif task.task_type == "INSPECTION":
        update_order_items(db, order, "INSPECTED")
    elif task.task_type == "UNLOAD":
        if order.pickup_slot_id:
            pickup_slot = db.get(PickupSlot, order.pickup_slot_id)
            if pickup_slot and pickup_slot.status != "BLOCKED":
                pickup_slot.status = "OCCUPIED"

        order.status = "PICKUP_READY"


def apply_robot_state_for_task(robot: Robot, task_type: str) -> None:
    if robot.robot_type == "PICKY":
        robot.picky_state = PICKY_STATE_BY_TASK.get(task_type, robot.picky_state)
    elif robot.robot_type == "COBOT":
        robot.cobot_state = COBOT_STATE_BY_TASK.get(task_type, robot.cobot_state)


def clear_robot_task(db: Session, task: Task) -> None:
    if not task.assigned_robot_id:
        return

    robot = db.get(Robot, task.assigned_robot_id)

    if not robot or robot.current_task_id != task.task_id:
        return

    robot.current_task_id = None

    if robot.robot_status not in UNAVAILABLE_ROBOT_STATUSES:
        robot.robot_status = "IDLE"

    if robot.robot_type == "PICKY":
        robot.picky_state = "STANDBY"
    elif robot.robot_type == "COBOT":
        robot.cobot_state = "STANDBY"


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
        .with_for_update(skip_locked=True)
        .first()
    )

    if not pickup_slot:
        return False

    pickup_slot.status = "RESERVED"
    order.pickup_slot_id = pickup_slot.slot_id
    return True


def update_order_item(db: Session, order_item_id: int | None, status: str) -> None:
    if order_item_id is None:
        return

    item = db.get(OrderItem, order_item_id)

    if item:
        item.status = status


def update_order_items(db: Session, order: Order, status: str) -> None:
    items = db.query(OrderItem).filter(OrderItem.order_id == order.order_id).all()

    for item in items:
        item.status = status


def update_stocking_item_status(db: Session, task: Task, status: str) -> None:
    if task.stocking_item_id is None:
        return

    stocking_item = db.get(StockingItem, task.stocking_item_id)

    if stocking_item and stocking_item.status not in FINAL_STOCKING_ITEM_STATUSES:
        stocking_item.status = status


def apply_stocking_success(
    db: Session,
    task: Task,
    previous_status: str | None = None,
) -> int | None:
    if previous_status == "SUCCESS" or task.stocking_item_id is None:
        return None

    stocking_item = db.get(StockingItem, task.stocking_item_id)

    if not stocking_item or stocking_item.status == "COMPLETED":
        return stocking_item.stock_delta if stocking_item else None

    product = db.get(Product, stocking_item.product_id)

    if not product:
        return None

    stock_delta = resolve_stock_delta(stocking_item)

    if stock_delta is None:
        return None

    product.stock_qty += stock_delta
    stocking_item.stock_delta = stock_delta
    stocking_item.status = "COMPLETED"
    return stock_delta
