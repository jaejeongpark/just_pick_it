import asyncio
import random
from datetime import UTC, datetime

from sqlalchemy.orm import Session

from app.database import SessionLocal
from app.models import ExceptionLog, Order, OrderItem, PickupSlot, Product, Robot, Task, TaskEvent
from app.services.realtime import broadcast_all_status


# Fleet Manager가 붙기 전 UI 시연을 위한 demo helper다.
# 실제 운영에서는 Fleet Manager가 상태를 판단하고 Control Server는 그 상태를 DB에 반영한다.
ORDER_TASK_TYPES = (
    "STANDBY_LOAD",
    "SORTING",
    "DELIVERY",
    "STANDBY_UNLOAD",
    "INSPECTION",
    "UNLOAD",
)

TASK_ROBOT_ID = {
    "STANDBY_LOAD": "AMR1",
    "SORTING": "COBOT1",
    "DELIVERY": "AMR1",
    "STANDBY_UNLOAD": "AMR1",
    "INSPECTION": "COBOT2",
    "UNLOAD": "COBOT2",
}

TASK_ROBOT_PREFIX = {
    "STANDBY_LOAD": "AMR",
    "SORTING": "COBOT",
    "DELIVERY": "AMR",
    "STANDBY_UNLOAD": "AMR",
    "INSPECTION": "COBOT",
    "UNLOAD": "COBOT",
}

ORDER_STATUS_BY_TASK = {
    "STANDBY_LOAD": "ORDER_WAIT",
    "SORTING": "SORTING",
    "DELIVERY": "DELIVERING",
    "STANDBY_UNLOAD": "DELIVERING",
    "INSPECTION": "INSPECTING",
    "UNLOAD": "DELIVERING",
}

ROBOT_STATUS_BY_TASK = {
    "STANDBY_LOAD": "STANDBY",
    "SORTING": "SORTING",
    "DELIVERY": "DELIVERING",
    "STANDBY_UNLOAD": "STANDBY",
    "INSPECTION": "INSPECTING",
    "UNLOAD": "UNLOADING",
}

FINAL_TASK_STATUSES = ("SUCCESS", "FAILED", "CANCELLED")
DEMO_STEP_DELAY_SECONDS = 2
DEMO_MAX_ADVANCE_STEPS = 10
DEMO_ESTIMATED_DURATION_SECONDS = DEMO_STEP_DELAY_SECONDS * 8
_demo_run_reserved = False


def demo_run_is_active() -> bool:
    return _demo_run_reserved


def reserve_demo_run() -> bool:
    global _demo_run_reserved

    if _demo_run_reserved:
        return False

    _demo_run_reserved = True
    return True


def release_demo_run() -> None:
    global _demo_run_reserved

    _demo_run_reserved = False


async def run_demo_order_sequence(
    step_delay_seconds: int = DEMO_STEP_DELAY_SECONDS,
) -> None:
    try:
        order_id = create_demo_order()
        await broadcast_all_status()
        await asyncio.sleep(step_delay_seconds)

        for _ in range(DEMO_MAX_ADVANCE_STEPS):
            if not advance_demo_order(order_id):
                break

            await broadcast_all_status()

            if demo_order_is_pickup_ready(order_id):
                break

            await asyncio.sleep(step_delay_seconds)

        await broadcast_all_status()
    finally:
        release_demo_run()


def create_demo_order() -> int:
    db = SessionLocal()

    try:
        prepare_demo_runtime_state(db)

        products = (
            db.query(Product)
            .filter(Product.stock_qty > 0)
            .order_by(Product.product_id)
            .all()
        )

        if not products:
            raise RuntimeError("no products available for demo order")

        item_count = min(len(products), random.choice((1, 2)))
        selected_products = random.sample(products, item_count)

        order = Order(status="ORDER_RECEIVED", priority=1)
        db.add(order)
        db.flush()

        order.order_no = f"ORD-{order.order_id:04d}"

        for product in selected_products:
            product.stock_qty -= 1
            db.add(
                OrderItem(
                    order_id=order.order_id,
                    product_id=product.product_id,
                    quantity=1,
                    status="WAITING",
                )
            )

        order_id = order.order_id
        db.commit()
        return order_id
    except Exception:
        db.rollback()
        raise
    finally:
        db.close()


def prepare_demo_runtime_state(db: Session) -> None:
    # 데모 버튼은 현재 DB 상태와 무관하게 새 시나리오를 보여주기 위한 기능이다.
    # 이전 테스트에서 남은 미완료 주문/task가 로봇을 잡고 있으면 새 주문이 ORDER_WAIT에 멈출 수 있다.
    stale_orders = (
        db.query(Order)
        .filter(Order.status.notin_(("PICKUP_READY", "COMPLETED", "ERROR")))
        .all()
    )
    stale_order_ids = [order.order_id for order in stale_orders]

    for order in stale_orders:
        order.status = "ERROR"

    if stale_order_ids:
        stale_tasks = (
            db.query(Task)
            .filter(Task.order_id.in_(stale_order_ids))
            .all()
        )

        for task in stale_tasks:
            if task.status not in FINAL_TASK_STATUSES:
                record_task_event(
                    db,
                    task,
                    "CANCELLED",
                    "DEMO_STALE_TASK_CANCELLED",
                    "Demo run cleaned stale task.",
                )
                task.status = "CANCELLED"
            clear_robot_task(db, task)

    robots = db.query(Robot).all()

    for robot in robots:
        if robot.status not in ("EMERGENCY_STOP", "ERROR", "OFFLINE"):
            robot.status = "IDLE"
            robot.current_task_id = None

    pickup_slots = db.query(PickupSlot).filter(PickupSlot.status == "RESERVED").all()

    for pickup_slot in pickup_slots:
        pickup_slot.status = "EMPTY"


def advance_demo_order(order_id: int) -> bool:
    db = SessionLocal()

    try:
        order = db.get(Order, order_id)

        if not order:
            return False

        if not order_tasks(db, order):
            create_order_workflow(db, order)
        else:
            advance_order_workflow(db, order)

        db.commit()
        return True
    except Exception:
        db.rollback()
        raise
    finally:
        db.close()


def demo_order_is_pickup_ready(order_id: int) -> bool:
    db = SessionLocal()

    try:
        order = db.get(Order, order_id)
        return bool(order and order.status == "PICKUP_READY")
    finally:
        db.close()


def create_order_workflow(db: Session, order: Order) -> None:
    existing_task = (
        db.query(Task)
        .filter(Task.order_id == order.order_id)
        .first()
    )

    if existing_task:
        start_next_task_for_order(db, order)
        return

    order_amr_id = find_order_amr_id(db, order)

    for task_type in ORDER_TASK_TYPES:
        assigned_robot_id = (
            order_amr_id
            if task_type in ("STANDBY_LOAD", "DELIVERY", "STANDBY_UNLOAD")
            else find_robot_id(db, task_type)
        )
        db.add(
            Task(
                order_id=order.order_id,
                assigned_robot_id=assigned_robot_id,
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
            record_task_event(db, task, "SUCCESS", "ORDER_COMPLETED")
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
        record_task_event(db, task, "FAILED", "ORDER_FAILED", detail)
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
        record_task_event(db, next_task, "ASSIGNED", "TASK_ASSIGNED")
        next_task.status = "ASSIGNED"
        order.status = "ORDER_WAIT"
        return

    robot = db.get(Robot, next_task.assigned_robot_id) if next_task.assigned_robot_id else None

    if not robot or not robot_is_available(db, robot):
        record_task_event(db, next_task, "ASSIGNED", "TASK_ASSIGNED")
        next_task.status = "ASSIGNED"
        order.status = "ORDER_WAIT"
        return

    if next_task.task_type == "INSPECTION":
        reserve_pickup_slot(db, order)

    record_task_event(db, next_task, "RUNNING", f"{next_task.task_type}_STARTED")
    next_task.status = "RUNNING"
    robot.current_task_id = next_task.task_id
    robot.status = ROBOT_STATUS_BY_TASK[next_task.task_type]
    order.status = ORDER_STATUS_BY_TASK[next_task.task_type]


def finish_task(db: Session, order: Order, task: Task) -> None:
    record_task_event(db, task, "SUCCESS", f"{task.task_type}_DONE")
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


def find_order_amr_id(db: Session, order: Order) -> str | None:
    amrs = (
        db.query(Robot)
        .filter(Robot.robot_id.like("AMR%"))
        .order_by(Robot.robot_id)
        .all()
    )

    if not amrs:
        return find_robot_id(db, "DELIVERY")

    # 데모에서는 주문별 AMR을 고정해서 DELIVERY와 UNLOAD가 같은 AMR을 쓰게 한다.
    index = (order.order_id - 1) % len(amrs)
    return amrs[index].robot_id


def robot_is_available(db: Session, robot: Robot) -> bool:
    if robot.status in ("EMERGENCY_STOP", "ERROR", "OFFLINE"):
        return False

    if not robot.current_task_id:
        return True

    current_task = db.get(Task, robot.current_task_id)
    return not current_task or current_task.status != "RUNNING"
