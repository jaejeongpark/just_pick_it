from datetime import UTC, datetime

from fastapi import APIRouter, BackgroundTasks, Depends, HTTPException
from sqlalchemy.orm import Session

from app.database import get_db
from app.models import ExceptionLog, Order, PickupSlot, Robot, Task, TaskEvent
from app.schemas import (
    FleetExceptionCreate,
    FleetExceptionRead,
    FleetOrderSummaryRead,
    FleetOrderStateUpdate,
    FleetPickupSlotAssignmentRead,
    FleetPickupSlotRead,
    FleetPickupSlotStateUpdate,
    FleetRobotStateUpdate,
    FleetStateUpdateRead,
    FleetTaskEventCreate,
    FleetTaskEventRead,
    FleetTaskStateUpdate,
    FleetTaskSummaryRead,
    OrderStatus,
    PickupSlotStatus,
    TaskStatus,
    TaskType,
)
from app.services.robot_runtime_policy import FINAL_TASK_STATUSES
from app.services.workflow_service import apply_task_runtime_state, assign_ready_tasks
from app.services.realtime import broadcast_all_status
from app.services.status_service import build_task_summary


router = APIRouter(prefix="/api/fleet", tags=["robot-runtime"])


def build_task_event_response(task_event: TaskEvent) -> dict:
    return {
        "event_id": task_event.event_id,
        "task_id": task_event.task_id,
        "robot_id": task_event.robot_id,
        "from_status": task_event.from_status,
        "to_status": task_event.to_status,
        "event_name": task_event.event_name,
        "reason": task_event.reason,
        "created_at": task_event.created_at.isoformat() if task_event.created_at else None,
    }


def build_order_summary_response(db: Session, order: Order) -> dict:
    pickup_slot = db.get(PickupSlot, order.pickup_slot_id) if order.pickup_slot_id else None
    current_task = (
        db.query(Task)
        .filter(
            Task.order_id == order.order_id,
            Task.status.notin_(FINAL_TASK_STATUSES),
        )
        .order_by(Task.task_id)
        .first()
    )

    return {
        "order_id": order.order_id,
        "order_no": order.order_no,
        "status": order.status,
        "priority": order.priority,
        "pickup_slot_id": order.pickup_slot_id,
        "pickup_slot_name": pickup_slot.slot_name if pickup_slot else None,
        "current_task_id": current_task.task_id if current_task else None,
        "current_task_type": current_task.task_type if current_task else None,
        "current_task_status": current_task.status if current_task else None,
        "assigned_robot_id": current_task.assigned_robot_id if current_task else None,
    }


def build_pickup_slot_response(db: Session, pickup_slot: PickupSlot) -> dict:
    active_order = (
        db.query(Order)
        .filter(Order.pickup_slot_id == pickup_slot.slot_id)
        .filter(Order.status != "COMPLETED")
        .order_by(Order.order_id.desc())
        .first()
    )

    return {
        "slot_id": pickup_slot.slot_id,
        "slot_name": pickup_slot.slot_name,
        "status": pickup_slot.status,
        "order_id": active_order.order_id if active_order else None,
        "order_no": active_order.order_no if active_order else None,
    }


def build_pickup_slot_assignment_response(order: Order, pickup_slot: PickupSlot) -> dict:
    return {
        "status": "ok",
        "order_id": order.order_id,
        "order_no": order.order_no,
        "pickup_slot_id": pickup_slot.slot_id,
        "slot_name": pickup_slot.slot_name,
        "slot_status": pickup_slot.status,
    }


def release_pickup_slot_if_unused(db: Session, slot_id: int | None) -> None:
    if slot_id is None:
        return

    has_active_order = (
        db.query(Order)
        .filter(Order.pickup_slot_id == slot_id)
        .filter(Order.status != "COMPLETED")
        .first()
        is not None
    )

    if has_active_order:
        return

    pickup_slot = db.get(PickupSlot, slot_id)

    if pickup_slot and pickup_slot.status != "BLOCKED":
        pickup_slot.status = "EMPTY"


def sync_order_pickup_slot(db: Session, order: Order, previous_slot_id: int | None) -> None:
    if previous_slot_id != order.pickup_slot_id:
        release_pickup_slot_if_unused(db, previous_slot_id)

    if order.pickup_slot_id is None:
        return

    pickup_slot = db.get(PickupSlot, order.pickup_slot_id)

    if not pickup_slot or pickup_slot.status == "BLOCKED":
        return

    if order.status == "COMPLETED":
        pickup_slot.status = "EMPTY"
    elif order.status == "PICKUP_READY":
        pickup_slot.status = "OCCUPIED"
    else:
        pickup_slot.status = "RESERVED"


@router.get("/tasks", response_model=list[FleetTaskSummaryRead])
def list_fleet_tasks(
    robot_id: str | None = None,
    status: TaskStatus | None = None,
    task_type: TaskType | None = None,
    order_id: int | None = None,
    db: Session = Depends(get_db),
):
    task_query = db.query(Task)

    if robot_id is not None:
        task_query = task_query.filter(Task.assigned_robot_id == robot_id)

    if status is not None:
        task_query = task_query.filter(Task.status == status)

    if task_type is not None:
        task_query = task_query.filter(Task.task_type == task_type)

    if order_id is not None:
        task_query = task_query.filter(Task.order_id == order_id)

    tasks = task_query.order_by(Task.task_id.desc()).all()
    return [build_task_summary(db, task) for task in tasks]


@router.get("/orders/{order_id}/tasks", response_model=list[FleetTaskSummaryRead])
def list_order_tasks(
    order_id: int,
    db: Session = Depends(get_db),
):
    if not db.get(Order, order_id):
        raise HTTPException(status_code=404, detail="order not found")

    tasks = (
        db.query(Task)
        .filter(Task.order_id == order_id)
        .order_by(Task.task_id)
        .all()
    )
    return [build_task_summary(db, task) for task in tasks]


@router.patch("/orders/{order_id}", response_model=FleetStateUpdateRead)
def update_order_state(
    order_id: int,
    state_update: FleetOrderStateUpdate,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    order = db.get(Order, order_id)

    if not order:
        raise HTTPException(status_code=404, detail="order not found")

    previous_slot_id = order.pickup_slot_id

    if state_update.status is not None:
        order.status = state_update.status

    if "pickup_slot_id" in state_update.model_fields_set and state_update.pickup_slot_id is not None:
        pickup_slot = db.get(PickupSlot, state_update.pickup_slot_id)

        if not pickup_slot:
            raise HTTPException(status_code=404, detail="pickup slot not found")

    if "pickup_slot_id" in state_update.model_fields_set:
        order.pickup_slot_id = state_update.pickup_slot_id

    sync_order_pickup_slot(db, order, previous_slot_id)
    db.commit()
    background_tasks.add_task(broadcast_all_status)
    return {"status": "ok"}


@router.patch("/tasks/{task_id}", response_model=FleetStateUpdateRead)
def update_task_state(
    task_id: int,
    state_update: FleetTaskStateUpdate,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    task = (
        db.query(Task)
        .filter(Task.task_id == task_id)
        .with_for_update()
        .one_or_none()
    )

    if not task:
        raise HTTPException(status_code=404, detail="task not found")

    if state_update.current_status is not None and task.status != state_update.current_status:
        raise HTTPException(
            status_code=409,
            detail={
                "message": "task status conflict",
                "expected_status": state_update.current_status,
                "current_status": task.status,
            },
        )

    previous_status = task.status

    if state_update.status is not None:
        task.status = state_update.status

    if state_update.assigned_robot_id is not None:
        robot = (
            db.query(Robot)
            .filter(Robot.robot_id == state_update.assigned_robot_id)
            .with_for_update()
            .one_or_none()
        )

        if not robot:
            raise HTTPException(status_code=404, detail="robot not found")

        task.assigned_robot_id = state_update.assigned_robot_id

    if state_update.result_message is not None:
        task.result_message = state_update.result_message

    if state_update.status is not None and previous_status != task.status:
        db.add(
            TaskEvent(
                task_id=task.task_id,
                robot_id=task.assigned_robot_id,
                from_status=previous_status,
                to_status=task.status,
                event_name=f"TASK_{task.status}",
                reason=task.result_message,
                created_at=datetime.now(UTC),
            )
        )

    apply_task_runtime_state(db, task)
    db.commit()
    background_tasks.add_task(broadcast_all_status)
    return {
        "status": "ok",
        "previous_status": previous_status,
        "current_status": task.status,
    }


@router.post("/tasks/{task_id}/events", response_model=FleetTaskEventRead, status_code=201)
def create_task_event(
    task_id: int,
    event_create: FleetTaskEventCreate,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    task = (
        db.query(Task)
        .filter(Task.task_id == task_id)
        .with_for_update()
        .one_or_none()
    )

    if not task:
        raise HTTPException(status_code=404, detail="task not found")

    if event_create.robot_id is not None and not db.get(Robot, event_create.robot_id):
        raise HTTPException(status_code=404, detail="robot not found")

    if (
        event_create.update_task_status
        and event_create.from_status is not None
        and task.status != event_create.from_status
    ):
        raise HTTPException(
            status_code=409,
            detail={
                "message": "task status conflict",
                "expected_status": event_create.from_status,
                "current_status": task.status,
            },
        )

    from_status = event_create.from_status or task.status
    task_event = TaskEvent(
        task_id=task.task_id,
        robot_id=event_create.robot_id or task.assigned_robot_id,
        from_status=from_status,
        to_status=event_create.to_status,
        event_name=event_create.event_name,
        reason=event_create.reason,
        created_at=datetime.now(UTC),
    )
    db.add(task_event)

    if event_create.update_task_status:
        task.status = event_create.to_status

    apply_task_runtime_state(db, task)
    db.commit()
    db.refresh(task_event)
    background_tasks.add_task(broadcast_all_status)
    return build_task_event_response(task_event)


@router.get("/tasks/{task_id}/events", response_model=list[FleetTaskEventRead])
def list_task_events(
    task_id: int,
    db: Session = Depends(get_db),
):
    if not db.get(Task, task_id):
        raise HTTPException(status_code=404, detail="task not found")

    task_events = (
        db.query(TaskEvent)
        .filter(TaskEvent.task_id == task_id)
        .order_by(TaskEvent.created_at, TaskEvent.event_id)
        .all()
    )
    return [
        build_task_event_response(task_event)
        for task_event in task_events
    ]


@router.patch("/robots/{robot_id}", response_model=FleetStateUpdateRead)
def update_robot_state(
    robot_id: str,
    state_update: FleetRobotStateUpdate,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    robot = (
        db.query(Robot)
        .filter(Robot.robot_id == robot_id)
        .with_for_update()
        .one_or_none()
    )

    if not robot:
        raise HTTPException(status_code=404, detail="robot not found")

    if state_update.status is not None:
        robot.status = state_update.status

    if "current_task_id" in state_update.model_fields_set and state_update.current_task_id is not None:
        task = db.get(Task, state_update.current_task_id)

        if not task:
            raise HTTPException(status_code=404, detail="task not found")

    if "current_task_id" in state_update.model_fields_set:
        robot.current_task_id = state_update.current_task_id

    if "battery_level" in state_update.model_fields_set:
        robot.battery_level = state_update.battery_level

    if "pos_x" in state_update.model_fields_set:
        robot.pos_x = state_update.pos_x

    if "pos_y" in state_update.model_fields_set:
        robot.pos_y = state_update.pos_y

    if "pos_theta" in state_update.model_fields_set:
        robot.pos_theta = state_update.pos_theta

    assign_ready_tasks(db, preferred_robot_id=robot_id)
    db.commit()
    background_tasks.add_task(broadcast_all_status)
    return {"status": "ok"}


@router.post("/assignments/run")
def run_task_assignment(
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    assigned_count = assign_ready_tasks(db)
    db.commit()
    background_tasks.add_task(broadcast_all_status)
    return {"status": "ok", "assigned_count": assigned_count}


@router.get("/orders", response_model=list[FleetOrderSummaryRead])
def list_fleet_orders(
    status: OrderStatus | None = None,
    include_completed: bool = False,
    db: Session = Depends(get_db),
):
    order_query = db.query(Order)

    if status is not None:
        order_query = order_query.filter(Order.status == status)
    elif not include_completed:
        order_query = order_query.filter(Order.status.notin_(("COMPLETED", "ERROR")))

    orders = (
        order_query
        .order_by(Order.priority, Order.order_id)
        .limit(50)
        .all()
    )
    return [build_order_summary_response(db, order) for order in orders]


@router.post("/orders/{order_id}/assign-pickup-slot", response_model=FleetPickupSlotAssignmentRead)
def assign_pickup_slot(
    order_id: int,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    order = db.get(Order, order_id)

    if not order:
        raise HTTPException(status_code=404, detail="order not found")

    if order.status == "COMPLETED":
        raise HTTPException(status_code=409, detail="completed order cannot be assigned a pickup slot")

    if order.pickup_slot_id is not None:
        pickup_slot = db.get(PickupSlot, order.pickup_slot_id)

        if not pickup_slot:
            raise HTTPException(status_code=404, detail="pickup slot not found")

        if pickup_slot.status == "BLOCKED":
            raise HTTPException(status_code=409, detail="assigned pickup slot is blocked")

        if pickup_slot.status == "EMPTY":
            pickup_slot.status = "RESERVED"
            db.commit()
            background_tasks.add_task(broadcast_all_status)

        return build_pickup_slot_assignment_response(order, pickup_slot)

    pickup_slot = (
        db.query(PickupSlot)
        .filter(PickupSlot.status == "EMPTY")
        .order_by(PickupSlot.slot_id)
        .with_for_update(skip_locked=True)
        .first()
    )

    if not pickup_slot:
        raise HTTPException(status_code=409, detail="empty pickup slot not found")

    pickup_slot.status = "RESERVED"
    order.pickup_slot_id = pickup_slot.slot_id

    db.commit()
    db.refresh(order)
    db.refresh(pickup_slot)
    background_tasks.add_task(broadcast_all_status)
    return build_pickup_slot_assignment_response(order, pickup_slot)


@router.get("/pickup-slots", response_model=list[FleetPickupSlotRead])
def list_pickup_slots(
    status: PickupSlotStatus | None = None,
    db: Session = Depends(get_db),
):
    slot_query = db.query(PickupSlot)

    if status is not None:
        slot_query = slot_query.filter(PickupSlot.status == status)

    pickup_slots = slot_query.order_by(PickupSlot.slot_id).all()
    return [
        build_pickup_slot_response(db, pickup_slot)
        for pickup_slot in pickup_slots
    ]


@router.patch("/pickup-slots/{slot_id}", response_model=FleetStateUpdateRead)
def update_pickup_slot_state(
    slot_id: int,
    state_update: FleetPickupSlotStateUpdate,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    pickup_slot = db.get(PickupSlot, slot_id)

    if not pickup_slot:
        raise HTTPException(status_code=404, detail="pickup slot not found")

    if state_update.status is not None:
        pickup_slot.status = state_update.status

    db.commit()
    background_tasks.add_task(broadcast_all_status)
    return {"status": "ok"}


@router.post("/exceptions", response_model=FleetExceptionRead, status_code=201)
def create_exception_report(
    exception_create: FleetExceptionCreate,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    if exception_create.robot_id is not None and not db.get(Robot, exception_create.robot_id):
        raise HTTPException(status_code=404, detail="robot not found")

    if exception_create.task_id is not None and not db.get(Task, exception_create.task_id):
        raise HTTPException(status_code=404, detail="task not found")

    if exception_create.order_id is not None and not db.get(Order, exception_create.order_id):
        raise HTTPException(status_code=404, detail="order not found")

    exception = ExceptionLog(
        robot_id=exception_create.robot_id,
        task_id=exception_create.task_id,
        order_id=exception_create.order_id,
        exception_type=exception_create.exception_type,
        detail=exception_create.detail,
        is_resolved=False,
        created_at=datetime.now(UTC),
    )
    db.add(exception)
    db.commit()
    db.refresh(exception)
    background_tasks.add_task(broadcast_all_status)
    return {
        "status": "ok",
        "exception_id": exception.exception_id,
    }
