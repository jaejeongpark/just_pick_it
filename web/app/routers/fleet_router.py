from datetime import UTC, datetime

from fastapi import APIRouter, BackgroundTasks, Depends, HTTPException
from sqlalchemy.orm import Session

from app.database import get_db
from app.models import ExceptionLog, Order, PickupSlot, Robot, Task, TaskEvent
from app.schemas import (
    FleetExceptionCreate,
    FleetExceptionRead,
    FleetOrderStateUpdate,
    FleetPickupSlotStateUpdate,
    FleetRobotStateUpdate,
    FleetStateUpdateRead,
    FleetTaskCreate,
    FleetTaskEventCreate,
    FleetTaskEventRead,
    FleetTaskRead,
    FleetTaskStateUpdate,
)
from app.services.realtime import broadcast_all_status


router = APIRouter(prefix="/api/fleet", tags=["fleet"])


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


@router.post("/tasks", response_model=FleetTaskRead, status_code=201)
def create_fleet_task(
    task_create: FleetTaskCreate,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    if task_create.order_id is not None and not db.get(Order, task_create.order_id):
        raise HTTPException(status_code=404, detail="order not found")

    if task_create.assigned_robot_id is not None and not db.get(Robot, task_create.assigned_robot_id):
        raise HTTPException(status_code=404, detail="robot not found")

    task = Task(
        order_id=task_create.order_id,
        assigned_robot_id=task_create.assigned_robot_id,
        task_type=task_create.task_type,
        status=task_create.status,
        source_zone_id=task_create.source_zone_id,
        target_zone_id=task_create.target_zone_id,
        result_message=task_create.result_message,
    )
    db.add(task)
    db.commit()
    db.refresh(task)
    background_tasks.add_task(broadcast_all_status)
    return {
        "status": "ok",
        "task_id": task.task_id,
    }


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
    task = db.get(Task, task_id)

    if not task:
        raise HTTPException(status_code=404, detail="task not found")

    if state_update.status is not None:
        task.status = state_update.status

    if state_update.assigned_robot_id is not None:
        robot = db.get(Robot, state_update.assigned_robot_id)

        if not robot:
            raise HTTPException(status_code=404, detail="robot not found")

        task.assigned_robot_id = state_update.assigned_robot_id

    if state_update.result_message is not None:
        task.result_message = state_update.result_message

    db.commit()
    background_tasks.add_task(broadcast_all_status)
    return {"status": "ok"}


@router.post("/tasks/{task_id}/events", response_model=FleetTaskEventRead, status_code=201)
def create_task_event(
    task_id: int,
    event_create: FleetTaskEventCreate,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    task = db.get(Task, task_id)

    if not task:
        raise HTTPException(status_code=404, detail="task not found")

    if event_create.robot_id is not None and not db.get(Robot, event_create.robot_id):
        raise HTTPException(status_code=404, detail="robot not found")

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
    robot = db.get(Robot, robot_id)

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

    db.commit()
    background_tasks.add_task(broadcast_all_status)
    return {"status": "ok"}


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

