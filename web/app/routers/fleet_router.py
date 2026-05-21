from datetime import UTC, datetime

from fastapi import APIRouter, BackgroundTasks, Depends, HTTPException, WebSocket, WebSocketDisconnect
from sqlalchemy.orm import Session

from app.database import get_db
from app.models import ExceptionLog, Order, OrderItem, PickupSlot, Product, Robot, RobotUnit, StockingItem, Task, TaskEvent, Zone
from app.schemas import (
    FleetExceptionCreate,
    FleetExceptionRead,
    FleetOrderSummaryRead,
    FleetOrderStateUpdate,
    FleetPickupSlotAssignmentRead,
    FleetPickupSlotRead,
    FleetPickupSlotStateUpdate,
    FleetRobotRuntimeRead,
    FleetRobotRunningTaskRead,
    FleetRobotStateUpdate,
    FleetStateUpdateRead,
    FleetStockingComplete,
    FleetStockingItemCreate,
    FleetStockingItemRead,
    FleetStockingItemUpdate,
    FleetTaskBulkCreate,
    FleetTaskBulkCreateRead,
    FleetTaskEventCreate,
    FleetTaskEventRead,
    FleetTaskStateUpdate,
    FleetTaskSummaryRead,
    FleetZoneRead,
    OrderStatus,
    PickupSlotStatus,
    StockingItemStatus,
    TaskStatus,
    TaskType,
)
from app.services.realtime import broadcast_all_status, broadcast_fleet_event, fleet_event_websockets
from app.services.robot_runtime_policy import FINAL_TASK_STATUSES, TASK_ROBOT_TYPE
from app.services.status_service import build_admin_status, build_robot_summary, build_task_summary, build_zone_pose
from app.services.stocking_service import (
    build_stocking_item_summary,
    create_stocking_item_record,
    resolve_stocking_policy,
)
from app.services.workflow_service import apply_task_runtime_state


router = APIRouter(prefix="/api/fleet", tags=["fleet-manager"])


def get_robot_by_identifier(db: Session, robot_identifier: int | str | None) -> Robot | None:
    if robot_identifier is None:
        return None

    if isinstance(robot_identifier, int):
        return db.get(Robot, robot_identifier)

    if robot_identifier.isdigit():
        robot = db.get(Robot, int(robot_identifier))

        if robot:
            return robot

    return db.query(Robot).filter(Robot.robot_name == robot_identifier).first()


def get_robot_from_payload(
    db: Session,
    robot_id: int | str | None = None,
    robot_name: str | None = None,
) -> Robot | None:
    if robot_id is not None:
        robot = get_robot_by_identifier(db, robot_id)

        if not robot:
            raise HTTPException(status_code=404, detail="robot not found")

        return robot

    if robot_name is not None:
        robot = db.query(Robot).filter(Robot.robot_name == robot_name).first()

        if not robot:
            raise HTTPException(status_code=404, detail="robot not found")

        return robot

    return None


def build_task_event_response(db: Session, task_event: TaskEvent) -> dict:
    robot = db.get(Robot, task_event.robot_id) if task_event.robot_id else None

    return {
        "event_id": task_event.event_id,
        "task_id": task_event.task_id,
        "robot_id": task_event.robot_id,
        "robot_name": robot.robot_name if robot else None,
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
        .order_by(Task.sequence_no, Task.task_id)
        .first()
    )
    robot = db.get(Robot, current_task.assigned_robot_id) if current_task and current_task.assigned_robot_id else None

    return {
        "order_id": order.order_id,
        "order_no": order.order_no,
        "status": order.status,
        "priority": order.priority,
        "pickup_slot_id": order.pickup_slot_id,
        "pickup_slot_name": pickup_slot.slot_name if pickup_slot else None,
        "assigned_unit_id": order.assigned_unit_id,
        "current_task_id": current_task.task_id if current_task else None,
        "current_task_type": current_task.task_type if current_task else None,
        "current_task_status": current_task.status if current_task else None,
        "assigned_robot_id": current_task.assigned_robot_id if current_task else None,
        "assigned_robot_name": robot.robot_name if robot else None,
    }


def find_robot_running_task(db: Session, robot: Robot) -> Task | None:
    if robot.current_task_id is not None:
        current_task = db.get(Task, robot.current_task_id)

        if current_task and current_task.status == "RUNNING":
            return current_task

    return (
        db.query(Task)
        .filter(
            Task.assigned_robot_id == robot.robot_id,
            Task.status == "RUNNING",
        )
        .order_by(Task.task_id.desc())
        .first()
    )


def build_robot_running_task_response(db: Session, robot: Robot) -> dict:
    running_task = find_robot_running_task(db, robot)

    return {
        "task_type": running_task.task_type if running_task else None,
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


def build_pickup_slot_assignment_response(db: Session, order: Order, pickup_slot: PickupSlot) -> dict:
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


def resolve_stocking_policy_or_400(
    requested_quantity: int | None,
    stocking_policy: str | None,
) -> str:
    try:
        return resolve_stocking_policy(requested_quantity, stocking_policy)
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc


def validate_robot_unit(db: Session, unit_id: int | None) -> None:
    if unit_id is not None and not db.get(RobotUnit, unit_id):
        raise HTTPException(status_code=404, detail="robot unit not found")


def validate_task_robot_type(robot: Robot | None, task_type: str) -> None:
    if robot is None:
        return

    expected_robot_type = TASK_ROBOT_TYPE.get(task_type)

    if expected_robot_type and robot.robot_type != expected_robot_type:
        raise HTTPException(
            status_code=400,
            detail=f"{task_type} task must be assigned to {expected_robot_type}",
        )


def validate_task_refs(db: Session, task_create) -> Robot | None:
    if task_create.stocking_item_id is not None:
        if task_create.order_id is not None or task_create.order_item_id is not None:
            raise HTTPException(
                status_code=400,
                detail="stocking task cannot reference order or order_item",
            )

        if not db.get(StockingItem, task_create.stocking_item_id):
            raise HTTPException(status_code=404, detail="stocking item not found")

    if task_create.order_id is not None and not db.get(Order, task_create.order_id):
        raise HTTPException(status_code=404, detail="order not found")

    order_item = (
        db.get(OrderItem, task_create.order_item_id)
        if task_create.order_item_id is not None
        else None
    )

    if task_create.order_item_id is not None and not order_item:
        raise HTTPException(status_code=404, detail="order item not found")

    if (
        order_item
        and task_create.order_id is not None
        and order_item.order_id != task_create.order_id
    ):
        raise HTTPException(status_code=400, detail="order item does not belong to order")

    if task_create.source_zone_id is not None and not db.get(Zone, task_create.source_zone_id):
        raise HTTPException(status_code=404, detail="source zone not found")

    if task_create.target_zone_id is not None and not db.get(Zone, task_create.target_zone_id):
        raise HTTPException(status_code=404, detail="target zone not found")

    robot = get_robot_from_payload(
        db,
        robot_id=task_create.assigned_robot_id,
        robot_name=task_create.assigned_robot_name,
    )
    validate_task_robot_type(robot, task_create.task_type)
    return robot


@router.websocket("/ws/events")
async def fleet_events_websocket(websocket: WebSocket):
    await fleet_event_websockets.connect(websocket)

    try:
        while True:
            await websocket.receive_text()
    except WebSocketDisconnect:
        await fleet_event_websockets.disconnect(websocket)


@router.get("/snapshot")
def get_fleet_snapshot(db: Session = Depends(get_db)):
    return build_admin_status(db)


@router.get("/zones", response_model=list[FleetZoneRead])
def list_fleet_zones(
    zone_type: str | None = "PRODUCT",
    db: Session = Depends(get_db),
):
    zone_query = db.query(Zone)

    normalized_zone_type = zone_type.upper() if zone_type else None

    if normalized_zone_type and normalized_zone_type != "ALL":
        zone_query = zone_query.filter(Zone.zone_type == normalized_zone_type)

    zones = zone_query.order_by(Zone.zone_type, Zone.zone_name).all()
    return [
        {
            "zone_id": zone.zone_id,
            "zone_name": zone.zone_name,
            "zone_type": zone.zone_type,
            "pose": build_zone_pose(zone),
        }
        for zone in zones
    ]


@router.post("/stocking-items", response_model=FleetStockingItemRead, status_code=201)
def create_stocking_item(
    stocking_create: FleetStockingItemCreate,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    product = db.get(Product, stocking_create.product_id)

    if not product:
        raise HTTPException(status_code=404, detail="product not found")

    validate_robot_unit(db, stocking_create.assigned_unit_id)
    try:
        stocking_item = create_stocking_item_record(
            db,
            product_id=stocking_create.product_id,
            requested_quantity=stocking_create.requested_quantity,
            detected_quantity=stocking_create.detected_quantity,
            stock_delta=stocking_create.stock_delta,
            stocking_policy=stocking_create.stocking_policy,
            status=stocking_create.status,
            assigned_unit_id=stocking_create.assigned_unit_id,
        )
    except ValueError as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc

    db.commit()
    db.refresh(stocking_item)
    background_tasks.add_task(broadcast_all_status)
    background_tasks.add_task(
        broadcast_fleet_event,
        {"event": "STOCKING_ITEM_CREATED", "stocking_item_id": stocking_item.stocking_item_id},
    )
    return build_stocking_item_summary(db, stocking_item)


@router.get("/stocking-items", response_model=list[FleetStockingItemRead])
def list_stocking_items(
    status: StockingItemStatus | None = None,
    include_completed: bool = False,
    db: Session = Depends(get_db),
):
    stocking_query = db.query(StockingItem)

    if status is not None:
        stocking_query = stocking_query.filter(StockingItem.status == status)
    elif not include_completed:
        stocking_query = stocking_query.filter(StockingItem.status.notin_(("COMPLETED", "CANCELLED")))

    stocking_items = (
        stocking_query
        .order_by(StockingItem.stocking_item_id.desc())
        .limit(50)
        .all()
    )
    return [build_stocking_item_summary(db, item) for item in stocking_items]


@router.patch("/stocking-items/{stocking_item_id}", response_model=FleetStockingItemRead)
def update_stocking_item(
    stocking_item_id: int,
    stocking_update: FleetStockingItemUpdate,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    stocking_item = (
        db.query(StockingItem)
        .filter(StockingItem.stocking_item_id == stocking_item_id)
        .with_for_update()
        .one_or_none()
    )

    if not stocking_item:
        raise HTTPException(status_code=404, detail="stocking item not found")

    if "assigned_unit_id" in stocking_update.model_fields_set:
        validate_robot_unit(db, stocking_update.assigned_unit_id)
        stocking_item.assigned_unit_id = stocking_update.assigned_unit_id

    requested_quantity = (
        stocking_update.requested_quantity
        if "requested_quantity" in stocking_update.model_fields_set
        else stocking_item.requested_quantity
    )
    stocking_policy = resolve_stocking_policy_or_400(
        requested_quantity,
        stocking_update.stocking_policy or stocking_item.stocking_policy,
    )

    if "requested_quantity" in stocking_update.model_fields_set:
        stocking_item.requested_quantity = stocking_update.requested_quantity

    if "detected_quantity" in stocking_update.model_fields_set:
        stocking_item.detected_quantity = stocking_update.detected_quantity

    if "stock_delta" in stocking_update.model_fields_set:
        stocking_item.stock_delta = stocking_update.stock_delta

    if stocking_update.stocking_policy is not None:
        stocking_item.stocking_policy = stocking_policy

    if stocking_update.status is not None:
        stocking_item.status = stocking_update.status

    db.commit()
    db.refresh(stocking_item)
    background_tasks.add_task(broadcast_all_status)
    return build_stocking_item_summary(db, stocking_item)


@router.post("/tasks/bulk", response_model=FleetTaskBulkCreateRead, status_code=201)
def create_fleet_tasks(
    task_bulk_create: FleetTaskBulkCreate,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    created_task_ids = []

    for task_create in task_bulk_create.tasks:
        robot = validate_task_refs(db, task_create)
        task = Task(
            order_id=task_create.order_id,
            order_item_id=task_create.order_item_id,
            stocking_item_id=task_create.stocking_item_id,
            sequence_no=task_create.sequence_no,
            assigned_robot_id=robot.robot_id if robot else None,
            task_type=task_create.task_type,
            status=task_create.status,
            priority=task_create.priority,
            source_zone_id=task_create.source_zone_id,
            target_zone_id=task_create.target_zone_id,
            result_message=task_create.result_message,
        )
        db.add(task)
        db.flush()
        created_task_ids.append(task.task_id)

    db.commit()
    background_tasks.add_task(broadcast_all_status)
    background_tasks.add_task(
        broadcast_fleet_event,
        {"event": "TASKS_CREATED", "task_ids": created_task_ids},
    )
    return {
        "status": "ok",
        "task_ids": created_task_ids,
        "created_count": len(created_task_ids),
    }


@router.get("/tasks", response_model=list[FleetTaskSummaryRead])
def list_fleet_tasks(
    robot_id: int | None = None,
    robot_name: str | None = None,
    status: TaskStatus | None = None,
    task_type: TaskType | None = None,
    order_id: int | None = None,
    db: Session = Depends(get_db),
):
    task_query = db.query(Task)

    robot = get_robot_from_payload(db, robot_id=robot_id, robot_name=robot_name)

    if robot is not None:
        task_query = task_query.filter(Task.assigned_robot_id == robot.robot_id)

    if status is not None:
        task_query = task_query.filter(Task.status == status)

    if task_type is not None:
        task_query = task_query.filter(Task.task_type == task_type)

    if order_id is not None:
        task_query = task_query.filter(Task.order_id == order_id)

    tasks = task_query.order_by(Task.priority, Task.sequence_no, Task.task_id).all()
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
        .order_by(Task.sequence_no, Task.task_id)
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

    if "assigned_unit_id" in state_update.model_fields_set:
        order.assigned_unit_id = state_update.assigned_unit_id

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

    robot = get_robot_from_payload(
        db,
        robot_id=state_update.assigned_robot_id,
        robot_name=state_update.assigned_robot_name,
    )

    if robot is not None:
        task.assigned_robot_id = robot.robot_id

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

    apply_task_runtime_state(db, task, previous_status=previous_status)
    db.commit()
    background_tasks.add_task(broadcast_all_status)
    background_tasks.add_task(
        broadcast_fleet_event,
        {"event": "TASK_STATUS_CHANGED", "task_id": task.task_id, "status": task.status},
    )
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

    robot = get_robot_from_payload(
        db,
        robot_id=event_create.robot_id,
        robot_name=event_create.robot_name,
    )

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
        robot_id=robot.robot_id if robot else task.assigned_robot_id,
        from_status=from_status,
        to_status=event_create.to_status,
        event_name=event_create.event_name,
        reason=event_create.reason,
        created_at=datetime.now(UTC),
    )
    db.add(task_event)

    if event_create.update_task_status:
        previous_status = task.status
        task.status = event_create.to_status
        apply_task_runtime_state(db, task, previous_status=previous_status)

    db.commit()
    db.refresh(task_event)
    background_tasks.add_task(broadcast_all_status)
    background_tasks.add_task(
        broadcast_fleet_event,
        {"event": task_event.event_name or "TASK_EVENT", "task_id": task.task_id},
    )
    return build_task_event_response(db, task_event)


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
        build_task_event_response(db, task_event)
        for task_event in task_events
    ]


@router.get("/robots/{robot_identifier}", response_model=FleetRobotRuntimeRead)
def get_robot_runtime(
    robot_identifier: str,
    db: Session = Depends(get_db),
):
    robot = get_robot_by_identifier(db, robot_identifier)

    if not robot:
        raise HTTPException(status_code=404, detail="robot not found")

    return build_robot_summary(db, robot)


@router.get("/robots/{robot_identifier}/running-task", response_model=FleetRobotRunningTaskRead)
def get_robot_running_task(
    robot_identifier: str,
    db: Session = Depends(get_db),
):
    robot = get_robot_by_identifier(db, robot_identifier)

    if not robot:
        raise HTTPException(status_code=404, detail="robot not found")

    return build_robot_running_task_response(db, robot)


@router.patch("/robots/{robot_identifier}", response_model=FleetStateUpdateRead)
def update_robot_state(
    robot_identifier: str,
    state_update: FleetRobotStateUpdate,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    existing_robot = get_robot_by_identifier(db, robot_identifier)

    if not existing_robot:
        raise HTTPException(status_code=404, detail="robot not found")

    robot = (
        db.query(Robot)
        .filter(Robot.robot_id == existing_robot.robot_id)
        .with_for_update()
        .one_or_none()
    )

    if not robot:
        raise HTTPException(status_code=404, detail="robot not found")

    requested_status = state_update.robot_status or state_update.status

    if requested_status is not None:
        robot.robot_status = requested_status

    if "picky_state" in state_update.model_fields_set:
        if robot.robot_type != "PICKY" and state_update.picky_state is not None:
            raise HTTPException(status_code=400, detail="picky_state is only for PICKY")
        robot.picky_state = state_update.picky_state

    if "cobot_state" in state_update.model_fields_set:
        if robot.robot_type != "COBOT" and state_update.cobot_state is not None:
            raise HTTPException(status_code=400, detail="cobot_state is only for COBOT")
        robot.cobot_state = state_update.cobot_state

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
    background_tasks.add_task(
        broadcast_fleet_event,
        {"event": "ROBOT_STATE_CHANGED", "robot_id": robot.robot_id, "robot_name": robot.robot_name},
    )
    return {"status": "ok"}


@router.post("/assignments/run")
def run_task_assignment():
    return {
        "status": "ok",
        "assigned_count": 0,
        "message": "task assignment is handled by Fleet Manager",
    }


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

        return build_pickup_slot_assignment_response(db, order, pickup_slot)

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
    return build_pickup_slot_assignment_response(db, order, pickup_slot)


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


@router.get("/zones")
def list_zones(db: Session = Depends(get_db)):
    zones = db.query(Zone).all()
    return [
        {
            "zone_id": z.zone_id,
            "zone_name": z.zone_name,
            "zone_type": z.zone_type,
            "pos_x": z.pos_x,
            "pos_y": z.pos_y,
        }
        for z in zones
    ]


@router.post("/exceptions", response_model=FleetExceptionRead, status_code=201)
def create_exception_report(
    exception_create: FleetExceptionCreate,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    robot = get_robot_from_payload(
        db,
        robot_id=exception_create.robot_id,
        robot_name=exception_create.robot_name,
    )

    if exception_create.task_id is not None and not db.get(Task, exception_create.task_id):
        raise HTTPException(status_code=404, detail="task not found")

    if exception_create.order_id is not None and not db.get(Order, exception_create.order_id):
        raise HTTPException(status_code=404, detail="order not found")

    exception = ExceptionLog(
        robot_id=robot.robot_id if robot else None,
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
    background_tasks.add_task(
        broadcast_fleet_event,
        {"event": "EXCEPTION_CREATED", "exception_id": exception.exception_id},
    )
    return {
        "status": "ok",
        "exception_id": exception.exception_id,
    }


@router.post("/stocking/complete")
def complete_stocking(
    stocking_complete: FleetStockingComplete,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    task = (
        db.query(Task)
        .filter(Task.task_id == stocking_complete.task_id)
        .with_for_update()
        .one_or_none()
    )

    if not task:
        raise HTTPException(status_code=404, detail="task not found")

    if task.task_type != "STOCKING_PLACE":
        raise HTTPException(status_code=409, detail="task is not STOCKING_PLACE")

    if task.stocking_item_id is None:
        raise HTTPException(status_code=409, detail="stocking item is not linked to task")

    stocking_item = (
        db.query(StockingItem)
        .filter(StockingItem.stocking_item_id == task.stocking_item_id)
        .with_for_update()
        .one_or_none()
    )

    if not stocking_item:
        raise HTTPException(status_code=404, detail="stocking item not found")

    product = db.get(Product, stocking_item.product_id)

    if not product:
        raise HTTPException(status_code=404, detail="product not found")

    if stocking_complete.detected_quantity is not None:
        stocking_item.detected_quantity = stocking_complete.detected_quantity

    if stocking_complete.stock_delta is not None:
        stocking_item.stock_delta = stocking_complete.stock_delta

    previous_status = task.status
    task.status = "SUCCESS"

    if stocking_complete.result_message is not None:
        task.result_message = stocking_complete.result_message

    apply_task_runtime_state(db, task, previous_status=previous_status)
    stock_delta = stocking_item.stock_delta

    db.add(
        TaskEvent(
            task_id=task.task_id,
            robot_id=task.assigned_robot_id,
            from_status=previous_status,
            to_status=task.status,
            event_name="STOCKING_COMPLETED",
            reason=task.result_message,
            created_at=datetime.now(UTC),
        )
    )
    db.commit()
    background_tasks.add_task(broadcast_all_status)
    background_tasks.add_task(
        broadcast_fleet_event,
        {"event": "STOCKING_COMPLETED", "task_id": task.task_id, "stock_delta": stock_delta},
    )
    return {
        "status": "ok",
        "task_id": task.task_id,
        "stocking_item_id": stocking_item.stocking_item_id,
        "product_id": product.product_id,
        "stock_delta": stock_delta,
        "stock_qty": product.stock_qty,
    }
