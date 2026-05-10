from fastapi import APIRouter, BackgroundTasks, Depends, HTTPException, WebSocket, WebSocketDisconnect
from sqlalchemy import text
from sqlalchemy.orm import Session

from app.database import get_db
from app.models import ExceptionLog, Order, OrderItem, PickupSlot, Product, Robot, Task
from app.schemas import (
    AdminLlmMessageCreate,
    AdminLlmMessageRead,
    AdminPickupSlotCreate,
    AdminRobotCreate,
    AdminTaskCreate,
    ProductCreate,
    ProductRead,
    ProductStockUpdate,
    ProductUpdate,
)
from app.services.demo_workflow import (
    DEMO_ESTIMATED_DURATION_SECONDS,
    DEMO_STEP_DELAY_SECONDS,
    demo_run_is_active,
    reserve_demo_run,
    run_demo_order_sequence,
)
from app.services.llm_client import build_llm_message
from app.services.realtime import admin_websockets, broadcast_all_status, get_admin_snapshot


router = APIRouter(prefix="/api/admin", tags=["admin"])


def build_order_summary(db: Session, order: Order):
    pickup_slot_name = None

    if order.pickup_slot_id:
        pickup_slot = db.get(PickupSlot, order.pickup_slot_id)
        if pickup_slot:
            pickup_slot_name = pickup_slot.slot_name

    order_items = (
        db.query(OrderItem, Product)
        .join(Product, OrderItem.product_id == Product.product_id)
        .filter(OrderItem.order_id == order.order_id)
        .order_by(OrderItem.item_id)
        .all()
    )

    return {
        "order_id": order.order_id,
        "order_no": order.order_no,
        "status": order.status,
        "pickup_slot_id": order.pickup_slot_id,
        "pickup_slot_name": pickup_slot_name,
        "items": [
            {
                "product_id": item.product_id,
                "product_name": product.name,
                "quantity": item.quantity,
                "status": item.status,
            }
            for item, product in order_items
        ],
    }


def build_exception_summary(exception: ExceptionLog):
    return {
        "exception_id": exception.exception_id,
        "robot_id": exception.robot_id,
        "task_id": exception.task_id,
        "order_id": exception.order_id,
        "exception_type": exception.exception_type,
        "detail": exception.detail,
        "is_resolved": exception.is_resolved,
        "created_at": exception.created_at.isoformat() if exception.created_at else None,
    }


def build_task_summary(db: Session, task: Task):
    order = db.get(Order, task.order_id) if task.order_id else None

    return {
        "task_id": task.task_id,
        "order_id": task.order_id,
        "order_no": order.order_no if order else None,
        "assigned_robot_id": task.assigned_robot_id,
        "task_type": task.task_type,
        "status": task.status,
        "result_message": task.result_message,
    }


def build_product_summary(product: Product):
    return {
        "product_id": product.product_id,
        "name": product.name,
        "image_url": product.image_url,
        "stock_qty": product.stock_qty,
        "storage_location": product.storage_location,
    }


def build_admin_status(db: Session):
    orders = (
        db.query(Order)
        .filter(Order.status != "COMPLETED")
        .order_by(Order.order_id.desc())
        .limit(20)
        .all()
    )
    order_history = (
        db.query(Order)
        .filter(Order.status == "COMPLETED")
        .order_by(Order.order_id.desc())
        .limit(50)
        .all()
    )
    robots = db.query(Robot).order_by(Robot.robot_id).all()
    tasks = db.query(Task).order_by(Task.task_id.desc()).limit(20).all()
    products = db.query(Product).order_by(Product.product_id).all()
    pickup_slots = db.query(PickupSlot).order_by(PickupSlot.slot_id).all()
    exceptions = (
        db.query(ExceptionLog)
        .filter(ExceptionLog.is_resolved.is_(False))
        .order_by(ExceptionLog.created_at.desc(), ExceptionLog.exception_id.desc())
        .limit(5)
        .all()
    )
    exception_history = (
        db.query(ExceptionLog)
        .filter(ExceptionLog.is_resolved.is_(True))
        .order_by(ExceptionLog.created_at.desc(), ExceptionLog.exception_id.desc())
        .limit(100)
        .all()
    )
    unresolved_exception_count = (
        db.query(ExceptionLog)
        .filter(ExceptionLog.is_resolved.is_(False))
        .count()
    )

    return {
        "orders": [
            build_order_summary(db, order)
            for order in orders
        ],
        "order_history": [
            build_order_summary(db, order)
            for order in order_history
        ],
        "robots": [
            {
                "robot_id": robot.robot_id,
                "status": robot.status,
                "battery_level": robot.battery_level,
                "current_task_id": robot.current_task_id,
                "pos_x": robot.pos_x,
                "pos_y": robot.pos_y,
                "pos_theta": robot.pos_theta,
            }
            for robot in robots
        ],
        "tasks": [
            build_task_summary(db, task)
            for task in tasks
        ],
        "products": [
            build_product_summary(product)
            for product in products
        ],
        "low_stock_count": sum(
            1 for product in products
            if product.stock_qty <= 1
        ),
        "pickup_slots": [
            {
                "slot_id": slot.slot_id,
                "slot_name": slot.slot_name,
                "status": slot.status,
            }
            for slot in pickup_slots
        ],
        "exceptions": [
            build_exception_summary(exception)
            for exception in exceptions
        ],
        "exception_history": [
            build_exception_summary(exception)
            for exception in exception_history
        ],
        "unresolved_exception_count": unresolved_exception_count,
    }


@router.get("/status")
def admin_status(db: Session = Depends(get_db)):
    return build_admin_status(db)


@router.websocket("/ws/status")
async def admin_status_websocket(websocket: WebSocket):
    await admin_websockets.connect(websocket)

    try:
        while True:
            await websocket.send_json(get_admin_snapshot())
            await websocket.receive_text()
    except WebSocketDisconnect:
        await admin_websockets.disconnect(websocket)
        return


@router.post("/emergency-stop")
def emergency_stop(
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    robots = db.query(Robot).all()

    for robot in robots:
        robot.status = "EMERGENCY_STOP"

    db.commit()
    background_tasks.add_task(broadcast_all_status)
    return {"status": "ok"}


@router.post("/resume")
def resume_system(
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    robots = db.query(Robot).filter(Robot.status == "EMERGENCY_STOP").all()

    for robot in robots:
        robot.status = "IDLE"

    db.commit()
    background_tasks.add_task(broadcast_all_status)
    return {"status": "ok"}


@router.post("/demo/run-order")
def run_order_demo(
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    if demo_run_is_active():
        raise HTTPException(status_code=409, detail="demo is already running")

    has_available_product = (
        db.query(Product.product_id)
        .filter(Product.stock_qty > 0)
        .first()
        is not None
    )
    has_empty_pickup_slot = (
        db.query(PickupSlot.slot_id)
        .filter(PickupSlot.status.in_(("EMPTY", "RESERVED")))
        .first()
        is not None
    )

    if not has_available_product:
        raise HTTPException(status_code=400, detail="no product stock available")

    if not has_empty_pickup_slot:
        raise HTTPException(status_code=400, detail="no empty pickup slot available")

    if not reserve_demo_run():
        raise HTTPException(status_code=409, detail="demo is already running")

    background_tasks.add_task(run_demo_order_sequence, DEMO_STEP_DELAY_SECONDS)
    return {
        "status": "started",
        "step_delay_seconds": DEMO_STEP_DELAY_SECONDS,
        "estimated_duration_seconds": DEMO_ESTIMATED_DURATION_SECONDS,
    }


@router.post("/products", response_model=ProductRead, status_code=201)
def create_product(
    product_create: ProductCreate,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    product = Product(
        name=product_create.name,
        image_url=product_create.image_url,
        stock_qty=product_create.stock_qty,
        storage_location=product_create.storage_location,
    )
    db.add(product)
    db.commit()
    db.refresh(product)
    background_tasks.add_task(broadcast_all_status)
    return product


@router.patch("/products/{product_id}/stock", response_model=ProductRead)
def update_product_stock(
    product_id: int,
    stock_update: ProductStockUpdate,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    product = db.get(Product, product_id)

    if not product:
        raise HTTPException(status_code=404, detail="product not found")

    product.stock_qty = stock_update.stock_qty
    db.commit()
    db.refresh(product)
    background_tasks.add_task(broadcast_all_status)
    return product


@router.patch("/products/{product_id}", response_model=ProductRead)
def update_product(
    product_id: int,
    product_update: ProductUpdate,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    product = db.get(Product, product_id)

    if not product:
        raise HTTPException(status_code=404, detail="product not found")

    product.name = product_update.name
    product.image_url = product_update.image_url
    product.stock_qty = product_update.stock_qty
    product.storage_location = product_update.storage_location
    db.commit()
    db.refresh(product)
    background_tasks.add_task(broadcast_all_status)
    return product


@router.delete("/products/{product_id}")
def delete_product(
    product_id: int,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    product = db.get(Product, product_id)

    if not product:
        raise HTTPException(status_code=404, detail="product not found")

    has_order_items = (
        db.query(OrderItem)
        .filter(OrderItem.product_id == product_id)
        .first()
        is not None
    )

    if has_order_items:
        raise HTTPException(status_code=400, detail="product is used by existing orders")

    db.delete(product)
    db.commit()
    background_tasks.add_task(broadcast_all_status)
    return {"status": "ok"}


@router.post("/robots")
def create_robot(
    robot_create: AdminRobotCreate,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    existing_robot = db.get(Robot, robot_create.robot_id)

    if existing_robot:
        raise HTTPException(status_code=400, detail="robot already exists")

    robot = Robot(
        robot_id=robot_create.robot_id,
        status=robot_create.status,
        ros_namespace=robot_create.ros_namespace,
        battery_level=robot_create.battery_level,
        pos_x=robot_create.pos_x,
        pos_y=robot_create.pos_y,
        pos_theta=robot_create.pos_theta,
    )
    db.add(robot)
    db.commit()
    background_tasks.add_task(broadcast_all_status)
    return {"status": "ok"}


@router.delete("/orders/{order_id}")
def delete_order(
    order_id: int,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    order = db.get(Order, order_id)

    if not order:
        raise HTTPException(status_code=404, detail="order not found")

    task_ids = [
        task_id
        for (task_id,) in db.query(Task.task_id).filter(Task.order_id == order_id).all()
    ]

    if task_ids:
        db.query(Robot).filter(Robot.current_task_id.in_(task_ids)).update(
            {"current_task_id": None},
            synchronize_session=False,
        )
        db.query(ExceptionLog).filter(ExceptionLog.task_id.in_(task_ids)).delete(
            synchronize_session=False,
        )

    db.query(ExceptionLog).filter(ExceptionLog.order_id == order_id).delete(
        synchronize_session=False,
    )
    db.delete(order)
    db.commit()
    background_tasks.add_task(broadcast_all_status)
    return {"status": "ok"}


@router.post("/tasks")
def create_task(
    task_create: AdminTaskCreate,
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
    background_tasks.add_task(broadcast_all_status)
    return {"status": "ok"}


@router.delete("/tasks/{task_id}")
def delete_task(
    task_id: int,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    task = db.get(Task, task_id)

    if not task:
        raise HTTPException(status_code=404, detail="task not found")

    db.query(Robot).filter(Robot.current_task_id == task_id).update(
        {"current_task_id": None},
        synchronize_session=False,
    )
    db.query(ExceptionLog).filter(ExceptionLog.task_id == task_id).delete(
        synchronize_session=False,
    )
    db.delete(task)
    db.commit()
    background_tasks.add_task(broadcast_all_status)
    return {"status": "ok"}


@router.delete("/robots/{robot_id}")
def delete_robot(
    robot_id: str,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    robot = db.get(Robot, robot_id)

    if not robot:
        raise HTTPException(status_code=404, detail="robot not found")

    db.query(Task).filter(Task.assigned_robot_id == robot_id).update(
        {"assigned_robot_id": None},
        synchronize_session=False,
    )
    db.query(ExceptionLog).filter(ExceptionLog.robot_id == robot_id).update(
        {"robot_id": None},
        synchronize_session=False,
    )
    db.execute(
        text("UPDATE task_event SET robot_id = NULL WHERE robot_id = :robot_id"),
        {"robot_id": robot_id},
    )
    db.delete(robot)
    db.commit()
    background_tasks.add_task(broadcast_all_status)
    return {"status": "ok"}


@router.post("/pickup-slots")
def create_pickup_slot(
    pickup_slot_create: AdminPickupSlotCreate,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    pickup_slot = PickupSlot(
        slot_name=pickup_slot_create.slot_name,
        status=pickup_slot_create.status,
    )
    db.add(pickup_slot)
    db.commit()
    background_tasks.add_task(broadcast_all_status)
    return {"status": "ok"}


@router.delete("/pickup-slots/{slot_id}")
def delete_pickup_slot(
    slot_id: int,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    pickup_slot = db.get(PickupSlot, slot_id)

    if not pickup_slot:
        raise HTTPException(status_code=404, detail="pickup slot not found")

    db.query(Order).filter(Order.pickup_slot_id == slot_id).update(
        {"pickup_slot_id": None},
        synchronize_session=False,
    )
    db.delete(pickup_slot)
    db.commit()
    background_tasks.add_task(broadcast_all_status)
    return {"status": "ok"}


@router.post("/llm/messages", response_model=AdminLlmMessageRead)
def create_llm_message(
    message_create: AdminLlmMessageCreate,
    db: Session = Depends(get_db),
):
    message = message_create.message.strip()
    context = {
        "low_stock_count": db.query(Product).filter(Product.stock_qty <= 1).count(),
        "unresolved_exception_count": (
            db.query(ExceptionLog)
            .filter(ExceptionLog.is_resolved.is_(False))
            .count()
        ),
    }
    return build_llm_message(message, context)


@router.post("/exceptions/{exception_id}/resolve")
def resolve_exception(
    exception_id: int,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    exception = db.get(ExceptionLog, exception_id)

    if not exception:
        raise HTTPException(status_code=404, detail="exception not found")

    exception.is_resolved = True
    db.commit()
    background_tasks.add_task(broadcast_all_status)
    return {"status": "ok"}
