from fastapi import APIRouter, BackgroundTasks, Depends, HTTPException, WebSocket, WebSocketDisconnect
from sqlalchemy.orm import Session

from app.database import get_db
from app.models import ExceptionLog, Order, PickupSlot, Product, Robot, Task
from app.schemas import (
    AdminLlmMessageCreate,
    AdminLlmMessageRead,
    AdminPickupSlotCreate,
    AdminTaskCreate,
    ProductCreate,
    ProductRead,
    ProductStockUpdate,
    ProductUpdate,
)
from app.services.llm_client import build_llm_message
from app.services.patrol_service import create_patrol_task_from_llm
from app.services.realtime import admin_websockets, broadcast_all_status, get_admin_snapshot
from app.services.status_service import build_admin_status


router = APIRouter(prefix="/api/admin", tags=["admin"])


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
        priority=task_create.priority,
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


@router.post("/llm/messages", response_model=AdminLlmMessageRead)
def create_llm_message(
    message_create: AdminLlmMessageCreate,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    message = message_create.message.strip()
    llm_response = build_llm_message(message)
    create_patrol_task_from_llm(db, message, llm_response)
    db.commit()

    if llm_response.get("task_id") is not None:
        background_tasks.add_task(broadcast_all_status)

    return llm_response


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
