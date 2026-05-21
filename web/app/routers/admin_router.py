from fastapi import APIRouter, BackgroundTasks, Depends, HTTPException, WebSocket, WebSocketDisconnect
from sqlalchemy.orm import Session

from app.database import get_db
from app.models import ExceptionLog, PickupSlot, Product, Robot, Zone
from app.schemas import (
    AdminLlmMessageCreate,
    AdminLlmMessageRead,
    AdminPickupSlotCreate,
    ProductCreate,
    ProductRead,
    ProductStockUpdate,
    ProductUpdate,
)
from app.services.llm_client import build_llm_message
from app.services.realtime import (
    admin_websockets,
    broadcast_all_status,
    broadcast_fleet_event,
    get_admin_snapshot,
)
from app.services.status_service import build_admin_status, build_product_summary
from app.services.stocking_service import create_stocking_item_record


router = APIRouter(prefix="/api/admin", tags=["admin"])


def resolve_storage_zone_id(
    db: Session,
    storage_zone_id: int | None,
    storage_location: str | None,
) -> int:
    if storage_zone_id is not None:
        if not db.get(Zone, storage_zone_id):
            raise HTTPException(status_code=404, detail="storage zone not found")
        return storage_zone_id

    if not storage_location:
        raise HTTPException(status_code=400, detail="storage zone is required")

    zone = (
        db.query(Zone)
        .filter(Zone.zone_name == storage_location)
        .first()
    )

    if not zone:
        raise HTTPException(status_code=404, detail="storage zone not found")

    return zone.zone_id


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
        robot.robot_status = "EMERGENCY_STOP"

    db.commit()
    background_tasks.add_task(broadcast_all_status)
    return {"status": "ok"}


@router.post("/resume")
def resume_system(
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    robots = db.query(Robot).filter(Robot.robot_status == "EMERGENCY_STOP").all()

    for robot in robots:
        robot.robot_status = "IDLE"

    db.commit()
    background_tasks.add_task(broadcast_all_status)
    return {"status": "ok"}


@router.post("/products", response_model=ProductRead, status_code=201)
def create_product(
    product_create: ProductCreate,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    storage_zone_id = resolve_storage_zone_id(
        db,
        product_create.storage_zone_id,
        product_create.storage_location,
    )
    product = Product(
        name=product_create.name,
        image_url=product_create.image_url,
        stock_qty=product_create.stock_qty,
        storage_zone_id=storage_zone_id,
    )
    db.add(product)
    db.commit()
    db.refresh(product)
    background_tasks.add_task(broadcast_all_status)
    return build_product_summary(db, product)


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
    return build_product_summary(db, product)


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

    storage_zone_id = resolve_storage_zone_id(
        db,
        product_update.storage_zone_id,
        product_update.storage_location,
    )
    product.name = product_update.name
    product.image_url = product_update.image_url
    product.stock_qty = product_update.stock_qty
    product.storage_zone_id = storage_zone_id
    db.commit()
    db.refresh(product)
    background_tasks.add_task(broadcast_all_status)
    return build_product_summary(db, product)


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

    if llm_response.get("action") == "STOCKING" and llm_response.get("product_id") is not None:
        product = db.get(Product, llm_response["product_id"])

        if not product:
            raise HTTPException(status_code=404, detail="product not found")

        try:
            stocking_item = create_stocking_item_record(
                db,
                product_id=product.product_id,
                requested_quantity=llm_response.get("requested_quantity"),
                stocking_policy=llm_response.get("stocking_policy"),
            )
        except ValueError as exc:
            raise HTTPException(status_code=400, detail=str(exc)) from exc

        db.flush()
        llm_response["stocking_item_id"] = stocking_item.stocking_item_id

    db.commit()

    if llm_response.get("action") == "STOCKING":
        background_tasks.add_task(
            broadcast_fleet_event,
            {
                "event": "STOCKING_COMMAND",
                "message": message,
                "command": llm_response,
            },
        )

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
