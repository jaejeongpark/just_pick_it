from fastapi import APIRouter, BackgroundTasks, Depends, HTTPException
from sqlalchemy.orm import Session

from app.database import get_db
from app.models import Order, OrderItem, PickupSlot, Product
from app.schemas import OrderCreate, OrderRead
from app.services.product_images import resolve_product_image_url
from app.services.workflow_service import ORDER_PRIORITY, complete_order_workflow, create_order_workflow
from app.services.realtime import broadcast_all_status


router = APIRouter(prefix="/api/orders", tags=["orders"])


def build_order_response(db: Session, order: Order) -> OrderRead:
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

    return OrderRead(
        order_id=order.order_id,
        order_no=order.order_no,
        status=order.status,
        pickup_slot_id=order.pickup_slot_id,
        pickup_slot_name=pickup_slot_name,
        items=[
            {
                "product_id": item.product_id,
                "product_name": product.name,
                "image_url": resolve_product_image_url(product),
                "quantity": item.quantity,
                "status": item.status,
            }
            for item, product in order_items
        ],
    )


@router.post("", response_model=OrderRead)
def create_order(
    order_request: OrderCreate,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    quantities_by_product_id = {}

    for item in order_request.items:
        quantities_by_product_id[item.product_id] = (
            quantities_by_product_id.get(item.product_id, 0) + item.quantity
        )

    product_ids = list(quantities_by_product_id.keys())
    products = (
        db.query(Product)
        .filter(Product.product_id.in_(product_ids))
        .with_for_update()
        .all()
    )
    products_by_id = {product.product_id: product for product in products}

    missing_ids = [product_id for product_id in product_ids if product_id not in products_by_id]
    if missing_ids:
        raise HTTPException(status_code=404, detail="product not found")

    for product_id, quantity in quantities_by_product_id.items():
        product = products_by_id[product_id]
        if product.stock_qty < quantity:
            raise HTTPException(status_code=400, detail="not enough stock")

    order = Order(status="ORDER_RECEIVED", priority=ORDER_PRIORITY)
    db.add(order)
    db.flush()

    order.order_no = f"ORD-{order.order_id:04d}"

    for product_id, quantity in quantities_by_product_id.items():
        product = products_by_id[product_id]
        product.stock_qty -= quantity
        db.add(
            OrderItem(
                order_id=order.order_id,
                product_id=product_id,
                quantity=quantity,
                status="WAITING",
            )
        )

    create_order_workflow(db, order)
    db.commit()
    db.refresh(order)
    background_tasks.add_task(broadcast_all_status)

    return build_order_response(db, order)


@router.get("", response_model=list[OrderRead])
def list_orders(db: Session = Depends(get_db)):
    orders = (
        db.query(Order)
        .filter(Order.status != "COMPLETED")
        .order_by(Order.order_id.desc())
        .limit(50)
        .all()
    )
    return [build_order_response(db, order) for order in orders]


@router.get("/{order_id}", response_model=OrderRead)
def get_order(order_id: int, db: Session = Depends(get_db)):
    order = db.get(Order, order_id)

    if not order:
        raise HTTPException(status_code=404, detail="order not found")

    return build_order_response(db, order)


@router.post("/{order_id}/complete", response_model=OrderRead)
def complete_order(
    order_id: int,
    background_tasks: BackgroundTasks,
    db: Session = Depends(get_db),
):
    order = db.get(Order, order_id)

    if not order:
        raise HTTPException(status_code=404, detail="order not found")

    if order.status != "PICKUP_READY":
        raise HTTPException(status_code=400, detail="order is not ready for pickup")

    complete_order_workflow(db, order)
    db.commit()
    db.refresh(order)
    background_tasks.add_task(broadcast_all_status)

    return build_order_response(db, order)
