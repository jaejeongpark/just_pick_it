from sqlalchemy.orm import Session

from just_pick_it_db.models import Product, DisplayItem
from just_pick_it_db.services.inventory_status import AUTO_DISPLAY_REQUEST_QTY, LOW_STOCK_MAX
from just_pick_it_db.services.product_images import resolve_product_image_url


FINAL_DISPLAY_ITEM_STATUSES = ("COMPLETED", "FAILED", "CANCELLED")


def build_display_item_summary(db: Session, display_item: DisplayItem) -> dict:
    product = db.get(Product, display_item.product_id)

    return {
        "display_item_id": display_item.display_item_id,
        "product_id": display_item.product_id,
        "product_name": product.name if product else None,
        "image_url": resolve_product_image_url(product) if product else None,
        "requested_quantity": display_item.requested_quantity,
        "processed_quantity": display_item.processed_quantity,
        "stock_delta": display_item.stock_delta,
        "display_policy": display_item.display_policy,
        "status": display_item.status,
        "assigned_unit_id": display_item.assigned_unit_id,
    }


def create_display_item_record(
    db: Session,
    *,
    product_id: int,
    requested_quantity: int | None = None,
    processed_quantity: int | None = None,
    stock_delta: int | None = None,
    display_policy: str | None = None,
    status: str = "REQUESTED",
    assigned_unit_id: int | None = None,
) -> DisplayItem:
    item = DisplayItem(
        product_id=product_id,
        requested_quantity=requested_quantity,
        processed_quantity=processed_quantity,
        stock_delta=stock_delta,
        display_policy=resolve_display_policy(requested_quantity, display_policy),
        status=status,
        assigned_unit_id=assigned_unit_id,
    )
    db.add(item)
    return item


def queue_auto_display_if_low_stock(db: Session, product: Product) -> DisplayItem | None:
    if product.stock_qty > LOW_STOCK_MAX:
        return None

    active_display_item = (
        db.query(DisplayItem)
        .filter(
            DisplayItem.product_id == product.product_id,
            ~DisplayItem.status.in_(FINAL_DISPLAY_ITEM_STATUSES),
        )
        .first()
    )
    if active_display_item is not None:
        return None

    stock_delta = AUTO_DISPLAY_REQUEST_QTY
    if stock_delta <= 0:
        return None

    return create_display_item_record(
        db,
        product_id=product.product_id,
        requested_quantity=stock_delta,
        stock_delta=stock_delta,
        display_policy="REQUESTED_QUANTITY",
        status="REQUESTED",
    )


def resolve_display_policy(
    requested_quantity: int | None,
    display_policy: str | None,
) -> str:
    policy = display_policy or (
        "REQUESTED_QUANTITY" if requested_quantity is not None else "ALL_PROCESSED"
    )

    if policy == "REQUESTED_QUANTITY" and requested_quantity is None:
        raise ValueError("requested_quantity is required for REQUESTED_QUANTITY policy")

    if policy == "ALL_PROCESSED" and requested_quantity is not None:
        raise ValueError("requested_quantity must be null for ALL_PROCESSED policy")

    return policy


def resolve_stock_delta(display_item: DisplayItem) -> int | None:
    if display_item.stock_delta is not None:
        return int(display_item.stock_delta)

    if display_item.requested_quantity is not None:
        return int(display_item.requested_quantity)

    if display_item.processed_quantity is not None:
        return int(display_item.processed_quantity)

    return None
