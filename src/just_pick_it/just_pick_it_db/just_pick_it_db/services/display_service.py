from sqlalchemy.orm import Session

from just_pick_it_db.models import Product, DisplayItem


FINAL_DISPLAY_ITEM_STATUSES = ("COMPLETED", "FAILED", "CANCELLED")


def build_display_item_summary(db: Session, display_item: DisplayItem) -> dict:
    product = db.get(Product, display_item.product_id)

    return {
        "display_item_id": display_item.display_item_id,
        "product_id": display_item.product_id,
        "product_name": product.name if product else None,
        "requested_quantity": display_item.requested_quantity,
        "detected_quantity": display_item.detected_quantity,
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
    detected_quantity: int | None = None,
    stock_delta: int | None = None,
    display_policy: str | None = None,
    status: str = "REQUESTED",
    assigned_unit_id: int | None = None,
) -> DisplayItem:
    item = DisplayItem(
        product_id=product_id,
        requested_quantity=requested_quantity,
        detected_quantity=detected_quantity,
        stock_delta=stock_delta,
        display_policy=resolve_display_policy(requested_quantity, display_policy),
        status=status,
        assigned_unit_id=assigned_unit_id,
    )
    db.add(item)
    return item


def resolve_display_policy(
    requested_quantity: int | None,
    display_policy: str | None,
) -> str:
    policy = display_policy or (
        "REQUESTED_QUANTITY" if requested_quantity is not None else "ALL_DETECTED"
    )

    if policy == "REQUESTED_QUANTITY" and requested_quantity is None:
        raise ValueError("requested_quantity is required for REQUESTED_QUANTITY policy")

    if policy == "ALL_DETECTED" and requested_quantity is not None:
        raise ValueError("requested_quantity must be null for ALL_DETECTED policy")

    return policy


def resolve_stock_delta(display_item: DisplayItem) -> int | None:
    if display_item.stock_delta is not None:
        return int(display_item.stock_delta)

    if display_item.requested_quantity is not None:
        return int(display_item.requested_quantity)

    if display_item.detected_quantity is not None:
        return int(display_item.detected_quantity)

    return None
