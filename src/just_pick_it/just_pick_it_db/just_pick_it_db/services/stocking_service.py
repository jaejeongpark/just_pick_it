from sqlalchemy.orm import Session

from just_pick_it_db.models import Product, StockingItem


FINAL_STOCKING_ITEM_STATUSES = ("COMPLETED", "CANCELLED")


def build_stocking_item_summary(db: Session, stocking_item: StockingItem) -> dict:
    product = db.get(Product, stocking_item.product_id)

    return {
        "stocking_item_id": stocking_item.stocking_item_id,
        "product_id": stocking_item.product_id,
        "product_name": product.name if product else None,
        "requested_quantity": stocking_item.requested_quantity,
        "detected_quantity": stocking_item.detected_quantity,
        "stock_delta": stocking_item.stock_delta,
        "stocking_policy": stocking_item.stocking_policy,
        "status": stocking_item.status,
        "assigned_unit_id": stocking_item.assigned_unit_id,
    }


def create_stocking_item_record(
    db: Session,
    *,
    product_id: int,
    requested_quantity: int | None = None,
    detected_quantity: int | None = None,
    stock_delta: int | None = None,
    stocking_policy: str | None = None,
    status: str = "REQUESTED",
    assigned_unit_id: int | None = None,
) -> StockingItem:
    item = StockingItem(
        product_id=product_id,
        requested_quantity=requested_quantity,
        detected_quantity=detected_quantity,
        stock_delta=stock_delta,
        stocking_policy=resolve_stocking_policy(requested_quantity, stocking_policy),
        status=status,
        assigned_unit_id=assigned_unit_id,
    )
    db.add(item)
    return item


def resolve_stocking_policy(
    requested_quantity: int | None,
    stocking_policy: str | None,
) -> str:
    policy = stocking_policy or (
        "REQUESTED_QUANTITY" if requested_quantity is not None else "ALL_DETECTED"
    )

    if policy == "REQUESTED_QUANTITY" and requested_quantity is None:
        raise ValueError("requested_quantity is required for REQUESTED_QUANTITY policy")

    if policy == "ALL_DETECTED" and requested_quantity is not None:
        raise ValueError("requested_quantity must be null for ALL_DETECTED policy")

    return policy


def resolve_stock_delta(stocking_item: StockingItem) -> int | None:
    if stocking_item.stock_delta is not None:
        return int(stocking_item.stock_delta)

    if stocking_item.requested_quantity is not None:
        return int(stocking_item.requested_quantity)

    if stocking_item.detected_quantity is not None:
        return int(stocking_item.detected_quantity)

    return None
