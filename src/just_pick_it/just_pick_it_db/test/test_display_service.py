from sqlalchemy import create_engine
from sqlalchemy.orm import Session

from just_pick_it_db.models import Base, Product, Zone
from just_pick_it_db.services.display_service import (
    create_display_item_record,
    has_appendable_display_item,
    has_open_display_item,
)


def _session() -> Session:
    engine = create_engine("sqlite:///:memory:")
    Base.metadata.create_all(engine)
    return Session(engine)


def _product(db: Session, name: str = "product") -> Product:
    zone = Zone(zone_name=f"{name}_zone", zone_type="PRODUCT", pos_x=0.0, pos_y=0.0, pos_z=0.0)
    db.add(zone)
    db.flush()

    product = Product(name=name, stock_qty=1, storage_zone_id=zone.zone_id)
    db.add(product)
    db.flush()
    return product


def _display_item(db: Session, product: Product, status: str = "REQUESTED"):
    return create_display_item_record(
        db,
        product_id=product.product_id,
        requested_quantity=1,
        stock_delta=1,
        display_policy="REQUESTED_QUANTITY",
        status=status,
    )


def test_create_display_item_reuses_not_started_batch() -> None:
    with _session() as db:
        first = _display_item(db, _product(db, "first"), status="ASSIGNED")
        second = _display_item(db, _product(db, "second"))

        assert second.display_batch_id == first.display_batch_id


def test_create_display_item_does_not_reuse_in_progress_batch() -> None:
    with _session() as db:
        running = _display_item(db, _product(db, "running"), status="IN_PROGRESS")
        requested = _display_item(db, _product(db, "requested"))

        assert requested.display_batch_id == requested.display_item_id
        assert requested.display_batch_id != running.display_batch_id


def test_appendable_display_context_excludes_in_progress_items() -> None:
    with _session() as db:
        _display_item(db, _product(db), status="IN_PROGRESS")

        assert has_open_display_item(db) is True
        assert has_appendable_display_item(db) is False
