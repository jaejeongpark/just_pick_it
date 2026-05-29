from sqlalchemy import case
from sqlalchemy.orm import Session

from just_pick_it_db.models import ExceptionLog, Order, OrderItem, PickupSlot, Product, Robot, DisplayItem, Task, Zone
from just_pick_it_db.services.inventory_status import is_low_stock, stock_level
from just_pick_it_db.services.product_images import resolve_product_image_url


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
        "priority": order.priority,
        "pickup_slot_id": order.pickup_slot_id,
        "pickup_slot_name": pickup_slot_name,
        "assigned_unit_id": order.assigned_unit_id,
        "items": [
            {
                "item_id": item.item_id,
                "product_id": item.product_id,
                "product_name": product.name,
                "image_url": resolve_product_image_url(product),
                "quantity": item.quantity,
                "status": item.status,
            }
            for item, product in order_items
        ],
    }


def build_exception_summary(db: Session, exception: ExceptionLog):
    robot = db.get(Robot, exception.robot_id) if exception.robot_id else None

    return {
        "exception_id": exception.exception_id,
        "robot_id": exception.robot_id,
        "robot_name": robot.robot_name if robot else None,
        "task_id": exception.task_id,
        "order_id": exception.order_id,
        "exception_type": exception.exception_type,
        "detail": exception.detail,
        "is_resolved": exception.is_resolved,
        "created_at": exception.created_at.isoformat() if exception.created_at else None,
    }


def build_zone_pose(zone: Zone | None):
    if not zone:
        return None

    return {
        "x": zone.pos_x,
        "y": zone.pos_y,
        "z": zone.pos_z,
        "theta": zone.pos_theta,
    }


def build_task_summary(db: Session, task: Task):
    order = db.get(Order, task.order_id) if task.order_id else None
    robot = db.get(Robot, task.assigned_robot_id) if task.assigned_robot_id else None
    order_item = db.get(OrderItem, task.order_item_id) if task.order_item_id else None
    display_item = db.get(DisplayItem, task.display_item_id) if task.display_item_id else None

    if order_item:
        product = db.get(Product, order_item.product_id)
        product_quantity = order_item.quantity
    elif display_item:
        product = db.get(Product, display_item.product_id)
        product_quantity = display_item.requested_quantity or display_item.detected_quantity
    else:
        product = None
        product_quantity = None

    source_zone = db.get(Zone, task.source_zone_id) if task.source_zone_id else None
    target_zone = db.get(Zone, task.target_zone_id) if task.target_zone_id else None

    return {
        "task_id": task.task_id,
        "order_id": task.order_id,
        "order_no": order.order_no if order else None,
        "pickup_slot_id": order.pickup_slot_id if order else None,
        "order_item_id": task.order_item_id,
        "display_item_id": task.display_item_id,
        "product_id": product.product_id if product else None,
        "product_name": product.name if product else None,
        "product_quantity": product_quantity,
        "requested_quantity": display_item.requested_quantity if display_item else None,
        "detected_quantity": display_item.detected_quantity if display_item else None,
        "stock_delta": display_item.stock_delta if display_item else None,
        "display_policy": display_item.display_policy if display_item else None,
        "display_status": display_item.status if display_item else None,
        "sequence_no": task.sequence_no,
        "assigned_robot_id": task.assigned_robot_id,
        "assigned_robot_name": robot.robot_name if robot else None,
        "task_type": task.task_type,
        "status": task.status,
        "priority": task.priority,
        "source_zone_id": task.source_zone_id,
        "source_zone_name": source_zone.zone_name if source_zone else None,
        "source_zone_pose": build_zone_pose(source_zone),
        "target_zone_id": task.target_zone_id,
        "target_zone_name": target_zone.zone_name if target_zone else None,
        "target_zone_pose": build_zone_pose(target_zone),
        "result_message": task.result_message,
    }


def build_product_summary(db: Session, product: Product):
    storage_zone = db.get(Zone, product.storage_zone_id)
    storage_zone_name = storage_zone.zone_name if storage_zone else None

    return {
        "product_id": product.product_id,
        "name": product.name,
        "image_url": resolve_product_image_url(product),
        "stock_qty": product.stock_qty,
        "stock_level": stock_level(product.stock_qty),
        "storage_zone_id": product.storage_zone_id,
        "storage_zone_name": storage_zone_name,
        "storage_zone_pose": build_zone_pose(storage_zone),
        # UI compatibility alias for screens that still display storage_location.
        "storage_location": storage_zone_name or str(product.storage_zone_id),
    }


def build_pickup_slot_summary(db: Session, pickup_slot: PickupSlot):
    return {
        "slot_id": pickup_slot.slot_id,
        "slot_name": pickup_slot.slot_name,
        "status": pickup_slot.status,
    }


def build_robot_summary(db: Session, robot: Robot):
    current_task = db.get(Task, robot.current_task_id) if robot.current_task_id else None

    return {
        "robot_id": robot.robot_id,
        "robot_name": robot.robot_name,
        "unit_id": robot.unit_id,
        "robot_type": robot.robot_type,
        "robot_status": robot.robot_status,
        # UI compatibility alias for screens that still read status.
        "status": robot.robot_status,
        "picky_state": robot.picky_state,
        "cobot_state": robot.cobot_state,
        "battery_level": robot.battery_level,
        "current_task_id": robot.current_task_id,
        "current_task_type": current_task.task_type if current_task else None,
        "current_task_status": current_task.status if current_task else None,
        "current_task": build_task_summary(db, current_task) if current_task else None,
        "pos_x": robot.pos_x,
        "pos_y": robot.pos_y,
        "pos_theta": robot.pos_theta,
    }


def build_admin_status(db: Session):
    robot_unit_order = case((Robot.unit_id.is_(None), 9999), else_=Robot.unit_id)
    robot_type_order = case(
        (Robot.robot_type == "PICKY", 0),
        (Robot.robot_type == "COBOT", 1),
        else_=2,
    )
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
    robots = (
        db.query(Robot)
        .order_by(robot_unit_order, robot_type_order, Robot.robot_name, Robot.robot_id)
        .all()
    )
    active_task_statuses = ["QUEUED", "ASSIGNED", "RUNNING", "PAUSED"]
    active_tasks = (
        db.query(Task)
        .filter(Task.status.in_(active_task_statuses))
        .order_by(Task.priority, Task.sequence_no, Task.task_id)
        .all()
    )
    recent_tasks_limit = max(0, 50 - len(active_tasks))
    recent_tasks = (
        db.query(Task)
        .filter(Task.status.notin_(active_task_statuses))
        .order_by(Task.task_id.desc())
        .limit(recent_tasks_limit)
        .all()
        if recent_tasks_limit > 0
        else []
    )
    tasks = active_tasks + recent_tasks
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
            build_robot_summary(db, robot)
            for robot in robots
        ],
        "tasks": [
            build_task_summary(db, task)
            for task in tasks
        ],
        "products": [
            build_product_summary(db, product)
            for product in products
        ],
        "low_stock_count": sum(
            1 for product in products
            if is_low_stock(product.stock_qty)
        ),
        "pickup_slots": [
            build_pickup_slot_summary(db, slot)
            for slot in pickup_slots
        ],
        "exceptions": [
            build_exception_summary(db, exception)
            for exception in exceptions
        ],
        "exception_history": [
            build_exception_summary(db, exception)
            for exception in exception_history
        ],
        "unresolved_exception_count": unresolved_exception_count,
    }


def build_customer_status(db: Session):
    products = db.query(Product).order_by(Product.product_id).all()
    orders = (
        db.query(Order)
        .filter(Order.status != "COMPLETED")
        .order_by(Order.order_id.desc())
        .limit(50)
        .all()
    )

    return {
        "products": [
            build_product_summary(db, product)
            for product in products
        ],
        "orders": [
            build_order_summary(db, order)
            for order in orders
        ],
    }
