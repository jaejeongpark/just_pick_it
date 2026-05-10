from sqlalchemy.orm import Session

from app.models import ExceptionLog, Order, OrderItem, PickupSlot, Product, Robot, Task


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
            build_product_summary(product)
            for product in products
        ],
        "orders": [
            build_order_summary(db, order)
            for order in orders
        ],
    }
