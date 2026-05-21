from sqlalchemy import Boolean, CheckConstraint, Column, DateTime, Float, ForeignKey, Integer, String, Text
from sqlalchemy.dialects.postgresql import ENUM
from sqlalchemy.orm import declarative_base


Base = declarative_base()

order_status_enum = ENUM(
    "ORDER_RECEIVED",
    "ORDER_WAIT",
    "SORTING",
    "DELIVERING",
    "INSPECTING",
    "PICKUP_READY",
    "COMPLETED",
    "ERROR",
    name="order_status",
    create_type=False,
)

order_item_status_enum = ENUM(
    "WAITING",
    "SORTED",
    "INSPECTED",
    "MISSING",
    "EXCESS",
    "MISMATCH",
    name="order_item_status",
    create_type=False,
)

pickup_slot_status_enum = ENUM(
    "EMPTY",
    "RESERVED",
    "OCCUPIED",
    "BLOCKED",
    name="pickup_slot_status",
    create_type=False,
)

robot_type_enum = ENUM(
    "PICKY",
    "COBOT",
    name="robot_type",
    create_type=False,
)

robot_status_enum = ENUM(
    "OFFLINE",
    "IDLE",
    "BUSY",
    "CHARGING",
    "EMERGENCY_STOP",
    "ERROR",
    name="robot_status",
    create_type=False,
)

picky_state_enum = ENUM(
    "CHARGING",
    "STANDBY",
    "MOVING_TO_PRODUCT",
    "WAITING_FOR_COBOT",
    "MOVING_TO_PICKUP",
    "MOVING_TO_STOCK",
    "MOVING_TO_STORAGE",
    "RETURNING",
    "DOCKING",
    "ERROR_RECOVERY",
    name="picky_state",
    create_type=False,
)

cobot_state_enum = ENUM(
    "STANDBY",
    "SORTING",
    "LOADING",
    "INSPECTING",
    "UNLOADING",
    "STOCKING_SORTING",
    "STOCKING_LOADING",
    "STOCKING_PLACING",
    "STOWING_ARM",
    "SAFETY_STOPPED",
    name="cobot_state",
    create_type=False,
)

task_type_enum = ENUM(
    "MOVE_TO_PRODUCT",
    "SORTING_AND_LOAD",
    "MOVE_TO_PICKUP",
    "INSPECTION",
    "UNLOAD",
    "MOVE_TO_STOCK",
    "STOCKING_PICK",
    "MOVE_TO_STORAGE",
    "STOCKING_PLACE",
    "RETURN_HOME",
    "CHARGE",
    name="task_type",
    create_type=False,
)

task_status_enum = ENUM(
    "QUEUED",
    "ASSIGNED",
    "RUNNING",
    "PAUSED",
    "SUCCESS",
    "FAILED",
    "CANCELLED",
    name="task_status",
    create_type=False,
)

exception_type_enum = ENUM(
    "OBSTACLE_DETECTED",
    "LOW_BATTERY",
    "NAVIGATION_FAILED",
    "HARDWARE_ERROR",
    "TIMEOUT",
    "SORTING_FAIL",
    "INSPECTION_FAIL",
    "HUMAN_DETECTED",
    "SYSTEM_ERROR",
    name="exception_type",
    create_type=False,
)

stocking_policy_enum = ENUM(
    "REQUESTED_QUANTITY",
    "ALL_DETECTED",
    name="stocking_policy",
    create_type=False,
)

stocking_item_status_enum = ENUM(
    "REQUESTED",
    "ASSIGNED",
    "IN_PROGRESS",
    "COMPLETED",
    "FAILED",
    "CANCELLED",
    name="stocking_item_status",
    create_type=False,
)


class Zone(Base):
    __tablename__ = "zone"

    zone_id = Column(Integer, primary_key=True)
    zone_name = Column(String(50), nullable=False)
    zone_type = Column(String(30), nullable=False)
    pos_x = Column(Float, nullable=False)
    pos_y = Column(Float, nullable=False)
    pos_z = Column(Float, nullable=False)
    pos_theta = Column(Float)


class Product(Base):
    __tablename__ = "product"

    product_id = Column(Integer, primary_key=True, index=True)
    name = Column(String(100), nullable=False)
    image_url = Column(Text)
    stock_qty = Column(Integer, nullable=False, default=0)
    storage_zone_id = Column(Integer, ForeignKey("zone.zone_id"), nullable=False)


class PickupSlot(Base):
    __tablename__ = "pickup_slot"

    slot_id = Column(Integer, primary_key=True)
    slot_name = Column(String(50))
    status = Column(pickup_slot_status_enum, nullable=False, default="EMPTY")


class RobotUnit(Base):
    __tablename__ = "robot_unit"

    unit_id = Column(Integer, primary_key=True)
    unit_name = Column(String(50), nullable=False)
    description = Column(Text)


class Order(Base):
    __tablename__ = "orders"

    order_id = Column(Integer, primary_key=True)
    order_no = Column(String(30), unique=True)
    status = Column(order_status_enum, nullable=False, default="ORDER_RECEIVED")
    priority = Column(Integer, nullable=False, default=2)
    pickup_slot_id = Column(Integer, ForeignKey("pickup_slot.slot_id"))
    assigned_unit_id = Column(Integer, ForeignKey("robot_unit.unit_id"))


class OrderItem(Base):
    __tablename__ = "order_item"

    item_id = Column(Integer, primary_key=True)
    order_id = Column(Integer, ForeignKey("orders.order_id"), nullable=False)
    product_id = Column(Integer, ForeignKey("product.product_id"), nullable=False)
    quantity = Column(Integer, nullable=False)
    status = Column(order_item_status_enum, nullable=False, default="WAITING")


class StockingItem(Base):
    __tablename__ = "stocking_item"
    __table_args__ = (
        CheckConstraint(
            "(stocking_policy = 'REQUESTED_QUANTITY' AND requested_quantity IS NOT NULL) "
            "OR (stocking_policy = 'ALL_DETECTED' AND requested_quantity IS NULL)",
            name="ck_stocking_item_policy_quantity",
        ),
    )

    stocking_item_id = Column(Integer, primary_key=True)
    product_id = Column(Integer, ForeignKey("product.product_id"), nullable=False)
    requested_quantity = Column(Integer)
    detected_quantity = Column(Integer)
    stock_delta = Column(Integer)
    stocking_policy = Column(stocking_policy_enum, nullable=False)
    status = Column(stocking_item_status_enum, nullable=False, default="REQUESTED")
    assigned_unit_id = Column(Integer, ForeignKey("robot_unit.unit_id"))


class Robot(Base):
    __tablename__ = "robot"
    __table_args__ = (
        CheckConstraint(
            "(robot_type = 'PICKY' AND cobot_state IS NULL) "
            "OR (robot_type = 'COBOT' AND picky_state IS NULL)",
            name="ck_robot_type_state",
        ),
    )

    robot_id = Column(Integer, primary_key=True)
    robot_name = Column(String(30), unique=True, nullable=False)
    unit_id = Column(Integer, ForeignKey("robot_unit.unit_id"))
    robot_type = Column(robot_type_enum, nullable=False)
    robot_status = Column(robot_status_enum, nullable=False, default="IDLE")
    picky_state = Column(picky_state_enum)
    cobot_state = Column(cobot_state_enum)
    current_task_id = Column(Integer, ForeignKey("task.task_id"))
    ros_namespace = Column(String(50))
    battery_level = Column(Integer)
    pos_x = Column(Float)
    pos_y = Column(Float)
    pos_theta = Column(Float)


class Task(Base):
    __tablename__ = "task"
    __table_args__ = (
        CheckConstraint(
            "NOT (order_item_id IS NOT NULL AND stocking_item_id IS NOT NULL)",
            name="ck_task_item_or_stocking_item",
        ),
        CheckConstraint(
            "stocking_item_id IS NULL OR (order_id IS NULL AND order_item_id IS NULL)",
            name="ck_task_stocking_without_order",
        ),
    )

    task_id = Column(Integer, primary_key=True)
    order_id = Column(Integer, ForeignKey("orders.order_id"))
    order_item_id = Column(Integer, ForeignKey("order_item.item_id"))
    stocking_item_id = Column(Integer, ForeignKey("stocking_item.stocking_item_id"))
    sequence_no = Column(Integer, nullable=False)
    assigned_robot_id = Column(Integer, ForeignKey("robot.robot_id"))
    task_type = Column(task_type_enum, nullable=False)
    status = Column(task_status_enum, nullable=False, default="QUEUED")
    priority = Column(Integer, nullable=False, default=2)
    source_zone_id = Column(Integer, ForeignKey("zone.zone_id"))
    target_zone_id = Column(Integer, ForeignKey("zone.zone_id"))
    result_message = Column(Text)


class TaskEvent(Base):
    __tablename__ = "task_event"

    event_id = Column(Integer, primary_key=True)
    task_id = Column(Integer, ForeignKey("task.task_id"), nullable=False)
    robot_id = Column(Integer, ForeignKey("robot.robot_id"))
    from_status = Column(task_status_enum)
    to_status = Column(task_status_enum, nullable=False)
    event_name = Column(String(50))
    reason = Column(Text)
    created_at = Column(DateTime(timezone=True))


class ExceptionLog(Base):
    __tablename__ = "exception_log"

    exception_id = Column(Integer, primary_key=True)
    robot_id = Column(Integer, ForeignKey("robot.robot_id"))
    task_id = Column(Integer, ForeignKey("task.task_id"))
    order_id = Column(Integer, ForeignKey("orders.order_id"))
    exception_type = Column(exception_type_enum, nullable=False)
    detail = Column(Text)
    is_resolved = Column(Boolean, nullable=False, default=False)
    created_at = Column(DateTime(timezone=True))
