from sqlalchemy import Boolean, Column, DateTime, Float, ForeignKey, Integer, String, Text
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

robot_status_enum = ENUM(
    "IDLE",
    "MOVING",
    "WAITING",
    "STANDBY",
    "SORTING",
    "LOADING",
    "PARKING",
    "INSPECTING",
    "UNLOADING",
    "PATROLLING",
    "CHARGING",
    "RETURNING",
    "DOCKING",
    "EMERGENCY_STOP",
    "ERROR",
    "OFFLINE",
    name="robot_status",
    create_type=False,
)

task_type_enum = ENUM(
    "STANDBY_LOAD",
    "STANDBY_UNLOAD",
    "SORTING",
    "LOAD",
    "INSPECTION",
    "UNLOAD",
    "PATROL",
    "CHARGE",
    "RETURN_HOME",
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
    "FIRE_DETECTED",
    name="exception_type",
    create_type=False,
)


class Product(Base):
    __tablename__ = "product"

    product_id = Column(Integer, primary_key=True, index=True)
    name = Column(String(100), nullable=False)
    image_url = Column(Text)
    stock_qty = Column(Integer, nullable=False, default=0)
    storage_location = Column(String(50), nullable=False)


class Zone(Base):
    __tablename__ = "zone"

    zone_id = Column(Integer, primary_key=True)
    zone_name = Column(String(50), nullable=False)
    pos_x = Column(Float, nullable=False)
    pos_y = Column(Float, nullable=False)
    pos_z = Column(Float, nullable=False)
    pos_theta = Column(Float)


class PickupSlot(Base):
    __tablename__ = "pickup_slot"

    slot_id = Column(Integer, primary_key=True)
    slot_name = Column(String(50))
    status = Column(pickup_slot_status_enum, nullable=False, default="EMPTY")


class Order(Base):
    __tablename__ = "orders"

    order_id = Column(Integer, primary_key=True)
    order_no = Column(String(30), unique=True)
    status = Column(order_status_enum, nullable=False, default="ORDER_RECEIVED")
    priority = Column(Integer, nullable=False, default=2)
    pickup_slot_id = Column(Integer, ForeignKey("pickup_slot.slot_id"))


class OrderItem(Base):
    __tablename__ = "order_item"

    item_id = Column(Integer, primary_key=True)
    order_id = Column(Integer, ForeignKey("orders.order_id"), nullable=False)
    product_id = Column(Integer, ForeignKey("product.product_id"), nullable=False)
    quantity = Column(Integer, nullable=False)
    status = Column(order_item_status_enum, nullable=False, default="WAITING")


class Robot(Base):
    __tablename__ = "robot"

    robot_id = Column(String(30), primary_key=True)
    status = Column(robot_status_enum, nullable=False, default="IDLE")
    current_task_id = Column(Integer)
    ros_namespace = Column(String(50))
    battery_level = Column(Integer)
    pos_x = Column(Float)
    pos_y = Column(Float)
    pos_theta = Column(Float)


class Task(Base):
    __tablename__ = "task"

    task_id = Column(Integer, primary_key=True)
    order_id = Column(Integer, ForeignKey("orders.order_id"))
    assigned_robot_id = Column(String(30), ForeignKey("robot.robot_id"))
    task_type = Column(task_type_enum, nullable=False)
    status = Column(task_status_enum, nullable=False, default="QUEUED")
    priority = Column(Integer, nullable=False, default=1)
    source_zone_id = Column(Integer, ForeignKey("zone.zone_id"))
    target_zone_id = Column(Integer, ForeignKey("zone.zone_id"))
    result_message = Column(Text)


class TaskEvent(Base):
    __tablename__ = "task_event"

    event_id = Column(Integer, primary_key=True)
    task_id = Column(Integer, ForeignKey("task.task_id"), nullable=False)
    robot_id = Column(String(30), ForeignKey("robot.robot_id"))
    from_status = Column(task_status_enum)
    to_status = Column(task_status_enum, nullable=False)
    event_name = Column(String(50))
    reason = Column(Text)
    created_at = Column(DateTime(timezone=True))


class ExceptionLog(Base):
    __tablename__ = "exception_log"

    exception_id = Column(Integer, primary_key=True)
    robot_id = Column(String(30), ForeignKey("robot.robot_id"))
    task_id = Column(Integer, ForeignKey("task.task_id"))
    order_id = Column(Integer, ForeignKey("orders.order_id"))
    exception_type = Column(exception_type_enum, nullable=False)
    detail = Column(Text)
    is_resolved = Column(Boolean, nullable=False, default=False)
    created_at = Column(DateTime(timezone=True))
