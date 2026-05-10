from typing import Literal

from pydantic import BaseModel, ConfigDict, Field


class ProductRead(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    product_id: int
    name: str
    image_url: str | None
    stock_qty: int
    storage_location: str


class ProductStockUpdate(BaseModel):
    stock_qty: int = Field(ge=0)


class ProductCreate(BaseModel):
    name: str = Field(min_length=1, max_length=100)
    stock_qty: int = Field(ge=0)
    storage_location: str = Field(min_length=1, max_length=50)
    image_url: str | None = None


class ProductUpdate(BaseModel):
    name: str = Field(min_length=1, max_length=100)
    stock_qty: int = Field(ge=0)
    storage_location: str = Field(min_length=1, max_length=50)
    image_url: str | None = None


class AdminLlmMessageCreate(BaseModel):
    message: str = Field(min_length=1)


class AdminLlmMessageRead(BaseModel):
    result: str
    message: str
    action: str | None = None
    task_id: int | None = None
    assigned_robot_id: str | None = None
    target_zone_id: int | None = None
    target_zone_name: str | None = None
    provider: str = "mock"


class OrderItemCreate(BaseModel):
    product_id: int
    quantity: int = Field(gt=0)


class OrderCreate(BaseModel):
    items: list[OrderItemCreate] = Field(min_length=1)


class OrderItemRead(BaseModel):
    product_id: int
    product_name: str
    quantity: int
    status: str


class OrderRead(BaseModel):
    order_id: int
    order_no: str
    status: str
    pickup_slot_id: int | None = None
    pickup_slot_name: str | None = None
    items: list[OrderItemRead] = []


OrderStatus = Literal[
    "ORDER_RECEIVED",
    "ORDER_WAIT",
    "SORTING",
    "DELIVERING",
    "INSPECTING",
    "PICKUP_READY",
    "COMPLETED",
    "ERROR",
]

TaskStatus = Literal[
    "QUEUED",
    "ASSIGNED",
    "RUNNING",
    "PAUSED",
    "SUCCESS",
    "FAILED",
    "CANCELLED",
]

RobotStatus = Literal[
    "IDLE",
    "MOVING",
    "WAITING",
    "SORTING",
    "DELIVERING",
    "INSPECTING",
    "UNLOADING",
    "PATROLLING",
    "CHARGING",
    "RETURNING",
    "PARKING",
    "EMERGENCY_STOP",
    "ERROR",
    "OFFLINE",
]

PickupSlotStatus = Literal[
    "EMPTY",
    "RESERVED",
    "OCCUPIED",
    "BLOCKED",
]

TaskType = Literal[
    "SORTING",
    "DELIVERY",
    "INSPECTION",
    "UNLOAD",
    "PATROL",
    "CHARGE",
    "RETURN_HOME",
]

ExceptionType = Literal[
    "OBSTACLE_DETECTED",
    "LOW_BATTERY",
    "NAVIGATION_FAILED",
    "HARDWARE_ERROR",
    "TIMEOUT",
    "SORTING_FAIL",
    "INSPECTION_FAIL",
    "HUMAN_DETECTED",
    "SYSTEM_ERROR",
]


class AdminRobotCreate(BaseModel):
    robot_id: str = Field(min_length=1, max_length=30)
    status: RobotStatus = "IDLE"
    ros_namespace: str | None = None
    battery_level: int | None = Field(default=None, ge=0, le=100)
    pos_x: float | None = None
    pos_y: float | None = None
    pos_theta: float | None = None


class AdminPickupSlotCreate(BaseModel):
    slot_name: str = Field(min_length=1, max_length=50)
    status: PickupSlotStatus = "EMPTY"


class AdminTaskCreate(BaseModel):
    task_type: TaskType
    status: TaskStatus = "QUEUED"
    order_id: int | None = None
    assigned_robot_id: str | None = None
    source_zone_id: int | None = None
    target_zone_id: int | None = None
    result_message: str | None = None


class FleetTaskCreate(BaseModel):
    task_type: TaskType
    order_id: int | None = None
    assigned_robot_id: str | None = None
    status: TaskStatus = "QUEUED"
    source_zone_id: int | None = None
    target_zone_id: int | None = None
    result_message: str | None = None


class FleetOrderStateUpdate(BaseModel):
    status: OrderStatus | None = None
    pickup_slot_id: int | None = None


class FleetTaskStateUpdate(BaseModel):
    status: TaskStatus | None = None
    assigned_robot_id: str | None = None
    result_message: str | None = None


class FleetRobotStateUpdate(BaseModel):
    status: RobotStatus | None = None
    current_task_id: int | None = None
    battery_level: int | None = Field(default=None, ge=0, le=100)
    pos_x: float | None = None
    pos_y: float | None = None
    pos_theta: float | None = None


class FleetPickupSlotStateUpdate(BaseModel):
    status: PickupSlotStatus | None = None


class FleetStateUpdateRead(BaseModel):
    status: str


class FleetTaskRead(BaseModel):
    status: str
    task_id: int


class FleetTaskSummaryRead(BaseModel):
    task_id: int
    order_id: int | None = None
    order_no: str | None = None
    assigned_robot_id: str | None = None
    task_type: str
    status: str
    source_zone_id: int | None = None
    target_zone_id: int | None = None
    result_message: str | None = None


class FleetPickupSlotRead(BaseModel):
    slot_id: int
    slot_name: str
    status: str
    order_id: int | None = None
    order_no: str | None = None


class FleetPickupSlotAssignmentRead(BaseModel):
    status: str
    order_id: int
    order_no: str
    pickup_slot_id: int
    slot_name: str
    slot_status: str


class FleetTaskEventCreate(BaseModel):
    robot_id: str | None = None
    from_status: TaskStatus | None = None
    to_status: TaskStatus
    event_name: str | None = Field(default=None, max_length=50)
    reason: str | None = None
    update_task_status: bool = True


class FleetTaskEventRead(BaseModel):
    event_id: int
    task_id: int
    robot_id: str | None
    from_status: str | None
    to_status: str
    event_name: str | None
    reason: str | None
    created_at: str | None


class FleetExceptionCreate(BaseModel):
    exception_type: ExceptionType
    robot_id: str | None = None
    task_id: int | None = None
    order_id: int | None = None
    detail: str | None = None


class FleetExceptionRead(BaseModel):
    status: str
    exception_id: int
