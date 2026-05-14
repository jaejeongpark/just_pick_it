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
    provider: str = "local"


class OrderItemCreate(BaseModel):
    product_id: int
    quantity: int = Field(gt=0)


class OrderCreate(BaseModel):
    items: list[OrderItemCreate] = Field(min_length=1)


class OrderItemRead(BaseModel):
    product_id: int
    product_name: str
    image_url: str | None = None
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
]

PickupSlotStatus = Literal[
    "EMPTY",
    "RESERVED",
    "OCCUPIED",
    "BLOCKED",
]

TaskType = Literal[
    "STANDBY_LOAD",
    "STANDBY_UNLOAD",
    "SORTING",
    "LOAD",
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
    "FIRE_DETECTED",
]


class AdminPickupSlotCreate(BaseModel):
    slot_name: str = Field(min_length=1, max_length=50)
    status: PickupSlotStatus = "EMPTY"


class FleetOrderStateUpdate(BaseModel):
    status: OrderStatus | None = None
    pickup_slot_id: int | None = None


class FleetTaskStateUpdate(BaseModel):
    current_status: TaskStatus | None = None
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
    previous_status: str | None = None
    current_status: str | None = None


class FleetZonePoseRead(BaseModel):
    x: float
    y: float
    z: float
    theta: float | None = None


class FleetTaskSummaryRead(BaseModel):
    task_id: int
    order_id: int | None = None
    order_no: str | None = None
    assigned_robot_id: str | None = None
    task_type: str
    status: str
    priority: int
    source_zone_id: int | None = None
    source_zone_name: str | None = None
    source_zone_pose: FleetZonePoseRead | None = None
    target_zone_id: int | None = None
    target_zone_name: str | None = None
    target_zone_pose: FleetZonePoseRead | None = None
    result_message: str | None = None


class FleetRobotRuntimeRead(BaseModel):
    robot_id: str
    status: str
    battery_level: int | None = None
    current_task_id: int | None = None
    current_task_type: str | None = None
    current_task_status: str | None = None
    current_task: FleetTaskSummaryRead | None = None
    pos_x: float | None = None
    pos_y: float | None = None
    pos_theta: float | None = None


class FleetRobotRunningTaskRead(BaseModel):
    task_type: str | None = None


class FleetOrderSummaryRead(BaseModel):
    order_id: int
    order_no: str
    status: str
    priority: int
    pickup_slot_id: int | None = None
    pickup_slot_name: str | None = None
    current_task_id: int | None = None
    current_task_type: str | None = None
    current_task_status: str | None = None
    assigned_robot_id: str | None = None


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
