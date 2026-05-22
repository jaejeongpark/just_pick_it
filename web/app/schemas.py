from typing import Literal

from pydantic import BaseModel, ConfigDict, Field

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

OrderItemStatus = Literal[
    "WAITING",
    "SORTED",
    "INSPECTED",
    "MISSING",
    "EXCESS",
    "MISMATCH",
]

PickupSlotStatus = Literal[
    "EMPTY",
    "RESERVED",
    "OCCUPIED",
    "BLOCKED",
]

RobotType = Literal[
    "PICKY",
    "COBOT",
]

RobotStatus = Literal[
    "OFFLINE",
    "IDLE",
    "BUSY",
    "CHARGING",
    "EMERGENCY_STOP",
    "ERROR",
]

PickyState = Literal[
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
]

CobotState = Literal[
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
]

TaskType = Literal[
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
    "DOCK_IN",
    "CHARGE",
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

StockingPolicy = Literal[
    "REQUESTED_QUANTITY",
    "ALL_DETECTED",
]

StockingItemStatus = Literal[
    "REQUESTED",
    "ASSIGNED",
    "IN_PROGRESS",
    "COMPLETED",
    "FAILED",
    "CANCELLED",
]


class FleetZonePoseRead(BaseModel):
    x: float
    y: float
    z: float
    theta: float | None = None


class FleetZoneRead(BaseModel):
    zone_id: int
    zone_name: str
    zone_type: str
    pose: FleetZonePoseRead


class ProductRead(BaseModel):
    model_config = ConfigDict(from_attributes=True)

    product_id: int
    name: str
    image_url: str | None
    stock_qty: int
    storage_zone_id: int
    storage_zone_name: str | None = None
    storage_zone_pose: FleetZonePoseRead | None = None
    storage_location: str


class ProductStockUpdate(BaseModel):
    stock_qty: int = Field(ge=0)


class ProductCreate(BaseModel):
    name: str = Field(min_length=1, max_length=100)
    stock_qty: int = Field(ge=0)
    storage_zone_id: int | None = None
    storage_location: str | None = Field(default=None, min_length=1, max_length=50)
    image_url: str | None = None


class ProductUpdate(BaseModel):
    name: str = Field(min_length=1, max_length=100)
    stock_qty: int = Field(ge=0)
    storage_zone_id: int | None = None
    storage_location: str | None = Field(default=None, min_length=1, max_length=50)
    image_url: str | None = None


class AdminLlmMessageCreate(BaseModel):
    message: str = Field(min_length=1)


class AdminLlmMessageRead(BaseModel):
    result: str
    message: str
    action: str | None = None
    task_id: int | None = None
    assigned_robot_id: int | None = None
    assigned_robot_name: str | None = None
    target_zone_id: int | None = None
    target_zone_name: str | None = None
    product_id: int | None = None
    product_name: str | None = None
    requested_quantity: int | None = None
    stocking_policy: StockingPolicy | None = None
    stocking_item_id: int | None = None
    provider: str = "local"


class OrderItemCreate(BaseModel):
    product_id: int
    quantity: int = Field(gt=0)


class OrderCreate(BaseModel):
    items: list[OrderItemCreate] = Field(min_length=1)


class OrderItemRead(BaseModel):
    item_id: int | None = None
    product_id: int
    product_name: str
    image_url: str | None = None
    quantity: int
    status: str


class OrderRead(BaseModel):
    order_id: int
    order_no: str
    status: str
    priority: int | None = None
    pickup_slot_id: int | None = None
    pickup_slot_name: str | None = None
    assigned_unit_id: int | None = None
    items: list[OrderItemRead] = []


class AdminPickupSlotCreate(BaseModel):
    slot_name: str = Field(min_length=1, max_length=50)
    status: PickupSlotStatus = "EMPTY"


class FleetOrderStateUpdate(BaseModel):
    status: OrderStatus | None = None
    pickup_slot_id: int | None = None
    assigned_unit_id: int | None = None


class FleetTaskStateUpdate(BaseModel):
    current_status: TaskStatus | None = None
    status: TaskStatus | None = None
    assigned_robot_id: int | str | None = None
    assigned_robot_name: str | None = None
    result_message: str | None = None


class FleetRobotStateUpdate(BaseModel):
    status: RobotStatus | None = None
    robot_status: RobotStatus | None = None
    picky_state: PickyState | None = None
    cobot_state: CobotState | None = None
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


class FleetTaskCreate(BaseModel):
    order_id: int | None = None
    order_item_id: int | None = None
    stocking_item_id: int | None = None
    sequence_no: int = Field(ge=1)
    assigned_robot_id: int | str | None = None
    assigned_robot_name: str | None = None
    task_type: TaskType
    status: TaskStatus = "QUEUED"
    priority: int = 2
    source_zone_id: int | None = None
    target_zone_id: int | None = None
    result_message: str | None = None


class FleetTaskBulkCreate(BaseModel):
    tasks: list[FleetTaskCreate] = Field(min_length=1)


class FleetTaskBulkCreateRead(BaseModel):
    status: str
    task_ids: list[int]
    created_count: int


class FleetTaskSummaryRead(BaseModel):
    task_id: int
    order_id: int | None = None
    order_no: str | None = None
    order_item_id: int | None = None
    stocking_item_id: int | None = None
    product_id: int | None = None
    product_name: str | None = None
    product_quantity: int | None = None
    requested_quantity: int | None = None
    detected_quantity: int | None = None
    stock_delta: int | None = None
    stocking_policy: str | None = None
    stocking_status: str | None = None
    sequence_no: int
    assigned_robot_id: int | None = None
    assigned_robot_name: str | None = None
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
    robot_id: int
    robot_name: str
    unit_id: int | None = None
    robot_type: str
    robot_status: str
    status: str
    picky_state: str | None = None
    cobot_state: str | None = None
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
    assigned_unit_id: int | None = None
    current_task_id: int | None = None
    current_task_type: str | None = None
    current_task_status: str | None = None
    assigned_robot_id: int | None = None
    assigned_robot_name: str | None = None


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
    robot_id: int | str | None = None
    robot_name: str | None = None
    from_status: TaskStatus | None = None
    to_status: TaskStatus
    event_name: str | None = Field(default=None, max_length=50)
    reason: str | None = None
    update_task_status: bool = True


class FleetTaskEventRead(BaseModel):
    event_id: int
    task_id: int
    robot_id: int | None
    robot_name: str | None = None
    from_status: str | None
    to_status: str
    event_name: str | None
    reason: str | None
    created_at: str | None


class FleetExceptionCreate(BaseModel):
    exception_type: ExceptionType
    robot_id: int | str | None = None
    robot_name: str | None = None
    task_id: int | None = None
    order_id: int | None = None
    detail: str | None = None


class FleetExceptionRead(BaseModel):
    status: str
    exception_id: int


class FleetStockingItemCreate(BaseModel):
    product_id: int
    requested_quantity: int | None = Field(default=None, gt=0)
    detected_quantity: int | None = Field(default=None, ge=0)
    stock_delta: int | None = Field(default=None, ge=0)
    stocking_policy: StockingPolicy | None = None
    status: StockingItemStatus = "REQUESTED"
    assigned_unit_id: int | None = None


class FleetStockingItemUpdate(BaseModel):
    requested_quantity: int | None = Field(default=None, gt=0)
    detected_quantity: int | None = Field(default=None, ge=0)
    stock_delta: int | None = Field(default=None, ge=0)
    stocking_policy: StockingPolicy | None = None
    status: StockingItemStatus | None = None
    assigned_unit_id: int | None = None


class FleetStockingItemRead(BaseModel):
    stocking_item_id: int
    product_id: int
    product_name: str | None = None
    requested_quantity: int | None = None
    detected_quantity: int | None = None
    stock_delta: int | None = None
    stocking_policy: str
    status: str
    assigned_unit_id: int | None = None


class FleetStockingComplete(BaseModel):
    task_id: int
    detected_quantity: int | None = Field(default=None, ge=0)
    stock_delta: int | None = Field(default=None, ge=0)
    result_message: str | None = None
