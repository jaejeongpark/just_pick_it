"""Fleet Manager HTTP API request schemas.

명령 엔드포인트(POST/PATCH)의 입력 검증용 Pydantic v1 모델이다.
응답은 FleetRepository 가 만드는 dict 를 그대로 반환하므로 별도 응답 모델은 두지 않는다.
"""

from __future__ import annotations

from pydantic import BaseModel, Field


# =====================================
# Customer/order requests
# =====================================

class OrderItemIn(BaseModel):
    product_id: int
    quantity: int = Field(gt=0)


class OrderCreateIn(BaseModel):
    items: list[OrderItemIn] = Field(min_items=1)


# =====================================
# Admin product requests
# =====================================

class ProductCreateIn(BaseModel):
    name: str = Field(min_length=1, max_length=100)
    stock_qty: int = Field(ge=0)
    storage_zone_id: int | None = None
    storage_location: str | None = Field(default=None, min_length=1, max_length=50)
    image_url: str | None = None


class ProductUpdateIn(BaseModel):
    name: str = Field(min_length=1, max_length=100)
    stock_qty: int = Field(ge=0)
    storage_zone_id: int | None = None
    storage_location: str | None = Field(default=None, min_length=1, max_length=50)
    image_url: str | None = None


class ProductStockUpdateIn(BaseModel):
    stock_qty: int = Field(ge=0)


# =====================================
# Admin operation requests
# =====================================

class PickupSlotCreateIn(BaseModel):
    slot_name: str = Field(min_length=1, max_length=50)
    status: str = "EMPTY"


class PickupSlotStateUpdateIn(BaseModel):
    status: str | None = None


class DisplayItemCreateIn(BaseModel):
    product_id: int
    requested_quantity: int | None = Field(default=None, gt=0)
    detected_quantity: int | None = Field(default=None, ge=0)
    stock_delta: int | None = Field(default=None, ge=0)
    display_policy: str | None = None
    assigned_unit_id: int | None = None


# =====================================
# Fleet state write requests
# =====================================

class OrderItemQuantityUpdateIn(BaseModel):
    item_id: int
    quantity: int = Field(gt=0)


class FleetOrderStateUpdateIn(BaseModel):
    status: str | None = None
    pickup_slot_id: int | None = None
    assigned_unit_id: int | None = None
    item_quantities: list[OrderItemQuantityUpdateIn] | None = None


class FleetTaskStateUpdateIn(BaseModel):
    current_status: str | None = None
    status: str | None = None
    assigned_robot_id: int | str | None = None
    assigned_robot_name: str | None = None
    result_message: str | None = None


class FleetRobotStateUpdateIn(BaseModel):
    status: str | None = None
    robot_status: str | None = None
    picky_state: str | None = None
    cobot_state: str | None = None
    current_task_id: int | None = None
    battery_level: int | None = Field(default=None, ge=0, le=100)
    pos_x: float | None = None
    pos_y: float | None = None
    pos_theta: float | None = None


class FleetTaskCreateIn(BaseModel):
    order_id: int | None = None
    order_item_id: int | None = None
    display_item_id: int | None = None
    sequence_no: int | None = Field(default=None, ge=1)
    assigned_robot_id: int | str | None = None
    assigned_robot_name: str | None = None
    task_type: str = Field(min_length=1)
    status: str = "QUEUED"
    priority: int = Field(default=2, ge=1)
    source_zone_id: int | None = None
    target_zone_id: int | None = None
    result_message: str | None = None


class FleetTaskBulkCreateIn(BaseModel):
    tasks: list[FleetTaskCreateIn] = Field(min_items=1)
