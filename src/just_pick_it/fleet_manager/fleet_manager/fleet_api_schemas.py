"""Fleet Manager HTTP API 요청 본문 스키마.

명령 엔드포인트(POST/PATCH)의 입력 검증용 Pydantic 모델. 응답은 FleetRepository 가
만드는 dict 를 그대로 반환하므로 별도 응답 모델은 두지 않는다.
"""

from __future__ import annotations

from pydantic import BaseModel, Field


class OrderItemIn(BaseModel):
    product_id: int
    quantity: int = Field(gt=0)


class OrderCreateIn(BaseModel):
    items: list[OrderItemIn] = Field(min_length=1)


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


class PickupSlotCreateIn(BaseModel):
    slot_name: str = Field(min_length=1, max_length=50)
    status: str = "EMPTY"
