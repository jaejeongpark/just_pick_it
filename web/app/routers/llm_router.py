from __future__ import annotations

from urllib.parse import urljoin

import httpx
from fastapi import APIRouter
from pydantic import BaseModel, Field

from app.config import FLEET_API_BASE_URL
from app.services.llm_client import build_llm_message


# =====================================
# Router
# =====================================

router = APIRouter(tags=["llm-command"])


# =====================================
# Schemas
# =====================================

class LlmMessageIn(BaseModel):
    message: str = Field(min_length=1)


# =====================================
# Fleet API forwarding
# =====================================

def _fleet_api_url(path: str) -> str:
    return urljoin(FLEET_API_BASE_URL.rstrip("/") + "/", path.lstrip("/"))


def _fleet_error_message(response: httpx.Response) -> str:
    try:
        return response.json().get("detail")
    except ValueError:
        return response.text


async def _create_order(parsed: dict) -> dict:
    payload = {
        "items": [
            {
                "product_id": item.get("product_id"),
                "quantity": item.get("quantity"),
            }
            for item in parsed.get("items", [])
        ]
    }

    try:
        async with httpx.AsyncClient(timeout=10.0) as client:
            response = await client.post(_fleet_api_url("/api/orders"), json=payload)
    except httpx.RequestError as exc:
        return {
            **parsed,
            "result": "error",
            "message": f"Fleet API 주문 생성 실패: {exc}",
        }

    if response.status_code >= 400:
        return {
            **parsed,
            "result": "error",
            "message": f"Fleet API 주문 생성 실패: {_fleet_error_message(response)}",
        }

    order = response.json()
    return {
        **parsed,
        "result": "ok",
        "message": f"주문이 생성되었습니다. order_id={order.get('order_id')}, order_no={order.get('order_no')}",
        "order": order,
        "order_id": order.get("order_id"),
        "order_no": order.get("order_no"),
    }


# =====================================
# Routes
# =====================================

@router.post("/api/customer/llm/messages")
async def create_customer_llm_message(body: LlmMessageIn) -> dict:
    """고객 음성/텍스트 주문 메시지를 LLM parser/client 로 전달한다."""
    parsed = build_llm_message(body.message, context={"surface": "customer"})
    if parsed.get("result") == "error":
        return parsed

    if str(parsed.get("action") or "").upper() == "ORDER":
        return await _create_order(parsed)

    return parsed
