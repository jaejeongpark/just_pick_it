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

class AdminLlmMessageIn(BaseModel):
    message: str = Field(min_length=1)


# =====================================
# Fleet API helpers
# =====================================

def _fleet_api_url(path: str) -> str:
    return urljoin(FLEET_API_BASE_URL.rstrip("/") + "/", path.lstrip("/"))


async def _create_stocking_item(parsed: dict) -> dict:
    product_id = parsed.get("product_id")
    if product_id is None:
        return {
            **parsed,
            "result": "error",
            "message": "입고 명령은 파싱됐지만 product_id가 없습니다. LLM parser가 product_id를 반환해야 합니다.",
        }

    payload = {
        "product_id": product_id,
        "requested_quantity": parsed.get("requested_quantity"),
        "stocking_policy": parsed.get("stocking_policy"),
    }

    try:
        async with httpx.AsyncClient(timeout=10.0) as client:
            response = await client.post(_fleet_api_url("/api/admin/stocking-items"), json=payload)
    except httpx.RequestError as exc:
        return {
            **parsed,
            "result": "error",
            "message": f"Fleet API 입고 요청 생성 실패: {exc}",
        }

    if response.status_code >= 400:
        try:
            detail = response.json().get("detail")
        except ValueError:
            detail = response.text
        return {
            **parsed,
            "result": "error",
            "message": f"Fleet API 입고 요청 생성 실패: {detail}",
        }

    stocking_item = response.json()
    return {
        **parsed,
        "result": "ok",
        "message": (
            f"입고 요청이 생성되었습니다. "
            f"stocking_item_id={stocking_item.get('stocking_item_id')}, "
            f"상품={stocking_item.get('product_name') or parsed.get('product_name') or parsed.get('product_id')}"
        ),
        "stocking_item_id": stocking_item.get("stocking_item_id"),
        "product_id": stocking_item.get("product_id", parsed.get("product_id")),
        "product_name": stocking_item.get("product_name", parsed.get("product_name")),
        "requested_quantity": stocking_item.get("requested_quantity", parsed.get("requested_quantity")),
        "stocking_policy": stocking_item.get("stocking_policy", parsed.get("stocking_policy")),
        "stocking_status": stocking_item.get("status"),
    }


# =====================================
# Routes
# =====================================

@router.post("/api/admin/llm/messages")
async def create_llm_message(body: AdminLlmMessageIn) -> dict:
    """관리자 AI 명령을 처리한다.

    LLM parser/client 는 Web Gateway 에 남긴다. 다만 DB 쓰기는 직접 하지 않고,
    STOCKING 으로 파싱된 경우 Fleet API 에 stocking_item 생성을 위임한다.
    """
    parsed = build_llm_message(body.message)
    if parsed.get("result") == "error":
        return parsed

    if str(parsed.get("action") or "").upper() == "STOCKING":
        return await _create_stocking_item(parsed)

    return parsed
