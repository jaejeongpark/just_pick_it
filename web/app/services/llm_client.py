from __future__ import annotations

import base64
import io
import json
import os
from typing import Any

import httpx
from openai import OpenAI

from app.config import FLEET_API_BASE_URL

_FALLBACK_PRODUCTS = [
    {"product_id": 1, "product_name": "수박"},
    {"product_id": 2, "product_name": "식빵"},
    {"product_id": 3, "product_name": "환타"},
    {"product_id": 4, "product_name": "크림빵"},
    {"product_id": 5, "product_name": "초코파이"},
    {"product_id": 6, "product_name": "생수"},
]


def _fetch_products() -> list[dict]:
    """Fleet API에서 상품 목록 조회. 실패 시 fallback 사용."""
    try:
        resp = httpx.get(f"{FLEET_API_BASE_URL}/api/products", timeout=5.0)
        if resp.status_code == 200:
            data = resp.json()
            if isinstance(data, list) and data:
                return [
                    {
                        "product_id": p["product_id"],
                        "product_name": p.get("product_name") or p.get("name"),
                    }
                    for p in data
                ]
    except Exception:
        pass
    return _FALLBACK_PRODUCTS


def _transcribe(client: OpenAI, data_url: str, model: str) -> str:
    """data URL 형식의 오디오를 텍스트로 변환한다."""
    header, b64 = data_url.split(",", 1)
    ext = header.split(";")[0].split("/")[1]  # e.g. audio/webm -> webm
    buf = io.BytesIO(base64.b64decode(b64))
    buf.name = f"audio.{ext}"
    return client.audio.transcriptions.create(
        model=model,
        file=buf,
        language="ko",
    ).text


def _parse_items(client: OpenAI, text: str, products: list[dict], model: str) -> dict:
    """텍스트에서 주문 항목을 추출한다."""
    catalog = "\n".join(
        f"- product_id={p['product_id']}, 상품명={p['product_name']}"
        for p in products
    )
    system = (
        "주문 명령에서 상품명과 수량을 추출해 JSON으로만 반환하세요.\n\n"
        f"판매 상품:\n{catalog}\n\n"
        '주문이면: {"action": "ORDER", "items": [{"product_id": <int>, "product_name": "<str>", "quantity": <int>}]}\n'
        '일반 대화이면: {"action": "CHAT", "items": []}\n\n'
        "규칙: 목록에 없는 상품 무시, 수량 미지정 시 1, JSON만 반환"
    )
    resp = client.chat.completions.create(
        model=model,
        messages=[
            {"role": "system", "content": system},
            {"role": "user", "content": text},
        ],
        response_format={"type": "json_object"},
        temperature=0,
    )
    return json.loads(resp.choices[0].message.content)


def build_llm_message(
    message: str,
    context: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """LLM 명령 파싱 진입점.

    LLM 담당자는 이 함수만 실제 구현으로 교체하면 된다.
    data URL 형식 오디오(data:audio/... 또는 data:video/...)가 오면 STT 후 파싱,
    일반 텍스트면 바로 파싱한다.
    """
    api_key = os.getenv("OPENAI_API_KEY")
    if not api_key:
        return {
            "result": "error",
            "message": "OPENAI_API_KEY 환경변수가 설정되지 않았습니다.",
            "action": "CHAT",
            "items": [],
            "provider": "none",
        }

    stt_model = os.getenv("STT_MODEL", "gpt-4o-mini-transcribe")
    parse_model = os.getenv("LLM_MODEL", "gpt-4o-mini")
    client = OpenAI(api_key=api_key)
    stt_client = client
    llm_client = client

    # 1. STT: data URL이면 오디오로 처리, 아니면 텍스트로 그대로 사용
    is_audio = message.startswith("data:audio") or message.startswith("data:video")
    if is_audio:
        try:
            text = _transcribe(stt_client, message, stt_model)
        except Exception as exc:
            return {
                "result": "error",
                "message": f"음성 변환 실패: {exc}",
                "action": "CHAT",
                "items": [],
                "provider": stt_model,
            }
        provider = f"{stt_model} + {parse_model}"
    else:
        text = message
        provider = parse_model

    # 2. 상품 카탈로그 조회
    products = _fetch_products()

    # 3. LLM으로 주문 파싱
    try:
        parsed = _parse_items(llm_client, text, products, parse_model)
    except Exception as exc:
        return {
            "result": "error",
            "message": f"주문 파싱 실패: {exc}",
            "action": "CHAT",
            "items": [],
            "provider": provider,
        }

    action = str(parsed.get("action", "CHAT")).upper()
    items = parsed.get("items", [])

    if action == "ORDER" and items:
        summary = ", ".join(f"{it['product_name']} {it['quantity']}개" for it in items)
        msg = f"{summary} 주문 요청을 생성합니다."
    else:
        action = "CHAT"
        msg = text

    return {
        "result": "ok",
        "message": msg,
        "action": action,
        "items": items,
        "provider": provider,
    }
