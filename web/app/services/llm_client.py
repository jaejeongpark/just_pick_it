import json
import re
from typing import Any

import anthropic

from app.config import (
    ANTHROPIC_API_KEY,
    CLAUDE_MAX_TOKENS,
    CLAUDE_MODEL,
    CLAUDE_TIMEOUT_SECONDS,
)


# Control Server가 현재 이해하는 관리자 자연어 명령 종류.
# Claude가 다른 값을 보내더라도 여기 없는 action은 CHAT으로 정규화한다.
VALID_ACTIONS = {
    "PATROL",
    "INVENTORY_SUMMARY",
    "EXCEPTION_SUMMARY",
    "CHAT",
}


# Claude에게 "일반 대화"가 아니라 "명령 해석기" 역할을 맡긴다.
# 응답은 UI와 Fleet 연동에서 바로 쓰기 쉬운 구조화 데이터로 제한한다.
SYSTEM_PROMPT = """
You are the command parser for the Just Pick It robot control server.
Parse the administrator's Korean or English command into a control-server action.

Rules:
- Use PATROL only when the administrator asks for patrol/surveillance.
- If a zone number is present, put it in target_zone_id.
- If a zone name such as "B 구역" is present, put it in target_zone_name.
- Do not claim that a robot task was actually created.
- Keep message short and practical in Korean.
""".strip()


# Anthropic 공식 Messages API의 tool use 형식에 맞춘 JSON schema.
# 일반 텍스트 JSON을 파싱하는 방식보다 안정적으로 필드를 받을 수 있다.
COMMAND_TOOL = {
    "name": "parse_admin_command",
    "description": "Parse an administrator natural-language command for the robot control server.",
    "input_schema": {
        "type": "object",
        "properties": {
            "result": {
                "type": "string",
                "enum": ["ok", "error"],
            },
            "message": {
                "type": "string",
                "description": "Short Korean response for the administrator UI.",
            },
            "action": {
                "type": "string",
                "enum": ["PATROL", "INVENTORY_SUMMARY", "EXCEPTION_SUMMARY", "CHAT"],
            },
            "target_zone_id": {
                "type": ["integer", "null"],
                "description": "Parsed patrol zone id, if the command includes one.",
            },
            "target_zone_name": {
                "type": ["string", "null"],
                "description": "Parsed patrol zone name, if the command includes one.",
            },
            "assigned_robot_id": {
                "type": ["string", "null"],
                "description": "Robot id only if the command explicitly names one.",
            },
            "task_id": {
                "type": ["integer", "null"],
                "description": "Existing task id only if the command explicitly names one.",
            },
        },
        "required": [
            "result",
            "message",
            "action",
            "target_zone_id",
            "target_zone_name",
            "assigned_robot_id",
            "task_id",
        ],
        "additionalProperties": False,
    },
}


def build_llm_message(message: str, context: dict[str, Any] | None = None) -> dict[str, Any]:
    """Return the admin LLM response used by /api/admin/llm/messages.

    현재 데모/로컬 개발에서는 Claude API key가 없을 수 있다.
    그래서 key가 없으면 mock으로 동작하고, key가 있으면 Claude를 직접 호출한다.
    """

    context = context or {}

    if ANTHROPIC_API_KEY:
        return request_claude_message(message)

    response = build_mock_message(message, context)
    response["provider"] = "mock"
    return response


def request_claude_message(message: str) -> dict[str, Any]:
    """Call Claude through Anthropic's official Python SDK.

    공식 SDK를 쓰면 Messages API의 URL, x-api-key, anthropic-version 헤더를
    직접 관리하지 않아도 된다. 환경변수는 web/.env의 ANTHROPIC_API_KEY와
    CLAUDE_MODEL만 채우면 된다.
    """

    client = anthropic.Anthropic(
        api_key=ANTHROPIC_API_KEY,
        timeout=CLAUDE_TIMEOUT_SECONDS,
    )

    try:
        response = client.messages.create(
            model=CLAUDE_MODEL,
            max_tokens=CLAUDE_MAX_TOKENS,
            temperature=0,
            system=SYSTEM_PROMPT,
            messages=[
                {
                    "role": "user",
                    "content": message,
                }
            ],
            tools=[COMMAND_TOOL],
            tool_choice={
                "type": "tool",
                "name": "parse_admin_command",
            },
        )
    except Exception as exc:
        # UI가 500으로 깨지지 않게, Claude 연결 문제를 관리자 메시지로 돌려준다.
        # 실제 운영에서는 여기서 exception_log 생성이나 별도 알림으로 연결할 수 있다.
        return {
            "result": "error",
            "message": f"Claude API 호출에 실패했습니다: {exc}",
            "action": "CHAT",
            "provider": "claude",
        }

    return parse_claude_response(response)


def parse_claude_response(response: Any) -> dict[str, Any]:
    """Convert Claude SDK response into AdminLlmMessageRead shape."""

    for content in getattr(response, "content", []):
        content_type = getattr(content, "type", None)

        if content_type == "tool_use":
            return normalize_llm_response(getattr(content, "input", {}), provider="claude")

    # tool_use가 아닌 텍스트 응답이 온 경우를 대비한 fallback.
    # tool_choice를 강제했기 때문에 일반적으로는 거의 타지 않는다.
    text = extract_text_from_claude_response(response)

    try:
        parsed = json.loads(text)
    except json.JSONDecodeError:
        return {
            "result": "ok",
            "message": text or "Claude 응답이 비어 있습니다.",
            "action": "CHAT",
            "provider": "claude",
        }

    return normalize_llm_response(parsed, provider="claude")


def extract_text_from_claude_response(response: Any) -> str:
    text = ""

    for content in getattr(response, "content", []):
        if getattr(content, "type", None) == "text":
            text += getattr(content, "text", "")

    return text.strip()


def build_mock_message(message: str, context: dict[str, Any]) -> dict[str, Any]:
    """Local fallback used when ANTHROPIC_API_KEY is not set."""

    lower_message = message.lower()

    if "순찰" in lower_message or "patrol" in lower_message:
        target_zone_id, target_zone_name = parse_zone(message)
        zone_text = target_zone_name or (f"{target_zone_id}번 구역" if target_zone_id else "지정 구역")
        return {
            "result": "ok",
            "message": f"{zone_text} 순찰 명령으로 해석했습니다. Claude 키 설정 후 실제 명령 해석으로 전환됩니다.",
            "action": "PATROL",
            "task_id": None,
            "assigned_robot_id": None,
            "target_zone_id": target_zone_id,
            "target_zone_name": target_zone_name,
        }

    if "재고" in lower_message or "stock" in lower_message:
        low_stock_count = context.get("low_stock_count", 0)
        return {
            "result": "ok",
            "message": f"현재 재고 확인이 필요한 상품은 {low_stock_count}개입니다.",
            "action": "INVENTORY_SUMMARY",
        }

    if "예외" in lower_message or "exception" in lower_message:
        unresolved_exception_count = context.get("unresolved_exception_count", 0)
        return {
            "result": "ok",
            "message": f"현재 미처리 예외는 {unresolved_exception_count}건입니다.",
            "action": "EXCEPTION_SUMMARY",
        }

    return {
        "result": "ok",
        "message": "Claude API 키가 없어 mock 응답으로 처리했습니다.",
        "action": "CHAT",
    }


def parse_zone(message: str) -> tuple[int | None, str | None]:
    """Parse simple zone expressions such as '1번 구역' or 'B 구역'."""

    numeric_match = re.search(r"(\d+)\s*(번|구역|존|zone)", message, re.IGNORECASE)

    if numeric_match:
        return int(numeric_match.group(1)), None

    name_match = re.search(r"([A-Za-z가-힣]+)\s*(구역|존|zone)", message, re.IGNORECASE)

    if name_match:
        return None, name_match.group(1).upper()

    return None, None


def normalize_llm_response(parsed: dict[str, Any], provider: str) -> dict[str, Any]:
    """Keep Claude/mock output inside the API response contract."""

    action = parsed.get("action")

    if action not in VALID_ACTIONS:
        action = "CHAT"

    return {
        "result": parsed.get("result") or "ok",
        "message": parsed.get("message") or "Claude 응답을 처리했습니다.",
        "action": action,
        "task_id": parsed.get("task_id"),
        "assigned_robot_id": parsed.get("assigned_robot_id"),
        "target_zone_id": parsed.get("target_zone_id"),
        "target_zone_name": parsed.get("target_zone_name"),
        "provider": provider,
    }
