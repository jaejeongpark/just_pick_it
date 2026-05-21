from typing import Any


def build_llm_message(
    _message: str,
    _context: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """LLM integration handoff point.

    The Admin UI and `/api/admin/llm/messages` endpoint are intentionally kept
    wired. The LLM owner should replace this stub with:
    1. natural-language parsing,
    2. product/quantity matching,
    3. Fleet Manager stocking command or task creation.
    """

    return {
        "result": "ok",
        "message": "LLM 명령 파싱은 아직 연결 대기 상태입니다. 담당 모듈에서 구현해주세요.",
        "action": "CHAT",
        "task_id": None,
        "assigned_robot_id": None,
        "assigned_robot_name": None,
        "target_zone_id": None,
        "target_zone_name": None,
        "product_id": None,
        "product_name": None,
        "requested_quantity": None,
        "stocking_policy": None,
        "provider": "stub",
    }
