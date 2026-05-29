from typing import Any


def build_llm_message(
    message: str,
    context: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Admin AI 명령 파싱 진입점.

    LLM 담당자는 이 함수만 실제 구현으로 교체하면 된다.
    반환값이 action=DISPLAY 이고 product_id/requested_quantity 등이 채워지면
    llm_router 가 Fleet API 에 display_item 생성을 위임한다.
    """

    return {
        "result": "ok",
        "message": "LLM 명령 파싱은 아직 연결 대기 상태입니다. 담당 모듈에서 구현해주세요.",
        "action": "CHAT",
        "product_id": None,
        "product_name": None,
        "requested_quantity": None,
        "display_policy": None,
        "display_item_id": None,
        "provider": "stub",
    }
