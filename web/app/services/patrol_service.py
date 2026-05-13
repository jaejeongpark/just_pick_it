from sqlalchemy.orm import Session

from app.models import Task, Zone
from app.services.workflow_service import PATROL_PRIORITY


ZONE_NAME_ALIASES = {
    "A": "A_ZONE",
    "A구역": "A_ZONE",
    "A존": "A_ZONE",
    "상차": "LOADING_ZONE",
    "로딩": "LOADING_ZONE",
    "상차대기": "STANDBY_LOADING_ZONE",
    "하차": "UNLOADING_ZONE",
    "하차대기": "STANDBY_UNLOADING_ZONE",
    "충전": "CHARGING_ZONE",
    "홈": "HOME",
    "상품": "PRODUCT_ZONE",
    "물품": "PRODUCT_ZONE",
}


def create_patrol_task_from_llm(
    db: Session,
    message: str,
    llm_response: dict,
) -> None:
    if llm_response.get("action") != "PATROL":
        return

    patrol_zone = resolve_patrol_zone(db, llm_response)

    if not patrol_zone:
        llm_response.update(
            {
                "result": "error",
                "message": "순찰 구역을 찾을 수 없습니다. DB zone_name 또는 zone_id를 확인해주세요.",
                "assigned_robot_id": None,
                "task_id": None,
                "target_zone_id": None,
            }
        )
        return

    task = Task(
        task_type="PATROL",
        status="QUEUED",
        priority=PATROL_PRIORITY,
        assigned_robot_id=None,
        target_zone_id=patrol_zone.zone_id,
        result_message=message,
    )
    db.add(task)
    db.flush()

    llm_response.update(
        {
            "result": "ok",
            "message": f"순찰 task #{task.task_id}를 생성했습니다. 대기 중인 AMR이 있으면 배정됩니다.",
            "assigned_robot_id": None,
            "task_id": task.task_id,
            "target_zone_id": patrol_zone.zone_id,
            "target_zone_name": patrol_zone.zone_name,
        }
    )


def resolve_patrol_zone(db: Session, llm_response: dict) -> Zone | None:
    target_zone_id = llm_response.get("target_zone_id")

    if target_zone_id is not None:
        zone = db.get(Zone, target_zone_id)

        if zone:
            return zone

    return find_zone_by_name(db, llm_response.get("target_zone_name"))


def find_zone_by_name(db: Session, zone_name: str | None) -> Zone | None:
    if not zone_name:
        return None

    alias_key = zone_name.replace("구역", "").replace("존", "").strip()
    aliased_zone_name = (
        ZONE_NAME_ALIASES.get(alias_key)
        or ZONE_NAME_ALIASES.get(zone_name)
    )
    normalized_target = normalize_zone_name(aliased_zone_name or zone_name)

    zones = db.query(Zone).all()

    for zone in zones:
        normalized_zone_name = normalize_zone_name(zone.zone_name)

        if normalized_zone_name == normalized_target:
            return zone

        if normalized_zone_name == f"{normalized_target}_ZONE":
            return zone

    return None


def normalize_zone_name(zone_name: str) -> str:
    return zone_name.replace(" ", "").replace("-", "_").upper()
