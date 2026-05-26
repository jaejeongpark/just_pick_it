"""status_service re-export shim. 실제 구현은 just_pick_it_db.services.status_service (Phase 1 이전). Phase 4 제거 예정."""

from just_pick_it_db.services.status_service import *  # noqa: F401,F403
