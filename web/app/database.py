"""웹용 DB 세션 진입점.

engine 과 SessionLocal 은 공용 패키지 ``just_pick_it_db.session`` 을 재사용한다(Phase 1).
``get_db`` 는 FastAPI 요청 수명주기 전용이라 웹에만 둔다.
"""

import os

from app.config import DATABASE_URL

# 공용 session 모듈이 동일한 DATABASE_URL 로 engine 을 만들도록 환경변수로 전달한다.
# app.config 가 .env 와 기본값을 이미 반영한 값을 그대로 넘긴다.
os.environ["DATABASE_URL"] = DATABASE_URL

from just_pick_it_db.session import (  # noqa: E402
    check_database_connection,
    get_engine,
    get_session_factory,
)

engine = get_engine()
SessionLocal = get_session_factory()

__all__ = ["engine", "SessionLocal", "get_db", "check_database_connection"]


def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
