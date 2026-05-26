"""DB 세션 관리.

이 모듈은 just_pick_it 시스템에서 PostgreSQL에 접근하는 단일 진입점이다.

설계 의도:
- FastAPI 요청 수명주기에 묶이지 않는다. (`get_db` 제너레이터는 두지 않는다)
- ROS2 ``MultiThreadedExecutor`` 처럼 여러 스레드가 동시에 콜백을 실행하는 환경에서도
  안전하게 쓰도록 ``scoped_session`` 을 사용한다. ``scoped_session`` 은 스레드마다 다른
  Session 을 돌려주므로 Session 공유로 인한 데이터 손상을 막는다.
- engine 은 lazy 하게(처음 필요할 때) 생성한다. 그래야 import 시점이 아니라
  실제 사용 시점의 ``DATABASE_URL`` 환경변수를 읽는다.

웹은 이 모듈의 ``get_session_factory`` 를 재사용하고, 자신의 요청 수명주기용
``get_db`` 만 얇게 따로 둔다.
"""

from __future__ import annotations

import os
import threading
from contextlib import contextmanager
from typing import Iterator

from sqlalchemy import create_engine, text
from sqlalchemy.engine import Engine
from sqlalchemy.orm import Session, scoped_session, sessionmaker

DEFAULT_DATABASE_URL = "postgresql://just_pick_it_user:just_pick_it_pw@localhost:5432/just_pick_it"

_engine: Engine | None = None
_session_factory: sessionmaker | None = None
_scoped_session: scoped_session | None = None

# executor 스레드와 uvicorn 스레드가 동시에 첫 사용 시 engine/factory 를 중복 생성하지
# 않도록 lazy 초기화를 보호한다(double-checked locking).
# get_session_factory 가 락을 잡은 채 get_engine 을 호출하는 등 같은 스레드 재진입이
# 있으므로 재진입 가능한 RLock 을 쓴다(일반 Lock 이면 자기 자신과 데드락).
_init_lock = threading.RLock()


def database_url() -> str:
    """사용할 DATABASE_URL 을 환경변수에서 읽는다."""
    return os.getenv("DATABASE_URL", DEFAULT_DATABASE_URL)


def get_engine() -> Engine:
    """SQLAlchemy engine 을 lazy 하게 생성해 반환한다.

    pool_size 는 ROS2 executor 의 동시 스레드 수에 맞춘다. 환경변수 ``DB_POOL_SIZE`` 로
    조정할 수 있고, 기본값은 MultiThreadedExecutor 의 일반적인 워커 수를 고려한 값이다.
    """
    global _engine
    if _engine is None:
        with _init_lock:
            if _engine is None:
                pool_size = int(os.getenv("DB_POOL_SIZE", "10"))
                max_overflow = int(os.getenv("DB_MAX_OVERFLOW", "5"))
                _engine = create_engine(
                    database_url(),
                    pool_size=pool_size,
                    max_overflow=max_overflow,
                    pool_pre_ping=True,
                )
    return _engine


def get_session_factory() -> sessionmaker:
    """sessionmaker 를 lazy 하게 생성해 반환한다.

    웹의 기존 ``SessionLocal`` 이 이 factory 를 가리키도록 재사용한다.
    """
    global _session_factory
    if _session_factory is None:
        with _init_lock:
            if _session_factory is None:
                _session_factory = sessionmaker(
                    autocommit=False,
                    autoflush=False,
                    bind=get_engine(),
                )
    return _session_factory


def get_scoped_session() -> scoped_session:
    """스레드 안전한 scoped_session 을 반환한다.

    ROS2 콜백/타이머에서 ``Session = get_scoped_session()`` 로 받아 쓰고,
    작업 단위가 끝나면 ``Session.remove()`` 로 스레드 로컬 Session 을 정리한다.
    직접 다루기보다 ``session_scope()`` 컨텍스트 매니저 사용을 권장한다.
    """
    global _scoped_session
    if _scoped_session is None:
        with _init_lock:
            if _scoped_session is None:
                _scoped_session = scoped_session(get_session_factory())
    return _scoped_session


@contextmanager
def session_scope() -> Iterator[Session]:
    """트랜잭션 경계를 갖는 Session 컨텍스트 매니저.

    사용 예::

        with session_scope() as db:
            db.add(obj)
        # 블록을 정상 종료하면 commit, 예외가 나면 rollback 후 재던짐.

    ROS2 콜백 안에서 DB 작업을 이 블록으로 감싸면, 콜백마다 스레드 로컬 Session 이
    열리고 닫혀 ``MultiThreadedExecutor`` 환경에서 안전하다.
    """
    scoped = get_scoped_session()
    db = scoped()
    try:
        yield db
        db.commit()
    except Exception:
        db.rollback()
        raise
    finally:
        scoped.remove()


def check_database_connection() -> None:
    """DB 연결이 살아 있는지 확인한다. 실패 시 예외를 던진다."""
    with get_engine().connect() as connection:
        connection.execute(text("SELECT 1"))


def dispose_engine() -> None:
    """engine 과 커넥션 풀을 정리한다. 노드 종료 시 호출한다."""
    global _engine, _session_factory, _scoped_session
    if _scoped_session is not None:
        _scoped_session.remove()
    if _engine is not None:
        _engine.dispose()
    _engine = None
    _session_factory = None
    _scoped_session = None
