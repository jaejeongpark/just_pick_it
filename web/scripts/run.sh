#!/usr/bin/env bash
set -euo pipefail

# Just Pick It 웹 서버 실행 스크립트
#
# 사용 시점:
# - web/scripts/setup.sh로 세팅을 끝낸 뒤 로컬 DB와 FastAPI 서버를 켤 때
#
# 사용법:
#   cd ~/just_pick_it
#   web/scripts/run.sh
#
# 실행 후 접속:
# - Customer UI: http://localhost:8000/customer
# - Admin UI   : http://localhost:8000/admin
#
# 중지:
# - 실행 중인 터미널에서 Ctrl+C

# web/ 폴더 절대 경로
WEB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# 프로젝트에서 공통으로 쓰는 로컬 PostgreSQL 접속 URL 기본값
DEFAULT_DB_URL="postgresql://just_pick_it_user:just_pick_it_pw@localhost:5432/just_pick_it"

# venv가 없으면 패키지 설치가 안 된 상태이므로 setup.sh를 먼저 실행해야 한다.
if [ ! -d "$WEB_DIR/.venv" ]; then
  echo "[web-run] missing .venv. Run web/scripts/setup.sh first." >&2
  exit 1
fi

# web/.env가 있으면 APP_HOST, APP_PORT, DATABASE_URL 등을 환경변수로 읽는다.
if [ -f "$WEB_DIR/.env" ]; then
  set -a
  # shellcheck disable=SC1091
  source "$WEB_DIR/.env"
  set +a
fi

# .env에 값이 없으면 기본값으로 실행한다.
APP_HOST="${APP_HOST:-0.0.0.0}"
APP_PORT="${APP_PORT:-8000}"
DB_URL="${DATABASE_URL:-$DEFAULT_DB_URL}"

start_postgresql() {
  if command -v systemctl >/dev/null 2>&1; then
    echo "[web-run] starting PostgreSQL with systemctl"
    sudo systemctl start postgresql
    return
  fi

  if command -v service >/dev/null 2>&1; then
    echo "[web-run] starting PostgreSQL with service"
    sudo service postgresql start
    return
  fi

  echo "[web-run] cannot start PostgreSQL automatically. Start it manually, then rerun." >&2
  exit 1
}

if ! command -v pg_isready >/dev/null 2>&1; then
  echo "[web-run] pg_isready not found. Run web/scripts/setup.sh first." >&2
  exit 1
fi

if ! pg_isready -d "$DB_URL" >/dev/null 2>&1; then
  start_postgresql
fi

if ! pg_isready -d "$DB_URL" >/dev/null 2>&1; then
  echo "[web-run] PostgreSQL is not ready: $DB_URL" >&2
  echo "[web-run] Run web/scripts/setup.sh first, or check PostgreSQL service status." >&2
  exit 1
fi

if command -v psql >/dev/null 2>&1; then
  if ! psql "$DB_URL" -tAc "SELECT to_regclass('public.orders')" | grep -q orders; then
    echo "[web-run] database schema is missing. Run web/scripts/setup.sh first." >&2
    exit 1
  fi
fi

# 로컬 DB가 준비된 뒤 venv를 활성화하고 uvicorn을 실행한다.
# exec를 쓰기 때문에 이 터미널 프로세스는 uvicorn 서버로 바뀐다.
cd "$WEB_DIR"
source "$WEB_DIR/.venv/bin/activate"
echo "[web-run] Customer: http://localhost:${APP_PORT}/customer"
echo "[web-run] Admin   : http://localhost:${APP_PORT}/admin"
echo "[web-run] DB      : $DB_URL"
exec uvicorn app.main:app --reload --host "$APP_HOST" --port "$APP_PORT"
