#!/usr/bin/env bash
set -euo pipefail

# Just Pick It 웹/DB 로컬 세팅 스크립트
#
# 사용 시점:
# - 처음 프로젝트를 받은 팀원이 웹 UI를 실행하기 전
# - requirements.txt가 바뀌어 Python 패키지를 다시 설치해야 할 때
# - PostgreSQL DB/user/schema/seed를 한 번에 준비하고 싶을 때
#
# 사용법:
#   cd ~/just_pick_it
#   web/scripts/setup.sh
#
# 데모 DB를 초기화하고 다시 만들고 싶을 때:
#   RESET_DB=1 web/scripts/setup.sh
# Python venv를 지우고 다시 만들고 싶을 때:
#   RESET_VENV=1 web/scripts/setup.sh
#
# 주의:
# - Ubuntu/apt 환경에서는 PostgreSQL이 없으면 자동 설치를 시도한다.
# - apt가 없는 환경이면 PostgreSQL을 직접 설치한 뒤 다시 실행한다.
# - PostgreSQL 설치, DB/user 생성, 서비스 시작에는 sudo 비밀번호가 필요할 수 있다.

# WEB_DIR: web/ 폴더 절대 경로
# ROOT_DIR: just_pick_it 루트 절대 경로
WEB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT_DIR="$(cd "$WEB_DIR/.." && pwd)"

# 프로젝트에서 공통으로 쓰는 로컬 PostgreSQL 접속 URL 기본값
DEFAULT_DB_URL="postgresql://just_pick_it_user:just_pick_it_pw@localhost:5432/just_pick_it"

echo "[web-setup] root: $ROOT_DIR"
echo "[web-setup] web : $WEB_DIR"

PYTHON_BIN="${PYTHON_BIN:-python3.12}"
REQUIRED_PYTHON_VERSION="3.12"

ensure_python312_apt_source() {
  PY312_CANDIDATE="$(apt-cache policy python3.12 2>/dev/null | awk '/Candidate:/ {print $2}')"
  if [ -n "$PY312_CANDIDATE" ] && [ "$PY312_CANDIDATE" != "(none)" ]; then
    return 0
  fi

  echo "[web-setup] python3.12 package not found in current apt sources"
  echo "[web-setup] adding deadsnakes PPA for Python $REQUIRED_PYTHON_VERSION"
  sudo apt-get install -y software-properties-common
  sudo add-apt-repository -y ppa:deadsnakes/ppa
  sudo apt-get update
}

install_python312() {
  if ! command -v apt-get >/dev/null 2>&1; then
    echo "[web-setup] $PYTHON_BIN is required, but apt-get was not found." >&2
    echo "[web-setup] Install Python $REQUIRED_PYTHON_VERSION manually, then run setup again." >&2
    exit 1
  fi

  echo "[web-setup] $PYTHON_BIN not found; installing Python $REQUIRED_PYTHON_VERSION with apt"
  sudo apt-get update

  if sudo apt-get install -y python3.12 python3.12-venv; then
    return 0
  fi

  ensure_python312_apt_source
  sudo apt-get install -y python3.12 python3.12-venv
}

if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
  install_python312
fi

if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
  echo "[web-setup] $PYTHON_BIN install finished, but command is still not available." >&2
  exit 1
fi

PYTHON_VERSION="$($PYTHON_BIN -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
if [ "$PYTHON_VERSION" != "$REQUIRED_PYTHON_VERSION" ]; then
  echo "[web-setup] Python $REQUIRED_PYTHON_VERSION is required, but $PYTHON_BIN is $PYTHON_VERSION" >&2
  echo "[web-setup] Set PYTHON_BIN=/path/to/python3.12 or install python3.12." >&2
  exit 1
fi

if ! "$PYTHON_BIN" -m venv --help >/dev/null 2>&1; then
  if command -v apt-get >/dev/null 2>&1; then
    echo "[web-setup] Python venv module not found; installing python3.12-venv"
    sudo apt-get update
    if ! sudo apt-get install -y python3.12-venv; then
      ensure_python312_apt_source
      sudo apt-get install -y python3.12-venv
    fi
  else
    echo "[web-setup] Python venv module is required. Install python3.12-venv manually." >&2
    exit 1
  fi
fi

# psql은 PostgreSQL에 schema/seed를 적용할 때 필요하다.
# Ubuntu 팀원 환경에서는 setup.sh가 PostgreSQL 설치까지 처리한다.
if ! command -v psql >/dev/null 2>&1; then
  if command -v apt-get >/dev/null 2>&1; then
    echo "[web-setup] PostgreSQL not found; installing with apt"
    sudo apt-get update
    sudo apt-get install -y postgresql postgresql-contrib
  else
    echo "[web-setup] psql is required, but apt-get was not found." >&2
    echo "[web-setup] Install PostgreSQL manually, then run web/scripts/setup.sh again." >&2
    exit 1
  fi
fi

if ! command -v psql >/dev/null 2>&1; then
  echo "[web-setup] PostgreSQL install finished, but psql is still not available." >&2
  exit 1
fi

# Python 3.12 venv를 web/.venv에 만든다.
# 기존 venv가 3.12가 아니면 RESET_VENV=1로 재생성한다.
if [ "${RESET_VENV:-0}" = "1" ] && [ -d "$WEB_DIR/.venv" ]; then
  echo "[web-setup] RESET_VENV=1, removing existing venv"
  rm -rf "$WEB_DIR/.venv"
fi

if [ ! -d "$WEB_DIR/.venv" ]; then
  echo "[web-setup] creating venv with $PYTHON_BIN"
  "$PYTHON_BIN" -m venv "$WEB_DIR/.venv"
else
  VENV_VERSION="$($WEB_DIR/.venv/bin/python -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
  if [ "$VENV_VERSION" != "$REQUIRED_PYTHON_VERSION" ]; then
    echo "[web-setup] existing web/.venv uses Python $VENV_VERSION, but $REQUIRED_PYTHON_VERSION is required." >&2
    echo "[web-setup] Re-run with RESET_VENV=1 web/scripts/setup.sh" >&2
    exit 1
  fi
fi

# FastAPI, SQLAlchemy, uvicorn 등 웹 서버 의존성을 설치한다.
echo "[web-setup] installing python packages"
"$WEB_DIR/.venv/bin/pip" install -r "$WEB_DIR/requirements.txt"

# .env가 없으면 예시 파일을 복사한다.
# DB URL, host, port는 web/.env에서 조정한다.
if [ ! -f "$WEB_DIR/.env" ]; then
  echo "[web-setup] creating web/.env from .env.example"
  cp "$WEB_DIR/.env.example" "$WEB_DIR/.env"
fi

# web/.env가 있으면 DATABASE_URL을 읽는다.
# 기본 로컬 세팅은 just_pick_it_user/just_pick_it DB를 만든 뒤 이 URL로 schema/seed를 적용한다.
if [ -f "$WEB_DIR/.env" ]; then
  set -a
  # shellcheck disable=SC1091
  source "$WEB_DIR/.env"
  set +a
fi

# DATABASE_URL이 비어 있으면 프로젝트 기본 로컬 DB를 사용한다.
DB_URL="${DATABASE_URL:-$DEFAULT_DB_URL}"

# PostgreSQL 서비스가 꺼져 있으면 시작한다.
# systemctl이 없는 환경에서는 이 단계는 건너뛴다.
if command -v systemctl >/dev/null 2>&1; then
  if ! systemctl is-active --quiet postgresql; then
    echo "[web-setup] starting PostgreSQL"
    sudo systemctl start postgresql
  fi
fi

# just_pick_it_user와 just_pick_it DB가 없으면 만든다.
# 이미 있으면 아무것도 하지 않는다.
echo "[web-setup] ensuring database/user"
if ! sudo -u postgres psql -tAc "SELECT 1 FROM pg_roles WHERE rolname='just_pick_it_user'" | grep -q 1; then
  sudo -u postgres psql -c "CREATE USER just_pick_it_user WITH PASSWORD 'just_pick_it_pw';"
fi

if ! sudo -u postgres psql -tAc "SELECT 1 FROM pg_database WHERE datname='just_pick_it'" | grep -q 1; then
  sudo -u postgres createdb just_pick_it
fi

# PostgreSQL 15+에서는 public schema 권한 문제로 CREATE TABLE이 막힐 수 있다.
# just_pick_it_user가 schema.sql을 적용할 수 있게 public schema 권한을 맞춘다.
sudo -u postgres psql -d just_pick_it -c "ALTER SCHEMA public OWNER TO just_pick_it_user;"
sudo -u postgres psql -d just_pick_it -c "GRANT ALL ON SCHEMA public TO just_pick_it_user;"
sudo -u postgres psql -d just_pick_it -c "GRANT ALL PRIVILEGES ON DATABASE just_pick_it TO just_pick_it_user;"

# RESET_DB=1이면 public schema를 통째로 다시 만든다.
# 테스트 주문/task/exception 데이터를 모두 초기화하고 싶을 때만 사용한다.
if [ "${RESET_DB:-0}" = "1" ]; then
  echo "[web-setup] RESET_DB=1, resetting public schema"
  sudo -u postgres psql -d just_pick_it -c "DROP SCHEMA public CASCADE; CREATE SCHEMA public AUTHORIZATION just_pick_it_user;"
  sudo -u postgres psql -d just_pick_it -c "GRANT ALL ON SCHEMA public TO just_pick_it_user;"
fi

# orders 테이블이 이미 있으면 DB가 준비되어 있다고 보고 schema/seed 적용을 건너뛴다.
# 처음 세팅이거나 RESET_DB=1 이후라면 schema.sql과 seed.sql을 적용한다.
if psql "$DB_URL" -tAc "SELECT to_regclass('public.orders')" | grep -q orders; then
  echo "[web-setup] schema already exists; skipping schema/seed"
  echo "[web-setup] use RESET_DB=1 web/scripts/setup.sh to recreate demo DB"
else
  echo "[web-setup] applying schema and seed"
  psql "$DB_URL" -f "$ROOT_DIR/db/schema.sql"
  psql "$DB_URL" -f "$ROOT_DIR/db/seed.sql"
fi

echo "[web-setup] done"
echo "[web-setup] run: web/scripts/run.sh"
