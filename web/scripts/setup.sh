#!/usr/bin/env bash
set -euo pipefail

# Just Pick It Web Gateway 환경 세팅 스크립트
#
# 담당 범위:
# - Python 3.12 확인
# - web/.venv 생성
# - web/requirements.txt 설치
# - web/.env 생성
#
# DB, rosdep, colcon build는 루트의 reset_ws.sh가 담당한다.

WEB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROOT_DIR="$(cd "$WEB_DIR/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3.12}"
REQUIRED_PYTHON_VERSION="3.12"

log() {
  echo "[web-setup] $*"
}

fail() {
  echo "[web-setup] $*" >&2
  exit 1
}

log "root: $ROOT_DIR"
log "web : $WEB_DIR"

if ! command -v "$PYTHON_BIN" >/dev/null 2>&1; then
  if command -v apt-get >/dev/null 2>&1; then
    log "$PYTHON_BIN not found; installing python3.12/python3.12-venv"
    sudo apt-get update
    sudo apt-get install -y python3.12 python3.12-venv
  else
    fail "$PYTHON_BIN is required. Install Python 3.12 manually."
  fi
fi

PYTHON_VERSION="$($PYTHON_BIN -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
[ "$PYTHON_VERSION" = "$REQUIRED_PYTHON_VERSION" ] || fail "Python $REQUIRED_PYTHON_VERSION is required, but $PYTHON_BIN is $PYTHON_VERSION"

if ! "$PYTHON_BIN" -m venv --help >/dev/null 2>&1; then
  if command -v apt-get >/dev/null 2>&1; then
    log "Python venv module not found; installing python3.12-venv"
    sudo apt-get update
    sudo apt-get install -y python3.12-venv
  else
    fail "python3.12-venv is required."
  fi
fi

if [ "${RESET_VENV:-0}" = "1" ] && [ -d "$WEB_DIR/.venv" ]; then
  log "RESET_VENV=1, removing existing web/.venv"
  rm -rf "$WEB_DIR/.venv"
fi

if [ ! -d "$WEB_DIR/.venv" ]; then
  log "creating venv with $PYTHON_BIN"
  "$PYTHON_BIN" -m venv "$WEB_DIR/.venv"
else
  VENV_VERSION="$($WEB_DIR/.venv/bin/python -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
  [ "$VENV_VERSION" = "$REQUIRED_PYTHON_VERSION" ] || fail "web/.venv is Python $VENV_VERSION. Re-run with RESET_VENV=1."
fi

log "installing web python packages"
# ROS workspace 를 source 한 터미널에서 실행해도 Web venv 설치가
# colcon install/* 패키지 메타데이터를 보지 않도록 PYTHONPATH 를 끊는다.
env -u PYTHONPATH "$WEB_DIR/.venv/bin/python" -m pip install -r "$WEB_DIR/requirements.txt"

if [ ! -f "$WEB_DIR/.env" ]; then
  log "creating web/.env from .env.example"
  cp "$WEB_DIR/.env.example" "$WEB_DIR/.env"
fi

if ! grep -q '^FLEET_API_BASE_URL=' "$WEB_DIR/.env"; then
  echo "FLEET_API_BASE_URL=http://localhost:8100" >> "$WEB_DIR/.env"
fi

if ! grep -q '^FLEET_API_WS_BASE_URL=' "$WEB_DIR/.env"; then
  echo "FLEET_API_WS_BASE_URL=ws://localhost:8100" >> "$WEB_DIR/.env"
fi


log "done"
log "run Web Gateway only: web/scripts/run.sh"
log "run full stack from repo root: bash scripts/runtime/run_all.sh"
