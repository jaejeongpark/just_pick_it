#!/usr/bin/env bash
set -euo pipefail

# Just Pick It 로컬 통합 실행 스크립트
#
# 실행 대상:
# - PostgreSQL 준비 확인
# - Fleet Manager API(:8100) 실행
# - Web Gateway(:8000) 실행
#
# 사전 세팅:
#   ./reset_ws.sh

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB_DIR="$ROOT_DIR/web"
ROS_DISTRO_REQUIRED="jazzy"
# 팀 공통 ROS_DOMAIN_ID. Fleet Manager가 로봇(/picky1 등)과 통신하려면
# 보드와 같은 도메인이어야 한다. 환경변수로 오버라이드 가능.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-25}"
FLEET_API_BASE_URL="${FLEET_API_BASE_URL:-http://localhost:8100}"
FLEET_API_WAIT_TIMEOUT="${FLEET_API_WAIT_TIMEOUT:-30}"
DATABASE_URL="${DATABASE_URL:-postgresql://just_pick_it_user:just_pick_it_pw@localhost:5432/just_pick_it}"
FLEET_PID=""
WEB_PID=""
STARTED_FLEET=0

log() {
  echo "[run-all] $*"
}

source_if_exists() {
  local path="$1"
  [ -f "$path" ] || return 1
  set +u
  # shellcheck disable=SC1090
  source "$path"
  set -u
}

cleanup() {
  local code=$?
  trap - EXIT INT TERM
  if [ -n "$WEB_PID" ] && kill -0 "$WEB_PID" >/dev/null 2>&1; then
    log "stopping Web Gateway"
    kill "$WEB_PID" >/dev/null 2>&1 || true
    wait "$WEB_PID" >/dev/null 2>&1 || true
  fi
  if [ -n "$FLEET_PID" ] && kill -0 "$FLEET_PID" >/dev/null 2>&1; then
    log "stopping Fleet Manager"
    kill "$FLEET_PID" >/dev/null 2>&1 || true
    wait "$FLEET_PID" >/dev/null 2>&1 || true
  fi
  exit "$code"
}

ensure_ready() {
  [ -d "$WEB_DIR/.venv" ] || { echo "[run-all] web/.venv missing. Run ./reset_ws.sh first." >&2; exit 1; }
  source_if_exists "/opt/ros/${ROS_DISTRO_REQUIRED}/setup.bash" || { echo "[run-all] ROS Jazzy setup missing." >&2; exit 1; }
  source_if_exists "$ROOT_DIR/install/setup.bash" || { echo "[run-all] workspace setup missing. Run ./reset_ws.sh first." >&2; exit 1; }
  pg_isready -d "$DATABASE_URL" >/dev/null 2>&1 || { echo "[run-all] PostgreSQL/DB not ready. Run ./reset_ws.sh first." >&2; exit 1; }
}

fleet_api_ready() {
  command -v curl >/dev/null 2>&1 && curl -fsS --max-time 1 "$FLEET_API_BASE_URL/api/health/db" >/dev/null 2>&1
}

wait_for_fleet_api() {
  log "waiting for Fleet API: $FLEET_API_BASE_URL"
  for _ in $(seq 1 "$FLEET_API_WAIT_TIMEOUT"); do
    fleet_api_ready && { log "Fleet API ready"; return; }
    if [ -n "$FLEET_PID" ] && ! kill -0 "$FLEET_PID" >/dev/null 2>&1; then
      echo "[run-all] Fleet Manager exited before API became ready." >&2
      wait "$FLEET_PID" || true
      exit 1
    fi
    sleep 1
  done
  echo "[run-all] Fleet API not ready after ${FLEET_API_WAIT_TIMEOUT}s." >&2
  exit 1
}

cd "$ROOT_DIR"
ensure_ready
trap cleanup EXIT INT TERM

if fleet_api_ready; then
  log "Fleet API already running: $FLEET_API_BASE_URL"
else
  log "starting Fleet Manager"
  ros2 launch fleet_manager fleet_manager.launch.xml &
  FLEET_PID="$!"
  STARTED_FLEET=1
  wait_for_fleet_api
fi

log "starting Web Gateway"
"$WEB_DIR/scripts/run.sh" &
WEB_PID="$!"

while true; do
  if [ -n "$WEB_PID" ] && ! kill -0 "$WEB_PID" >/dev/null 2>&1; then
    wait "$WEB_PID"
    exit $?
  fi

  if [ "$STARTED_FLEET" -eq 1 ] && [ -n "$FLEET_PID" ] && ! kill -0 "$FLEET_PID" >/dev/null 2>&1; then
    echo "[run-all] Fleet Manager exited unexpectedly." >&2
    wait "$FLEET_PID" || true
    exit 1
  fi

  sleep 1
done
