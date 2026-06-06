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
WEB_PORT="${WEB_PORT:-8000}"
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
  if port_is_listening "$WEB_PORT"; then
    stop_existing_port_owners "Web Gateway" "$WEB_PORT"
  fi
  if [ -n "$FLEET_PID" ] && kill -0 "$FLEET_PID" >/dev/null 2>&1; then
    log "stopping Fleet Manager"
    kill "$FLEET_PID" >/dev/null 2>&1 || true
    wait "$FLEET_PID" >/dev/null 2>&1 || true
  fi
  if port_is_listening 8100; then
    stop_existing_port_owners "Fleet API" 8100
  fi
  stop_existing_fleet_manager_processes
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

port_is_listening() {
  local port="$1"
  ss -ltn "sport = :${port}" 2>/dev/null | grep -q LISTEN
}

port_owner_pids() {
  local port="$1"
  ss -ltnp "sport = :${port}" 2>/dev/null |
    sed -n 's/.*pid=\([0-9][0-9]*\).*/\1/p' |
    sort -u
}

stop_existing_port_owners() {
  local label="$1"
  local port="$2"
  local pids
  pids="$(port_owner_pids "$port" || true)"
  [ -n "$pids" ] || return 0

  log "port ${port} already in use; stopping existing ${label} owner(s)"
  while read -r pid; do
    [ -n "$pid" ] || continue
    log "stopping pid=$pid cmd=$(ps -p "$pid" -o args= 2>/dev/null || echo unknown)"
    kill "$pid" >/dev/null 2>&1 || true
  done <<< "$pids"

  for _ in $(seq 1 10); do
    port_is_listening "$port" || return 0
    sleep 0.5
  done

  if command -v fuser >/dev/null 2>&1; then
    log "port ${port} still busy; using fuser cleanup"
    fuser -k "${port}/tcp" >/dev/null 2>&1 || true
    sleep 1
  fi

  if port_is_listening "$port"; then
    echo "[run-all] port ${port} is still busy after cleanup." >&2
    exit 1
  fi
}

fleet_manager_process_pids() {
  ps -eo pid=,args= |
    awk -v root="$ROOT_DIR" '
      $0 ~ root "/install/fleet_manager/lib/fleet_manager/fleet_manager_node" ||
      $0 ~ "ros2 launch fleet_manager fleet_manager.launch.xml" {
        print $1
      }
    '
}

stop_existing_fleet_manager_processes() {
  local pids
  pids="$(fleet_manager_process_pids || true)"
  [ -n "$pids" ] || return 0

  log "stopping existing Fleet Manager process(es)"
  while read -r pid; do
    [ -n "$pid" ] || continue
    log "stopping pid=$pid cmd=$(ps -p "$pid" -o args= 2>/dev/null || echo unknown)"
    kill "$pid" >/dev/null 2>&1 || true
  done <<< "$pids"

  sleep 1
  pids="$(fleet_manager_process_pids || true)"
  [ -n "$pids" ] || return 0

  while read -r pid; do
    [ -n "$pid" ] || continue
    log "forcing Fleet Manager pid=$pid"
    kill -9 "$pid" >/dev/null 2>&1 || true
  done <<< "$pids"
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

stop_existing_port_owners "Fleet API" 8100
stop_existing_fleet_manager_processes
log "starting Fleet Manager"
ros2 launch fleet_manager fleet_manager.launch.xml &
FLEET_PID="$!"
STARTED_FLEET=1
wait_for_fleet_api

stop_existing_port_owners "Web Gateway" "$WEB_PORT"
log "starting Web Gateway"
export APP_PORT="$WEB_PORT"
export FLEET_API_BASE_URL
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
