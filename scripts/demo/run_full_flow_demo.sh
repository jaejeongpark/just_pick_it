#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DEMO_ENV_FILE="$ROOT_DIR/scripts/demo/full_flow_demo.env"
FLEET_API_BASE_URL="http://localhost:8100"
FLEET_API_WS_BASE_URL="ws://localhost:8100"
FLEET_API_WAIT_TIMEOUT=30
ROS_DISTRO_REQUIRED="jazzy"
DATABASE_URL="postgresql://just_pick_it_user:just_pick_it_pw@localhost:5432/just_pick_it"
WEB_PORT=8000

if [ ! -f "$DEMO_ENV_FILE" ]; then
  echo "[full-flow-demo] missing demo config: scripts/demo/full_flow_demo.env" >&2
  echo "[full-flow-demo] create it once:" >&2
  echo "[full-flow-demo]   cp scripts/demo/full_flow_demo.env.example scripts/demo/full_flow_demo.env" >&2
  echo "[full-flow-demo] then set DEMO_ROS_DOMAIN_ID in scripts/demo/full_flow_demo.env" >&2
  exit 1
fi

set -a
# shellcheck disable=SC1090
source "$DEMO_ENV_FILE"
set +a

FLEET_PID=""
FAKE_ROBOT_PID=""
WEB_PID=""
DEMO_PARAMS_FILE=""

log() {
  echo "[full-flow-demo] $*"
}

require_config_value() {
  local name="$1"
  if [ -z "${!name:-}" ]; then
    echo "[full-flow-demo] missing required config: $name" >&2
    echo "[full-flow-demo] edit scripts/demo/full_flow_demo.env" >&2
    exit 1
  fi
}

require_config_defined() {
  local name="$1"
  if [ -z "${!name+x}" ]; then
    echo "[full-flow-demo] missing required config: $name" >&2
    echo "[full-flow-demo] edit scripts/demo/full_flow_demo.env" >&2
    exit 1
  fi
}

validate_demo_config() {
  require_config_value DEMO_ROS_DOMAIN_ID
  require_config_defined DEMO_MOCK_PICKY_IDS
  require_config_value DEMO_PICKY_LINEAR_SPEED_MPS
  require_config_value DEMO_PICKY_POSE_HZ
  require_config_value DEMO_DOCK_LINEAR_SPEED_MPS
  require_config_value DEMO_PICKY_BATTERY_STANDBY_THRESHOLD
  require_config_value DEMO_PICKY_BATTERY_DRAIN_PER_FLOW
  require_config_value DEMO_PICKY_CHARGE_COMPLETE_SECONDS
  require_config_value DEMO_STATE_PUBLISH_INTERVAL_SECONDS
  require_config_defined DEMO_MOCK_COBOT_IDS
  require_config_value DEMO_COBOT_SORTING_SECONDS
  require_config_value DEMO_COBOT_LOADING_SECONDS
  require_config_value DEMO_COBOT_INSPECTING_SECONDS
  require_config_value DEMO_COBOT_UNLOADING_SECONDS
  require_config_value DEMO_COBOT_SCANNING_SECONDS
  require_config_value DEMO_COBOT_PLACING_SECONDS
  require_config_value DEMO_COBOT_STOWING_ARM_SECONDS
  require_config_value DEMO_COBOT_AUTO_COMPLETE
}

source_if_exists() {
  local path="$1"
  [ -f "$path" ] || return 1
  set +u
  # shellcheck disable=SC1090
  source "$path"
  set -u
}

fleet_api_ready() {
  command -v curl >/dev/null 2>&1 &&
    curl -fsS --max-time 1 "$FLEET_API_BASE_URL/api/health/db" >/dev/null 2>&1
}

fleet_api_is_listening() {
  ss -ltn 'sport = :8100' 2>/dev/null | grep -q LISTEN
}

fleet_api_owner_pid() {
  ss -ltnp 'sport = :8100' 2>/dev/null |
    sed -n 's/.*pid=\([0-9][0-9]*\).*/\1/p' |
    head -n 1
}

stop_existing_fleet_api() {
  local pid
  pid="$(fleet_api_owner_pid || true)"
  [ -n "$pid" ] || return 0

  log "stopping existing Fleet API pid=$pid cmd=$(ps -p "$pid" -o args= 2>/dev/null || echo unknown)"
  kill "$pid" >/dev/null 2>&1 || true
  for _ in $(seq 1 10); do
    fleet_api_is_listening || return 0
    sleep 0.5
  done

  if kill -0 "$pid" >/dev/null 2>&1; then
    log "existing Fleet API still running; forcing pid=$pid"
    kill -9 "$pid" >/dev/null 2>&1 || true
    sleep 1
  fi

  if fleet_api_is_listening; then
    echo "[full-flow-demo] Fleet API is still running after cleanup." >&2
    exit 1
  fi
}

fleet_manager_process_pids() {
  ps -eo pid=,comm=,args= |
    awk -v root="$ROOT_DIR" '
      $2 == "awk" || $2 == "bash" || $2 == "sh" || $2 == "timeout" { next }
      index($0, root "/install/fleet_manager/lib/fleet_manager/fleet_manager_node") ||
      index($0, "/opt/ros/jazzy/bin/ros2 launch fleet_manager fleet_manager.launch.xml") {
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

fake_robot_process_pids() {
  ps -eo pid=,comm=,args= |
    awk -v root="$ROOT_DIR" '
      $2 == "awk" || $2 == "bash" || $2 == "sh" || $2 == "timeout" { next }
      index($0, root "/scripts/demo/fake_robot_servers.py") {
        print $1
      }
    '
}

stop_existing_fake_robot_servers() {
  local pids
  pids="$(fake_robot_process_pids || true)"
  [ -n "$pids" ] || return 0

  log "stopping existing fake robot server process(es)"
  while read -r pid; do
    [ -n "$pid" ] || continue
    log "stopping pid=$pid cmd=$(ps -p "$pid" -o args= 2>/dev/null || echo unknown)"
    kill "$pid" >/dev/null 2>&1 || true
  done <<< "$pids"

  sleep 1
  pids="$(fake_robot_process_pids || true)"
  [ -n "$pids" ] || return 0

  while read -r pid; do
    [ -n "$pid" ] || continue
    log "forcing fake robot server pid=$pid"
    kill -9 "$pid" >/dev/null 2>&1 || true
  done <<< "$pids"
}

cleanup() {
  local code=$?
  trap - EXIT INT TERM

  if [ -n "$WEB_PID" ] && kill -0 "$WEB_PID" >/dev/null 2>&1; then
    log "stopping Web Gateway"
    kill "$WEB_PID" >/dev/null 2>&1 || true
    wait "$WEB_PID" >/dev/null 2>&1 || true
  fi
  stop_existing_web_gateway

  if [ -n "$FAKE_ROBOT_PID" ] && kill -0 "$FAKE_ROBOT_PID" >/dev/null 2>&1; then
    log "stopping fake robot servers"
    kill "$FAKE_ROBOT_PID" >/dev/null 2>&1 || true
    wait "$FAKE_ROBOT_PID" >/dev/null 2>&1 || true
  fi
  stop_existing_fake_robot_servers

  if [ -n "$FLEET_PID" ] && kill -0 "$FLEET_PID" >/dev/null 2>&1; then
    log "stopping demo Fleet Manager"
    kill "$FLEET_PID" >/dev/null 2>&1 || true
    wait "$FLEET_PID" >/dev/null 2>&1 || true
  fi
  if fleet_api_ready; then
    stop_existing_fleet_api
  fi
  stop_existing_fleet_manager_processes

  if [ -n "$DEMO_PARAMS_FILE" ] && [ -f "$DEMO_PARAMS_FILE" ]; then
    rm -f "$DEMO_PARAMS_FILE"
  fi

  exit "$code"
}

ensure_ready() {
  command -v curl >/dev/null 2>&1 || {
    echo "[full-flow-demo] curl is required." >&2
    exit 1
  }
  source_if_exists "/opt/ros/${ROS_DISTRO_REQUIRED}/setup.bash" || {
    echo "[full-flow-demo] ROS ${ROS_DISTRO_REQUIRED} setup missing." >&2
    exit 1
  }
  source_if_exists "$ROOT_DIR/install/setup.bash" || {
    echo "[full-flow-demo] workspace setup missing. Build/source the workspace first." >&2
    exit 1
  }
  if command -v pg_isready >/dev/null 2>&1; then
    pg_isready -d "$DATABASE_URL" >/dev/null 2>&1 || {
      echo "[full-flow-demo] PostgreSQL/DB is not ready." >&2
      exit 1
    }
  fi
}

write_demo_params() {
  DEMO_PARAMS_FILE="$(mktemp /tmp/just_pick_it_demo_fleet_manager.XXXXXX.yaml)"
  cat > "$DEMO_PARAMS_FILE" <<YAML
fleet_manager:
  ros__parameters:
    robot_ids:
      - PICKY1
      - COBOT1
      - PICKY2
      - COBOT2
    api_enabled: true
    api_host: "0.0.0.0"
    api_port: 8100
    api_push_interval_sec: 1.0
    waiting_work_poll_period_sec: 1.0
    robot_state_flush_period_sec: 1.0
    reconcile_delay_sec: 2.0
YAML
}

wait_for_fleet_api() {
  log "waiting for Fleet API: $FLEET_API_BASE_URL"
  for _ in $(seq 1 "$FLEET_API_WAIT_TIMEOUT"); do
    fleet_api_ready && {
      log "Fleet API ready"
      return
    }
    if [ -n "$FLEET_PID" ] && ! kill -0 "$FLEET_PID" >/dev/null 2>&1; then
      echo "[full-flow-demo] Fleet Manager exited before API became ready." >&2
      wait "$FLEET_PID" || true
      exit 1
    fi
    sleep 1
  done

  echo "[full-flow-demo] Fleet API not ready after ${FLEET_API_WAIT_TIMEOUT}s." >&2
  exit 1
}

start_fleet_manager() {
  if fleet_api_ready; then
    stop_existing_fleet_api
  fi
  stop_existing_fleet_manager_processes

  write_demo_params
  export ROS_DOMAIN_ID="$DEMO_ROS_DOMAIN_ID"
  log "starting Fleet Manager only for demo (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
  ros2 launch fleet_manager fleet_manager.launch.xml params_file:="$DEMO_PARAMS_FILE" &
  FLEET_PID="$!"
  wait_for_fleet_api
}

web_owner_pids() {
  ss -ltnp "sport = :${WEB_PORT}" 2>/dev/null |
    sed -n 's/.*pid=\([0-9][0-9]*\).*/\1/p' |
    sort -u
}

web_is_listening() {
  ss -ltn "sport = :${WEB_PORT}" 2>/dev/null | grep -q LISTEN
}

stop_existing_web_gateway() {
  local pids
  pids="$(web_owner_pids || true)"
  [ -n "$pids" ] || return 0

  log "port ${WEB_PORT} already in use; stopping existing Web Gateway owner(s)"
  while read -r pid; do
    [ -n "$pid" ] || continue
    log "stopping pid=$pid cmd=$(ps -p "$pid" -o args= 2>/dev/null || echo unknown)"
    kill "$pid" >/dev/null 2>&1 || true
  done <<< "$pids"

  for _ in $(seq 1 10); do
    web_is_listening || return
    sleep 0.5
  done

  if command -v fuser >/dev/null 2>&1; then
    log "port ${WEB_PORT} still busy; using fuser cleanup"
    fuser -k "${WEB_PORT}/tcp" >/dev/null 2>&1 || true
    sleep 1
  fi

  if web_is_listening; then
    echo "[full-flow-demo] Web port ${WEB_PORT} is still busy." >&2
    exit 1
  fi
}

start_fake_robot_servers() {
  stop_existing_fake_robot_servers
  export ROS_DOMAIN_ID="$DEMO_ROS_DOMAIN_ID"
  export DEMO_MOCK_PICKY_IDS
  export DEMO_PICKY_LINEAR_SPEED_MPS
  export DEMO_PICKY_POSE_HZ
  export DEMO_DOCK_LINEAR_SPEED_MPS
  export DEMO_PICKY_BATTERY_STANDBY_THRESHOLD
  export DEMO_PICKY_BATTERY_DRAIN_PER_FLOW
  export DEMO_PICKY_CHARGE_COMPLETE_SECONDS
  export DEMO_STATE_PUBLISH_INTERVAL_SECONDS
  export DEMO_MOCK_COBOT_IDS
  export DEMO_COBOT_SORTING_SECONDS
  export DEMO_COBOT_LOADING_SECONDS
  export DEMO_COBOT_INSPECTING_SECONDS
  export DEMO_COBOT_UNLOADING_SECONDS
  export DEMO_COBOT_SCANNING_SECONDS
  export DEMO_COBOT_PLACING_SECONDS
  export DEMO_COBOT_STOWING_ARM_SECONDS
  export DEMO_COBOT_AUTO_COMPLETE
  log "starting fake robot servers (ROS_DOMAIN_ID=$ROS_DOMAIN_ID)"
  python3 "$ROOT_DIR/scripts/demo/fake_robot_servers.py" &
  FAKE_ROBOT_PID="$!"
  sleep 2
  if ! kill -0 "$FAKE_ROBOT_PID" >/dev/null 2>&1; then
    echo "[full-flow-demo] fake robot servers exited during startup." >&2
    wait "$FAKE_ROBOT_PID" || true
    exit 1
  fi
}

start_web_gateway() {
  stop_existing_web_gateway
  export APP_PORT="$WEB_PORT"
  export FLEET_API_BASE_URL
  export FLEET_API_WS_BASE_URL
  log "starting Web Gateway on port ${WEB_PORT}"
  "$ROOT_DIR/web/scripts/run.sh" &
  WEB_PID="$!"
}

monitor_demo_stack() {
  log "demo stack is running"
  log "Customer UI: http://localhost:${WEB_PORT}/customer"
  log "Admin UI   : http://localhost:${WEB_PORT}/admin"
  log "Fleet API  : ${FLEET_API_BASE_URL}"
  log "Press Ctrl-C to stop the demo stack."

  while true; do
    if [ -n "$WEB_PID" ] && ! kill -0 "$WEB_PID" >/dev/null 2>&1; then
      echo "[full-flow-demo] Web Gateway exited unexpectedly." >&2
      wait "$WEB_PID" || true
      exit 1
    fi
    if [ -n "$FAKE_ROBOT_PID" ] && ! kill -0 "$FAKE_ROBOT_PID" >/dev/null 2>&1; then
      echo "[full-flow-demo] fake robot servers exited unexpectedly." >&2
      wait "$FAKE_ROBOT_PID" || true
      exit 1
    fi
    if [ -n "$FLEET_PID" ] && ! kill -0 "$FLEET_PID" >/dev/null 2>&1; then
      echo "[full-flow-demo] Fleet Manager exited unexpectedly." >&2
      wait "$FLEET_PID" || true
      exit 1
    fi
    sleep 1
  done
}

cd "$ROOT_DIR"
validate_demo_config
ensure_ready
trap cleanup EXIT INT TERM

start_fleet_manager
start_fake_robot_servers
start_web_gateway
monitor_demo_stack
