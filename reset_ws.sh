#!/usr/bin/env bash
set -euo pipefail

# Just Pick It 워크스페이스 재세팅 스크립트
#
# 고정 기준:
# - Ubuntu 24.04
# - ROS 2 Jazzy
# - Python 3.12
# - colcon build는 전체 워크스페이스를 symlink로 빌드
# - build/install/log 전체 삭제 후 --symlink-install 사용
#
# 사용법:
#   cd ~/just_pick_it
#   ./reset_ws.sh
#
# 옵션:
#   RESET_DB=0 ./reset_ws.sh       # DB schema/seed 유지
#   RESET_VENV=0 ./reset_ws.sh    # web/.venv 유지
#   SKIP_WEB=1 ./reset_ws.sh      # web venv 세팅 생략
#   SKIP_DB=1 ./reset_ws.sh       # DB 세팅 생략
#   SKIP_ROSDEP=1 ./reset_ws.sh   # rosdep 생략

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_DISTRO_REQUIRED="jazzy"
PYTHON_REQUIRED="3.12"
DB_NAME="${DB_NAME:-just_pick_it}"
DB_USER="${DB_USER:-just_pick_it_user}"
DB_PASSWORD="${DB_PASSWORD:-just_pick_it_pw}"
DB_HOST="${DB_HOST:-localhost}"
DB_PORT="${DB_PORT:-5432}"
DATABASE_URL="${DATABASE_URL:-postgresql://${DB_USER}:${DB_PASSWORD}@${DB_HOST}:${DB_PORT}/${DB_NAME}}"
RESET_DB="${RESET_DB:-1}"
RESET_VENV="${RESET_VENV:-1}"
SKIP_WEB="${SKIP_WEB:-0}"
SKIP_DB="${SKIP_DB:-0}"
SKIP_ROSDEP="${SKIP_ROSDEP:-0}"


log() {
  echo "[ws-reset] $*"
}

fail() {
  echo "[ws-reset] $*" >&2
  exit 1
}

source_if_exists() {
  local path="$1"
  [ -f "$path" ] || return 1
  set +u
  # shellcheck disable=SC1090
  source "$path"
  set -u
}

remove_path_entries_under() {
  local var_name="$1"
  local prefix="$2"
  local value="${!var_name:-}"
  local result=""
  local entry=""
  local old_ifs="$IFS"

  [ -n "$value" ] || return 0

  IFS=':'
  for entry in $value; do
    [ -n "$entry" ] || continue
    case "$entry" in
      "$prefix"|"$prefix"/*)
        continue
        ;;
    esac
    if [ -n "$result" ]; then
      result="$result:$entry"
    else
      result="$entry"
    fi
  done
  IFS="$old_ifs"

  if [ -n "$result" ]; then
    export "$var_name=$result"
  else
    unset "$var_name"
  fi
}

sanitize_workspace_overlay_env() {
  log "removing stale workspace overlay from environment"
  remove_path_entries_under AMENT_PREFIX_PATH "$ROOT_DIR/install"
  remove_path_entries_under CMAKE_PREFIX_PATH "$ROOT_DIR/install"
  remove_path_entries_under COLCON_PREFIX_PATH "$ROOT_DIR/install"
  remove_path_entries_under PYTHONPATH "$ROOT_DIR/install"
  remove_path_entries_under LD_LIBRARY_PATH "$ROOT_DIR/install"
  remove_path_entries_under PKG_CONFIG_PATH "$ROOT_DIR/install"
  remove_path_entries_under PATH "$ROOT_DIR/install"
}

require_ubuntu_2404() {
  [ -f /etc/os-release ] || fail "/etc/os-release not found"
  # shellcheck disable=SC1091
  source /etc/os-release
  [ "${ID:-}" = "ubuntu" ] || fail "Ubuntu 24.04 기준입니다. 현재 ID=${ID:-unknown}"
  [ "${VERSION_ID:-}" = "24.04" ] || fail "Ubuntu 24.04 기준입니다. 현재 VERSION_ID=${VERSION_ID:-unknown}"
}

require_python312() {
  if ! command -v python3.12 >/dev/null 2>&1; then
    log "python3.12 not found; installing python3.12/python3.12-venv"
    sudo apt-get update
    sudo apt-get install -y python3.12 python3.12-venv
  fi

  local version
  version="$(python3.12 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
  [ "$version" = "$PYTHON_REQUIRED" ] || fail "Python $PYTHON_REQUIRED 기준입니다. 현재 python3.12=$version"
}

require_ros_jazzy() {
  local setup="/opt/ros/${ROS_DISTRO_REQUIRED}/setup.bash"
  [ -f "$setup" ] || fail "ROS 2 ${ROS_DISTRO_REQUIRED} setup not found: $setup"
  source_if_exists "$setup"
  [ "${ROS_DISTRO:-}" = "$ROS_DISTRO_REQUIRED" ] || fail "ROS_DISTRO=${ROS_DISTRO:-empty}; ${ROS_DISTRO_REQUIRED} 기준입니다."
}

start_postgresql() {
  if command -v systemctl >/dev/null 2>&1; then
    sudo systemctl start postgresql
    return
  fi
  if command -v service >/dev/null 2>&1; then
    sudo service postgresql start
    return
  fi
  fail "PostgreSQL을 자동 시작할 수 없습니다. 수동으로 시작한 뒤 다시 실행하세요."
}

ensure_postgresql_tools() {
  if command -v psql >/dev/null 2>&1 && command -v pg_isready >/dev/null 2>&1; then
    return
  fi
  log "PostgreSQL client/server tools not found; installing postgresql"
  sudo apt-get update
  sudo apt-get install -y postgresql postgresql-contrib
}

setup_database() {
  [ "$SKIP_DB" = "1" ] && { log "SKIP_DB=1, DB setup skipped"; return; }

  ensure_postgresql_tools
  start_postgresql

  log "ensuring PostgreSQL role/database: ${DB_USER}/${DB_NAME}"
  if ! sudo -u postgres psql -tAc "SELECT 1 FROM pg_roles WHERE rolname='${DB_USER}'" | grep -q 1; then
    sudo -u postgres psql -c "CREATE USER ${DB_USER} WITH PASSWORD '${DB_PASSWORD}';"
  fi

  if ! sudo -u postgres psql -tAc "SELECT 1 FROM pg_database WHERE datname='${DB_NAME}'" | grep -q 1; then
    sudo -u postgres createdb "$DB_NAME"
  fi

  sudo -u postgres psql -d "$DB_NAME" -c "ALTER SCHEMA public OWNER TO ${DB_USER};"
  sudo -u postgres psql -d "$DB_NAME" -c "GRANT ALL ON SCHEMA public TO ${DB_USER};"
  sudo -u postgres psql -d "$DB_NAME" -c "GRANT ALL PRIVILEGES ON DATABASE ${DB_NAME} TO ${DB_USER};"

  if [ "$RESET_DB" = "1" ]; then
    log "RESET_DB=1, resetting public schema"
    sudo -u postgres psql -d "$DB_NAME" -c "DROP SCHEMA public CASCADE; CREATE SCHEMA public AUTHORIZATION ${DB_USER};"
    sudo -u postgres psql -d "$DB_NAME" -c "GRANT ALL ON SCHEMA public TO ${DB_USER};"
  fi

  if psql "$DATABASE_URL" -tAc "SELECT to_regclass('public.orders')" | grep -q orders; then
    log "DB schema already exists"
  else
    log "applying db/schema.sql and db/seed.sql"
    psql "$DATABASE_URL" -f "$ROOT_DIR/db/schema.sql"
    psql "$DATABASE_URL" -f "$ROOT_DIR/db/seed.sql"
  fi
}

setup_web_gateway_env() {
  [ "$SKIP_WEB" = "1" ] && { log "SKIP_WEB=1, web setup skipped"; return; }
  log "running web/scripts/setup.sh"
  RESET_VENV="$RESET_VENV" "$ROOT_DIR/web/scripts/setup.sh"
}

run_rosdep() {
  [ "$SKIP_ROSDEP" = "1" ] && { log "SKIP_ROSDEP=1, rosdep skipped"; return; }
  command -v rosdep >/dev/null 2>&1 || {
    log "rosdep not found; installing python3-rosdep"
    sudo apt-get update
    sudo apt-get install -y python3-rosdep
  }

  log "running rosdep for entire workspace"
  rosdep install --from-paths "$ROOT_DIR/src" --ignore-src -r -y
}

clean_colcon_artifacts() {
  log "cleaning colcon artifacts: build/ install/ log/"
  rm -rf "$ROOT_DIR/build" "$ROOT_DIR/install" "$ROOT_DIR/log"
}
build_workspace() {
  log "colcon build --symlink-install"
  colcon build --symlink-install
}

cd "$ROOT_DIR"
log "root: $ROOT_DIR"
log "fixed target: Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12 / full symlink build"

require_ubuntu_2404
require_python312
require_ros_jazzy
sanitize_workspace_overlay_env
clean_colcon_artifacts
setup_web_gateway_env
setup_database
run_rosdep
build_workspace

log "done"
log "run: ./run_all.sh"
log "or: source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch fleet_manager fleet_manager.launch.xml"
