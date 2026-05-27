#!/usr/bin/env bash
set -euo pipefail

# Just Pick It Web Gateway 실행 스크립트
#
# 담당 범위:
# - web/.venv 활성화
# - FastAPI Web Gateway(:8000) 실행
#
# 실제 API/DB 처리는 Fleet Manager API(:8100)가 담당한다.
# Fleet Manager까지 함께 켜려면 루트의 ./run_all.sh를 사용한다.

WEB_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ ! -d "$WEB_DIR/.venv" ]; then
  echo "[web-run] missing web/.venv. Run web/scripts/setup.sh first." >&2
  exit 1
fi

if [ -f "$WEB_DIR/.env" ]; then
  set -a
  # shellcheck disable=SC1091
  source "$WEB_DIR/.env"
  set +a
fi

APP_HOST="${APP_HOST:-0.0.0.0}"
APP_PORT="${APP_PORT:-8000}"
FLEET_API_BASE_URL="${FLEET_API_BASE_URL:-http://localhost:8100}"
FLEET_API_WS_BASE_URL="${FLEET_API_WS_BASE_URL:-ws://localhost:8100}"
export FLEET_API_BASE_URL FLEET_API_WS_BASE_URL

if command -v curl >/dev/null 2>&1; then
  if ! curl -fsS --max-time 1 "$FLEET_API_BASE_URL/api/health/db" >/dev/null 2>&1; then
    echo "[web-run] warning: Fleet API is not responding yet: $FLEET_API_BASE_URL" >&2
    echo "[web-run] start Fleet API: source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch fleet_manager fleet_manager.launch.xml" >&2
    echo "[web-run] or run full stack from repo root: ./run_all.sh" >&2
  fi
fi

cd "$WEB_DIR"
source "$WEB_DIR/.venv/bin/activate"
echo "[web-run] Customer: http://localhost:${APP_PORT}/customer"
echo "[web-run] Admin   : http://localhost:${APP_PORT}/admin"
echo "[web-run] Fleet API: $FLEET_API_BASE_URL"
exec uvicorn app.main:app --reload --host "$APP_HOST" --port "$APP_PORT"
