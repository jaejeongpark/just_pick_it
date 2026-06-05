#!/usr/bin/env bash
set -euo pipefail

# Just Pick It 데모 데이터를 seed 기준으로 되돌립니다.
# DB schema 자체까지 다시 만들려면 ./reset_ws.sh 를 사용합니다. DB를 유지하려면 RESET_DB=0을 붙입니다.

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DB_NAME="${DB_NAME:-just_pick_it}"
DB_USER="${DB_USER:-just_pick_it_user}"
DB_PASSWORD="${DB_PASSWORD:-just_pick_it_pw}"
DB_HOST="${DB_HOST:-localhost}"
DB_PORT="${DB_PORT:-5432}"
DATABASE_URL="${DATABASE_URL:-postgresql://${DB_USER}:${DB_PASSWORD}@${DB_HOST}:${DB_PORT}/${DB_NAME}}"

if ! command -v psql >/dev/null 2>&1; then
  echo "[demo-reset] psql command not found. Run ./reset_ws.sh first." >&2
  exit 1
fi

echo "[demo-reset] clearing demo tables and resetting primary keys"
psql "$DATABASE_URL" <<'SQL'
TRUNCATE TABLE
  task_event,
  exception_log,
  task,
  stocking_item,
  order_item,
  orders,
  robot,
  robot_unit,
  pickup_slot,
  product,
  zone
RESTART IDENTITY CASCADE;
SQL

echo "[demo-reset] applying seed data"
psql "$DATABASE_URL" -f "$ROOT_DIR/db/seed.sql"

echo "[demo-reset] done"
