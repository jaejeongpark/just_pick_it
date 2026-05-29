#!/usr/bin/env bash
set -euo pipefail

# Just Pick It 데모 DB를 schema + seed 기준으로 빠르게 되돌립니다.
# venv/rosdep/colcon build는 건드리지 않습니다.

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

echo "[demo-reset] resetting public schema"
psql "$DATABASE_URL" <<'SQL'
DROP SCHEMA IF EXISTS public CASCADE;
CREATE SCHEMA public;
GRANT ALL ON SCHEMA public TO public;
SQL
psql "$DATABASE_URL" -v db_user="$DB_USER" <<'SQL'
GRANT ALL ON SCHEMA public TO :"db_user";
SQL

echo "[demo-reset] applying schema and seed data"
psql "$DATABASE_URL" -f "$ROOT_DIR/db/schema.sql"
psql "$DATABASE_URL" -f "$ROOT_DIR/db/seed.sql"

echo "[demo-reset] done"
