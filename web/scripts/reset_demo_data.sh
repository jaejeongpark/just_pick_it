#!/usr/bin/env bash

set -euo pipefail

# Just Pick It 데모 데이터를 초기 상태로 되돌리는 스크립트입니다.
# 시연 전/후에 주문, 작업, 예외, 재고, 로봇, 픽업 슬롯을 깨끗하게 다시 맞출 때 사용합니다.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROOT_DIR="$(cd "$WEB_DIR/.." && pwd)"

# web/.env가 있으면 그 안의 DATABASE_URL을 먼저 읽습니다.
if [ -f "$WEB_DIR/.env" ]; then
  set -a
  # shellcheck disable=SC1091
  source "$WEB_DIR/.env"
  set +a
fi

# DATABASE_URL을 따로 지정하지 않으면 우리 프로젝트 기본 로컬 DB를 사용합니다.
DB_URL="${DATABASE_URL:-postgresql://just_pick_it_user:just_pick_it_pw@localhost:5432/just_pick_it}"

# psql이 없으면 schema/seed 적용을 할 수 없으므로 먼저 알려줍니다.
if ! command -v psql >/dev/null 2>&1; then
  echo "[demo-reset] psql command not found. Install PostgreSQL client first."
  exit 1
fi

echo "[demo-reset] clearing demo tables and resetting primary keys"

# 주문/작업/예외는 상품, 로봇, 픽업 슬롯과 연결되어 있으므로 전체 데모 테이블을 함께 초기화합니다.
# RESTART IDENTITY는 product_id, order_id 같은 PK 번호를 1번부터 다시 시작하게 합니다.
psql "$DB_URL" <<'SQL'
TRUNCATE TABLE
  task_event,
  exception_log,
  task,
  order_item,
  orders,
  robot,
  pickup_slot,
  product,
  zone
RESTART IDENTITY CASCADE;
SQL

echo "[demo-reset] applying seed data"

# db/seed.sql에는 기본 zone, product, pickup_slot, robot 테스트 데이터가 들어 있습니다.
psql "$DB_URL" -f "$ROOT_DIR/db/seed.sql"

echo "[demo-reset] done"
