# Just Pick It Web

FastAPI 기반 Control Server와 고객/관리자 UI를 두는 폴더입니다.

웹은 쇼핑몰 자체가 아니라 **로봇 시스템 관제와 주문 흐름을 확인하기 위한 Control Server + UI**입니다.  
최신 구현 계획은 [`../docs/web_plan.md`](../docs/web_plan.md), 작업 진행 기록은 [`WEB_STATUS.md`](WEB_STATUS.md)를 기준으로 봅니다.
주문/로봇 상태 전이 흐름은 [`WORKFLOW.md`](WORKFLOW.md)에 따로 정리했습니다.
로봇/Fleet/Vision/LLM에서 호출할 API 사용법은 [`API_USAGE.md`](API_USAGE.md)에 정리했습니다.

## Quick Start

처음 실행하는 팀원은 아래 순서만 따라 하면 됩니다.

### 초보자용 실행 순서

1. 웹/DB 자동 세팅을 실행합니다.

```bash
cd ~/autonomous_sys_ws
web/scripts/setup.sh
```

Ubuntu/apt 환경에서는 PostgreSQL이 없으면 `setup.sh`가 설치까지 시도합니다.  
설치와 DB 생성 과정에서 sudo 비밀번호를 물어볼 수 있습니다.

2. 웹 서버를 실행합니다.

```bash
web/scripts/run.sh
```

3. 브라우저에서 접속합니다.

```text
Customer UI : http://localhost:8000/customer
Admin UI    : http://localhost:8000/admin
API Docs    : http://localhost:8000/docs
DB Health   : http://localhost:8000/api/health/db
```

4. 서버를 끄고 싶으면 `web/scripts/run.sh`를 실행한 터미널에서 `Ctrl+C`를 누릅니다.

처음 세팅이 끝난 뒤에는 보통 `web/scripts/run.sh`만 실행하면 됩니다.

### 빠른 명령어

자동 세팅:

```bash
cd ~/autonomous_sys_ws
web/scripts/setup.sh
```

서버 실행:

```bash
cd ~/autonomous_sys_ws
web/scripts/run.sh
```

데모 데이터 초기화:

```bash
cd ~/autonomous_sys_ws
web/scripts/reset_demo_data.sh
```

Fleet runtime API smoke test:

```bash
cd ~/autonomous_sys_ws
web/.venv/bin/python web/scripts/smoke_runtime_flow.py --reset-db
```

초기화 후 상품은 seed 기준 상품 6종이 각 2개씩 들어갑니다.

브라우저:

```text
Customer UI : http://localhost:8000/customer
Admin UI    : http://localhost:8000/admin
API Docs    : http://localhost:8000/docs
DB Health   : http://localhost:8000/api/health/db
```

DB 구조까지 다시 만들고 싶으면:

```bash
cd ~/autonomous_sys_ws
RESET_DB=1 web/scripts/setup.sh
```

이미 DB 구조가 세팅된 상태에서 시연 데이터만 seed 기준으로 초기화하고 싶으면:

```bash
cd ~/autonomous_sys_ws
web/scripts/reset_demo_data.sh
```

## 현재 연결 상태

웹과 DB 연결은 **고객/관리자 UI + DB 연동 + 실제 Robot Control Node 상태 보고 API 기준으로 동작하는 상태**입니다.

완료된 것:

```text
- FastAPI 서버 실행
- PostgreSQL 연결
- /api/health/db DB 상태 확인
- 상품 목록 조회
- 고객 주문 생성
- 주문 생성 시 orders/order_item 저장
- 주문 생성 시 product.stock_qty 차감
- 고객 주문/재고 WebSocket 실시간 갱신
- 고객 픽업 완료 처리
- 관리자 /api/admin/status 통합 조회
- 관리자 WebSocket 실시간 갱신
- 관리자 대시보드/로봇/작업·주문/예외/재고 페이지
- 로봇/주문/task/픽업슬롯/예외/재고 상태 표시
- 관리자 UI에서 로봇/주문/task/픽업슬롯/재고 상태 수정
- 실제 Robot Control Node 연동용 상태 보고 API
- task_event 기록/조회 API
- Vision/Robot 예외 보고 API
- exception 처리 완료
- 긴급정지 / 재개
```

웹 쪽에서 아직 실제 외부 시스템과 연결하지 않은 것:

```text
- 실제 Robot Control Node / Control Bridge / ROS2 연결
- Claude API key 설정 후 실제 LLM 호출 테스트
- Robot Control Node와 Vision Server 사이의 영상 분석 요청/응답 연결
- task_event를 관리자 UI 타임라인으로 보여주는 화면
- 재고 임계치 알림을 exception/alert로 자동 생성하는 정책
```

즉, 현재 웹은 **로컬 DB와 UI 관제 기준으로는 바로 시연 가능한 상태**이고, 남은 큰 작업은 외부 Robot Control Node/Vision 시스템을 실제로 붙이는 일입니다.
Vision Server는 Control Server가 직접 중계하지 않고 Robot Control Node가 직접 호출한 뒤 결과만 `/api/fleet/*`로 보고하는 방향입니다.
LLM은 `ANTHROPIC_API_KEY`를 넣으면 Claude API로 호출하고, 비워두면 Claude tool-use 응답과 같은 형태의 로컬 고정 JSON으로 동작합니다.

## Structure

```text
web/
├── app/
│   ├── main.py
│   ├── routers/
│   ├── services/
│   ├── static/
│   │   ├── css/
│   │   └── js/
│   └── templates/
├── scripts/
│   ├── setup.sh
│   ├── run.sh
│   └── reset_demo_data.sh
├── tests/
├── requirements.txt
├── .env.example
└── README.md
```

## Manual Setup

자동 스크립트를 쓰지 않는 경우:

```bash
cd ~/autonomous_sys_ws/web
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
cp .env.example .env
```

PostgreSQL이 켜져 있어야 합니다.

```bash
sudo systemctl status postgresql
sudo systemctl start postgresql
```

DB와 계정 생성, schema/seed 적용은 [`../db/README.md`](../db/README.md)를 참고합니다.

서버 실행:

```bash
cd ~/autonomous_sys_ws/web
source .venv/bin/activate
uvicorn app.main:app --reload --host 0.0.0.0 --port 8000
```

## Scripts

### `web/scripts/setup.sh`

다음을 자동으로 처리합니다.

```text
- Python venv 생성
- requirements 설치
- web/.env 생성
- PostgreSQL 설치 시도(Ubuntu/apt 환경)
- PostgreSQL 실행 확인
- just_pick_it DB/user 생성
- public schema 권한 설정
- schema/seed 적용
```

이미 DB schema가 있으면 schema/seed 적용은 건너뜁니다.  
테이블 구조, enum, index까지 다시 만들고 싶으면 `RESET_DB=1 web/scripts/setup.sh`를 사용합니다.
기본 로컬 DB(`just_pick_it_user` / `just_pick_it`) 기준으로 DB/user를 만들고, `web/.env`의 `DATABASE_URL`로 schema/seed를 적용합니다.
스크립트 내부에 각 단계별 한글 주석을 달아두었으므로, 동작이 궁금하면 파일을 직접 열어 확인합니다.

### `web/scripts/run.sh`

`.env`의 `APP_HOST`, `APP_PORT`를 읽고, `web/.venv`를 활성화한 뒤 uvicorn 서버를 실행합니다.
이 스크립트도 실행 목적과 중지 방법을 한글 주석으로 적어두었습니다.

### `web/scripts/reset_demo_data.sh`

시연 전/후에 DB 구조는 유지하고 데이터만 seed 기준으로 되돌립니다.

```text
- task_event, exception_log, task 삭제
- order_item, orders 삭제
- robot, pickup_slot, product, zone 삭제
- PK 번호를 1번부터 다시 시작
- db/seed.sql 다시 적용
```

주문과 작업은 상품/로봇/픽업 슬롯을 참조하므로 상품만 초기화하면 참조가 꼬일 수 있습니다.  
그래서 이 스크립트는 주문, 작업, 예외, 상품, 로봇, 픽업 슬롯, 존을 함께 초기화합니다.

초기화 후 상태:

```text
- 주문 없음
- 작업 없음
- 예외 없음
- 상품은 db/seed.sql 기준 테스트 상품 6종, 각 5개로 복구
- 로봇은 db/seed.sql 기준 AMR/COBOT 테스트 데이터로 복구
- 픽업 슬롯은 db/seed.sql 기준으로 복구
- product_id, order_id, task_id 등 PK 번호는 다시 1번부터 시작
```

주의: 기존에 직접 추가한 상품/주문/task도 모두 사라집니다.

`RESET_DB=1 web/scripts/setup.sh`와의 차이:

```text
RESET_DB=1 web/scripts/setup.sh
  - DB schema를 삭제 후 재생성
  - schema.sql과 seed.sql 재적용
  - 테이블 구조나 enum이 바뀌었을 때 사용

web/scripts/reset_demo_data.sh
  - 기존 DB schema는 유지
  - 테이블 데이터만 비운 뒤 seed.sql 재적용
  - 시연 전 데이터를 깨끗하게 만들 때 사용
```

## Environment

`web/.env.example`은 예시 파일입니다.  
`web/scripts/setup.sh`를 실행하면 `web/.env`가 없을 때 자동으로 복사됩니다.

실제 실행 시 읽는 파일:

```text
web/.env
```

읽는 곳:

```text
- web/app/config.py
  - FastAPI 앱이 DATABASE_URL, APP_HOST, APP_PORT, CLAUDE_* 값을 읽음
- web/scripts/setup.sh
  - DATABASE_URL을 읽어 schema/seed 적용 대상 DB를 결정
- web/scripts/run.sh
  - APP_HOST, APP_PORT를 읽고 web/.venv를 활성화해서 uvicorn 실행
- web/scripts/reset_demo_data.sh
  - DATABASE_URL을 읽어서 초기화 대상 DB 결정
```

기본값:

```text
DATABASE_URL=postgresql://just_pick_it_user:just_pick_it_pw@localhost:5432/just_pick_it
APP_HOST=0.0.0.0
APP_PORT=8000
ANTHROPIC_API_KEY=
CLAUDE_MODEL=claude-sonnet-4-6
CLAUDE_MAX_TOKENS=512
CLAUDE_TIMEOUT_SECONDS=10
```

`ANTHROPIC_API_KEY`가 비어 있으면 `/api/admin/llm/messages`는 `A_ZONE` 순찰 명령 JSON을 로컬로 반환합니다.
키를 넣으면 Anthropic 공식 Python SDK로 Claude Messages API를 호출합니다.

## Robot Runtime API

`/api/fleet/*` 경로명은 기존 구현 호환을 위해 유지합니다.  
현재 프로젝트 방향에서는 Fleet Manager 전용 API가 아니라 각 Robot Control Node 또는 Control Bridge가 task/robot/order 상태를 보고하는 runtime API로 사용합니다.

각 로봇 담당 노드는 자기 로봇의 task 조회, 상태 보고, 예외 보고를 수행하고, Control Server는 받은 상태를 DB에 반영합니다.
고객 주문 생성 시 기본 task는 자동 생성되며, 가능한 로봇에는 ready task가 `ASSIGNED` 상태로 자동 배정됩니다.

```text
GET   /api/fleet/tasks
POST  /api/fleet/tasks
GET   /api/fleet/orders
GET   /api/fleet/orders/{order_id}/tasks
PATCH /api/fleet/orders/{order_id}
POST  /api/fleet/assignments/run
POST  /api/fleet/orders/{order_id}/assign-pickup-slot
PATCH /api/fleet/tasks/{task_id}
POST  /api/fleet/tasks/{task_id}/events
GET   /api/fleet/tasks/{task_id}/events
PATCH /api/fleet/robots/{robot_id}
GET   /api/fleet/pickup-slots
PATCH /api/fleet/pickup-slots/{slot_id}
POST  /api/fleet/exceptions
```

조회 예시:

```bash
curl "http://localhost:8000/api/fleet/tasks?robot_id=AMR_1&status=ASSIGNED"
curl "http://localhost:8000/api/fleet/orders?status=ORDER_WAIT"
curl "http://localhost:8000/api/fleet/orders/7/tasks"
curl "http://localhost:8000/api/fleet/pickup-slots?status=EMPTY"
```

배정 재시도 예시:

```bash
curl -X POST http://localhost:8000/api/fleet/assignments/run
```

검수 시작 시점 픽업 슬롯 예약 예시:

```bash
curl -X POST http://localhost:8000/api/fleet/orders/7/assign-pickup-slot
```

일반 흐름에서는 `INSPECTION` task를 `RUNNING`으로 바꿀 때 자동 예약됩니다. 이 API는 수동 테스트나 예외 복구용입니다.

예시:

```bash
curl -X PATCH http://localhost:8000/api/fleet/robots/AMR_1 \
  -H "Content-Type: application/json" \
  -d '{"status":"MOVING","current_task_id":12,"battery_level":84,"pos_x":1.2,"pos_y":0.4}'
```

작업 이벤트 기록 예시:

```bash
curl -X POST http://localhost:8000/api/fleet/tasks/12/events \
  -H "Content-Type: application/json" \
  -d '{"robot_id":"AMR_1","to_status":"RUNNING","event_name":"STANDBY_UNLOAD_STARTED","reason":"AMR started moving to unloading standby zone"}'
```

예외 보고 예시:

```bash
curl -X POST http://localhost:8000/api/fleet/exceptions \
  -H "Content-Type: application/json" \
  -d '{"exception_type":"INSPECTION_FAIL","robot_id":"INSPECTION_COBOT","task_id":13,"order_id":7,"detail":"검수 결과가 주문과 일치하지 않음"}'
```

## Troubleshooting

### `psql: command not found`

Ubuntu/apt 환경이면 `web/scripts/setup.sh`가 PostgreSQL 설치를 시도합니다.  
apt가 없는 환경이면 PostgreSQL을 수동 설치해야 합니다.

```bash
sudo apt update
sudo apt install postgresql postgresql-contrib
```

설치 후 다시 실행합니다.

```bash
web/scripts/setup.sh
```

### `[sudo] password for ...`

정상입니다. PostgreSQL 서비스 시작, DB/user 생성, schema 권한 설정에는 `sudo` 권한이 필요할 수 있습니다.  
현재 Ubuntu 계정 비밀번호를 입력하면 됩니다.

### `permission denied for schema public`

PostgreSQL 15 이상에서 public schema 권한 때문에 생길 수 있습니다.  
보통 `web/scripts/setup.sh`가 자동으로 처리합니다.

수동으로 처리하려면:

```bash
sudo -u postgres psql -d just_pick_it
```

PostgreSQL 콘솔에서:

```sql
ALTER SCHEMA public OWNER TO just_pick_it_user;
GRANT ALL ON SCHEMA public TO just_pick_it_user;
\q
```

### `address already in use`

이미 8000 포트를 쓰는 서버가 켜져 있는 상태입니다.

확인:

```bash
ss -ltnp | grep 8000
```

기존 웹 서버 터미널에서 `Ctrl+C`로 끄거나, 다른 포트로 실행합니다.

```bash
cd ~/autonomous_sys_ws/web
source .venv/bin/activate
APP_PORT=8001 uvicorn app.main:app --reload --host 0.0.0.0 --port 8001
```

### `/api/health/db`가 실패함

PostgreSQL이 꺼져 있거나 `.env`의 `DATABASE_URL`이 잘못됐을 수 있습니다.

```bash
sudo systemctl status postgresql
sudo systemctl start postgresql
cat ~/autonomous_sys_ws/web/.env
```

기본값은 아래와 같아야 합니다.

```text
DATABASE_URL=postgresql://just_pick_it_user:just_pick_it_pw@localhost:5432/just_pick_it
```

### DB 데이터를 처음 상태로 되돌리고 싶음

주의: 기존 주문, task, exception 테스트 데이터가 지워집니다.

```bash
cd ~/autonomous_sys_ws
RESET_DB=1 web/scripts/setup.sh
```

### 화면이 예전 UI 그대로 보임

브라우저 캐시 때문일 수 있습니다.

```text
Ctrl + Shift + R
```

로 강력 새로고침합니다.
