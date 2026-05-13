# Just Pick It API 사용 가이드

이 문서는 웹/DB를 잘 모르는 팀원이 Swagger를 열지 않아도 각 Robot Control Node, Vision, LLM 연동 경계를 바로 이해할 수 있게 정리한 기준입니다.

## 기본 원칙

```text
로봇 담당 노드/Control Bridge/LLM은 DB에 직접 접근하지 않는다.
상태 변경은 Control Server API를 호출한다.
Vision Server는 Robot Control Node가 직접 호출하고, Control Server는 이미지/영상 데이터를 중계하지 않는다.
Control Server는 DB에 저장한 뒤 Admin/Customer UI로 WebSocket 갱신을 보낸다.
```

전체 흐름:

```text
Customer UI
  -> Control Server
      -> DB 저장
      -> Admin/Customer UI 갱신

Robot Control Node / Control Bridge / LLM
  -> Control Server API 호출
      -> DB 저장
      -> Admin/Customer UI 갱신

Robot Control Node
  -> Vision Server 직접 호출
      -> 결과 수신
          -> Control Server API로 상태/예외 보고
```

`/api/fleet/*` 경로명은 기존 구현 호환을 위해 유지합니다.  
현재 프로젝트 방향에서는 Fleet Manager 전용 API가 아니라 **각 로봇 담당 노드가 task/robot/order 상태를 보고하는 runtime API**로 사용합니다.

## Control Server가 담당하는 기능

| 기능 | 설명 |
|---|---|
| DB 접근 | 주문, 상품, task, robot, pickup slot, exception을 PostgreSQL에 저장 |
| 주문 task 생성/배정 | 고객 주문 생성 시 기본 task를 만들고, Control Server가 로봇 상태를 기준으로 배정 처리 |
| 상태 변경 API | Robot Control Node/Control Bridge/Admin UI가 보낸 상태를 검증하고 DB에 반영 |
| 픽업 슬롯 배정 | 검수 시작 시점에 `EMPTY` 슬롯 하나를 `RESERVED`로 예약 |
| 실시간 갱신 | DB 변경 후 Admin/Customer UI에 WebSocket broadcast |
| 예외 기록 | Robot Control Node 또는 Vision 연동 노드가 보낸 예외를 `exception_log`에 저장 |
| LLM 연결 | 관리자 자연어 명령을 Claude 또는 로컬 고정 JSON으로 처리 |

## 서버 주소와 호출 방식

웹 서버 실행:

```bash
cd ~/autonomous_sys_ws
web/scripts/run.sh
```

기본 주소:

```text
http://localhost:8000
```

API 호출 방식:

```text
GET    조회
POST   새 데이터 생성 또는 명령 요청
PATCH  일부 상태 변경
```

자주 보는 URL:

```text
Admin UI    : http://localhost:8000/admin
Customer UI : http://localhost:8000/customer
DB Health   : http://localhost:8000/api/health/db
```

## 누가 무엇을 호출하나

| 호출자 | 목적 | 주로 쓰는 API |
|---|---|---|
| Customer UI | 상품 조회, 주문 생성, 픽업 완료 | `GET /api/products`, `POST /api/orders`, `POST /api/orders/{order_id}/complete` |
| Admin UI | 관제 화면 조회, 수동 상태 변경 | `/api/admin/*`, `/api/fleet/*` |
| SORTING_COBOT 담당 노드 | 배정된 선별/상차 task 조회, 시작/완료 보고 | `GET /api/fleet/tasks?robot_id=SORTING_COBOT&status=ASSIGNED`, `PATCH /api/fleet/tasks/{task_id}` |
| AMR 담당 노드 | 대기/배터리/위치 상태 보고, 배정된 이동 task 조회/상태 보고 | `PATCH /api/fleet/robots/{robot_id}`, `GET /api/fleet/tasks?robot_id=AMR_1&status=ASSIGNED`, `PATCH /api/fleet/tasks/{task_id}` |
| INSPECTION_COBOT 담당 노드 | 배정된 검수/하차 task 조회, 결과 보고 | `GET /api/fleet/tasks?robot_id=INSPECTION_COBOT&status=ASSIGNED`, `PATCH /api/fleet/tasks/{task_id}` |
| Vision Server | Robot Control Node가 직접 호출하는 인식/검수/안전 감지 서버 | Control Server API 직접 호출 없음 |
| Vision 연동 노드 | Vision 결과 기반 예외/상태 보고 | `POST /api/fleet/exceptions`, `PATCH /api/fleet/tasks/{task_id}` |
| LLM 기능 | 자연어 명령 파싱, PATROL task 생성/배정 | `POST /api/admin/llm/messages` |

## 정상 주문 흐름

픽업 슬롯은 **검수 시작 시점**에 예약합니다. 주문 접수 시점에는 예약하지 않습니다.

| 단계 | 호출 시점 | 호출자 | API | 요청 모델 | 결과 |
|---|---|---|---|---|---|
| 1 | 고객 주문 | Customer UI | `POST /api/orders` | `{items:[{product_id, quantity}]}` | `orders/order_item/task` 생성, 재고 차감, 주문 상태 `ORDER_WAIT` |
| 2 | AMR 대기 상태 보고 | AMR Robot Control Node | `PATCH /api/fleet/robots/{robot_id}` | `{status:"STANDBY", battery_level, pos_x, pos_y, pos_theta}` | 배터리 조건이 맞으면 Control Server가 우선순위 높은 AMR task 배정 |
| 3 | 배정 task 조회 | 각 Robot Control Node | `GET /api/fleet/tasks?robot_id={robot_id}&status=ASSIGNED` | - | 자기 로봇의 다음 task 확인 |
| 4 | task 시작 | 각 Robot Control Node | `PATCH /api/fleet/tasks/{task_id}` | `{status:"RUNNING"}` | task 진행 중, robot/current_task/order 상태 자동 갱신 |
| 5 | task 완료 | 각 Robot Control Node | `PATCH /api/fleet/tasks/{task_id}` | `{status:"SUCCESS"}` | task 성공 후 robot/current_task 정리, 다음 ready task 자동 배정 |
| 6 | 검수 task 시작 | INSPECTION_COBOT | `PATCH /api/fleet/tasks/{inspection_task_id}` | `{status:"RUNNING"}` | 빈 픽업 슬롯이 `RESERVED` |
| 7 | 하차 완료 | INSPECTION_COBOT | `PATCH /api/fleet/tasks/{unload_task_id}` | `{status:"SUCCESS"}` | 픽업 슬롯 `OCCUPIED`, 주문 `PICKUP_READY` |
| 8 | 고객 수령 | Customer UI | `POST /api/orders/{order_id}/complete` | body 없음 | 주문 완료, 픽업 슬롯 `EMPTY` |

## 핵심 API 상세

### DB 상태 확인

| API | 사용 시점 | 실제 응답 모델 | 결과 |
|---|---|---|---|
| `GET /api/health/db` | 서버 실행 후 DB 연결 확인 | `{status:"ok"}` | PostgreSQL 연결 정상 여부 확인 |

### 상품 조회

| API | 사용 시점 | 실제 응답 모델 | 결과 |
|---|---|---|---|
| `GET /api/products` | 고객 UI 상품 목록 표시 | `[{product_id, name, image_url, stock_qty, storage_location}]` | 고객 상품 카드 표시 |

고객 UI에서는 `storage_location`을 화면에 보여주지 않습니다.

### 주문 생성

| API | 사용 시점 | 실제 요청 모델 | 실제 응답 모델 | 결과 |
|---|---|---|---|---|
| `POST /api/orders` | 고객이 주문하기 클릭 | `OrderCreate` | `OrderRead` | 주문 생성, 재고 차감 |

예시:

```json
{
  "items": [
    {"product_id": 1, "quantity": 1},
    {"product_id": 2, "quantity": 1}
  ]
}
```

### task 생성

| API | 사용 시점 | 실제 요청 모델 | 실제 응답 모델 | 결과 |
|---|---|---|---|---|
| `POST /api/orders` | 고객 주문 접수 | `OrderCreate` | `OrderRead` | 기본 주문 task 자동 생성 |
| `POST /api/fleet/tasks` | 수동 테스트 또는 특수 task 생성 | `FleetTaskCreate` | `FleetTaskRead` | task 수동 생성 |

수동 task 생성 예시:

```json
{
  "order_id": 7,
  "task_type": "SORTING",
  "status": "QUEUED",
  "assigned_robot_id": "SORTING_COBOT",
  "priority": 2
}
```

한 주문의 기본 task 예시:

```text
STANDBY_LOAD   -> QUEUED, AMR 상태 보고 시 배정
SORTING        -> QUEUED, SORTING_COBOT 고정
LOAD           -> QUEUED, SORTING_COBOT 고정
STANDBY_UNLOAD -> QUEUED, STANDBY_LOAD와 같은 AMR로 예약
INSPECTION     -> QUEUED, INSPECTION_COBOT 고정
UNLOAD         -> QUEUED, INSPECTION_COBOT 고정
```

주문 생성 직후 모든 task가 바로 실행되는 것은 아닙니다.
Control Server는 task 순서와 로봇 상태를 보고 실행 가능한 task만 `ASSIGNED`로 변경합니다.

### task 조회

| API | 사용 시점 | 실제 응답 모델 | 결과 |
|---|---|---|---|
| `GET /api/fleet/tasks` | 전체 task 조회 | `list[FleetTaskSummaryRead]` | 전체 task 확인 |
| `GET /api/fleet/tasks?robot_id=AMR_1` | 특정 로봇 작업 큐 확인 | 동일 | AMR_1에 배정된 task만 확인 |
| `GET /api/fleet/tasks?robot_id=AMR_1&status=ASSIGNED` | 특정 로봇의 다음 실행 task 확인 | 동일 | AMR_1이 지금 시작할 수 있는 task 확인 |
| `GET /api/fleet/tasks?status=QUEUED` | 아직 배정되지 않은 task 확인 | 동일 | 앞 단계/로봇 대기 중인 task 확인 |
| `GET /api/fleet/tasks?status=QUEUED&task_type=STANDBY_LOAD` | 특정 종류의 대기 task 확인 | 동일 | 디버깅/관리자 확인용 |
| `GET /api/fleet/orders/{order_id}/tasks` | 주문 하나의 task 흐름 확인 | 동일 | 주문 상세/작업 큐 확인 |

`FleetTaskSummaryRead`는 Robot Control Node가 AMR/Cobot에 넘길 실행 명령 payload의 기준입니다.

```text
FleetTaskSummaryRead = {
  task_id: int,
  order_id: int | null,
  order_no: string | null,
  assigned_robot_id: string | null,
  task_type: string,
  status: string,
  priority: int,
  source_zone_id: int | null,
  source_zone_name: string | null,
  source_zone_pose: {x: float, y: float, z: float, theta: float | null} | null,
  target_zone_id: int | null,
  target_zone_name: string | null,
  target_zone_pose: {x: float, y: float, z: float, theta: float | null} | null,
  result_message: string | null
}
```

### 주문 목록 조회

| API | 사용 시점 | 실제 응답 모델 | 결과 |
|---|---|---|---|
| `GET /api/fleet/orders` | Robot Runtime 기준 미완료 주문 확인 | `[{order_id, order_no, status, current_task_type, current_task_status, assigned_robot_id}]` | 남은 주문/현재 단계 확인 |
| `GET /api/fleet/orders?status=ORDER_WAIT` | 작업 대기 주문 확인 | 동일 | 아직 작업 중이 아닌 주문 확인 |
| `GET /api/fleet/orders?include_completed=true` | 완료 주문까지 포함해서 확인 | 동일 | 디버깅용 전체 주문 확인 |

AMR Robot Control Node는 작업을 직접 가져가지 않고, 상차 대기존 복귀/대기 상태와 배터리를 `/api/fleet/robots/{robot_id}`로 보고합니다.
Control Server는 그 시점에 priority가 높은 task부터 배정합니다.

### task 자동 배정

| 트리거 | 처리 |
|---|---|
| 주문 생성 | task 6개 생성, Cobot task 고정 배정 |
| AMR 상태 보고 | AMR이 `IDLE`/`STANDBY`이고 배터리 20 이상이면 주문 task(priority 2) 우선 배정, 없으면 순찰 task(priority 1) 배정 |
| task SUCCESS | 다음 순서 task가 고정 로봇이면 자동 `ASSIGNED` |

### task 상태 변경

| API | 사용 시점 | 요청 | 결과 |
|---|---|---|---|
| `PATCH /api/fleet/tasks/{task_id}` | task 시작/완료/실패 | `{status, assigned_robot_id, result_message}` | task 상태 저장, UI 갱신 |

예시:

```json
{
  "status": "RUNNING",
  "assigned_robot_id": "AMR_1"
}
```

```json
{
  "status": "SUCCESS"
}
```

### robot 상태 변경

| API | 사용 시점 | 요청 | 결과 |
|---|---|---|---|
| `PATCH /api/fleet/robots/{robot_id}` | 로봇 상태/배터리/위치 보고 | `{status, current_task_id, battery_level, pos_x, pos_y, pos_theta}` | 로봇 상태 저장, 미니맵/UI 갱신 |

예시:

```json
{
  "status": "MOVING",
  "current_task_id": 12,
  "battery_level": 84,
  "pos_x": 0.9,
  "pos_y": 0.8,
  "pos_theta": 0.0
}
```

### 주문 상태 변경

| API | 사용 시점 | 요청 | 결과 |
|---|---|---|---|
| `PATCH /api/fleet/orders/{order_id}` | 주문 단계 전환 | `{status, pickup_slot_id}` | 고객/관리자 주문 상태 갱신 |

예시:

```json
{
  "status": "INSPECTING"
}
```

```json
{
  "status": "PICKUP_READY"
}
```

검수 실패 후 픽업 슬롯 예약을 해제해야 하는 경우:

```json
{
  "status": "ERROR",
  "pickup_slot_id": null
}
```

### 픽업 슬롯 조회

| API | 사용 시점 | 응답 | 결과 |
|---|---|---|---|
| `GET /api/fleet/pickup-slots` | 픽업 슬롯 전체 확인 | `[{slot_id, slot_name, status, order_id, order_no}]` | 슬롯 상태 확인 |
| `GET /api/fleet/pickup-slots?status=EMPTY` | 빈 슬롯 확인 | 동일 | 배정 가능 슬롯 확인 |

### 픽업 슬롯 자동 배정

| API | 사용 시점 | 실제 요청 모델 | 실제 응답 모델 | 결과 |
|---|---|---|---|---|
| `POST /api/fleet/orders/{order_id}/assign-pickup-slot` | 검수 시작 시점 | body 없음 | `FleetPickupSlotAssignmentRead` | 빈 슬롯 1개 예약, 주문에 slot 저장 |

일반 검수 흐름에서는 `INSPECTION` task를 `RUNNING`으로 바꾸면 Control Server가 자동으로 예약합니다.
이 API는 수동 테스트나 예외 복구 때 사용할 수 있습니다.

응답 모델:

```text
FleetPickupSlotAssignmentRead = {
  status: string,
  order_id: int,
  order_no: string,
  pickup_slot_id: int,
  slot_name: string,
  slot_status: string
}
```

이 API를 쓰는 이유:

```text
GET으로 빈 슬롯을 읽고 PATCH로 따로 예약하면, 동시에 두 로봇이 같은 슬롯을 잡을 수 있다.
assign-pickup-slot은 Control Server가 한 번에 예약해서 충돌 가능성을 줄인다.
```

### 픽업 슬롯 상태 변경

| API | 사용 시점 | 요청 | 결과 |
|---|---|---|---|
| `PATCH /api/fleet/pickup-slots/{slot_id}` | 하차 완료 또는 슬롯 사용 불가 처리 | `{status}` | 슬롯 상태 저장, UI 갱신 |

일반 하차 흐름에서는 `UNLOAD` task를 `SUCCESS`로 바꾸면 Control Server가 자동으로 `OCCUPIED` 처리합니다.
이 API는 관리자가 슬롯을 `BLOCKED` 처리하거나 수동 복구할 때 사용합니다.

예시:

```json
{
  "status": "OCCUPIED"
}
```

### task event 기록

| API | 사용 시점 | 요청 | 결과 |
|---|---|---|---|
| `POST /api/fleet/tasks/{task_id}/events` | 작업 시작/완료/실패 이력 저장 | `{robot_id, to_status, event_name, reason}` | task_event 기록 |
| `GET /api/fleet/tasks/{task_id}/events` | 작업 이력 조회 | - | task 이벤트 타임라인 조회 |

예시:

```json
{
  "robot_id": "AMR_1",
  "to_status": "RUNNING",
  "event_name": "STANDBY_UNLOAD_STARTED",
  "reason": "AMR started moving to unloading standby zone"
}
```

### 예외 보고

| API | 사용 시점 | 요청 | 결과 |
|---|---|---|---|
| `POST /api/fleet/exceptions` | Robot Control Node 또는 Vision 연동 노드에서 예외 감지 | `{exception_type, robot_id, task_id, order_id, detail}` | Admin UI 예외/알람 표시 |

예시:

```json
{
  "exception_type": "INSPECTION_FAIL",
  "robot_id": "INSPECTION_COBOT",
  "task_id": 103,
  "order_id": 7,
  "detail": "검수 결과 주문 상품과 실제 상품이 일치하지 않음"
}
```

### LLM 자연어 명령

| API | 사용 시점 | 실제 요청 모델 | 실제 응답 모델 | 결과 |
|---|---|---|---|---|
| `POST /api/admin/llm/messages` | 관리자 자연어 명령 입력 | `AdminLlmMessageCreate` | `AdminLlmMessageRead` | 순찰 명령이면 zone을 확인한 뒤 가능한 AMR에 `PATROL` task 배정 |

요청 모델:

```text
AdminLlmMessageCreate = {
  message: string
}
```

응답 모델:

```text
AdminLlmMessageRead = {
  result: string,
  message: string,
  action: string | null,
  task_id: int | null,
  assigned_robot_id: string | null,
  target_zone_id: int | null,
  target_zone_name: string | null,
  provider: string
}
```

순찰 처리 흐름:

```text
Admin UI
-> POST /api/admin/llm/messages {message}
-> Control Server가 LLM/local fixed JSON으로 action, target_zone_id/target_zone_name 수신
-> Control Server가 DB zone으로 target_zone_id 확정
-> Control Server가 PATROL task QUEUED 생성
-> AMR Robot Control Node가 대기 상태/배터리를 PATCH /api/fleet/robots/{robot_id}로 보고
-> Control Server가 대기 중인 AMR에 PATROL task ASSIGNED
-> Robot Control Node가 FleetTaskSummaryRead의 target_zone_id/target_zone_pose로 AMR 이동 실행
-> Robot Control Node가 PATCH /api/fleet/tasks/{task_id}, PATCH /api/fleet/robots/{robot_id}, POST /api/fleet/exceptions로 결과 보고
```

LLM 응답은 명령 해석/task 생성 결과이고, AMR 실행 명령의 실제 payload는 배정 이후의 `FleetTaskSummaryRead`입니다.
`B 구역`처럼 `zone` 테이블에 없는 이름은 task를 만들지 않고 error 응답으로 돌려줍니다.

## 상태값 기준

### order.status

| 값 | 의미 |
|---|---|
| `ORDER_RECEIVED` | 주문 접수 |
| `ORDER_WAIT` | 작업 대기 |
| `SORTING` | 선별 중 |
| `DELIVERING` | 배송/운반 중 |
| `INSPECTING` | 검수 중 |
| `PICKUP_READY` | 픽업 가능 |
| `COMPLETED` | 고객 수령 완료 |
| `ERROR` | 주문 오류 |

### task.status

| 값 | 의미 |
|---|---|
| `QUEUED` | 큐 대기 |
| `ASSIGNED` | 로봇 할당됨 |
| `RUNNING` | 실행 중 |
| `PAUSED` | 일시정지 |
| `SUCCESS` | 성공 |
| `FAILED` | 실패 |
| `CANCELLED` | 취소 |

### task.task_type

| 값 | 의미 |
|---|---|
| `STANDBY_LOAD` | AMR 상차 대기존 이동/대기 |
| `STANDBY_UNLOAD` | AMR 하차 대기존 이동/대기 |
| `SORTING` | 상품 선별 |
| `LOAD` | 상품 상차 |
| `INSPECTION` | 상품 검수 |
| `UNLOAD` | 픽업 슬롯 하차 |
| `PATROL` | 순찰 |
| `CHARGE` | 충전 |
| `RETURN_HOME` | 복귀 |

### robot.status

| 값 | 의미 |
|---|---|
| `IDLE` | 대기 |
| `MOVING` | 이동 중 |
| `WAITING` | 대기 중 |
| `STANDBY` | AMR 상하차 대기 중 |
| `SORTING` | 선별 중 |
| `LOADING` | 상품 상차 중 |
| `PARKING` | 주차 중 |
| `INSPECTING` | 검수 중 |
| `UNLOADING` | 하차 중 |
| `PATROLLING` | 순찰 중 |
| `CHARGING` | 충전 중 |
| `RETURNING` | 복귀 중 |
| `DOCKING` | 도킹 중 |
| `EMERGENCY_STOP` | 긴급정지 |
| `ERROR` | 오류 |
| `OFFLINE` | 연결 끊김 |

### pickup_slot.status

| 값 | 의미 |
|---|---|
| `EMPTY` | 비어 있음 |
| `RESERVED` | 주문에 예약됨 |
| `OCCUPIED` | 상품 있음, 고객 픽업 대기 |
| `BLOCKED` | 사용 불가 |

## 구현 담당별 추천 사용 흐름

### SORTING_COBOT 담당

```text
1. GET /api/fleet/tasks?robot_id=SORTING_COBOT&status=ASSIGNED
2. 배정된 SORTING 또는 LOAD task 선택
3. PATCH /api/fleet/tasks/{task_id} {"status":"RUNNING"}
4. 실제 선별/상차 수행
5. 성공 시 PATCH /api/fleet/tasks/{task_id} {"status":"SUCCESS"}
6. LOAD 성공이면 Control Server가 order_item을 SORTED 처리
7. 실패 시 POST /api/fleet/exceptions
```

### AMR 담당

```text
1. 상차 대기존 복귀 또는 대기 상태에서 PATCH /api/fleet/robots/AMR_1로 상태/위치/배터리 보고
2. GET /api/fleet/tasks?robot_id=AMR_1&status=ASSIGNED
3. 배정된 STANDBY_LOAD / STANDBY_UNLOAD / PATROL task 선택
4. PATCH /api/fleet/tasks/{task_id} {"status":"RUNNING"}
5. 주행 중 주기적으로 PATCH /api/fleet/robots/AMR_1로 위치/배터리 보고
6. 도착/작업 성공 시 PATCH /api/fleet/tasks/{task_id} {"status":"SUCCESS"}
```

### INSPECTION_COBOT 담당

```text
1. GET /api/fleet/tasks?robot_id=INSPECTION_COBOT&status=ASSIGNED
2. 배정된 INSPECTION 또는 UNLOAD task 선택
3. PATCH /api/fleet/tasks/{task_id} {"status":"RUNNING"}
4. INSPECTION 시작이면 Control Server가 pickup_slot을 RESERVED 처리
5. 실제 검수/하차 수행
6. 성공 시 PATCH /api/fleet/tasks/{task_id} {"status":"SUCCESS"}
7. UNLOAD 성공이면 Control Server가 pickup_slot OCCUPIED, order PICKUP_READY 처리
8. 실패 시 POST /api/fleet/exceptions
```

## 자주 나는 실수

```text
DB를 직접 수정하지 않는다.
GET 요청은 request body가 없다.
PATCH는 바꾸고 싶은 필드만 보내면 된다.
pickup slot은 GET 후 PATCH 두 번으로 직접 예약하지 말고 assign-pickup-slot을 쓴다.
고객 UI는 PICKUP_READY 전까지 픽업 번호를 보여주지 않는다.
API 호출 후 Admin/Customer UI는 WebSocket으로 자동 갱신된다.
직접 psql로 DB를 수정하면 WebSocket broadcast가 안 되므로 새로고침이 필요할 수 있다.
```

## HTTP 응답 코드

| 코드 | 의미 | 주로 발생하는 상황 |
|---|---|---|
| `200` | 성공 | 조회/수정 성공 |
| `201` | 생성 성공 | task/event/exception 생성 |
| `404` | 대상 없음 | 없는 order/task/robot/slot ID 사용 |
| `409` | 현재 상태에서 처리 불가 | 완료 주문에 슬롯 배정, 빈 슬롯 없음 |
| `422` | 요청 형식 오류 | status 오타, 필수 필드 누락 |
