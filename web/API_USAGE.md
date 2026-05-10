# Just Pick It API 사용 가이드

이 문서는 웹/DB를 잘 모르는 팀원이 Swagger를 열지 않아도 로봇 노드, Fleet/Bridge, Vision, LLM 연동 API를 바로 사용할 수 있게 정리한 기준입니다.

## 기본 원칙

```text
로봇/Fleet/Vision/LLM은 DB에 직접 접근하지 않는다.
상태 변경은 Control Server API를 호출한다.
Control Server는 DB에 저장한 뒤 Admin/Customer UI로 WebSocket 갱신을 보낸다.
```

전체 흐름:

```text
Customer UI
  -> Control Server
      -> DB 저장
      -> Admin/Customer UI 갱신

Robot / Fleet / Bridge / Vision
  -> Control Server API 호출
      -> DB 저장
      -> Admin/Customer UI 갱신
```

## Control Server가 담당하는 기능

| 기능 | 설명 |
|---|---|
| DB 접근 | 주문, 상품, task, robot, pickup slot, exception을 PostgreSQL에 저장 |
| 상태 변경 API | 외부 로봇/Fleet/Bridge가 보낸 상태를 검증하고 DB에 반영 |
| 픽업 슬롯 배정 | 검수 시작 시점에 `EMPTY` 슬롯 하나를 `RESERVED`로 예약 |
| 실시간 갱신 | DB 변경 후 Admin/Customer UI에 WebSocket broadcast |
| 예외 기록 | Vision/Robot/Fleet에서 보낸 예외를 `exception_log`에 저장 |
| LLM 연결 | 관리자 자연어 명령을 Claude 또는 mock 응답으로 처리 |

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
| Admin UI | 관제 화면 조회, 수동 상태 변경, 데모 실행 | `/api/admin/*`, `/api/fleet/*` |
| Sorting Cobot 담당 노드 | 선별 task 조회/시작/완료 보고 | `GET /api/fleet/tasks`, `PATCH /api/fleet/tasks/{task_id}`, `PATCH /api/fleet/robots/{robot_id}` |
| AMR 담당 노드 | 배송/하차/복귀 task 조회/상태 보고 | `GET /api/fleet/tasks`, `PATCH /api/fleet/tasks/{task_id}`, `PATCH /api/fleet/robots/{robot_id}` |
| Inspection Cobot 담당 노드 | 검수 시작, 픽업 슬롯 예약, 검수 결과 보고 | `POST /api/fleet/orders/{order_id}/assign-pickup-slot`, `PATCH /api/fleet/tasks/{task_id}` |
| Vision Server | 검수/감지 실패 등 예외 보고 | `POST /api/fleet/exceptions` |
| LLM 기능 | 자연어 명령 파싱/응답 | `POST /api/admin/llm/messages` |

## 정상 주문 흐름

픽업 슬롯은 **검수 시작 시점**에 예약합니다. 주문 접수 시점에는 예약하지 않습니다.

| 단계 | 호출 시점 | 호출자 | API | 요청 예시 | 결과 |
|---|---|---|---|---|---|
| 1 | 고객 주문 | Customer UI | `POST /api/orders` | `{items:[{product_id, quantity}]}` | `orders` 생성, 재고 차감, 주문 상태 `ORDER_RECEIVED` |
| 2 | 주문 task 생성 | Fleet/Bridge 또는 담당 노드 | `POST /api/fleet/tasks` | `{order_id, task_type, status, assigned_robot_id}` | `task` 생성 |
| 3 | 선별 시작 | Sorting Cobot | `PATCH /api/fleet/tasks/{task_id}` | `{status:"RUNNING"}` | 선별 task 진행 중 |
| 4 | 선별 로봇 상태 보고 | Sorting Cobot | `PATCH /api/fleet/robots/COBOT1` | `{status:"SORTING", current_task_id}` | 로봇 상태 UI 갱신 |
| 5 | 선별 완료 | Sorting Cobot | `PATCH /api/fleet/tasks/{task_id}` | `{status:"SUCCESS"}` | 선별 task 성공 |
| 6 | 배송 시작 | AMR | `PATCH /api/fleet/tasks/{task_id}` | `{status:"RUNNING"}` | 배송 task 진행 중 |
| 7 | 배송 로봇 상태 보고 | AMR | `PATCH /api/fleet/robots/AMR1` | `{status:"DELIVERING", current_task_id, pos_x, pos_y, pos_theta}` | AMR 상태/위치 UI 갱신 |
| 8 | 배송 완료 | AMR | `PATCH /api/fleet/tasks/{task_id}` | `{status:"SUCCESS"}` | 배송 task 성공 |
| 9 | 검수 시작 | Inspection Cobot | `POST /api/fleet/orders/{order_id}/assign-pickup-slot` | body 없음 | 빈 픽업 슬롯 1개를 `RESERVED`로 예약 |
| 10 | 검수 진행 보고 | Inspection Cobot | `PATCH /api/fleet/tasks/{task_id}` | `{status:"RUNNING"}` | 검수 task 진행 중 |
| 11 | 주문 상태 변경 | Inspection Cobot/Fleet | `PATCH /api/fleet/orders/{order_id}` | `{status:"INSPECTING"}` | 고객/관리자 UI에 검수 중 표시 |
| 12 | 검수 성공 | Inspection Cobot | `PATCH /api/fleet/tasks/{task_id}` | `{status:"SUCCESS"}` | 검수 task 성공 |
| 13 | 하차 시작 | AMR 또는 하차 담당 노드 | `PATCH /api/fleet/tasks/{task_id}` | `{status:"RUNNING"}` | 하차 task 진행 중 |
| 14 | 하차 완료 | AMR 또는 하차 담당 노드 | `PATCH /api/fleet/pickup-slots/{slot_id}` | `{status:"OCCUPIED"}` | 픽업 슬롯이 상품 있음/픽업 대기 상태 |
| 15 | 픽업 가능 처리 | AMR 또는 하차 담당 노드 | `PATCH /api/fleet/orders/{order_id}` | `{status:"PICKUP_READY"}` | 고객 UI에 픽업 가능 + 슬롯 번호 표시 |
| 16 | 고객 수령 | Customer UI | `POST /api/orders/{order_id}/complete` | body 없음 | 주문 완료, 픽업 슬롯 `EMPTY` |

## 핵심 API 상세

### DB 상태 확인

| API | 사용 시점 | 응답 | 결과 |
|---|---|---|---|
| `GET /api/health/db` | 서버 실행 후 DB 연결 확인 | `{status:"ok"}` | PostgreSQL 연결 정상 여부 확인 |

### 상품 조회

| API | 사용 시점 | 응답 | 결과 |
|---|---|---|---|
| `GET /api/products` | 고객 UI 상품 목록 표시 | `[{product_id, name, image_url, stock_qty, storage_location}]` | 고객 상품 카드 표시 |

고객 UI에서는 `storage_location`을 화면에 보여주지 않습니다.

### 주문 생성

| API | 사용 시점 | 요청 | 응답 | 결과 |
|---|---|---|---|---|
| `POST /api/orders` | 고객이 주문하기 클릭 | `{items:[{product_id, quantity}]}` | `{order_id, order_no, status, items}` | 주문 생성, 재고 차감 |

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

| API | 사용 시점 | 요청 | 응답 | 결과 |
|---|---|---|---|---|
| `POST /api/fleet/tasks` | 주문을 로봇 작업으로 나눌 때 | `{order_id, task_type, status, assigned_robot_id}` | `{status:"ok", task_id}` | task 생성 |

예시:

```json
{
  "order_id": 7,
  "task_type": "SORTING",
  "status": "QUEUED",
  "assigned_robot_id": "COBOT1"
}
```

한 주문의 기본 task 예시:

```text
SORTING    -> COBOT1
DELIVERY   -> AMR1 또는 AMR2
INSPECTION -> COBOT2
UNLOAD     -> DELIVERY와 같은 AMR
```

### task 조회

| API | 사용 시점 | 응답 | 결과 |
|---|---|---|---|
| `GET /api/fleet/tasks` | 전체 task 조회 | `[{task_id, order_id, order_no, assigned_robot_id, task_type, status}]` | 전체 task 확인 |
| `GET /api/fleet/tasks?robot_id=AMR1` | 특정 로봇 작업 큐 확인 | 동일 | AMR1에 배정된 task만 확인 |
| `GET /api/fleet/tasks?status=QUEUED` | 대기 task 확인 | 동일 | 아직 시작 안 한 task 확인 |
| `GET /api/fleet/orders/{order_id}/tasks` | 주문 하나의 task 흐름 확인 | 동일 | 주문 상세/작업 큐 확인 |

### task 상태 변경

| API | 사용 시점 | 요청 | 결과 |
|---|---|---|---|
| `PATCH /api/fleet/tasks/{task_id}` | task 시작/완료/실패 | `{status, assigned_robot_id, result_message}` | task 상태 저장, UI 갱신 |

예시:

```json
{
  "status": "RUNNING",
  "assigned_robot_id": "AMR1"
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
  "status": "DELIVERING",
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

| API | 사용 시점 | 요청 | 응답 | 결과 |
|---|---|---|---|---|
| `POST /api/fleet/orders/{order_id}/assign-pickup-slot` | 검수 시작 시점 | body 없음 | `{pickup_slot_id, slot_name, slot_status}` | 빈 슬롯 1개 예약, 주문에 slot 저장 |

예시 응답:

```json
{
  "status": "ok",
  "order_id": 7,
  "order_no": "ORD-0007",
  "pickup_slot_id": 1,
  "slot_name": "Pickup Slot 1",
  "slot_status": "RESERVED"
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
  "robot_id": "AMR1",
  "to_status": "RUNNING",
  "event_name": "DELIVERY_STARTED",
  "reason": "AMR started delivery"
}
```

### 예외 보고

| API | 사용 시점 | 요청 | 결과 |
|---|---|---|---|
| `POST /api/fleet/exceptions` | Vision/Robot/Fleet에서 예외 감지 | `{exception_type, robot_id, task_id, order_id, detail}` | Admin UI 예외/알람 표시 |

예시:

```json
{
  "exception_type": "INSPECTION_FAIL",
  "robot_id": "COBOT2",
  "task_id": 103,
  "order_id": 7,
  "detail": "검수 결과 주문 상품과 실제 상품이 일치하지 않음"
}
```

### LLM 자연어 명령

| API | 사용 시점 | 요청 | 응답 | 결과 |
|---|---|---|---|---|
| `POST /api/admin/llm/messages` | 관리자 자연어 명령 입력 | `{message}` | `{result, message, action, target_zone_name}` | Claude key가 있으면 실제 LLM, 없으면 mock 응답 |

예시:

```json
{
  "message": "B 구역 순찰해줘"
}
```

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

### robot.status

| 값 | 의미 |
|---|---|
| `IDLE` | 대기 |
| `MOVING` | 이동 중 |
| `WAITING` | 대기 중 |
| `SORTING` | 선별 중 |
| `DELIVERING` | 배송 중 |
| `INSPECTING` | 검수 중 |
| `UNLOADING` | 하차 중 |
| `PATROLLING` | 순찰 중 |
| `CHARGING` | 충전 중 |
| `RETURNING` | 복귀 중 |
| `PARKING` | 파킹 중 |
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

### Sorting Cobot 담당

```text
1. GET /api/fleet/tasks?robot_id=COBOT1&status=QUEUED
2. 선별할 task 선택
3. PATCH /api/fleet/tasks/{task_id} {"status":"RUNNING"}
4. PATCH /api/fleet/robots/COBOT1 {"status":"SORTING","current_task_id":task_id}
5. 실제 선별 수행
6. 성공 시 PATCH /api/fleet/tasks/{task_id} {"status":"SUCCESS"}
7. PATCH /api/fleet/robots/COBOT1 {"status":"IDLE","current_task_id":null}
8. 실패 시 POST /api/fleet/exceptions
```

### AMR 담당

```text
1. GET /api/fleet/tasks?robot_id=AMR1&status=QUEUED
2. DELIVERY 또는 UNLOAD task 선택
3. PATCH /api/fleet/tasks/{task_id} {"status":"RUNNING"}
4. PATCH /api/fleet/robots/AMR1 {"status":"DELIVERING","current_task_id":task_id}
5. 주행 중 주기적으로 PATCH /api/fleet/robots/AMR1로 위치/배터리 보고
6. 도착/작업 성공 시 PATCH /api/fleet/tasks/{task_id} {"status":"SUCCESS"}
7. 필요하면 PATCH /api/fleet/orders/{order_id}로 주문 상태 변경
```

### Inspection Cobot 담당

```text
1. GET /api/fleet/tasks?robot_id=COBOT2&status=QUEUED
2. INSPECTION task 선택
3. 검수 시작 시 POST /api/fleet/orders/{order_id}/assign-pickup-slot
4. 응답으로 받은 pickup_slot_id 저장
5. PATCH /api/fleet/tasks/{task_id} {"status":"RUNNING"}
6. PATCH /api/fleet/robots/COBOT2 {"status":"INSPECTING","current_task_id":task_id}
7. 실제 검수 수행
8. 성공 시 PATCH /api/fleet/tasks/{task_id} {"status":"SUCCESS"}
9. 실패 시 POST /api/fleet/exceptions
10. 재시도하지 않을 거면 PATCH /api/fleet/orders/{order_id} {"status":"ERROR","pickup_slot_id":null}
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
