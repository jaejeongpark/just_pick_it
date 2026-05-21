# Just Pick It API 사용 가이드

이 문서는 Customer UI, Admin UI, Fleet Manager, LLM 담당 모듈이 Control Server와 어떤 API로 통신하는지 정리한 문서이다.

기본 주소:

```text
http://localhost:8000
```

API 문서:

```text
http://localhost:8000/docs
```

---

## 1. 기본 원칙

```text
Customer UI / Admin UI
  -> Control Server HTTP/WebSocket API 사용

Fleet Manager
  -> Control Server snapshot/event 수신
  -> Control Server /api/fleet/* API로 task/robot/order/exception 상태 저장

PICKY / COBOT
  -> Control Server와 직접 통신하지 않음
  -> Fleet Manager를 통해 명령 수신 및 상태 보고

LLM 담당 모듈
  -> /api/admin/llm/messages 구현부 또는 별도 모듈에서 자연어 파싱
  -> 입고 명령이면 Fleet Manager에 STOCKING_COMMAND 전달

Vision
  -> Control Server가 이미지/영상 중계하지 않음
  -> Fleet Manager 또는 로봇 쪽 모듈이 직접 호출하고 결과만 Control Server에 보고
```

`/api/fleet/*` 이름은 유지하지만, 의미는 “Fleet Manager가 Control Server에 상태를 저장하거나 조회하는 API”이다.

---

## 2. 호출 주체별 API

| 호출자 | 목적 | 주요 API |
|---|---|---|
| Customer UI | 상품 조회, 주문 생성, 주문 조회, 수령 완료 | `/api/products`, `/api/orders`, `/api/customer/status` |
| Admin UI | 통합 관제, 재고 관리, 예외 처리, LLM 명령 | `/api/admin/*` |
| Fleet Manager | snapshot 수신, task 생성/조회/상태 보고, robot 상태 보고 | `/api/fleet/*` |
| LLM 담당 모듈 | 자연어 입고 명령 파싱 | `/api/admin/llm/messages` |
| 외부 Vision/로봇 모듈 | Control Server 직접 호출 없음 | Fleet Manager 경유 |

---

## 3. Customer UI API

### 3-1. 상품 목록 조회

```http
GET /api/products
```

응답:

```json
[
  {
    "product_id": 1,
    "name": "우유",
    "image_url": "/static/img/milk.png",
    "stock_qty": 2,
    "storage_zone_id": 11,
    "storage_zone_name": "PRODUCT_SLOT_1",
    "storage_location": "PRODUCT_SLOT_1"
  }
]
```

### 3-2. 주문 생성

```http
POST /api/orders
```

요청:

```json
{
  "items": [
    {"product_id": 1, "quantity": 1},
    {"product_id": 2, "quantity": 1}
  ]
}
```

응답:

```json
{
  "order_id": 1,
  "order_no": "ORD-0001",
  "status": "ORDER_WAIT",
  "priority": 2,
  "pickup_slot_id": null,
  "pickup_slot_name": null,
  "assigned_unit_id": null,
  "items": [
    {
      "item_id": 1,
      "product_id": 1,
      "product_name": "우유",
      "image_url": "/static/img/milk.png",
      "quantity": 1,
      "status": "WAITING"
    }
  ]
}
```

처리:

| 항목 | 설명 |
|---|---|
| 재고 | 주문 수량만큼 즉시 차감 |
| orders | `ORDER_WAIT`, `priority=2` |
| order_item | 주문 상품 목록 생성 |
| task | Control Server가 만들지 않음. Fleet Manager가 이벤트를 받고 생성 |
| 이벤트 | `ORDER_CREATED`를 Fleet Manager WebSocket으로 broadcast |

### 3-3. 주문 목록 조회

```http
GET /api/orders
```

응답:

```json
[
  {
    "order_id": 1,
    "order_no": "ORD-0001",
    "status": "SORTING",
    "priority": 2,
    "pickup_slot_id": null,
    "pickup_slot_name": null,
    "assigned_unit_id": 1,
    "items": []
  }
]
```

`COMPLETED` 주문은 제외된다.

### 3-4. 주문 상세 조회

```http
GET /api/orders/{order_id}
```

### 3-5. 수령 완료

```http
POST /api/orders/{order_id}/complete
```

조건:

| 조건 | 설명 |
|---|---|
| 가능 상태 | `PICKUP_READY` |
| 처리 | 주문 `COMPLETED`, 픽업 슬롯 `EMPTY` |

### 3-6. 고객 화면 전체 상태

```http
GET /api/customer/status
WS /api/customer/ws/status
```

응답:

```json
{
  "products": [],
  "orders": []
}
```

---

## 4. Admin UI API

### 4-1. 관리자 통합 상태 조회

```http
GET /api/admin/status
WS /api/admin/ws/status
```

응답 주요 필드:

| 필드 | 의미 |
|---|---|
| `orders` | 완료되지 않은 주문 |
| `order_history` | 완료 주문 |
| `robots` | PICKY/COBOT 상태 |
| `tasks` | task 목록 |
| `products` | 상품/재고 목록 |
| `pickup_slots` | 픽업 슬롯 목록 |
| `exceptions` | 미처리 예외 |
| `exception_history` | 처리 완료 예외 |
| `low_stock_count` | 재고 부족 상품 수 |
| `unresolved_exception_count` | 미처리 예외 수 |

`robots` 예:

```json
[
  {
    "robot_id": 1,
    "robot_name": "PICKY1",
    "unit_id": 1,
    "robot_type": "PICKY",
    "robot_status": "BUSY",
    "status": "BUSY",
    "picky_state": "MOVING_TO_PRODUCT",
    "cobot_state": null,
    "battery_level": 100,
    "current_task_id": 3,
    "current_task_type": "MOVE_TO_PRODUCT",
    "current_task_status": "RUNNING",
    "pos_x": 0.9,
    "pos_y": 0.82,
    "pos_theta": 0.0
  }
]
```

`tasks` 예:

```json
[
  {
    "task_id": 3,
    "order_id": 1,
    "order_no": "ORD-0001",
    "order_item_id": 2,
    "product_id": 2,
    "product_name": "시리얼",
    "product_quantity": 1,
    "sequence_no": 3,
    "assigned_robot_id": 1,
    "assigned_robot_name": "PICKY1",
    "task_type": "MOVE_TO_PRODUCT",
    "status": "RUNNING",
    "priority": 2,
    "source_zone_name": "PRODUCT_ZONE_1",
    "target_zone_name": "PRODUCT_ZONE_2",
    "result_message": "시리얼 보관 위치로 이동 중"
  }
]
```

### 4-2. 긴급 정지 / 재개

```http
POST /api/admin/emergency-stop
POST /api/admin/resume
```

응답:

```json
{"status": "ok"}
```

### 4-3. 상품 등록

```http
POST /api/admin/products
```

요청:

```json
{
  "name": "우유",
  "stock_qty": 10,
  "storage_zone_id": 11,
  "image_url": "/static/img/milk.png"
}
```

`storage_zone_id` 대신 `storage_location`에 zone 이름을 넣을 수도 있다.

### 4-4. 상품 수정

```http
PATCH /api/admin/products/{product_id}
```

요청:

```json
{
  "name": "우유",
  "stock_qty": 10,
  "storage_zone_id": 11,
  "image_url": "/static/img/milk.png"
}
```

### 4-5. 재고 수량만 수정

```http
PATCH /api/admin/products/{product_id}/stock
```

요청:

```json
{
  "stock_qty": 5
}
```

### 4-6. 픽업 슬롯 생성

```http
POST /api/admin/pickup-slots
```

요청:

```json
{
  "slot_name": "Pickup_slot_5",
  "status": "EMPTY"
}
```

### 4-7. 예외 처리 완료

```http
POST /api/admin/exceptions/{exception_id}/resolve
```

응답:

```json
{"status": "ok"}
```

### 4-8. LLM 명령 입력

```http
POST /api/admin/llm/messages
```

요청:

```json
{
  "message": "우유 5개 입고해줘"
}
```

현재 응답:

```json
{
  "result": "ok",
  "message": "LLM 명령 파싱은 아직 연결 대기 상태입니다. 담당 모듈에서 구현해주세요.",
  "action": "CHAT",
  "task_id": null,
  "assigned_robot_id": null,
  "assigned_robot_name": null,
  "target_zone_id": null,
  "target_zone_name": null,
  "product_id": null,
  "product_name": null,
  "requested_quantity": null,
  "stocking_policy": null,
  "stocking_item_id": null,
  "provider": "stub"
}
```

향후 LLM 담당자가 구현해야 할 입고 응답:

```json
{
  "result": "ok",
  "message": "우유 5개 입고 명령을 인식했습니다.",
  "action": "STOCKING",
  "product_id": 1,
  "product_name": "우유",
  "requested_quantity": 5,
  "stocking_policy": "REQUESTED_QUANTITY",
  "stocking_item_id": 1,
  "provider": "llm"
}
```

`action="STOCKING"`이면 Control Server는 `stocking_item`을 만들고 Fleet Manager 이벤트 WebSocket으로 `STOCKING_COMMAND`를 보낸다.

---

## 5. Fleet Manager API

Fleet Manager는 Control Server DB를 직접 만지지 않고 아래 API만 사용한다.

### 5-1. 이벤트 수신

```http
WS /api/fleet/ws/events
```

이벤트 예:

```json
{"event": "ORDER_CREATED", "order_id": 1, "order_no": "ORD-0001"}
{"event": "TASKS_CREATED", "task_ids": [1, 2, 3]}
{"event": "TASK_STATUS_CHANGED", "task_id": 3, "status": "SUCCESS"}
{"event": "ROBOT_STATE_CHANGED", "robot_id": 1, "robot_name": "PICKY1"}
{"event": "EXCEPTION_CREATED", "exception_id": 1}
{"event": "STOCKING_COMMAND", "message": "우유 5개 입고해줘", "command": {}}
```

### 5-2. 전체 snapshot 조회

```http
GET /api/fleet/snapshot
```

응답은 `GET /api/admin/status`와 같은 형태이다. Fleet Manager 재시작 시 현재 주문, task, robot, pickup slot 상태를 한 번에 복구하는 용도이다.

### 5-2-1. zone 조회

```http
GET /api/fleet/zones
GET /api/fleet/zones?zone_type=PRODUCT
```

응답:

```json
[
  {
    "zone_id": 5,
    "zone_name": "PRODUCT_ZONE_1",
    "zone_type": "PRODUCT",
    "pose": {"x": 0.2, "y": 0.8, "z": 0.0, "theta": 0.0}
  }
]
```

`*_ZONE_*`은 PICKY가 이동/주차할 위치이고, `*_SLOT_*`은 COBOT이 상품을 집거나 내려놓는 물리 위치이다.

### 5-3. 입고 item 생성/조회/수정

```http
POST /api/fleet/stocking-items
GET /api/fleet/stocking-items
PATCH /api/fleet/stocking-items/{stocking_item_id}
```

생성 요청:

```json
{
  "product_id": 1,
  "requested_quantity": 5,
  "detected_quantity": null,
  "stock_delta": null,
  "stocking_policy": "REQUESTED_QUANTITY",
  "status": "REQUESTED",
  "assigned_unit_id": 1
}
```

응답:

```json
{
  "stocking_item_id": 1,
  "product_id": 1,
  "product_name": "우유",
  "requested_quantity": 5,
  "detected_quantity": null,
  "stock_delta": null,
  "stocking_policy": "REQUESTED_QUANTITY",
  "status": "REQUESTED",
  "assigned_unit_id": 1
}
```

수량이 없는 입고 명령이면 `requested_quantity`는 `null`, `stocking_policy`는 `ALL_DETECTED`로 둔다.

### 5-4. task bulk 생성

```http
POST /api/fleet/tasks/bulk
```

요청:

```json
{
  "tasks": [
    {
      "order_id": 1,
      "order_item_id": 2,
      "stocking_item_id": null,
      "sequence_no": 1,
      "assigned_robot_name": "PICKY1",
      "task_type": "MOVE_TO_PRODUCT",
      "status": "ASSIGNED",
      "priority": 2,
      "source_zone_id": 1,
      "target_zone_id": 5,
      "result_message": null
    }
  ]
}
```

응답:

```json
{
  "status": "ok",
  "task_ids": [1],
  "created_count": 1
}
```

주의:

| 필드 | 설명 |
|---|---|
| `sequence_no` | Fleet Manager가 경로 계산 후 정한 실행 순서 |
| `assigned_robot_id` | 숫자 ID 또는 로봇 이름 문자열 가능 |
| `assigned_robot_name` | 로봇 이름으로 배정할 때 사용 |
| `order_item_id` | 상품별 이동/선별 task에 연결 |
| `stocking_item_id` | 입고 task를 `stocking_item`과 연결할 때 사용 |

### 5-5. task 조회

```http
GET /api/fleet/tasks
```

쿼리:

| 쿼리 | 예 |
|---|---|
| `robot_id` | `/api/fleet/tasks?robot_id=1` |
| `robot_name` | `/api/fleet/tasks?robot_name=PICKY1` |
| `status` | `/api/fleet/tasks?status=RUNNING` |
| `task_type` | `/api/fleet/tasks?task_type=MOVE_TO_PRODUCT` |
| `order_id` | `/api/fleet/tasks?order_id=1` |

응답:

```json
[
  {
    "task_id": 3,
    "order_id": 1,
    "order_no": "ORD-0001",
    "order_item_id": 2,
    "stocking_item_id": null,
    "product_id": 2,
    "product_name": "시리얼",
    "product_quantity": 1,
    "requested_quantity": null,
    "detected_quantity": null,
    "stock_delta": null,
    "stocking_policy": null,
    "stocking_status": null,
    "sequence_no": 3,
    "assigned_robot_id": 1,
    "assigned_robot_name": "PICKY1",
    "task_type": "MOVE_TO_PRODUCT",
    "status": "RUNNING",
    "priority": 2,
    "source_zone_id": 4,
    "source_zone_name": "PRODUCT_ZONE_1",
    "source_zone_pose": {"x": 0.2, "y": 0.8, "z": 0.1, "theta": 0.0},
    "target_zone_id": 5,
    "target_zone_name": "PRODUCT_ZONE_2",
    "target_zone_pose": {"x": 0.32, "y": 0.8, "z": 0.1, "theta": 0.0},
    "result_message": "시리얼 보관 위치로 이동 중"
  }
]
```

### 5-6. 주문별 task 조회

```http
GET /api/fleet/orders/{order_id}/tasks
```

### 5-7. task 상태 변경

```http
PATCH /api/fleet/tasks/{task_id}
```

요청:

```json
{
  "current_status": "ASSIGNED",
  "status": "RUNNING",
  "assigned_robot_name": "PICKY1",
  "result_message": "상품 보관 위치로 이동 중"
}
```

응답:

```json
{
  "status": "ok",
  "previous_status": "ASSIGNED",
  "current_status": "RUNNING"
}
```

충돌 방지:

| 경우 | 결과 |
|---|---|
| `current_status`가 DB 현재 상태와 같음 | 변경 성공 |
| `current_status`가 DB 현재 상태와 다름 | `409 task status conflict` |

자동 후처리:

| task 상태 | 후처리 |
|---|---|
| `RUNNING` | robot `current_task_id`, `robot_status`, 세부 state 갱신 |
| `SUCCESS` | robot 작업 해제, item/order/pickup slot 후처리 |
| `FAILED` | order `ERROR`, robot 작업 해제 |
| `CANCELLED` | robot 작업 해제 |

### 5-8. task event 생성/조회

```http
POST /api/fleet/tasks/{task_id}/events
GET /api/fleet/tasks/{task_id}/events
```

요청:

```json
{
  "robot_name": "PICKY1",
  "from_status": "RUNNING",
  "to_status": "SUCCESS",
  "event_name": "ARRIVED_PRODUCT_ZONE",
  "reason": "상품 구역 도착",
  "update_task_status": true
}
```

응답:

```json
{
  "event_id": 1,
  "task_id": 3,
  "robot_id": 1,
  "robot_name": "PICKY1",
  "from_status": "RUNNING",
  "to_status": "SUCCESS",
  "event_name": "ARRIVED_PRODUCT_ZONE",
  "reason": "상품 구역 도착",
  "created_at": "2026-05-20T10:00:00+00:00"
}
```

### 5-9. robot 상태 조회/보고

조회:

```http
GET /api/fleet/robots/PICKY1
GET /api/fleet/robots/1
GET /api/fleet/robots/PICKY1/running-task
```

보고:

```http
PATCH /api/fleet/robots/PICKY1
```

PICKY 요청:

```json
{
  "robot_status": "BUSY",
  "picky_state": "MOVING_TO_PRODUCT",
  "current_task_id": 3,
  "battery_level": 88,
  "pos_x": 0.32,
  "pos_y": 0.8,
  "pos_theta": 0.0
}
```

COBOT 요청:

```json
{
  "robot_status": "BUSY",
  "cobot_state": "SORTING",
  "current_task_id": 4
}
```

응답:

```json
{"status": "ok"}
```

주의:

| 규칙 | 설명 |
|---|---|
| PICKY | `picky_state`만 보고한다. `cobot_state`는 넣지 않는다. |
| COBOT | `cobot_state`만 보고한다. `picky_state`는 넣지 않는다. |
| `status` | UI 호환 필드. 새 코드에서는 `robot_status`를 권장한다. |

### 5-10. 주문 상태 변경

```http
PATCH /api/fleet/orders/{order_id}
```

요청:

```json
{
  "status": "INSPECTING",
  "pickup_slot_id": 2,
  "assigned_unit_id": 1
}
```

응답:

```json
{"status": "ok"}
```

### 5-11. 주문 큐 조회

```http
GET /api/fleet/orders
GET /api/fleet/orders?status=ORDER_WAIT
GET /api/fleet/orders?include_completed=true
```

응답:

```json
[
  {
    "order_id": 1,
    "order_no": "ORD-0001",
    "status": "SORTING",
    "priority": 2,
    "pickup_slot_id": null,
    "pickup_slot_name": null,
    "assigned_unit_id": 1,
    "current_task_id": 3,
    "current_task_type": "MOVE_TO_PRODUCT",
    "current_task_status": "RUNNING",
    "assigned_robot_id": 1,
    "assigned_robot_name": "PICKY1"
  }
]
```

### 5-12. 픽업 슬롯 예약/조회/변경

```http
POST /api/fleet/orders/{order_id}/assign-pickup-slot
GET /api/fleet/pickup-slots
PATCH /api/fleet/pickup-slots/{slot_id}
```

픽업 슬롯 변경 요청:

```json
{
  "status": "OCCUPIED"
}
```

### 5-13. 예외 보고

```http
POST /api/fleet/exceptions
```

요청:

```json
{
  "exception_type": "OBSTACLE_DETECTED",
  "robot_name": "PICKY2",
  "task_id": 12,
  "order_id": 2,
  "detail": "픽업존 이동 경로에 장애물 감지"
}
```

응답:

```json
{
  "status": "ok",
  "exception_id": 1
}
```

### 5-14. 입고 완료 보고

```http
POST /api/fleet/stocking/complete
```

요청:

```json
{
  "task_id": 31,
  "detected_quantity": 5,
  "stock_delta": 5,
  "result_message": "우유 5개 입고 완료"
}
```

응답:

```json
{
  "status": "ok",
  "task_id": 31,
  "stocking_item_id": 1,
  "product_id": 1,
  "stock_delta": 5,
  "stock_qty": 7
}
```

`stock_delta`가 없으면 `stocking_item.requested_quantity`, `stocking_item.detected_quantity` 순서로 재고 증가량을 계산한다.

### 5-15. 배정 실행 API

```http
POST /api/fleet/assignments/run
```

현재 응답:

```json
{
  "status": "ok",
  "assigned_count": 0,
  "message": "task assignment is handled by Fleet Manager"
}
```

이 API는 예전 호출 호환용이다. 실제 task 배정은 Fleet Manager가 한다.

---

## 6. 주요 ENUM

### order_status

```text
ORDER_RECEIVED
ORDER_WAIT
SORTING
DELIVERING
INSPECTING
PICKUP_READY
COMPLETED
ERROR
```

### order_item_status

```text
WAITING
SORTED
INSPECTED
MISSING
EXCESS
MISMATCH
```

### pickup_slot_status

```text
EMPTY
RESERVED
OCCUPIED
BLOCKED
```

### robot_type

```text
PICKY
COBOT
```

### robot_status

```text
OFFLINE
IDLE
BUSY
CHARGING
EMERGENCY_STOP
ERROR
```

### picky_state

```text
CHARGING
STANDBY
MOVING_TO_PRODUCT
WAITING_FOR_COBOT
MOVING_TO_PICKUP
MOVING_TO_STOCK
MOVING_TO_STORAGE
RETURNING
DOCKING
ERROR_RECOVERY
```

### cobot_state

```text
STANDBY
SORTING
LOADING
INSPECTING
UNLOADING
STOCKING_SORTING
STOCKING_LOADING
STOCKING_PLACING
STOWING_ARM
SAFETY_STOPPED
```

### task_type

```text
MOVE_TO_PRODUCT
SORTING_AND_LOAD
MOVE_TO_PICKUP
INSPECTION
UNLOAD
MOVE_TO_STOCK
STOCKING_PICK
MOVE_TO_STORAGE
STOCKING_PLACE
RETURN_HOME
CHARGE
```

### task_status

```text
QUEUED
ASSIGNED
RUNNING
PAUSED
SUCCESS
FAILED
CANCELLED
```

### exception_type

```text
OBSTACLE_DETECTED
LOW_BATTERY
NAVIGATION_FAILED
HARDWARE_ERROR
TIMEOUT
SORTING_FAIL
INSPECTION_FAIL
HUMAN_DETECTED
SYSTEM_ERROR
```

---

## 7. task_type별 기본 담당

| task_type | 담당 | 설명 |
|---|---|---|
| `MOVE_TO_PRODUCT` | PICKY | 주문 상품 보관 구역으로 이동 |
| `SORTING_AND_LOAD` | COBOT | 주문 상품 선별 후 PICKY에 상차 |
| `MOVE_TO_PICKUP` | PICKY | 픽업존으로 이동 |
| `INSPECTION` | COBOT | 주문 상품 검수 |
| `UNLOAD` | COBOT | 픽업 슬롯에 하차 |
| `MOVE_TO_STOCK` | PICKY | 입고존으로 이동 |
| `STOCKING_PICK` | COBOT | 입고 상품 선별 후 PICKY에 상차 |
| `MOVE_TO_STORAGE` | PICKY | 상품 보관 위치로 이동 |
| `STOCKING_PLACE` | COBOT | 상품 보관 위치에 적재 |
| `RETURN_HOME` | PICKY | 대기/충전 구역 복귀 |
| `CHARGE` | PICKY | 충전 |

---

## 8. 빠른 curl 예시

서버 상태:

```bash
curl http://localhost:8000/api/health/db
```

관리자 snapshot:

```bash
curl http://localhost:8000/api/admin/status
```

Fleet snapshot:

```bash
curl http://localhost:8000/api/fleet/snapshot
```

PICKY1 조회:

```bash
curl http://localhost:8000/api/fleet/robots/PICKY1
```

PICKY1 task 조회:

```bash
curl "http://localhost:8000/api/fleet/tasks?robot_name=PICKY1"
```

진행 중 task 조회:

```bash
curl "http://localhost:8000/api/fleet/tasks?status=RUNNING"
```

예외 보고:

```bash
curl -X POST http://localhost:8000/api/fleet/exceptions \
  -H "Content-Type: application/json" \
  -d '{
    "exception_type": "OBSTACLE_DETECTED",
    "robot_name": "PICKY2",
    "detail": "테스트 예외"
  }'
```
