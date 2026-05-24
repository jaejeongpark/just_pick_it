# Just Pick It Workflow

이 문서는 시나리오별로 무엇이 호출되고, 어떤 테이블이 어떤 형태로 바뀌는지 읽기 위한 기준 문서이다.

현재 기준:

- Control Server는 주문, 재고, 로봇 상태, task 상태, 예외 기록을 저장하고 UI에 제공한다.
- Fleet Manager는 Task Manager, Traffic Manager, State Manager를 포함한다.
- Fleet Manager가 task 생성, task 배정, 경로/충돌 판단, 로봇 실행 명령을 담당한다.
- PICKY와 COBOT은 Control Server와 직접 통신하지 않고 Fleet Manager를 통해 제어된다.
- Control Server는 주문/상태 변경이 생기면 UI WebSocket과 Fleet Manager 이벤트 WebSocket으로 알린다.
- `/api/fleet/*`는 이름은 fleet이지만, 실제 의미는 Fleet Manager가 Control Server 상태 저장소를 읽고 쓰는 API이다.
- LLM UI와 API는 남겨두었지만, 현재 LLM client는 담당자 구현 대기용 stub이다.

---

## 1. 전체 구조

```text
Customer UI
  -> Control Server
      -> DB 저장
      -> Customer/Admin UI 실시간 갱신
      -> Fleet Manager 이벤트 발행

Fleet Manager
  -> Control Server snapshot/event 수신
  -> task 생성/배정
  -> PICKY/COBOT 실행 명령
  -> Control Server로 task/robot/exception 상태 보고

Admin UI
  -> Control Server
      -> 상태 조회, 재고 관리, 예외 처리, 긴급 정지, LLM 명령 입력
```

역할 분리:

| 구성 | 역할 |
|---|---|
| Control Server | DB 저장, API 제공, UI 실시간 갱신 |
| Fleet Manager | task 생성/배정, 경로 계획, 로봇 실행 제어 |
| PICKY | 주행 로봇 |
| COBOT | 로봇팔 |
| robot_unit | 함께 동작하는 PICKY와 COBOT 묶음 |

---

## 2. 기본 주문 시나리오

### 2-1. 고객이 주문한다

호출:

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

Control Server 처리:

| 순서 | 처리 |
|---|---|
| 1 | `product` row를 lock하고 재고를 확인한다. |
| 2 | `orders` row를 `ORDER_RECEIVED`로 생성한다. |
| 3 | 주문번호 `ORD-{order_id:04d}`를 만든다. |
| 4 | 주문 수량만큼 `product.stock_qty`를 차감한다. |
| 5 | `order_item` row를 생성한다. |
| 6 | 주문 상태를 `ORDER_WAIT`로 바꾼다. |
| 7 | UI 상태를 broadcast한다. |
| 8 | Fleet Manager에 `ORDER_CREATED` 이벤트를 보낸다. |

DB 변화:

| 테이블 | 변화 |
|---|---|
| `orders` | 주문 1건 생성, `status="ORDER_WAIT"`, `priority=2` |
| `order_item` | 주문 상품 목록 생성, 기본 `status="WAITING"` |
| `product` | 주문 수량만큼 재고 차감 |
| `task` | 이 시점에는 생성하지 않는다. task 생성은 Fleet Manager 담당이다. |

Fleet Manager 이벤트:

```json
{
  "event": "ORDER_CREATED",
  "order_id": 1,
  "order_no": "ORD-0001"
}
```

---

### 2-2. Fleet Manager가 주문을 받아 task를 만든다

Fleet Manager는 이벤트를 받거나 시작 시 snapshot을 조회한다.

```http
GET /api/fleet/snapshot
WS /api/fleet/ws/events
```

Fleet Manager 처리:

| 순서 | 처리 |
|---|---|
| 1 | `ORDER_WAIT` 주문을 확인한다. |
| 2 | 사용 가능한 `robot_unit`을 고른다. |
| 3 | 주문의 `order_item` 목록을 읽는다. |
| 4 | 상품 보관 위치와 PICKY 현재 위치를 기준으로 방문 순서를 계산한다. |
| 5 | 계산된 순서대로 `sequence_no`를 부여한다. |
| 6 | PICKY/COBOT에 배정할 task 목록을 생성한다. |
| 7 | `POST /api/fleet/tasks/bulk`로 Control Server에 task를 저장한다. |
| 8 | 필요하면 `PATCH /api/fleet/orders/{order_id}`로 `assigned_unit_id`를 기록한다. |

상품별 task 생성 규칙:

| 순서 | task_type | 담당 로봇 | order_item_id | 의미 |
|---|---|---|---|---|
| 반복 | `MOVE_TO_PRODUCT` | PICKY | 있음 | 상품 보관 구역으로 이동 |
| 반복 | `SORTING_AND_LOAD` | COBOT | 있음 | 상품 선별 후 PICKY에 상차 |
| 마지막 | `MOVE_TO_PICKUP` | PICKY | 없음 | 픽업존으로 이동 |
| 마지막 | `INSPECTION` | COBOT | 없음 | 주문 전체 상품 검수 |
| 마지막 | `UNLOAD` | COBOT | 없음 | 픽업 슬롯에 하차 |

예시:

```json
{
  "tasks": [
    {
      "order_id": 1,
      "order_item_id": 2,
      "sequence_no": 1,
      "assigned_robot_name": "PICKY1",
      "task_type": "MOVE_TO_PRODUCT",
      "status": "ASSIGNED",
      "priority": 2,
      "source_zone_id": 1,
      "target_zone_id": 5
    },
    {
      "order_id": 1,
      "order_item_id": 2,
      "sequence_no": 2,
      "assigned_robot_name": "COBOT1",
      "task_type": "SORTING_AND_LOAD",
      "status": "QUEUED",
      "priority": 2,
      "source_zone_id": 5,
      "target_zone_id": 1
    }
  ]
}
```

DB 변화:

| 테이블 | 변화 |
|---|---|
| `orders` | `assigned_unit_id` 기록 가능 |
| `task` | Fleet Manager가 만든 task 목록 저장 |
| `robot` | Fleet Manager가 작업을 시작시키면 `BUSY`와 세부 state로 갱신 |

---

### 2-3. PICKY가 상품 위치로 이동한다

Fleet Manager가 PICKY에 이동 명령을 내린 뒤 Control Server에 상태를 보고한다.

```http
PATCH /api/fleet/robots/PICKY1
```

요청:

```json
{
  "robot_status": "BUSY",
  "picky_state": "MOVING_TO_PRODUCT",
  "current_task_id": 1,
  "battery_level": 91,
  "pos_x": 0.32,
  "pos_y": 0.80,
  "pos_theta": 0.0
}
```

task 시작 보고:

```http
PATCH /api/fleet/tasks/1
```

```json
{
  "current_status": "ASSIGNED",
  "status": "RUNNING",
  "result_message": "상품 보관 위치로 이동 중"
}
```

도착 후 완료 보고:

```json
{
  "current_status": "RUNNING",
  "status": "SUCCESS",
  "result_message": "상품 보관 위치 도착"
}
```

DB 변화:

| 테이블 | 변화 |
|---|---|
| `task` | `ASSIGNED -> RUNNING -> SUCCESS` |
| `task_event` | 상태 변경 이력 기록 |
| `robot` | PICKY 상태/위치/current_task 갱신 |
| `orders` | 실행 중 task에 따라 `SORTING` 또는 `DELIVERING`으로 갱신 |

---

### 2-4. COBOT이 상품을 선별하고 상차한다

Fleet Manager가 같은 `robot_unit`의 COBOT에 선별/상차 명령을 내린다.

COBOT 상태 예:

```json
{
  "robot_status": "BUSY",
  "cobot_state": "SORTING",
  "current_task_id": 2
}
```

상차 중 상태 예:

```json
{
  "robot_status": "BUSY",
  "cobot_state": "LOADING",
  "current_task_id": 2
}
```

task 완료 보고:

```http
PATCH /api/fleet/tasks/2
```

```json
{
  "current_status": "RUNNING",
  "status": "SUCCESS",
  "result_message": "우유 선별/상차 완료"
}
```

완료 후 처리:

| 테이블 | 변화 |
|---|---|
| `task` | `SORTING_AND_LOAD` 성공 |
| `order_item` | 해당 상품 `status="SORTED"` |
| `robot` | COBOT `IDLE`, `cobot_state="STANDBY"` |

상품이 더 있으면 Fleet Manager가 다음 `MOVE_TO_PRODUCT` / `SORTING_AND_LOAD`를 이어서 실행한다.

---

### 2-5. PICKY가 픽업존으로 이동한다

모든 상품 상차가 끝나면 Fleet Manager는 PICKY에 픽업존 이동 task를 실행시킨다.

| task_type | 담당 |
|---|---|
| `MOVE_TO_PICKUP` | PICKY |

PICKY 상태:

```json
{
  "robot_status": "BUSY",
  "picky_state": "MOVING_TO_PICKUP",
  "current_task_id": 7
}
```

완료 후 PICKY는 COBOT 작업 대기 상태가 될 수 있다.

```json
{
  "robot_status": "BUSY",
  "picky_state": "WAITING_FOR_COBOT"
}
```

---

### 2-6. COBOT이 검수하고 하차한다

검수 시작:

| task_type | 담당 | 처리 |
|---|---|---|
| `INSPECTION` | COBOT | 주문 전체 상품 검수 |

`INSPECTION`이 `RUNNING`이 되면 Control Server는 빈 픽업 슬롯을 예약한다.

| 테이블 | 변화 |
|---|---|
| `pickup_slot` | `EMPTY -> RESERVED` |
| `orders` | `pickup_slot_id` 기록 |
| `orders` | `status="INSPECTING"` |
| `robot` | COBOT `BUSY`, `cobot_state="INSPECTING"` |

검수 성공:

| 테이블 | 변화 |
|---|---|
| `order_item` | 주문 내 상품 `status="INSPECTED"` |
| `task` | `INSPECTION.status="SUCCESS"` |

하차 시작/완료:

| task_type | 담당 | 처리 |
|---|---|---|
| `UNLOAD` | COBOT | 픽업 슬롯에 상품 하차 |

`UNLOAD` 성공 후:

| 테이블 | 변화 |
|---|---|
| `pickup_slot` | `RESERVED -> OCCUPIED` |
| `orders` | `status="PICKUP_READY"` |
| `robot` | COBOT `IDLE`, `cobot_state="STANDBY"` |

---

### 2-7. 고객이 수령 완료 처리한다

호출:

```http
POST /api/orders/{order_id}/complete
```

처리:

| 테이블 | 변화 |
|---|---|
| `orders` | `status="COMPLETED"` |
| `pickup_slot` | `OCCUPIED -> EMPTY` |

---

## 3. 입고 명령 시나리오

입고는 고객 주문과 다르게 `orders` / `order_item`을 만들지 않는다.
입고 대상 상품, 요청 수량, 실제 감지 수량, 재고 반영 수량은 `stocking_item`에 둔다.
Fleet Manager가 생성하는 입고 task는 `stocking_item_id`로 입고 대상과 연결한다.

### 3-1. 관리자가 자연어로 입고 명령을 입력한다

호출:

```http
POST /api/admin/llm/messages
```

요청:

```json
{
  "message": "우유 5개 입고해줘"
}
```

현재 상태:

- UI와 API는 남겨져 있다.
- `llm_client.py`는 담당자 구현 대기용 stub이다.
- 담당자가 LLM parser를 구현하면 `action="STOCKING"` 응답을 만들고 Fleet Manager 이벤트로 연결한다.

현재 stub 응답:

```json
{
  "result": "ok",
  "message": "LLM 명령 파싱은 아직 연결 대기 상태입니다. 담당 모듈에서 구현해주세요.",
  "action": "CHAT",
  "provider": "stub"
}
```

향후 입고 응답 예:

```json
{
  "result": "ok",
  "message": "우유 5개 입고 명령을 인식했습니다.",
  "action": "STOCKING",
  "product_id": 1,
  "product_name": "우유",
  "requested_quantity": 5,
  "stocking_policy": "REQUESTED_QUANTITY",
  "provider": "llm"
}
```

Control Server가 Fleet Manager에 보낼 이벤트:

```json
{
  "event": "STOCKING_COMMAND",
  "message": "우유 5개 입고해줘",
  "command": {
    "action": "STOCKING",
    "product_id": 1,
    "product_name": "우유",
    "requested_quantity": 5,
    "stocking_policy": "REQUESTED_QUANTITY"
  }
}
```

---

### 3-2. Fleet Manager가 입고 task를 만든다

입고 task 흐름:

| 순서 | task_type | 담당 로봇 | 의미 |
|---|---|---|---|
| 1 | `MOVE_TO_STOCK` | PICKY | 입고존으로 이동 |
| 2 | `STOCKING_PICK` | COBOT | 입고존에서 대상 상품을 선별하고 PICKY에 상차 |
| 3 | `MOVE_TO_STORAGE` | PICKY | 대상 상품 보관 구역으로 이동 |
| 4 | `STOCKING_PLACE` | COBOT | 상품을 보관 위치에 적재 |

수량 정책:

| 조건 | stocking_item |
|---|---|
| 명령에 수량 있음 | `requested_quantity`, `stocking_policy="REQUESTED_QUANTITY"` |
| 명령에 수량 없음 | `requested_quantity=null`, `stocking_policy="ALL_DETECTED"` |
| 실제 감지 수량 | `detected_quantity` |
| 최종 재고 증가량 | `stock_delta` |

stocking_item 생성 예:

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

---

### 3-3. 입고 완료 후 재고를 반영한다

호출:

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

처리:

| 테이블 | 변화 |
|---|---|
| `task` | 해당 입고 task `SUCCESS` |
| `stocking_item` | `detected_quantity`, `stock_delta`, `status="COMPLETED"` |
| `product` | `stock_qty += stock_delta` |
| `task_event` | `STOCKING_COMPLETED` 기록 |
| UI | 재고/작업 상태 실시간 갱신 |

---

## 4. 예외 시나리오

로봇/비전/시스템 예외는 Fleet Manager가 Control Server에 보고한다.

호출:

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

처리:

| 테이블 | 변화 |
|---|---|
| `exception_log` | 미처리 예외 생성 |
| `task` | 필요하면 Fleet Manager가 별도 `FAILED` 보고 |
| `robot` | 필요하면 Fleet Manager가 `ERROR` 또는 `EMERGENCY_STOP` 보고 |
| Admin UI | 예외 알림 표시 |

관리자가 예외를 처리하면:

```http
POST /api/admin/exceptions/{exception_id}/resolve
```

`exception_log.is_resolved=true`로 바뀐다.

---

## 5. 긴급 정지 / 재개

긴급 정지:

```http
POST /api/admin/emergency-stop
```

처리:

| 대상 | 변화 |
|---|---|
| 모든 robot | `robot_status="EMERGENCY_STOP"` |
| UI | 즉시 갱신 |

재개:

```http
POST /api/admin/resume
```

처리:

| 대상 | 변화 |
|---|---|
| `EMERGENCY_STOP` robot | `robot_status="IDLE"` |
| 세부 state | Fleet Manager가 실제 재개 시점에 다시 보고 |

---

## 6. 현재 seed 시나리오

`web/scripts/reset_demo_data.sh` 후 기본 데이터:

| 데이터 | 내용 |
|---|---|
| 상품 | 우유, 시리얼, 바나나 우유, 식빵, 투게더, 바나나 |
| 로봇 | PICKY1, COBOT1, PICKY2, COBOT2 |
| 로봇 세트 | PICKY_UNIT_1, PICKY_UNIT_2 |
| 구역 | 대기존 2개, 입고존 1개, 상품 주차존 6개, 상품 슬롯 6개, 픽업존 4개, 픽업 슬롯 4개 |
| 주문 | 0건 |
| 주문 상품 | 0건 |
| task | 0건 |
| 입고 item | 0건 |
| 예외 | 0건 |

시드 데이터의 의도:

| 화면 | 확인 가능 항목 |
|---|---|
| 관리자 홈 | 초기 로봇/재고/픽업 슬롯 상태 |
| 미니맵 | PICKY 위치만 표시 |
| 로봇 상태 | PICKY/COBOT 4대 상태와 세부 state |
| 작업 목록 | Fleet Manager가 생성한 task만 표시 |
| 예외 화면 | Fleet Manager가 보고한 예외만 표시 |

---

## 7. 상태 전이 요약

주문 상태:

```text
ORDER_RECEIVED
  -> ORDER_WAIT
  -> SORTING
  -> DELIVERING
  -> INSPECTING
  -> PICKUP_READY
  -> COMPLETED
```

실패 시:

```text
작업 실패 -> ERROR
```

PICKY 주요 state:

```text
STANDBY
  -> MOVING_TO_PRODUCT
  -> WAITING_FOR_COBOT
  -> MOVING_TO_PICKUP
  -> RETURNING
  -> CHARGING / STANDBY
```

입고 PICKY state:

```text
STANDBY
  -> MOVING_TO_STOCK
  -> WAITING_FOR_COBOT
  -> MOVING_TO_STORAGE
  -> WAITING_FOR_COBOT
  -> RETURNING
```

COBOT 주요 state:

```text
STANDBY
  -> SORTING
  -> LOADING
  -> STOWING_ARM
  -> STANDBY
```

검수/하차:

```text
STANDBY
  -> INSPECTING
  -> UNLOADING
  -> STOWING_ARM
  -> STANDBY
```

입고 COBOT state:

```text
STANDBY
  -> STOCKING_SORTING
  -> STOCKING_LOADING
  -> STOCKING_PLACING
  -> STOWING_ARM
  -> STANDBY
```

---

## 8. 빠른 확인 명령

서버 실행:

```bash
cd ~/just_pick_it
web/scripts/run.sh
```

DB 초기화:

```bash
cd ~/just_pick_it
web/scripts/reset_demo_data.sh
```

Fleet snapshot 확인:

```bash
curl http://localhost:8000/api/fleet/snapshot
```

PICKY1 상태 확인:

```bash
curl http://localhost:8000/api/fleet/robots/PICKY1
```

PICKY1 task 확인:

```bash
curl "http://localhost:8000/api/fleet/tasks?robot_name=PICKY1"
```
