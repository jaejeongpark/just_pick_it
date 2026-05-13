# Just Pick It Workflow

이 문서는 시나리오별로 무엇이 호출되고, 어떤 테이블이 어떤 형태로 바뀌는지 읽기 위한 기준 문서입니다.

현재 기준:

- Control Server가 task를 생성하고 배정한다.
- Robot Control Node는 DB를 직접 보지 않고 Control Server API만 호출한다.
- Cobot task는 생성 시점에 고정 로봇으로 배정한다.
- AMR task는 주문 생성 시점에 바로 배정하지 않고, AMR이 대기 상태와 배터리를 보고할 때 배정한다.
- 주문 task priority는 `2`, 순찰 task priority는 `1`이다.
- Robot Control Node는 배정된 `ASSIGNED` task를 조회하고, 시작/완료/실패를 보고한다.

---

## 1. 기본 주문 시나리오

### 1-1. 고객이 주문한다

호출:

```text
POST /api/orders

request
{
  "items": [
    {"product_id": 1, "quantity": 1}
  ]
}
```

Control Server 처리:

| 순서 | 처리 |
|---|---|
| 1 | product row를 lock하고 재고를 확인한다. |
| 2 | `orders` row를 생성한다. |
| 3 | 주문번호 `ORD-{order_id:04d}`를 생성한다. |
| 4 | 주문 수량만큼 `product.stock_qty`를 차감한다. |
| 5 | `order_item` row를 생성한다. |
| 6 | 기본 task 6개를 생성한다. |
| 7 | Cobot task는 고정 로봇을 미리 채운다. |
| 8 | AMR task는 AMR 상태 보고 전까지 비워둔다. |

DB 변화:

| 테이블 | row 형태 |
|---|---|
| `orders` | `{order_id: 1, order_no: "ORD-0001", status: "ORDER_WAIT", priority: 2, pickup_slot_id: null}` |
| `order_item` | `{order_id: 1, product_id: 1, quantity: 1, status: "WAITING"}` |
| `product` | 주문 수량만큼 `stock_qty` 감소 |

생성되는 task:

| 순서 | task_type | status | priority | assigned_robot_id |
|---|---|---|---|---|
| 1 | `STANDBY_LOAD` | `QUEUED` | 2 | null |
| 2 | `SORTING` | `QUEUED` | 2 | `SORTING_COBOT` |
| 3 | `LOAD` | `QUEUED` | 2 | `SORTING_COBOT` |
| 4 | `STANDBY_UNLOAD` | `QUEUED` | 2 | null |
| 5 | `INSPECTION` | `QUEUED` | 2 | `INSPECTION_COBOT` |
| 6 | `UNLOAD` | `QUEUED` | 2 | `INSPECTION_COBOT` |

### 1-2. AMR이 상차 대기존 복귀/대기 상태를 보고한다

호출:

```text
PATCH /api/fleet/robots/AMR_1

request
{
  "status": "STANDBY",
  "current_task_id": null,
  "battery_level": 90,
  "pos_x": 0.9,
  "pos_y": 0.8,
  "pos_theta": 0.0
}
```

Control Server 처리:

| 조건 | 처리 |
|---|---|
| AMR 상태가 `IDLE` 또는 `STANDBY` | 배정 가능 |
| battery_level >= 20 | 배정 가능 |
| 진행 중 task 없음 | 배정 가능 |
| priority 2 주문 task 존재 | 순찰보다 먼저 주문 task 배정 |

DB 변화:

| 테이블 | 변화 |
|---|---|
| `robot` | AMR 상태/위치/배터리 갱신 |
| `task` | `STANDBY_LOAD.status`: `QUEUED` -> `ASSIGNED` |
| `task` | `STANDBY_LOAD.assigned_robot_id`: null -> `AMR_1` |
| `task` | 같은 주문의 `STANDBY_UNLOAD.assigned_robot_id`: null -> `AMR_1` |
| `task_event` | `TASK_ASSIGNED` 기록 |

### 1-3. AMR이 배정 task를 조회하고 실행한다

호출:

```text
GET /api/fleet/tasks?robot_id=AMR_1&status=ASSIGNED
```

응답 형태:

```text
[
  {
    "task_id": 1,
    "order_id": 1,
    "order_no": "ORD-0001",
    "assigned_robot_id": "AMR_1",
    "task_type": "STANDBY_LOAD",
    "status": "ASSIGNED",
    "priority": 2,
    "target_zone_name": "STANDBY_LOADING_ZONE",
    "target_zone_pose": {"x": 0.85, "y": 0.4, "z": 0.0, "theta": 0.0}
  }
]
```

시작 보고:

```text
PATCH /api/fleet/tasks/1

request
{
  "status": "RUNNING",
  "result_message": "moving to standby loading zone"
}
```

DB 변화:

| 테이블 | 변화 |
|---|---|
| `task` | `STANDBY_LOAD.status`: `ASSIGNED` -> `RUNNING` |
| `robot` | `AMR_1.current_task_id=1`, `AMR_1.status="STANDBY"` |
| `orders` | `ORDER_WAIT` 유지 |

완료 보고:

```text
PATCH /api/fleet/tasks/1

request
{
  "status": "SUCCESS",
  "result_message": "standby load completed"
}
```

DB 변화:

| 테이블 | 변화 |
|---|---|
| `task` | `STANDBY_LOAD.status`: `RUNNING` -> `SUCCESS` |
| `robot` | `AMR_1.current_task_id=null`, `AMR_1.status="IDLE"` |
| `task` | 다음 task `SORTING.status`: `QUEUED` -> `ASSIGNED` |

### 1-4. SORTING_COBOT이 선별/상차를 진행한다

조회:

```text
GET /api/fleet/tasks?robot_id=SORTING_COBOT&status=ASSIGNED
```

`SORTING` 시작/완료:

```text
PATCH /api/fleet/tasks/{sorting_task_id} {"status":"RUNNING"}
PATCH /api/fleet/tasks/{sorting_task_id} {"status":"SUCCESS"}
```

완료 후 DB 변화:

| 테이블 | 변화 |
|---|---|
| `task` | `SORTING.status`: `SUCCESS` |
| `task` | 다음 task `LOAD.status`: `QUEUED` -> `ASSIGNED` |

`LOAD` 시작/완료:

```text
PATCH /api/fleet/tasks/{load_task_id} {"status":"RUNNING"}
PATCH /api/fleet/tasks/{load_task_id} {"status":"SUCCESS"}
```

완료 후 DB 변화:

| 테이블 | 변화 |
|---|---|
| `task` | `LOAD.status`: `SUCCESS` |
| `order_item` | 해당 주문 item `status="SORTED"` |
| `task` | 예약된 `STANDBY_UNLOAD.status`: `QUEUED` -> `ASSIGNED` |

### 1-5. AMR이 하차 대기 task를 수행한다

조회:

```text
GET /api/fleet/tasks?robot_id=AMR_1&status=ASSIGNED
```

실행:

```text
PATCH /api/fleet/tasks/{standby_unload_task_id} {"status":"RUNNING"}
PATCH /api/fleet/tasks/{standby_unload_task_id} {"status":"SUCCESS"}
```

DB 변화:

| 테이블 | 변화 |
|---|---|
| `task` | `STANDBY_UNLOAD.status`: `SUCCESS` |
| `robot` | `AMR_1.current_task_id=null`, `AMR_1.status="IDLE"` |
| `task` | 다음 task `INSPECTION.status`: `QUEUED` -> `ASSIGNED` |

AMR은 이후 상차 대기존으로 복귀한 뒤 다시 `PATCH /api/fleet/robots/{robot_id}`로 상태와 배터리를 보고한다. 그 시점에 다음 주문이 있으면 Control Server가 그 AMR에 다음 주문을 배정한다.

### 1-6. INSPECTION_COBOT이 검수/하차를 진행한다

`INSPECTION` 시작 시:

```text
PATCH /api/fleet/tasks/{inspection_task_id} {"status":"RUNNING"}
```

DB 변화:

| 테이블 | 변화 |
|---|---|
| `orders` | `status="INSPECTING"` |
| `pickup_slot` | 빈 슬롯 `EMPTY` -> `RESERVED` |
| `orders` | `pickup_slot_id` 저장 |

`INSPECTION` 완료 시:

```text
PATCH /api/fleet/tasks/{inspection_task_id} {"status":"SUCCESS"}
```

DB 변화:

| 테이블 | 변화 |
|---|---|
| `order_item` | 해당 주문 item `status="INSPECTED"` |
| `task` | 다음 task `UNLOAD.status`: `QUEUED` -> `ASSIGNED` |

`UNLOAD` 완료 시:

```text
PATCH /api/fleet/tasks/{unload_task_id} {"status":"SUCCESS"}
```

DB 변화:

| 테이블 | 변화 |
|---|---|
| `pickup_slot` | `RESERVED` -> `OCCUPIED` |
| `orders` | `status="PICKUP_READY"` |

### 1-7. 고객이 수령 완료한다

호출:

```text
POST /api/orders/1/complete
```

DB 변화:

| 테이블 | 변화 |
|---|---|
| `orders` | `status="COMPLETED"` |
| `pickup_slot` | `OCCUPIED` -> `EMPTY` |

---

## 2. LLM 순찰 명령 시나리오

### 2-1. 관리자가 순찰 명령을 입력한다

호출:

```text
POST /api/admin/llm/messages

request
{
  "message": "A구역 순찰해줘"
}
```

Control Server 처리:

| 순서 | 처리 |
|---|---|
| 1 | LLM 또는 local parser가 `PATROL`, `A_ZONE`으로 해석한다. |
| 2 | `zone` 테이블에서 `A_ZONE`을 찾는다. |
| 3 | `PATROL` task를 생성한다. |
| 4 | 주문보다 낮은 priority `1`을 저장한다. |

DB 생성:

| 테이블 | row 형태 |
|---|---|
| `task` | `{task_type:"PATROL", status:"QUEUED", priority:1, assigned_robot_id:null, target_zone_id:A_ZONE}` |

### 2-2. AMR이 대기 상태를 보고하면 순찰이 배정된다

호출:

```text
PATCH /api/fleet/robots/AMR_2

request
{
  "status": "STANDBY",
  "current_task_id": null,
  "battery_level": 90
}
```

배정 규칙:

| 조건 | 처리 |
|---|---|
| priority 2 주문 task 있음 | 주문 task 먼저 배정 |
| priority 2 주문 task 없음 | priority 1 순찰 task 배정 |

DB 변화:

| 테이블 | 변화 |
|---|---|
| `task` | `PATROL.status`: `QUEUED` -> `ASSIGNED` |
| `task` | `PATROL.assigned_robot_id`: null -> `AMR_2` |

AMR 조회:

```text
GET /api/fleet/tasks?robot_id=AMR_2&status=ASSIGNED
```

이후 AMR은 `target_zone_pose`로 이동하고 `RUNNING`, `SUCCESS`를 보고한다.

---

## 3. 작업 중 HUMAN_DETECTED 시나리오

### 3-1. Vision Server가 사람 감지를 반환한다

Robot Control Node는 Vision Server를 직접 호출한다.

```text
AMR Robot Control Node -> Vision Server
{
  "task_id": 4,
  "robot_id": "AMR_1",
  "camera_frame": "<binary|base64>",
  "zone_name": "UNLOADING_ZONE"
}
```

Vision 응답:

```text
{
  "person_detected": true,
  "fire_detected": false,
  "confidence": 0.94
}
```

### 3-2. Control Server에 예외를 기록한다

호출:

```text
POST /api/fleet/exceptions

request
{
  "exception_type": "HUMAN_DETECTED",
  "robot_id": "AMR_1",
  "task_id": 4,
  "order_id": 1,
  "detail": "zone=UNLOADING_ZONE, confidence=0.94"
}
```

DB 생성:

| 테이블 | row 형태 |
|---|---|
| `exception_log` | `{exception_type:"HUMAN_DETECTED", robot_id:"AMR_1", task_id:4, order_id:1, is_resolved:false}` |

주의:

- `POST /api/fleet/exceptions`는 예외만 기록한다.
- 이 API 하나만으로 task, robot, order 상태가 자동 변경되지는 않는다.
- 실제 정지는 ROS2 안전 계층에서 즉시 수행하고, Control Server에는 별도로 상태를 보고한다.

정지 상태 보고 예:

```text
PATCH /api/fleet/robots/AMR_1
{
  "status": "EMERGENCY_STOP",
  "current_task_id": 4
}
```

작업을 실패 처리할 경우:

```text
PATCH /api/fleet/tasks/4
{
  "status": "FAILED",
  "result_message": "human detected"
}
```

DB 변화:

| 테이블 | 변화 |
|---|---|
| `task` | `status="FAILED"` |
| `orders` | `status="ERROR"` |

### 3-3. 관리자가 예외를 처리 완료한다

호출:

```text
POST /api/admin/exceptions/{exception_id}/resolve
```

DB 변화:

| 테이블 | 변화 |
|---|---|
| `exception_log` | `is_resolved=false` -> `true` |

---

## 4. 우선순위 기준

| task 종류 | priority | 설명 |
|---|---|---|
| 주문 task | 2 | 고객 주문 처리 우선 |
| 순찰 task | 1 | 대기 AMR이 있고 주문 task가 없을 때 수행 |

---

## 5. 핵심 테이블

| 테이블 | 역할 |
|---|---|
| `orders` | 주문 상태와 priority 저장 |
| `order_item` | 주문 상품 단위 상태 저장 |
| `task` | 주문/순찰/충전 등 실행 작업 저장 |
| `robot` | 로봇 상태, 위치, 배터리, current_task 저장 |
| `pickup_slot` | 픽업 슬롯 상태 저장 |
| `task_event` | task 상태 전이 이력 저장 |
| `exception_log` | 예외 발생/처리 이력 저장 |
