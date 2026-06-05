# Fleet Manager API 스펙 (초안)

작성일: 2026-05-26
상태: 초안 (Phase 0 산출물). 관련 방향성은 `docs/Control_Service_통합_계획.md` 참고.

이 문서는 통합 후 **Fleet Manager가 외부(웹 프런트)에 노출하는 HTTP/REST + WebSocket API**의 초안이다.
통합 결정에 따라 DB는 Fleet Manager가 단독 소유하며, 웹은 DB에 직접 접근하지 않고 이 API만 호출한다.

```text
브라우저 ──HTTP/WS──> 웹 프런트(페이지 렌더링/정적) ──이 API(HTTP/WS)──> Fleet Manager ──> DB
```

---

## 0. 설계 원칙

1. **기존 경로 재사용.** 현재 웹이 DB에 대고 제공하던 엔드포인트 경로(`/api/orders`, `/api/admin/status` 등)를
   Fleet Manager가 그대로 제공한다. 웹은 호출 대상 host만 Fleet Manager로 바꾸면 되어 변경 비용이 작다.
2. **읽기와 명령 분리.** 조회는 `GET`, 상태를 바꾸는 동작은 `POST`/`PATCH`. 실시간 상태 흐름은 WebSocket.
3. **응답 데이터 형태 유지.** 현재 `status_service`/`schemas`가 만드는 JSON 구조를 그대로 유지해 웹 템플릿/JS 수정 최소화.
4. **로봇 제어 위임.** API 핸들러는 DB만 다루고, 로봇을 실제로 움직이는 동작(emergency 전파 등)은
   rclpy executor로 위임한다(HTTP 스레드에서 rclpy 직접 호출 금지, 통합 계획 2.3/3.4 참고).
5. **내부 동작은 API가 아니다.** 기존 `/api/fleet/*`(Fleet↔웹 브리지)는 통합 후 **사라진다.**
   task 생성, 상태 전이, 주문 감지 등은 Fleet Manager 내부 함수 호출로 처리되며 HTTP 표면에 노출하지 않는다.

### 공통 규약

- Base URL: `http://<fleet_manager_host>:<port>` (포트는 Phase에서 확정. 기존 관례상 8000 후보)
- Content-Type: `application/json`
- 인증: **현재 미구현(범위 외).** 단 관리자 명령 엔드포인트는 추후 인증/인가 필요. (TODO)
- 시각 표기: ISO 8601 문자열.
- 오류 형식 (FastAPI 기본):
  ```json
  { "detail": "에러 메시지" }
  ```
  | 상황 | status |
  |------|--------|
  | 정상 조회/명령 | 200 |
  | 생성 성공 | 201 |
  | 요청 값 오류(재고 부족 등) | 400 |
  | 대상 없음(상품/주문 미존재) | 404 |
  | 상태 충돌(이미 완료된 주문 등) | 409 (또는 400) |

### 상태 enum 참고 (`web/app/schemas.py` 기준)

- OrderStatus: `ORDER_RECEIVED, ORDER_WAIT, SORTING, DELIVERING, INSPECTING, PICKUP_READY, COMPLETED, ERROR`
- RobotStatus: `OFFLINE, IDLE, BUSY, CHARGING, EMERGENCY_STOP, ERROR`
- PickupSlotStatus: `EMPTY, RESERVED, OCCUPIED, BLOCKED`
- ExceptionType: `OBSTACLE_DETECTED, LOW_BATTERY, NAVIGATION_FAILED, HARDWARE_ERROR, TIMEOUT, SORTING_FAIL, INSPECTION_FAIL, HUMAN_DETECTED, SYSTEM_ERROR`
- (PickyState, CobotState, TaskType, TaskStatus, StockingPolicy 등은 schemas.py 정의를 그대로 따른다)

---

## 1. 조회 API (GET)

### 1.1 상품 목록

```
GET /api/products
```
응답 200: `ProductRead[]`
```json
[
  {
    "product_id": 1,
    "name": "콜라",
    "image_url": "/static/products/1.png",
    "stock_qty": 12,
    "stock_level": "OK",
    "storage_zone_id": 3,
    "storage_zone_name": "PRODUCT_SLOT_1",
    "storage_zone_pose": { "x": 1.0, "y": 0.5, "z": 0.0, "theta": 0.0 },
    "storage_location": "PRODUCT_SLOT_1"
  }
]
```

### 1.2 주문 목록 / 단건

```
GET /api/orders                # COMPLETED 제외, 최신순 50건
GET /api/orders/{order_id}
```
응답 200: `OrderRead` (단건) 또는 `OrderRead[]`
```json
{
  "order_id": 12,
  "order_no": "ORD-0012",
  "status": "SORTING",
  "priority": 2,
  "pickup_slot_id": 2,
  "pickup_slot_name": "PICKUP_SLOT_2",
  "assigned_unit_id": 1,
  "items": [
    { "item_id": 30, "product_id": 1, "product_name": "콜라",
      "image_url": "/static/products/1.png", "quantity": 2, "status": "WAITING" }
  ]
}
```
오류: 404 (주문 없음)

### 1.3 고객 상태 스냅샷

```
GET /api/customer/status
```
응답 200: 고객 화면용 요약(상품 가용성 + 진행 주문). `build_customer_status` 구조 유지.

### 1.4 관리자 상태 스냅샷

```
GET /api/admin/status
```
응답 200: 관제 화면 전체 스냅샷. `build_admin_status` 구조 유지. 주요 키:
```json
{
  "orders": [ /* OrderSummary[] (진행중 20건) */ ],
  "order_history": [ /* COMPLETED 50건 */ ],
  "robots": [ /* RobotSummary[]: robot_status, picky_state, cobot_state, battery_level, pos_x/y/theta, current_task */ ],
  "tasks": [ /* TaskSummary[]: 활성 + 최근 */ ],
  "products": [ /* ProductSummary[] */ ],
  "low_stock_count": 1,
  "pickup_slots": [ /* {slot_id, slot_name, status} */ ],
  "exceptions": [ /* 미해결 5건 */ ],
  "exception_history": [ /* 해결됨 100건 */ ],
  "unresolved_exception_count": 0
}
```

---

## 2. 명령 API (POST / PATCH)

웹이 직접 DB에 쓰지 못하므로, 아래 동작은 Fleet Manager에 위임한다.

### 2.1 주문 생성 (고객)

```
POST /api/orders
```
요청: `OrderCreate`
```json
{ "items": [ { "product_id": 1, "quantity": 2 }, { "product_id": 4, "quantity": 1 } ] }
```
처리: 재고 검증(부족 시 400) → 주문/주문항목 생성 → 재고 차감 → 주문 워크플로 진입 → 내부 상태 갱신 브로드캐스트.
응답 200: 생성된 `OrderRead`.
오류: 404(상품 없음), 400(재고 부족).

### 2.2 주문 픽업 완료 (고객)

```
POST /api/orders/{order_id}/complete
```
조건: 주문 status가 `PICKUP_READY`일 때만 허용.
응답 200: `OrderRead`. 오류: 404, 400(픽업 준비 상태 아님).

### 2.3 비상 정지 / 재개 (관리자)

```
POST /api/admin/emergency-stop
POST /api/admin/resume
```
처리:
- emergency-stop: 모든 로봇 `EMERGENCY_STOP`, RUNNING task `PAUSED` + task_event 기록. **로봇 EmergencyControl 전파는 executor로 위임.**
- resume: PAUSED task `RUNNING` 복귀 또는 로봇 `IDLE`. 재개 전파도 executor 위임.
- 기존 `/api/fleet/ws/events` 우회 경로는 불필요(같은 프로세스이므로 내부 호출로 대체).

응답 200:
```json
{ "status": "ok", "paused_task_ids": [5, 6] }     // emergency-stop
{ "status": "ok", "resumed_task_ids": [5] }        // resume
```

### 2.4 상품 관리 (관리자)

```
POST  /api/admin/products                  # 생성, 201
PATCH /api/admin/products/{product_id}      # 전체 수정
PATCH /api/admin/products/{product_id}/stock # 재고만 수정
```
요청: `ProductCreate` / `ProductUpdate` / `ProductStockUpdate`.
```json
// ProductCreate
{ "name": "사이다", "stock_qty": 10, "storage_zone_id": 3, "image_url": null }
// ProductStockUpdate
{ "stock_qty": 20 }
```
응답: `ProductRead`. 오류: 404(상품/zone 없음), 400(보관 zone 누락).

### 2.5 픽업 슬롯 생성 (관리자)

```
POST /api/admin/pickup-slots
```
요청: `AdminPickupSlotCreate` `{ "slot_name": "PICKUP_SLOT_5", "status": "EMPTY" }`
응답 200: `{ "status": "ok" }`

### 2.6 LLM 명령 (관리자)

```
POST /api/admin/llm/messages
```
요청: `AdminLlmMessageCreate` `{ "message": "콜라 20개 입고해줘" }`
처리: 자연어 파싱(현재 stub) → STOCKING이면 stocking_item 생성 → 입고 워크플로 진입.
응답 200: `AdminLlmMessageRead` (action, stocking_item_id 등 포함).
참고: 통합 후 stocking_item 감지/처리는 내부 함수로 직접 연결(기존 STOCKING_COMMAND 이벤트 우회 불필요).

### 2.7 예외 해결 처리 (관리자)

```
POST /api/admin/exceptions/{exception_id}/resolve
```
응답 200: `{ "status": "ok" }`. 오류: 404.

---

## 3. 실시간 WebSocket

상태 변화(로봇 위치/상태, 주문 진행, task 전이)를 화면에 흘려보내는 통로.
기존 `realtime.py`의 브로드캐스트 역할이 Fleet Manager로 이동한다.

### 3.1 고객 / 관리자 상태 스트림

```
WS /api/customer/ws/status
WS /api/admin/ws/status
```
- 연결 시 최초 스냅샷 1회 전송.
- 상태 변경 이벤트가 생기면 갱신된 스냅샷을 push.
- 메시지 본문은 각각 `build_customer_status` / `build_admin_status`와 동일 구조(2.x GET 응답과 동형).

### 3.2 push 트리거 (내부)

다음 내부 이벤트가 발생하면 해당 스냅샷을 재송출한다(이전 `broadcast_all_status` 대체):
- 주문 생성/완료, task 생성/상태 전이, 로봇 상태/위치 변경, 재고 변경, 예외 발생/해결.

> 설계 메모: 통합 전에는 robot/task 상태가 HTTP PATCH로 들어와 broadcast가 트리거됐다.
> 통합 후에는 TaskManager 등 내부 모듈이 DB를 갱신하는 지점에서 직접 push를 호출하므로, WebSocket 우회가 사라지고 지연이 준다.

---

## 4. 통합으로 사라지는 표면 (기존 `/api/fleet/*`)

아래는 **Fleet Manager가 웹의 DB에 접근하려고** 두었던 브리지였다. 통합 후 DB가 Fleet 내부에 있으므로 HTTP가 아닌 내부 함수 호출로 대체되어 **API 표면에서 제거**한다.

- `GET /api/fleet/snapshot`, `/zones`, `/orders`, `/pickup-slots`, `/tasks`, `/stocking-items`
- `POST /api/fleet/tasks/bulk`, `/tasks/{id}/events`, `/exceptions`, `/stocking/complete`
- `PATCH /api/fleet/orders/{id}`, `/tasks/{id}`, `/robots/{id}`, `/pickup-slots/{id}`, `/stocking-items/{id}`
- `POST /api/fleet/assignments/run`, `/orders/{id}/assign-pickup-slot`
- `WS /api/fleet/ws/events` (emergency/resume 전파 우회 경로)

이들의 비즈니스 로직(`workflow_service`, `status_service`, `stocking_service`)은 폐기가 아니라 Fleet Manager 내부로 이사해 재사용한다(통합 계획 3장).

---

## 5. 미결 / 후속 결정

- [ ] API 서버 포트와 호스트 바인딩 확정.
- [ ] 관리자 엔드포인트 인증/인가 방식.
- [ ] 웹 프런트가 브라우저 요청을 **프록시**할지, 브라우저가 일부 GET/WS를 Fleet에 **직접** 연결할지 (CORS 영향).
- [ ] 상태 충돌 시 409 vs 400 정책 통일.
- [ ] WebSocket push를 전체 스냅샷 재전송 방식으로 유지할지, 변경분(delta)만 보낼지.
