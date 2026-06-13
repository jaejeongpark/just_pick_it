# Web/Fleet API 사용 가이드

현재 구조에서 브라우저는 Web Gateway를 호출하고, Web Gateway가 Fleet API로 프록시한다.

```text
Browser -> http://localhost:8000/api/* -> http://localhost:8100/api/*
```

## Base URLs

| 대상 | URL | 역할 |
|---|---|---|
| Web Gateway | `http://localhost:8000` | 화면 제공, `/api/*` 프록시 |
| Fleet API | `http://localhost:8100` | DB 접근, task/order/robot 상태 API |

## 호출 원칙

- UI JavaScript는 항상 `localhost:8000/api/*`를 호출한다.
- Fleet Manager 내부 로직은 `FleetRepository`를 직접 사용한다.
- Web Gateway는 DB를 직접 import하지 않는다.
- PICKY/COBOT은 Web Gateway를 호출하지 않고 ROS Action/Service를 통해 Fleet Manager와 통신한다.

## Customer UI

```http
GET /api/customer/status
GET /api/products
GET /api/orders
POST /api/orders
POST /api/orders/{order_id}/complete
WS  /api/customer/ws/status
```

주문 생성 예:

```bash
curl -X POST http://localhost:8000/api/orders \
  -H 'Content-Type: application/json' \
  -d '{"items":[{"product_id":1,"quantity":1}]}'
```

## Admin UI

```http
GET  /api/admin/status
POST /api/admin/products
PATCH /api/admin/products/{product_id}
PATCH /api/admin/products/{product_id}/stock
POST /api/admin/pickup-slots
POST /api/admin/exceptions/{exception_id}/resolve
POST /api/admin/emergency-stop
POST /api/admin/resume
WS   /api/admin/ws/status
```

## Fleet API 조회

```http
GET /api/fleet/snapshot
GET /api/fleet/zones
GET /api/fleet/tasks
GET /api/fleet/orders
GET /api/fleet/orders/{order_id}/tasks
GET /api/fleet/pickup-slots
```

## Fleet API 상태 쓰기

```http
POST   /api/fleet/tasks/bulk
PATCH  /api/fleet/tasks/{task_id}
DELETE /api/fleet/tasks/{task_id}
PATCH  /api/fleet/orders/{order_id}
PATCH  /api/fleet/robots/{robot_identifier}
PATCH  /api/fleet/pickup-slots/{slot_id}
```

이 API들은 외부 디버깅/관리용으로 열려 있지만, 일반 운영 흐름에서는 `TaskManager`와
`FleetRepository`가 프로세스 내부에서 직접 상태를 갱신한다.

## 고객 음성 주문 메시지

```http
POST /api/customer/llm/messages
```

요청 (텍스트 또는 data URL 형식 오디오):

```json
{
  "message": "수박 두개 식빵 한개 주문해줘"
}
```

응답 (다중 상품 주문 성공 시):

```json
{
  "result": "ok",
  "message": "주문이 생성되었습니다. order_id=1, order_no=ORD-0001",
  "action": "ORDER",
  "items": [
    {"product_id": 1, "product_name": "수박", "quantity": 2},
    {"product_id": 2, "product_name": "식빵", "quantity": 1}
  ],
  "provider": "gpt-4o-mini-transcribe + gpt-4o-mini",
  "order_id": 1,
  "order_no": "ORD-0001"
}
```

`llm_client.py`가 음성 → STT(`gpt-4o-mini-transcribe`) → 텍스트 파싱(`gpt-4o-mini`) 순서로 처리하고,
`action="ORDER"`이면 Web Gateway가 Fleet API `POST /api/orders`를 호출한다.

## WebSocket

브라우저는 Web Gateway에 연결한다.

```text
ws://localhost:8000/api/customer/ws/status
ws://localhost:8000/api/admin/ws/status
```

Web Gateway는 같은 경로를 Fleet API로 프록시한다.

```text
ws://localhost:8100/api/customer/ws/status
ws://localhost:8100/api/admin/ws/status
```

## Health check

```bash
curl http://localhost:8000/api/health/db
curl http://localhost:8100/api/health/db
```
