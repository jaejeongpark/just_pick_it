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

## LLM 진열 명령

```http
POST /api/admin/llm/messages
```

요청:

```json
{
  "message": "생수 3개 진열"
}
```

현재 `llm_client.py`는 stub이다. 담당자가 `action="DISPLAY"` 형태로 파싱하면
Web Gateway가 Fleet API `POST /api/admin/display-items`를 호출한다.

직접 진열 요청 생성:

```bash
curl -X POST http://localhost:8000/api/admin/display-items \
  -H 'Content-Type: application/json' \
  -d '{"product_id":1,"requested_quantity":3,"display_policy":"REQUESTED_QUANTITY"}'
```

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
