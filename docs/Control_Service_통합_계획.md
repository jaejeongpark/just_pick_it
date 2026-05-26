# Control Service → Fleet Manager 통합 계획

작성일: 2026-05-26

이 문서는 `System_Architecture ver_2.0`의 Software/System Architecture를 기준으로,
현재 별도 웹서버로 편성된 **Control Service**의 책임을 **ROS2 Fleet Manager**로 통합하고
**DB 소유권을 Fleet Manager로 옮기는** 작업의 방향성과 해야 할 일을 정리한다.

결론을 한쪽으로 확정하지 않고, 토폴로지와 DB 접근 방식 각각에 대해 두 안을 비교한 뒤
권고안과 단계별 작업 목록을 제시한다.

---

## 1. 현재 구조와 문제 정의

### 1.1 아키텍처 현황

```text
[User Interface]            [Server]                                 [Hardware]
Customer Browser  --HTTP-->  Control Server (FastAPI, web/)  --ROS2-->  Picky x 2
Admin Browser     --HTTP-->    │  ├ Web Service (페이지/REST)
                               │  ├ Control Service (비즈니스 로직)
                               │  └ DB (PostgreSQL, 단독 소유)
                               │
                               └ ROS2 Fleet Manager
                                   ├ Task Manager
                                   └ Traffic Manager
```

핵심 사실:

- **DB는 Control Server가 단독 소유**한다. SQLAlchemy `engine` / `SessionLocal` (`web/app/database.py`),
  ORM 모델(`web/app/models.py`), 비즈니스 로직(`web/app/services/*`)이 모두 웹 프로세스 안에 있다.
- **Fleet Manager(ROS2)는 DB에 직접 닿지 못한다.** 모든 읽기/쓰기를 Control Server의 HTTP API로 처리한다.
  접점은 `src/.../fleet_manager/control_server_client.py`의 `ControlServerClient` 하나로 모여 있다.
- Control Server는 `/api/fleet/*` 네임스페이스에 Fleet Manager 전용 브리지 엔드포인트 약 20여 개를 둔다
  (snapshot, zones, orders, tasks/bulk, tasks/{id}, robots/{id}, pickup-slots, exceptions, stocking-items 등).

### 1.2 비효율의 실체

| 동작 | 현재 비용 |
|------|-----------|
| 대기 주문/입고 감지 | Fleet Manager가 5초 주기 HTTP polling (`_poll_waiting_work_if_picky_idle`) |
| 주문 1건 정규화 (`get_order_work`) | `get_order_detail` + `list_products` + `list_zones` 등 **HTTP 3~4회** |
| 상태 변경 1건 (task/order/robot) | 매번 HTTP PATCH 왕복 |
| Emergency/Resume 전파 | Control Server → WebSocket(`/api/fleet/ws/events`) → Fleet Manager 수신 thread |

즉 Fleet Manager가 DB 한 줄을 읽거나 쓸 때마다 **네트워크 왕복 + JSON 직렬화/역직렬화 + FastAPI 라우팅**을 거친다.
실시간 제어 루프 안에서 이 chatty 통신은 지연·실패 지점을 늘리고, 같은 비즈니스 로직(상태 전이 규칙 등)을
HTTP 계약 양쪽에서 이중으로 신경 쓰게 만든다.

### 1.3 통합의 목표

1. Control Service가 하던 일(DB CRUD, 상태 전이 로직, 스냅샷 생성)을 Fleet Manager로 흡수한다.
2. **DB 쓰기 소유권을 Fleet Manager로 단일화**한다. Fleet Manager는 HTTP를 거치지 않고 DB에 직접 접근한다.
3. 고객/관리자 UI는 계속 동작해야 한다 (이 부분의 처리 방식이 아래 토폴로지 선택의 핵심이다).

---

## 2. 통합 토폴로지 — 두 안 비교

웹 UI(브라우저 페이지, 고객/관리자 WebSocket 상태 송출, LLM/Vision 연동 창구)를 어디에 둘지에 따라 두 갈래로 나뉜다.

### 안 A — FastAPI를 Fleet Manager 프로세스에 내장 (단일 프로세스)

ROS2 노드 프로세스 안에서 uvicorn/FastAPI를 별도 스레드로 함께 띄운다.
Control Service가 완전히 사라지고 Fleet Manager가 ROS2 + 웹 + DB를 모두 담당한다.

```text
Fleet Manager Process
├ rclpy executor (Task/Traffic/StateMonitor/Gateway)
├ FastAPI/uvicorn thread (Customer/Admin 페이지, REST, WS)
└ DB 접근 계층 (단일 소유)
```

- **장점**
  - 사용자가 말한 "전부 통합"에 가장 가깝다. 프로세스가 하나이므로 DB·상태를 메모리에서 공유한다.
  - 상태 변경 → UI 브로드캐스트가 인프로세스 호출로 즉시 일어난다. WebSocket 이벤트 우회 경로(`/api/fleet/ws/events`)가 불필요해진다.
  - 비즈니스 로직이 한 곳에만 존재한다.
- **단점 / 리스크**
  - rclpy executor와 asyncio(uvicorn) **두 이벤트 루프를 한 프로세스에서 공존**시켜야 한다. 스레드 경계·GIL·블로킹 호출 관리가 까다롭다.
  - 웹 트래픽(브라우저 다수 접속)과 실시간 제어가 자원을 공유한다. 무거운 페이지 렌더링이 제어 루프를 방해하지 않도록 격리가 필요하다.
  - 배포 단위가 커지고, 웹만 재시작/로봇만 재시작 같은 독립 운영이 어려워진다.

### 안 B — 웹은 얇은 read-only 프런트로 분리 유지

Fleet Manager가 DB를 단독 소유/쓰기하고, 웹은 **같은 DB를 읽어 화면만 렌더링**하는 얇은 프런트로 축소한다.
쓰기 경로의 HTTP(`/api/fleet/*` PATCH/POST)는 제거되지만 프로세스는 둘로 유지된다.

```text
Fleet Manager Process            Web Frontend Process
├ rclpy executor                 ├ FastAPI (페이지/조회 REST/WS)
└ DB 쓰기 + 읽기 (소유)   <----   └ DB 읽기 전용
        같은 PostgreSQL
```

- **장점**
  - 관심사 분리 유지: 제어 루프와 웹 서빙이 독립 프로세스라 장애·부하가 서로 격리된다.
  - 이벤트 루프 공존 문제 없음. 각 프로세스가 자기 런타임만 신경 쓴다.
  - 웹/로봇을 독립적으로 배포·재시작할 수 있다.
  - 마이그레이션이 점진적이다. `ControlServerClient`의 쓰기 호출부터 DB 직접 호출로 바꾸고, 읽기 화면은 나중에 옮길 수 있다.
- **단점 / 리스크**
  - **DB가 두 프로세스에서 동시 접근**된다(웹=읽기, Fleet=쓰기). 쓰기 소유권을 Fleet으로 못박아야 정합성이 깨지지 않는다.
  - 고객 주문 생성, 관리자 emergency-stop 등 **현재 웹이 직접 DB에 쓰는 동작**을 어떻게 처리할지 결정해야 한다
    (Fleet Manager로 명령 전달 후 Fleet이 DB에 쓰게 하거나, 명령 테이블/큐를 통해 위임).
  - "DB도 Fleet이 관리"라는 목표는 달성하지만, 웹이 별도 프로세스로 남아 "전부 통합"의 체감은 안 A보다 약하다.

### 토폴로지 권고

- 단기 안정성·점진적 이행을 중시하면 **안 B**가 현실적이다. 쓰기 경로를 먼저 DB 직접화하여 비효율을 즉시 제거하고,
  웹은 read-only로 격리해 리스크를 낮춘다.
- 장기적으로 "단일 백엔드"를 지향하고 운영 단순성을 원하면 **안 A**가 목표 그림에 더 부합한다.
- 절충안: **안 B로 먼저 이행(쓰기 DB 직접화 + 웹 read-only) → 안정화 후 안 A로 프로세스 합치기.**
  공용 DB 접근 계층을 패키지로 분리해 두면 두 단계 모두에서 재사용되므로 이 경로가 손실이 적다.

---

## 3. DB 접근 방식 — 두 안 비교

Fleet Manager가 DB에 직접 접근할 때 코드 자산을 어떻게 다룰지.

### 안 ① 기존 SQLAlchemy 자산을 공용 패키지로 추출해 재사용

`web/app/models.py`와 `web/app/services/*`(workflow/status/stocking/inventory/robot_runtime_policy)를
웹/Fleet 양쪽이 import하는 공용 패키지(예: `just_pick_it_db`)로 분리한다.

- **장점**
  - 검증된 ORM 모델과 상태 전이 로직(`workflow_service`의 `ORDER_STATUS_BY_RUNNING_TASK`, `PICKY_STATE_BY_TASK` 등)을 그대로 옮긴다. 재구현 버그 위험이 작다.
  - 비즈니스 규칙이 한 패키지에만 존재해 양쪽 동작이 자동으로 일치한다.
- **단점 / 리스크**
  - ROS2 패키지가 SQLAlchemy/psycopg 의존성을 갖게 된다 (colcon 빌드·rosdep·가상환경 정리 필요).
  - 공용 패키지가 FastAPI 세션 수명주기(`get_db` 제너레이터)에 묶여 있던 부분을 ROS2 타이머/콜백 컨텍스트에 맞게 세션 관리로 다듬어야 한다.

### 안 ② Fleet Manager 전용 DB 레이어 신설

ROS2 쪽에 별도 DB 접근 모듈(psycopg 또는 독립 SQLAlchemy)을 새로 만들고 필요한 쿼리만 이식한다.

- **장점**
  - Fleet Manager가 쓰는 쿼리만 가볍게 가져갈 수 있고, 웹 코드와의 결합이 없다.
  - 제어 루프에 맞춘 커넥션 풀/세션 전략을 자유롭게 설계할 수 있다.
- **단점 / 리스크**
  - 상태 전이 규칙·스냅샷 생성 로직이 웹과 Fleet 두 곳에 중복되어, 한쪽만 고치면 정합성이 깨진다.
  - 이미 검증된 로직을 재작성하므로 회귀 버그 위험이 크다.

### DB 접근 권고

- **안 ① (공용 패키지 추출)을 권고**한다. 핵심 자산은 `workflow_service`의 상태 전이 규칙과 `status_service`의 스냅샷 빌더인데,
  이를 중복시키면 통합의 이점(로직 단일화)이 사라진다.
- 단, `get_db` 같은 FastAPI 결합부는 공용 패키지에서 제거하고, 세션 팩토리만 노출하여
  웹(요청 수명주기)과 Fleet(타이머/콜백 수명주기)이 각자 세션을 열도록 한다.

---

## 4. 통합 대상 인벤토리

통합 시 "Control Service가 하던 일"을 분류한다.

### 4.1 Fleet Manager로 흡수할 것 (DB 쓰기·로직)

`/api/fleet/*`의 쓰기 엔드포인트와 대응 `ControlServerClient` 메서드 → DB 직접 호출로 대체.

| 현재 HTTP 엔드포인트 | ControlServerClient 메서드 | 흡수 후 |
|---|---|---|
| `POST /api/fleet/tasks/bulk` | `create_tasks_bulk` | DB INSERT (task) |
| `PATCH /api/fleet/tasks/{id}` | `update_task_status` | DB UPDATE + `apply_task_runtime_state` |
| `POST /api/fleet/tasks/{id}/events` | `create_task_event` | DB INSERT (task_event) + 상태 전이 |
| `PATCH /api/fleet/orders/{id}` | `update_order_status` | DB UPDATE (orders) |
| `PATCH /api/fleet/robots/{id}` | `update_robot_state` | DB UPDATE (robot) |
| `PATCH /api/fleet/pickup-slots/{id}` | `update_pickup_slot_status` | DB UPDATE (pickup_slot) |
| `POST /api/fleet/exceptions` | `create_exception` | DB INSERT (exception_log) |
| `PATCH /api/fleet/stocking-items/{id}` | `update_stocking_item` | DB UPDATE (stocking_item) |
| `POST /api/fleet/stocking/complete` | `complete_stocking` | DB 트랜잭션 (stock_qty + stocking_item) |
| `GET /api/fleet/snapshot`, `/zones`, `/orders`, `/products`, `/pickup-slots` 등 | 각 조회 메서드 | DB SELECT (status_service 재사용) |

흡수할 로직 자산:
- `web/app/services/workflow_service.py` — task↔order↔robot 상태 전이의 단일 진실 원천.
- `web/app/services/status_service.py` — admin/customer 스냅샷 빌더.
- `web/app/services/stocking_service.py`, `inventory_status.py`, `robot_runtime_policy.py`.

### 4.2 처리 방식을 결정해야 할 것 (웹이 직접 DB에 쓰는 동작)

- **고객 주문 생성** (`POST /api/orders`, `order_router.py`) — 외부(브라우저)에서 들어오는 쓰기.
- **관리자 동작** — emergency-stop / resume / 상품 등록·재고 수정 / pickup-slot 생성 / 입고 요청(`admin_router.py`).
- 안 A에서는 Fleet Manager 프로세스 내 FastAPI가 그대로 처리.
- 안 B에서는 (1) 웹이 명령만 ROS2로 전달하고 DB 쓰기는 Fleet이 수행, 또는 (2) 주문/명령 입력 테이블에 웹이 쓰고 Fleet이 폴링/알림으로 소비하는 방식 중 택일.

### 4.3 그대로 남는 것 (UI / 외부 연동)

- Customer/Admin HTML 페이지 (`page_router.py`, `templates/`).
- 고객/관리자 WebSocket 상태 송출 (`realtime.py`의 `admin_websockets`, `customer_websockets`).
- AI Server(LLM/Vision) 연동 창구 (`llm_client.py` — 현재 stub).

---

## 5. 단계별 작업 목록

아래는 절충 경로(안 B 먼저 → 추후 안 A) 기준의 작업 순서다. 안 ① DB 자산 추출을 전제로 한다.

### Phase 0 — 준비

- [ ] 토폴로지(A vs B)와 DB 접근 방식(① vs ②) 최종 결정. 이 문서의 권고는 "안 B 우선 + 안 ①".
- [ ] DB 동시 접근 정책 확정: **쓰기는 Fleet 단독, 웹은 읽기 전용**으로 명문화.
- [ ] 운영 흐름 회귀 테스트 시나리오 정의 (주문 1건 end-to-end, 입고 1건, emergency/resume).

### Phase 1 — 공용 DB 패키지 추출

- [ ] `web/app/models.py` + `services/*`를 공용 패키지(예: `src/just_pick_it/just_pick_it_db` 또는 pip 설치 가능 모듈)로 이동.
- [ ] FastAPI 결합부(`get_db` 제너레이터) 제거, 세션 팩토리(`SessionLocal`)와 순수 로직만 노출.
- [ ] 웹(`web/`)이 공용 패키지를 import하도록 리팩터링. **기존 동작 무변경 확인** (회귀 테스트 통과).
- [ ] ROS2 빌드에서 이 패키지 의존성 해결(rosdep/venv/`package.xml` 또는 `requirements`).

### Phase 2 — Fleet Manager 쓰기 경로 DB 직접화

- [ ] Fleet Manager에 DB 세션 관리 도입 (커넥션 풀, 타이머/콜백당 세션 수명).
- [ ] `ControlServerClient`의 **쓰기 메서드**(4.1 표)를 공용 로직 직접 호출로 대체.
      `ControlServerClient`는 인터페이스를 유지한 채 내부 구현만 HTTP→DB로 교체하면 `TaskManager` 변경을 최소화할 수 있다.
- [ ] 상태 전이는 반드시 공용 `workflow_service` 로직을 거치게 해 중복 규칙을 만들지 않는다.
- [ ] 트랜잭션 경계 점검: `stocking/complete`처럼 다중 테이블을 함께 갱신하는 동작은 단일 트랜잭션으로 처리.

### Phase 3 — Fleet Manager 읽기 경로 DB 직접화

- [ ] `get_snapshot/list_orders/list_zones/list_products/list_pickup_slots` 등 조회를 DB SELECT로 대체.
- [ ] 주문/입고 감지를 HTTP polling → DB polling (또는 PostgreSQL `LISTEN/NOTIFY` 기반 이벤트)으로 전환해 왕복 제거.
- [ ] `get_order_work`의 다회 HTTP 조회를 단일 트랜잭션 내 JOIN 조회로 통합.

### Phase 4 — 이벤트/명령 경로 정리

- [ ] Emergency/Resume: 안 B에서는 웹이 명령 테이블에 쓰고 Fleet이 소비하거나, 웹이 ROS2로 직접 트리거.
      `/api/fleet/ws/events` 우회 경로 제거 검토.
- [ ] 고객 주문·관리자 명령의 DB 쓰기 주체를 4.2 결정에 맞게 구현.
- [ ] 웹을 **read-only**로 전환하고 쓰기 코드 제거 또는 명령 위임으로 교체.

### Phase 5 — (선택) 단일 프로세스(안 A)로 합치기

- [ ] Fleet Manager 프로세스 안에 FastAPI/uvicorn 스레드 기동, rclpy executor와 이벤트 루프 공존 구조 설계.
- [ ] 상태 변경 → UI 브로드캐스트를 인프로세스 호출로 전환, WebSocket 우회 제거.
- [ ] 부하 격리(웹 트래픽이 제어 루프를 방해하지 않도록) 검증.

### Phase 6 — 정리

- [ ] 사용되지 않게 된 `/api/fleet/*` 엔드포인트와 `ControlServerClient` HTTP 코드 제거.
- [ ] 아키텍처 문서(`docs/3_System_Architecture.pdf`) ver_3.0 갱신: Control Server 박스 제거 또는 read-only 프런트로 표기.
- [ ] `TASK_PLAN.md` / `README.md`의 구조 설명 갱신.

---

## 6. 리스크와 점검 포인트

| 항목 | 내용 |
|------|------|
| DB 동시 접근 정합성 | 쓰기 소유권을 Fleet 단독으로 못박지 않으면 race/덮어쓰기 위험. 웹은 읽기 전용 커넥션 권한으로 강제 가능. |
| 로직 중복 | DB 접근 방식 안 ②를 택하면 상태 전이 규칙이 이중화됨. 안 ① 권고 이유. |
| 이벤트 루프 공존(안 A) | rclpy + asyncio 동시 운용은 블로킹/스레드 안전성 함정이 많음. Phase 5에서 충분한 검증 필요. |
| 트랜잭션 경계 | HTTP 시절 엔드포인트 단위로 암묵 보장되던 원자성을 DB 직접화 후 명시적 트랜잭션으로 재현해야 함. |
| 마이그레이션 중 이중 경로 | Phase 2~3 동안 일부는 HTTP, 일부는 DB 직접일 수 있음. `ControlServerClient` 인터페이스 유지로 전환 충격을 흡수. |
| 단일 장애점 | 통합 후 Fleet Manager가 제어·DB·(안 A면)웹까지 담당 → 장애 시 영향 범위가 커짐. 모니터링/재시작 전략 필요. |

---

## 7. 요약 권고

1. **토폴로지**: 안 B(웹 read-only 분리)로 먼저 이행해 비효율을 제거하고, 안정화 후 필요 시 안 A(단일 프로세스)로 합친다.
2. **DB 접근**: 안 ①(기존 SQLAlchemy 자산을 공용 패키지로 추출)로 로직 단일화를 유지한다.
3. **쓰기 소유권**: DB 쓰기는 Fleet Manager 단독, 웹은 읽기 전용.
4. **이행 순서**: 공용 패키지 추출 → 쓰기 DB 직접화 → 읽기 DB 직접화 → 이벤트/명령 정리 → (선택)프로세스 통합 → HTTP 잔재 제거.
