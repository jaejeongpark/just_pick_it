# Control Service → Fleet Manager 통합 계획

작성일: 2026-05-26
갱신일: 2026-05-27 (Fleet API/Web Gateway 전환 및 실행 기준 반영)

이 문서는 `System_Architecture ver_2.0`의 Software/System Architecture를 기준으로,
현재 별도 웹서버로 편성된 **Control Service**의 책임을 **ROS2 Fleet Manager**로 통합하고
**DB 소유권을 Fleet Manager로 옮기는** 작업의 방향성과 해야 할 일을 정리한다.

---

## 0. 확정된 방향 (요약)

- **DB에 직접 접근하는 주체는 Fleet Manager 하나뿐이다** (읽기·쓰기 모두). DB 커넥션은 Fleet Manager 프로세스에만 존재한다.
- **웹은 DB에 직접 연결하지 않는다.** 웹은 화면을 그리는 프레젠테이션 클라이언트이며, **읽기·쓰기 모두 Fleet Manager를 거친다.** (DB를 직접 읽지도 쓰지도 않음)
- 웹은 별도 프로세스로 유지하되 **DB 접속 권한 자체를 갖지 않는다.** 조회는 Fleet Manager에 요청하고, 쓰기 동작은 Fleet Manager에 명령으로 위임한다.
- **웹 ↔ Fleet Manager 전달 방식은 HTTP/REST API + WebSocket으로 확정**한다. Fleet Manager가 API 서버를 열고, 웹은 순수 HTTP/WS 클라이언트가 된다 (조회=GET, 명령=POST, 실시간 상태=WebSocket push).
- DB 접근 코드는 **기존 SQLAlchemy 자산을 Fleet Manager 안으로 이사**시켜 재사용하고, **ORM을 유지**한다.

이 구조는 "API 서버만 DB를 만지고 프런트엔드는 DB에 직접 붙지 않는다"는 일반적인 웹 서비스 표준을 그대로 따른 것이다.

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
2. **DB 소유권을 Fleet Manager로 단일화**한다. Fleet Manager는 HTTP를 거치지 않고 DB에 직접 접근하고, **다른 어떤 프로세스도 DB에 직접 붙지 않는다.**
3. 고객/관리자 UI는 계속 동작해야 한다. 웹은 DB에 직접 접근하지 않는 프레젠테이션 클라이언트로 분리하고, 읽기·쓰기 모두 Fleet Manager를 경유한다.

---

## 2. 목표 토폴로지

### 2.1 확정 구조 — Fleet Manager 단독 DB 게이트웨이

```text
Fleet Manager Process (유일한 DB 소유자)
├ rclpy executor (Task/Traffic/StateMonitor/Gateway)
├ DB 접근 계층 (읽기 + 쓰기, 단독)
└ Fleet API (HTTP/REST + WebSocket)
        │  (HTTP/REST + WebSocket)
        ▼
Web Frontend Process (프레젠테이션 전용)
├ Customer/Admin 페이지 렌더링
└ DB 접속 권한 없음. 읽기·쓰기 모두 Fleet Manager 경유
        │  PostgreSQL
        ▼
   DB ── Fleet Manager만 접속
```

- 웹은 별도 프로세스로 유지하되 **DB 연결 문자열·접속 권한을 갖지 않는다.**
- 쓰기(주문 생성, emergency-stop 등)는 웹이 Fleet Manager에 명령을 전달하고 Fleet Manager가 DB에 쓴다.
- 읽기(화면 표시)도 웹이 Fleet Manager에 조회를 요청한다. 즉 **읽기·쓰기 모두 Fleet Manager 경유**다.

### 2.2 이 구조의 장점

- **DB 동시 접근 정합성 문제가 원천적으로 사라진다.** 커넥션이 한 프로세스에만 있으므로 race/덮어쓰기 위험이 없다.
- 비즈니스 로직(상태 전이 규칙 등)이 Fleet Manager 한 곳에만 존재한다.
- 제어 루프와 웹 서빙이 별도 프로세스라 부하·장애가 서로 격리된다. 웹/로봇을 독립적으로 재시작할 수 있다.

### 2.3 웹↔Fleet 전달 방식 — HTTP/REST API + WebSocket (확정)

웹은 DB에 직접 못 붙으므로 Fleet Manager가 데이터를 내주는 통로가 필요하다.
**Fleet Manager가 HTTP/REST API 서버를 열고, 웹은 순수 HTTP/WS 클라이언트로 호출하는 방식으로 확정**한다.

```text
브라우저 ──HTTP/WS──> Web Gateway(페이지 + proxy) ──HTTP/WS──> Fleet Manager API ──> DB
```

상호작용은 두 성격으로 나뉜다.

| 성격 | 통로 | 예시 |
|------|------|------|
| 요청-응답 (조회) | `GET /api/...` | 주문/로봇/zone/재고/예외 목록 조회, 화면 로드 |
| 요청-응답 (명령) | `POST /api/...` | 주문 생성, emergency-stop/resume, 상품/재고 수정, 입고 요청 |
| 실시간 푸시 (스트림) | WebSocket | 로봇 위치·상태·주문 상태 변화 (현재 `realtime.py` 역할이 Fleet으로 이동) |

채택 이유:

- 웹 프런트 ↔ 백엔드 서비스 사이에서 **가장 표준적인 3계층 패턴**이다(브라우저 → 웹 → 백엔드 API → DB).
- 웹이 이미 FastAPI(HTTP)라 **변경 비용이 가장 작다.** "DB 직접 소유" → "Fleet API 호출"로만 바뀐다.
- 주문·명령(요청-응답)과 상태 푸시(WebSocket)를 하나의 친숙한 스택으로 모두 커버한다.
- 지금도 쓰던 HTTP를 **방향만 뒤집는 것**이라 학습 비용이 거의 없다.

대안과의 비교(메시지 큐, ROS2 네이티브)는 부록 성격으로 6장 리스크/메모에 남긴다.
로봇이 수십 대로 늘고 다수 서비스가 상태를 공유하게 되면 메시지 큐 재검토 여지가 있으나, 현재 2대 규모에선 과하다.

> 주의: 이 방식은 Fleet Manager 프로세스 안에서 **rclpy executor와 asyncio(API 서버)가 공존**해야 한다.
> uvicorn은 별도 스레드로 띄우고, HTTP 핸들러는 스레드별 DB 세션(`scoped_session`, 3.4)을 사용하며,
> 로봇을 실제로 움직이는 동작은 HTTP 스레드에서 rclpy를 직접 호출하지 말고 **executor로 위임(큐/내부 ROS2 호출)**한다.

---

## 3. DB 접근 방식 (확정 + 근거)

### 3.1 코드 자산: 기존 SQLAlchemy 자산을 Fleet Manager로 이사 (재사용)

`web/app/models.py`(테이블 정의)와 `web/app/services/*`(상태 전이·스냅샷 로직)은 **이미 동작하며 검증된 자산**이다.
통합은 "이 코드를 어디서 실행하느냐"를 바꾸는 일이지 로직을 새로 발명하는 일이 아니다. 새로 작성하면 같은 버그를 다시 만들 위험만 커진다.

- 웹이 더 이상 DB를 쓰지 않으므로, 원래 고려했던 "웹·Fleet 공용 패키지로 공유" 대신
  **DB 코드를 Fleet Manager 안으로 이사**시키면 된다. 웹에는 DB 코드가 0줄 남는다.
- 대안(전용 DB 레이어 신설/재작성)은 상태 전이 규칙이 중복·재구현되어 회귀 버그 위험이 커지므로 채택하지 않는다.

### 3.2 기술: ORM(SQLAlchemy) 유지

- 이 스키마는 order ↔ order_item ↔ product ↔ task ↔ robot 처럼 **테이블 간 관계가 많다.** 관계 조회/갱신은 ORM이 raw SQL보다 안전하고 읽기 쉽다.
- 이미 SQLAlchemy로 작성되어 있어 그대로 옮기면 된다.
- raw SQL은 성능이 특히 중요한 일부 무거운 쿼리에만 부분적으로 섞는다. 전체를 raw로 가지 않는다.

### 3.3 구조: DAO/Repository 계층을 한 겹 둔다

- `TaskManager` 같은 로직이 SQL·세션을 직접 만지지 않고, "주문 조회 / task 저장" 같은 함수만 부르게 한다.
- 지금 `ControlServerClient`가 HTTP를 감춰주던 역할을, **같은 인터페이스로 DB를 감춰주는 계층**으로 바꾸면
  `TaskManager` 변경을 최소화하면서 내부 구현만 HTTP → DB로 교체할 수 있다.

### 3.4 반드시 챙길 함정 — 스레드별 세션 관리

Fleet Manager는 `MultiThreadedExecutor`로 동작한다(여러 콜백이 **여러 스레드에서 동시에** 실행됨, `fleet_manager_node.py`).
그런데 **SQLAlchemy의 Session 객체는 스레드 안전하지 않다.** 하나의 Session을 여러 스레드가 공유하면 데이터가 깨진다.

표준 해법:

- **콜백/타이머마다 세션을 새로 열고 닫는다** (요청 단위로 세션을 쓰는 웹 `get_db`와 같은 발상).
- 또는 **`scoped_session`**(스레드마다 자동으로 다른 세션을 주는 SQLAlchemy 기능)을 사용한다.
- 커넥션 풀 크기를 executor 스레드 수에 맞춘다.
- 웹의 `get_db()` 제너레이터는 FastAPI 요청 수명주기에 묶여 있으므로 그대로 가져오지 말고,
  **세션 팩토리(`SessionLocal`)만 옮겨 와 ROS2 콜백 수명주기에 맞게 다시 감싼다.**

---

## 4. 통합 대상 인벤토리

> 전제: 웹은 DB에 직접 접근하지 않는다. 아래 4.1~4.2는 모두 Fleet Manager가 DB를 만지고, 웹은 그 결과를 Fleet Manager 경유로만 주고받는다.

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

### 4.2 Fleet Manager로 위임할 것 (현재 웹이 직접 DB에 쓰는 동작)

웹은 DB에 못 쓰므로, 아래 동작은 **웹이 Fleet API에 명령을 보내고 Fleet Manager가 DB에 쓴다** (Web Gateway proxy 경유).

- **고객 주문 생성** (`POST /api/orders`, `order_router.py`).
- **관리자 동작** — emergency-stop / resume / 상품 등록·재고 수정 / pickup-slot 생성 / 입고 요청 (`admin_router.py`).

### 4.3 웹에 남는 것 (UI / 외부 연동) — 단, DB 접근 없음 (읽기·쓰기 모두 Fleet 경유)

- Customer/Admin HTML 페이지 (`page_router.py`, `templates/`).
- 고객/관리자 WebSocket 상태 송출 (`realtime.py`) — 표시할 데이터는 Fleet Manager에서 받아온다 (DB 직접 조회 아님).
- AI Server LLM 연동 창구(`web/app/services/llm_client.py`). 단, LLM 결과가 DB 쓰기로 이어질 때는 Web Gateway가 Fleet API에 위임한다.

---

## 5. 단계별 작업 목록

### Phase 0 — 준비

- [x] 웹↔Fleet 전달 방식 확정: **HTTP/REST API + WebSocket** (2.3).
- [x] Fleet Manager API 스펙 초안 작성 → `docs/Fleet_Manager_API_스펙_초안.md`.
- [ ] DB 권한 정책 확정: **Fleet Manager 계정만 DB 접속**, 웹은 DB 접속 권한을 아예 갖지 않음 (읽기 권한도 부여하지 않음).
- [ ] 운영 흐름 회귀 테스트 시나리오 정의 (주문 1건 end-to-end, 입고 1건, emergency/resume).

### Phase 1 — DB 코드 자산을 Fleet Manager로 이사 (완료)

- [x] `web/app/models.py` + DB 비결합 `services/*`(inventory_status, robot_runtime_policy, product_images, stocking_service, status_service, workflow_service)를 신설 ROS2 패키지 `src/just_pick_it/just_pick_it_db`로 이동. (`realtime.py`는 WebSocket 전송 계층이라 web 잔류)
- [x] FastAPI 결합부(`get_db` 제너레이터)는 공용 패키지에 두지 않음. 세션 팩토리/`scoped_session`/`session_scope`만 노출(`just_pick_it_db/session.py`). 웹은 자신의 `get_db`만 얇게 유지.
- [x] **스레드별 세션 관리** 적용: `get_scoped_session()` + `session_scope()` 컨텍스트 매니저, 풀 크기 env(`DB_POOL_SIZE`) 조정.
- [x] ROS2 빌드 의존성 해결: `package.xml`에 `python3-sqlalchemy`/`python3-psycopg2`, `fleet_manager`가 `just_pick_it_db` 의존. colcon 빌드 성공, 시스템 Python·install 공간·web venv 양쪽 import 검증 완료.
- [x] web 무회귀: 전환 중에는 기존 `app.models`/`app.services.*` 경로를 re-export shim으로 유지했고, Phase 5에서 Web Gateway가 Fleet API proxy 전용으로 바뀌면서 제거 완료.

> 메모: 검증 중 실행 DB의 `orders.assigned_unit_id` 컬럼 부재(스키마 드리프트)를 확인했다.
> 이는 Phase 1 변경과 무관한 사전 문제이며 현재 기준에서는 `./reset_ws.sh` 또는 `./reset_demo_data.sh`로 스키마/seed 재적용 시 해소된다.

### Phase 2 — Fleet Manager 쓰기 경로 DB 직접화 (완료)

- [x] DAO/Repository 계층 도입: `ControlServerClient`를 **`FleetRepository`**(`fleet_repository.py`)로 개명하고 내부 구현을 HTTP → DB(`just_pick_it_db`)로 교체. public 메서드 이름/시그니처/반환 형태 유지(무회귀).
- [x] 4.1 표의 쓰기 메서드(`update_task_status`, `create_task_event`, `update_order_status`, `update_robot_state`, `update_pickup_slot_status`, `create_exception`, `update_stocking_item`, `create_tasks_bulk`)를 DB 직접 호출로 대체. 상태 전이는 공용 `workflow_service.apply_task_runtime_state`를 거친다.
- [x] 다중 테이블 동시 갱신(`complete_stocking`)을 단일 `session_scope()` 트랜잭션으로 처리.
- [x] not-found/검증 실패는 `_RepoError`로 잡아 `None`/빈 list 반환(이전 HTTP 4xx→None 계약 유지). `node`/`task_manager`의 `control_server`→`fleet_repo`, `self._control`→`self._repo` 개명.
- [x] 실 DB 대상 읽기·쓰기 검증 완료(로봇 상태 변경, task 생성/전이, 이벤트, 예외 생성, 실패 케이스).

> 메모: 비상 정지/재개는 아직 Control Server WS(`/api/fleet/ws/events`) 경유로 남아 있다(명령 경로, Phase 4 대상). 데이터 read/write의 HTTP 의존은 제거됐다.

### Phase 3 — Fleet Manager 읽기 경로 DB 직접화 (대부분 완료)

- [x] `get_snapshot/list_orders/list_zones/list_products/list_pickup_slots/list_tasks/list_order_tasks/list_requested_stocking_items/get_order_detail` 등 조회를 DB SELECT로 대체(Phase 2에서 `FleetRepository`로 함께 전환). 데이터 조회의 HTTP/Control Server 의존 제거.
- [x] `get_order_work`/`get_stocking_work`의 다회 조회(주문+상품맵+zone맵)를 **단일 `session_scope` 트랜잭션**으로 합침. 일관된 스냅샷 읽기 + 세션/커넥션 1회. 실 DB 검증 완료.
- [ ] PostgreSQL `LISTEN/NOTIFY` 기반 이벤트화는 **Phase 4 이후로 의도적 보류**. 이유: Phase 4 에서 주문 생성이 Fleet Manager 로 위임되면 Fleet 이 주문을 직접 만들어 즉시 인지하므로, 외부 INSERT 감지를 위한 NOTIFY 자체가 불필요해질 가능성이 크다(현재 5초 polling 으로 충분).

### Phase 4 — Fleet Manager API 서버 구축 (완료)

- [x] **HTTP API 서버 골격**(`fleet_api_server.py`): ROS2 노드 프로세스 안에서 uvicorn 을 데몬 스레드로 기동(rclpy executor + asyncio 공존). 데몬 스레드 signal 핸들러 비활성화로 공존 문제 해결. 노드 `api_enabled/api_host/api_port` 파라미터, start/stop 수명주기 연결.
- [x] 읽기 엔드포인트: `GET /api/health/db`, `/api/admin/status`, `/api/customer/status`, `/api/products`, `/api/orders`, `/api/orders/{id}`. 핸들러는 `FleetRepository`(스레드 안전)만 호출.
- [x] DB 명령 엔드포인트: `POST /api/orders`(재고 검증/차감), `/api/orders/{id}/complete`, `POST/PATCH /api/admin/products`(+`/stock`), `POST /api/admin/pickup-slots`, `POST /api/admin/exceptions/{id}/resolve`. `RepoError(status_code)`로 404/400 매핑, 요청 검증은 `fleet_api_schemas.py`.
- [x] ROS 핸드오프 명령(emergency-stop/resume): DB 전이(`FleetRepository.apply_emergency_stop/apply_resume`) + 노드 `trigger_emergency_stop`가 `RobotCommandGateway` 전파 + `task_manager.handle_*`를 묶음. WS listener 경로와 `_propagate_emergency` 공유.
- [x] 실시간 WebSocket push: `WS /api/admin/ws/status`, `/api/customer/ws/status`. 연결 시 즉시 스냅샷 + 이벤트 루프 주기 push(`api_push_interval_sec`, DB 조회는 `run_in_executor`).
- [x] 입고 요청 생성 API: `POST /api/admin/stocking-items` 가 `stocking_item`을 REQUESTED 상태로 생성한다. LLM parser 자체는 Web Gateway에 남긴다.

> 검증: 각 증분을 TestClient + 실기동(uvicorn 스레드 바인드/응답/정지, 실 WebSocket 클라이언트) + 실 DB 로 확인.

### Phase 5 — 웹을 Fleet API 클라이언트로 전환 (완료, LLM 제외)

목표: 웹이 DB 에 직접 접근하지 않고, 읽기·쓰기 모두 Fleet API(Phase 4)를 경유하게 한다. 끝나면 웹에 DB 코드가 0줄 남는다.

**5.0 전달 경로 결정 (완료)**
- [x] 브라우저 ↔ Fleet 연결 방식은 **웹 프록시(A안)** 로 확정. 브라우저는 기존처럼 Web Gateway 의 `/api/*` 를 호출하고, Web Gateway 가 Fleet API 로 forward 한다.
- [x] Fleet API base URL 을 웹 설정값으로 추가: `FLEET_API_BASE_URL`, `FLEET_API_WS_BASE_URL`.

**5.1 웹에 Fleet API 클라이언트 도입 (완료)**
- [x] `httpx` 기반 HTTP proxy 와 `websockets` 기반 WebSocket proxy 도입: `web/app/routers/fleet_api_proxy_router.py`.
- [x] Fleet API 연결 실패 시 Web Gateway 는 503 으로 응답한다.

**5.2 읽기 라우터 전환 (완료)**
- [x] 기존 product/order/customer/admin/status 조회 라우터를 DB 직접 조회 대신 `/api/{path:path}` proxy 로 통합.
- [x] `GET /api/products`, `/api/orders`, `/api/customer/status`, `/api/admin/status` 등은 Fleet API 로 전달된다.

**5.3 쓰기 라우터 전환 (완료)**
- [x] 주문 생성/완료, 관리자 상품/슬롯/예외/emergency/resume 요청도 Web Gateway 에서 Fleet API 로 전달된다.
- [x] Web Gateway 는 요청 body/header 를 넘기고, Fleet API 응답 status/body 를 그대로 브라우저에 반환한다.

**5.4 WebSocket 전환 (완료)**
- [x] `customer`/`admin` 상태 WebSocket 을 Fleet API WebSocket 으로 프록시한다.
- [x] 웹 `realtime.py`/WebSocketManager 방식은 제거됐다.

**5.5 웹 DB 코드 제거 (완료)**
- [x] `web/app/database.py`, `web/app/models.py`, `web/app/services/*` 제거.
- [x] 기존 DB 직접 라우터(`admin_router`, `customer_router`, `fleet_router`, `order_router`, `product_router`, `health_router`) 제거.
- [x] `web/requirements.txt` 에서 `sqlalchemy`, `psycopg2` 제거. Web Gateway 는 DB driver 를 설치하지 않는다.
- [x] `web/scripts/setup.sh` 는 Web Gateway venv/requirements/.env 만 담당한다. DB/rosdep/colcon 은 root script 로 이동.
- [ ] DB 계정 권한 회수: 현재 로컬 단일 DB 계정 기준에서는 코드/설정상 웹 DB 접근은 제거됨. 별도 운영 계정을 나누는 시점에 웹 계정 DB 권한 없음으로 재확인한다.

**5.6 LLM 명령 처리 (구조 완료, 실제 parser 구현 대기)**
- [x] `POST /api/admin/llm/messages`는 Web Gateway가 직접 처리한다.
- [x] LLM client/parser 위치는 `web/app/services/llm_client.py`로 고정한다. 실제 의미 분석 구현은 LLM 담당자가 이 파일을 교체/확장한다.
- [x] parser 결과가 `action=STOCKING`이면 Web Gateway가 Fleet API `POST /api/admin/stocking-items`로 위임한다.
- [x] Fleet API는 `stocking_item`을 `REQUESTED` 상태로 생성하고, TaskManager가 IDLE polling 때 기존 입고 흐름으로 가져간다.
- [ ] 실제 LLM provider 연결 및 자연어 의미 분석 로직 구현.

**5.7 무회귀 확인 (진행 중)**
- [ ] 고객 주문 end-to-end, 관리자 화면 실시간 갱신, emergency-stop/resume 흐름을 웹 UI 기준으로 검증.

### Phase 6 — 잔재 제거 및 문서화 (진행 중)

- [x] 웹 `fleet_router`(`/api/fleet/*`) 삭제: Fleet 이 더 이상 웹을 호출하지 않으므로 제거 완료.
- [ ] 노드의 Fleet event WS listener(`_send_emergency_stop`, `_start_fleet_event_listener`, `/api/fleet/ws/events`) 제거: emergency 가 Fleet API 로 직접 들어오는 흐름만 남길지 최종 확인 후 정리.
- [ ] 아키텍처 문서(`docs/3_System_Architecture.pdf`) ver_3.0 갱신: Control Server 를 DB 비접근 프레젠테이션 프런트로 표기, DB 화살표는 Fleet Manager 에만 연결.
- [x] 팀원 실행 가이드 추가: `docs/Fleet_API_통합_팀원_가이드.md`.
- [x] `인수인계서.md`, `db/README.md`, `web/README.md` 실행 기준 갱신.
- [ ] `TASK_PLAN.md` / 최상위 `README.md` 구조 설명 갱신.

### Phase 7 — 실행 기준 고정 및 팀원 재세팅 자동화 (추가, 완료)

- [x] 고정 실행 기준 명시: Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12.
- [x] root `reset_ws.sh` 추가: Web Gateway setup, PostgreSQL 준비/DB 초기화, rosdep, `build/install/log` 삭제, 전체 `colcon build --symlink-install` 를 한 번에 수행.
- [x] root `run_all.sh` 추가: PostgreSQL 확인, Fleet Manager/Fleet API 실행, Web Gateway 실행을 한 번에 묶음.
- [x] root `reset_demo_data.sh` 추가: schema 전체가 아니라 데모 테이블/seed 만 빠르게 되돌리는 용도.
- [x] `web/scripts/run_all.sh`, `web/scripts/reset_demo_data.sh` 제거: web 폴더는 Web Gateway 전용으로 유지.
- [x] `web/scripts/run.sh` 는 Web Gateway only, `web/scripts/setup.sh` 는 Web Gateway venv setup only 로 책임 축소.


---

## 6. 리스크와 점검 포인트

| 항목 | 내용 |
|------|------|
| 스레드 세션 안전성 | `MultiThreadedExecutor` + SQLAlchemy Session은 공유 시 데이터 손상. `scoped_session`/콜백당 세션 필수 (3.4). |
| API 서버·rclpy 공존 | HTTP/REST 방식(확정) 때문에 한 프로세스에서 uvicorn(asyncio)과 rclpy executor가 공존. uvicorn은 별도 스레드, 로봇 제어 동작은 executor로 위임 (2.3). |
| 전달 방식 확장성 | 로봇 다수·다중 서비스로 커지면 메시지 큐 재검토 여지. 현재 2대 규모에선 HTTP/REST로 충분. |
| 트랜잭션 경계 | HTTP 엔드포인트 단위로 암묵 보장되던 원자성을 DB 직접화 후 명시적 트랜잭션으로 재현해야 함. |
| 마이그레이션 중 이중 경로 | Phase 2~3 동안 일부는 HTTP, 일부는 DB 직접일 수 있음. `ControlServerClient` 인터페이스 유지로 전환 충격 흡수. |
| 단일 장애점 | 통합 후 Fleet Manager가 제어 + DB + 웹 데이터 제공을 담당 → 장애 영향 범위 확대. 모니터링/재시작 전략 필요. |
| 권한 강제 | 웹의 DB 비접근을 코드 규칙이 아니라 **DB 계정 권한**으로 못박는다. 웹에는 읽기 권한조차 부여하지 않아 실수로도 DB에 직접 붙을 수 없게 한다. |

---

## 7. 요약 권고

1. **토폴로지**: Fleet Manager가 DB 단독 게이트웨이. 웹은 DB 비접근 프레젠테이션 클라이언트로 분리, **읽기·쓰기 모두 Fleet Manager 경유**.
2. **전달 방식**: **HTTP/REST API + WebSocket** 확정. Fleet Manager가 API 서버를 열고 웹은 HTTP/WS 클라이언트 (조회 GET, 명령 POST, 실시간 WebSocket).
2. **DB 접근 코드**: 기존 SQLAlchemy 자산을 Fleet Manager로 이사해 재사용. 재작성하지 않는다.
3. **기술**: ORM(SQLAlchemy) 유지, DAO/Repository 계층으로 감싼다.
4. **필수 안전장치**: 스레드별 세션 관리 + DB 계정 권한으로 웹 쓰기 차단.
5. **이행 순서**: DB 코드 이사 → 쓰기 DB 직접화 → 읽기 DB 직접화 → Fleet API 서버 구축 → Web Gateway proxy 전환 → HTTP 잔재 제거.
