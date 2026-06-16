# Fleet Manager 인터페이스 계약

Fleet Manager가 외부(웹/로봇/내부 모듈)와 주고받는 모든 계약을 한 곳에 모았다. 설계·동작 개요는 `docs/Fleet_manager.md`, 작업 현황은 `docs/Fleet_manager_TODO.md`.

대상별:
- 전체 구조 파악: §0 구성요소 개요
- 웹 담당: §1 Fleet API (HTTP/REST), §2 WebSocket
- 로봇(PICKY/COBOT) 담당: §3 텔레메트리 토픽, §4 명령 인터페이스, §5 PICKY 계약, §6 COBOT 계약, §8 시나리오
- Fleet 내부(TaskManager): §7 TrafficManager API

---

## 0. 구성요소 개요

Just Pick It의 중앙 제어 노드인 **Fleet Manager**는 하나의 ROS2 프로세스 안에서
DB 접근 / 작업 스케줄링 / 경로 관리 / 로봇 명령 / 웹 API를 조립한다.
`fleet_manager_node.py`의 `FleetManagerNode`만 `rclpy.Node`를 상속하고,
나머지 기능 클래스는 별도 노드가 아니라 이 노드에 조립되는 일반 Python 객체로 동작한다.
담당 경계와 설계 배경은 `docs/Fleet_manager.md` §2 참고.

```text
FleetManagerNode
  ├── FleetRepository      DB 접근 단일 계층
  ├── FleetApiServer       HTTP/WebSocket API (uvicorn, 데몬 스레드)
  ├── TrafficManager       PICKY 경로 탐색/예약/충돌 회피
  ├── RobotStateMonitor    로봇 텔레메트리 구독 -> DB + Traffic
  ├── RobotCommandGateway  task -> ROS2 Action/Service 명령
  └── TaskManager          주문/진열 polling, task 생성/전이/dispatch
```

### 0.1 컴포넌트별 상세

#### Fleet Manager Node — `fleet_manager_node.py`
- Fleet Manager의 메인 ROS2 노드. 아래 기능 클래스들은 각각 별도 노드가 아니라 이 노드에 조립되는 Python 객체로 동작한다.
- 노드 시작 시 설정을 읽고, 6개 컴포넌트를 생성·연결하며, 주기 타이머(대기 작업 polling, 재시작 복구)를 건다.

#### Fleet API Schemas — `fleet_api_schemas.py`
- HTTP 명령 엔드포인트(POST/PATCH)의 입력 검증용 Pydantic 모델 모음. 주문·상품·작업 등 요청 본문의 형식을 검사한다.
- 응답은 Repository가 만든 dict를 그대로 쓰므로 응답 모델은 따로 두지 않는다.

#### Fleet API Server — `fleet_api_server.py`
- 웹 프런트에 노출하는 HTTP/REST + WebSocket API 서버. ROS2 executor와 같은 프로세스 안에서 별도 스레드의 uvicorn으로 돈다.
- DB 접근은 Repository로, 로봇 전파가 필요한 명령만 메인 노드로 위임한다.

#### Fleet Repository — `fleet_repository.py`
- Fleet Manager의 단일 DB 접근 계층. 주문·작업·로봇·zone·진열 데이터를 읽고 쓰는 유일한 진입점이다.
- PostgreSQL(`just_pick_it_db`)에 직접 접근하고, 상태 전이 등 비즈니스 로직은 DB services를 재사용한다. (`RepoError`는 not-found/검증 실패 예외)

#### Robot Command Gateway — `robot_command_gateway.py`
- Fleet의 task를 ROS2 Action/Service 명령으로 변환해 로봇에 보내는 출력 어댑터. TaskManager가 topic/message 구조를 직접 몰라도 되게 한다.
- PICKY 이동(MoveCommand)·도킹(DockCommand)·비상정지(EmergencyControl)를 지원하고, Action 피드백/결과를 다시 콜백으로 돌려준다.

#### Task Manager — `task_manager.py`
- Fleet Manager 내부 task의 생성·배정·상태 전이 담당. DB를 polling해 대기 중인 주문/진열 요청을 찾고, 가용 로봇에 배정하며, task로 변환한다.
- TrafficManager와 협업해 PICKY 경로를 예약하고, task 진행/실패를 DB에 보고한다. (ROS2 노드가 아니며 메인 노드의 logger만 사용)

#### Traffic Manager — `traffic_manager.py`
- BFS 기반 경로 계획 + 다중 AMR 충돌 회피 모듈. zone 인접 그래프 위에서 경로를 탐색·예약한다.
- 외부 I/O가 없다: 로봇 상태는 RobotStateMonitor가 `notify_state()`로 전달하고, zone 좌표는 생성자에서 주입받는다. (`PathResult`는 탐색/예약 결과 구조체)

#### (참고) Robot State Monitor — `robot_state_monitor.py`
- 목록 외 항목. PICKY 텔레메트리(picky_state / battery / pose)를 ROS2 토픽으로 구독해 DB와 TrafficManager에 반영하는 클래스.

### 0.2 한 문장 요약 (슬라이드용)

| 구성요소 | 한 문장 설명 |
|---|---|
| **Fleet Manager Node** | 6개 기능 클래스를 한 프로세스에 조립·배선하고 주기 타이머를 돌리는 메인 ROS2 노드. |
| **Fleet API Schemas** | HTTP 요청 본문을 검증하는 Pydantic 입력 모델 모음. |
| **Fleet API Server** | 웹 프런트에 HTTP/REST + WebSocket을 노출하는 API 서버(별도 스레드 uvicorn). |
| **Fleet Repository** | 모든 DB 읽기·쓰기를 책임지는 단일 DB 접근 계층. |
| **Robot Command Gateway** | task를 ROS2 Action/Service 명령으로 바꿔 로봇에 보내는 출력 어댑터. |
| **Task Manager** | 주문/진열을 polling해 로봇에 배정하고 task 상태를 전이시키는 스케줄러. |
| **Traffic Manager** | zone 그래프 BFS로 경로를 계획하고 AMR 충돌을 회피하는 모듈. |
| *(참고)* **Robot State Monitor** | 로봇 텔레메트리를 구독해 DB와 Traffic에 반영하는 모듈. |

---

## 1. Fleet API (HTTP/REST)

- Base URL: `http://<fleet_host>:8100`. 브라우저는 같은 origin `http://localhost:8000/api/*`를 호출하고 Web Gateway가 Fleet API로 forward한다.
- Content-Type: `application/json`. 시각: ISO 8601.
- 오류(FastAPI 기본): `{ "detail": "..." }`. 200 정상 / 201 생성 / 400 값 오류(재고 부족 등) / 404 대상 없음 / 409 상태 충돌.
- 인증: 현재 미구현(관리자 엔드포인트 인증은 TODO).

### 상태 enum (`web/app/schemas.py` 기준)

- OrderStatus: `ORDER_RECEIVED, ORDER_WAIT, SORTING, DELIVERING, INSPECTING, PICKUP_READY, COMPLETED, ERROR`
- RobotStatus: `OFFLINE, IDLE, BUSY, CHARGING, EMERGENCY_STOP, ERROR`
- PickupSlotStatus: `EMPTY, RESERVED, OCCUPIED, BLOCKED`
- ExceptionType: `OBSTACLE_DETECTED, LOW_BATTERY, NAVIGATION_FAILED, HARDWARE_ERROR, TIMEOUT, SORTING_FAIL, INSPECTION_FAIL, HUMAN_DETECTED, SYSTEM_ERROR`
- (PickyState, CobotState, TaskType, TaskStatus, DisplayPolicy 등은 schemas.py 정의를 따른다)

### 1.1 조회 (GET)

| 엔드포인트 | 응답 | 비고 |
|---|---|---|
| `GET /api/products` | `ProductRead[]` | 상품 목록 |
| `GET /api/orders` | `OrderRead[]` | COMPLETED 제외 최신순 |
| `GET /api/orders/{id}` | `OrderRead` | 404 가능 |
| `GET /api/customer/status` | 고객 스냅샷 | `build_customer_status` |
| `GET /api/admin/status` | 관제 스냅샷 | `build_admin_status`(orders/robots/tasks/products/pickup_slots/exceptions 등) |
| `GET /api/health/db` | `{"status":"ok"}` | 503 연결 실패 |

### 1.2 명령 (POST/PATCH)

| 엔드포인트 | 처리 |
|---|---|
| `POST /api/orders` | 재고 검증/차감 -> 주문 생성 -> ORDER_WAIT |
| `POST /api/orders/{id}/complete` | PICKUP_READY일 때만 COMPLETED |
| `POST /api/admin/products`, `PATCH .../{id}`, `PATCH .../{id}/stock` | 상품 생성/수정/재고 |
| `POST /api/admin/pickup-slots` | 슬롯 생성 |
| `POST /api/admin/exceptions/{id}/resolve` | 예외 해결 |
| `POST /api/admin/display-items` | 진열 요청(REQUESTED) 생성 |
| `POST /api/admin/emergency-stop`, `POST /api/admin/resume` | DB 전이 + 로봇 전파 + dispatch 게이트 |

### 1.3 `/api/fleet/*` (관리/검증용, 유지)

원래 Fleet↔웹 브리지였으나, 통합 후에는 **admin UI에서 task/로봇/슬롯을 직접 조회·수정해 검증 테스트하기 위한 운영·디버그 엔드포인트**로 유지한다(결정 D3). 경로는 여전히 Fleet API → FleetRepository → DB라 DB 소유권 정책에 위배되지 않는다.

- 조회: `GET /api/fleet/snapshot`, `/zones`, `/tasks`, `/orders`, `/orders/{id}/tasks`, `/pickup-slots`
- 쓰기: `POST /api/fleet/tasks/bulk`, `PATCH /api/fleet/tasks/{id}`, `DELETE /api/fleet/tasks/{id}`, `PATCH /api/fleet/orders/{id}`, `PATCH /api/fleet/robots/{id}`, `PATCH /api/fleet/pickup-slots/{id}`

> 주의: `PATCH /api/fleet/robots/{id}`는 admin 수동 보정용이다. 로봇의 정기 상태 보고는 HTTP가 아니라 ROS2 텔레메트리(§3)로 들어온다.

---

## 2. Fleet API WebSocket

```text
WS /api/admin/ws/status      연결 시 즉시 스냅샷 1회 + 주기 push(build_admin_status)
WS /api/customer/ws/status   연결 시 즉시 스냅샷 1회 + 주기 push(build_customer_status)
```

- Web Gateway가 프록시한다.
- 현재는 `api_push_interval_sec`(기본 1.0s)마다 전체 스냅샷을 재전송한다(이벤트 기반 delta 전환은 후속 여지).

---

## 3. 로봇 텔레메트리 토픽 (PICKY -> Fleet)

로봇 상태는 HTTP가 아니라 ROS2 토픽으로 보고한다(System Architecture 준수). `RobotStateMonitor`가 구독해 DB에 반영한다.

| 데이터 | 토픽 | 타입 | 발행 측 |
|---|---|---|---|
| picky_state | `/pickyX/picky_state` | `std_msgs/String` | State Manager |
| battery | `/pickyX/battery/percent` | `std_msgs/Float32` (이미 %) | pinky_bringup |
| pose | `/pickyX/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` (map frame) | AMCL |

반영 규칙:
- picky_state는 수신 즉시 `TrafficManager.notify_state()`로 전달(경로/도크 자동 해제 지연 방지).
- battery/pose/picky_state는 최신값만 캐시했다가 `robot_state_flush_period_sec`(1Hz)마다 변경분만 `update_robot_state(picky_state, battery_level, pos_x/y/theta)`로 DB 반영.
- **`robot_status`는 텔레메트리로 갱신하지 않는다**(task 전이 전용, 결정 D2).
- battery_level이 임계값(30%, `CHARGE_BATTERY_THRESHOLD`)을 초과하는 구간에 진입할 때 robot별 1회만 `handle_battery_update`를 호출(충전 완료 트리거).

---

## 4. 로봇 명령 인터페이스 (Fleet -> 로봇)

`RobotCommandGateway`가 task를 ROS2 Action/Service로 변환한다. robot_name -> namespace는 소문자(`PICKY1` -> `/picky1`).

| 명령 | 인터페이스 | 대상 task | 비고 |
|---|---|---|---|
| 이동 | `/{ns}/move_command` `just_pick_it_interfaces/action/MoveCommand` | MOVE_TO_PRODUCT/PICKUP/STOCK/DISPLAY, RETURN_HOME | goal=목적지 pose만, feedback=`current_waypoint_index` |
| 도킹 | `/{ns}/dock_command` `just_pick_it_interfaces/action/DockCommand` | DOCK_IN | goal=`task_id`,`dock_name`,`start_zone_name` |
| 비상 | `/{ns}/emergency_control` `just_pick_it_interfaces/srv/EmergencyControl` | (전체) | 표준 SetBool 아님 |
| COBOT 작업 | `/{ns}/execute_task` `just_pick_it_interfaces/action/ExecuteTask` (정의 대기) | SORTING_AND_LOAD/INSPECTION/UNLOAD/DISPLAY_SCAN/DISPLAY_PLACE | feedback `status=STOWING_ARM`이면 다음 PICKY 이동 선계획 |
| 충전 | (액션 없음) | CHARGE | 배터리 상태로 완료 판단하는 logical task |

`EmergencyControl.srv`:
```
bool emergency_stop / string reason / int32 task_id / string request_id
---
bool accepted / string status / string message
```
요청: `emergency_stop=true` 즉시 안전 정지, `false` 해제·재개. State Manager는 service server를 항상 띄우고, true 수신 시 동작을 안전 정지하고 emergency 중 action을 SUCCESS로 반환하지 않는다. 재개 불가 시 `accepted=false`+사유.

내부 콜백 경로:
```text
PICKY feedback -> Gateway -> TaskManager.handle_move_feedback() -> TrafficManager.update_path_progress()
PICKY/COBOT result -> Gateway -> TaskManager.handle_task_result() -> FleetRepository.update_task_status() -> WS push
COBOT STOWING_ARM feedback -> Gateway -> TaskManager.preplan_after_cobot_stowing()
```

---

## 5. PICKY State Manager 계약

| task_type | picky_state(이동) | 도착 시 picky_state |
|---|---|---|
| MOVE_TO_PRODUCT/PICKUP/STOCK/DISPLAY | MOVING_TO_* | `WAITING_FOR_COBOT` |
| RETURN_HOME | RETURNING | `STANDBY` |
| DOCK_IN | DOCKING | (도킹 완료 후) `CHARGING` |

**MoveCommand 수신**: task_type에 맞는 picky_state 발행 → waypoint 순서대로 주행 → 통과마다 feedback `current_waypoint_index`(통과한 index, 0=시작은 생략 가능) → 도착 시 result `success=true`, 실패 시 `success=false`+메시지.

**DockCommand 수신**: `picky_state=DOCKING` 발행 → `start_zone_name`에서 `dock_name` 방향 로컬 도킹(라인/PID/ArUco/후진은 State Manager 내부) → feedback `phase/progress/message` → 완료 시 `success=true`. `CHARGING_DOCK_*`는 DB pose가 아니라 논리 도크 이름이며, Fleet은 DOCK_IN 전 TrafficManager로 도크만 예약하고 정밀 진입은 로봇에 맡긴다.

규칙: 물리 동작이 안전하게 끝난 뒤에만 SUCCESS. PICKY는 임의 이동 금지 — 다음 MOVE는 Fleet이 새 goal로 내려준다. COBOT 작업 중 PICKY는 `WAITING_FOR_COBOT` 유지.

> 재시작 복구(A'')는 이 "도착 picky_state" 계약에 의존한다. 도착 시 상태 전이가 명확해야 복구가 완료를 재동기할 수 있다.

---

## 6. COBOT State Manager 계약

| task_type | 작업 상태 | 팔 복귀 |
|---|---|---|
| SORTING_AND_LOAD | SORTING/LOADING | STOWING_ARM |
| INSPECTION | INSPECTING | STOWING_ARM |
| UNLOAD | UNLOADING | STOWING_ARM |
| DISPLAY_SCAN | SCANNING | STOWING_ARM |
| DISPLAY_PLACE | PLACING | STOWING_ARM |

`SORTING_AND_LOAD`는 주문(`order_item`)과 진열(`display_item`) 양쪽에서 재사용한다. 진열 흐름에서도 현재 Fleet 계약상 cobot_state는 `SORTING`으로 기록한다(`LOADING` 세분화는 COBOT feedback 확정 후 반영).

`ExecuteTask.action` goal은 task/order/display/product/zone id와 수량 정보를 전달한다. 현재 action 메시지 정의는 COBOT 담당 확정 대기 상태다. `RobotCommandGateway.send_cobot_task()`는 `ExecuteTask.action`이 아직 생성되지 않았거나 `/{cobot_ns}/execute_task` action server가 없으면 False를 반환하고, 이 경우 task는 ASSIGNED로 유지되어 다음 dispatch cycle에 재시도된다.

규칙: goal 수신 → cobot_state 전이 → 작업 수행 → 본동작 후 바로 SUCCESS 금지, `STOWING_ARM` 전이 + STOWING_ARM 시작 feedback → 팔 완전 복귀 후 result `success=true`. COBOT SUCCESS는 STOWING_ARM 완료를 의미. 실패를 숨기고 SUCCESS 보내면 안 된다.

**STOWING_ARM 선계획**: COBOT이 STOWING_ARM에 들어가면 PICKY는 아직 못 움직이지만 Fleet은 다음 이동 task를 미리 생성/예약할 수 있다(`preplan_after_cobot_stowing`). 미리 만든 MOVE task는 sequence gate로 이전 COBOT SUCCESS 전에는 dispatch되지 않는다.

| STOWING_ARM trigger | 선계획 |
|---|---|
| SORTING_AND_LOAD (주문) | 남은 상품 있으면 다음 MOVE_TO_PRODUCT/SORTING_AND_LOAD, 없으면 MOVE_TO_PICKUP/INSPECTION/UNLOAD |
| SORTING_AND_LOAD (진열) | 다음 MOVE_TO_DISPLAY 경로 선예약 |
| INSPECTION/UNLOAD/DISPLAY_SCAN/DISPLAY_PLACE | 이동 선계획 없음 |

**실패 보상**: COBOT이 STOWING_ARM까지 갔다 최종 실패하면 — 미리 생성한 후속 task CANCELLED, 미리 예약한 path release, 실패 task FAILED, exception 기록.

---

## 7. TrafficManager API

`TrafficManager`는 zone 기반 BFS 경로 탐색과 다중 PICKY 충돌 회피를 담당한다. 외부 I/O(HTTP/DB/ROS 송신) 없이 in-memory 레지스트리(`_robot_paths`, `_robot_reservations`, `_robot_dock`, `_robot_states`)만 다룬다. **도크 점유(`_robot_dock`)는 TrafficManager 단독 보유**로 외부에 노출/통지하지 않는다.

| | TrafficManager | TaskManager |
|---|---|---|
| zone 그래프/경로 탐색, 충돌 회피, 도크 선정 | O | X |
| task 순서/상품 후보 선정 | X | O |

### PathResult
```python
@dataclass(frozen=True)
class PathResult:
    ok: bool
    waypoints: tuple[str, ...] = ()   # 시작~목적지 zone_name, ok=False면 ()
    cost: float | None = None         # 현재 hop 수
    reason: str | None = None         # ok=True면 None
```

### 메서드
- `reserve_path(robot_id, task_id, source_zone, target_zone) -> PathResult` — 단일 목적지 이동. BFS+등록을 단일 lock에서 원자적으로. 실패 시 잠시 후 재시도.
- `reserve_nearest_from(robot_id, task_id|None, source_zone, candidates: {zone: 수량}) -> PathResult` — 후보 중 cost 최소 zone을 atomic 예약(MOVE_TO_PRODUCT). 선택 zone=`waypoints[-1]`. `task_id=None`으로 path 먼저 잡고 INSERT 후 `attach_task_id`로 연결.
- `attach_task_id(robot_id, task_id) -> bool` — 임시 예약(task_id 미배정)에 사후 task_id 연결.
- `reserve_return_home_path(robot_id, task_id, source_zone) -> PathResult` — STANDBY_ZONE 중 최소비용. 도크 예약은 안 함.
- `reserve_dock_path(robot_id, task_id, source_zone) -> PathResult` — 빈 도크를 안쪽(CHARGING_DOCK_1) 우선 선정+경로+도크 예약을 원자적으로.
- `update_path_progress(robot_id, task_id, current_waypoint_index)` — 통과 구간 점유 해제. stale(task_id 불일치)면 무시.
- `release_path(robot_id, task_id|None)` — SUCCESS/FAILED/CANCELLED/timeout 시 예약 해제. `None`은 임시 예약 정리용.
- `notify_state(robot_id, state)` — RobotStateMonitor 전용. `_robot_states` 갱신 + (MOVING/OCCUPYING 아닌 상태로 가면) path/예약 자동 해제 안전망 + `CHARGING` 이탈 시 도크 해제.
- `get_robot_state` / `get_all_states` — 읽기 전용.
- **재시작 복구용**: `nearest_zone(x,y) -> zone` (pose→그래프 노드), `nearest_dock(x,y) -> dock` (최근접 충전 도크), `rebuild_dock(robot_id, dock_name)` (도크 점유 복원).

### 점유 차단 규칙 (`_build_blocked_sets`)
- path 등록 + state ∈ `OCCUPYING_STATES`(`WAITING_FOR_COBOT`): 마지막 노드만 차단.
- path 등록 + 그 외 모든 state(`STANDBY` 포함): 경로 전체 노드+엣지 차단. reserve 성공 자체가 "곧 점유" 약속이라 picky_state가 MOVING으로 갱신되기 전 race window도 닫힌다.
- path 미등록: 차단 없음.
- **도메인 제약**: 같은 노드에 두 로봇이 동시에 머물 수 없으므로 차단된 노드는 목적지여도 도달 불가.

`MOVING_STATES = {MOVING_TO_PRODUCT/PICKUP/STOCK/DISPLAY, RETURNING, DOCKING}`, `OCCUPYING_STATES = {WAITING_FOR_COBOT}`.

### 표준 호출 순서
1. **단일 이동**: `reserve_path` → (feedback마다) `update_path_progress` → (종료) `release_path`.
2. **후보 선정+사후 task_id**: `reserve_nearest_from(task_id=None)` → INSERT → `attach_task_id` → 패턴1. INSERT 실패 시 `release_path(robot, None)`.
3. **RETURN_HOME**: `reserve_return_home_path` → 종료 시 `release_path`.
4. **DOCK_IN**: `reserve_dock_path` → 종료 시 `release_path`. 도크 점유는 picky_state `CHARGING` 이탈 시 `notify_state`가 자동 해제.

### 알려진 한계
- **DOCK_IN 실패 시 도크 해제**: `_robot_dock`은 `release_path`로 풀리지 않고 오직 `notify_state`의 CHARGING 이탈 안전망으로만 해제된다(의도된 정책 — TrafficManager에 DB/외부 동기화 책임을 주지 않기 위함). 다음 MOVE dispatch로 도크 이탈 시 자연 해제된다고 가정.
- **cost 단위**: 현재 hop 수. 필요 시 `_zone_coords` 기반 유클리드 거리로 교체.

zone 그래프/좌표는 `traffic_manager.py`의 `ZONE_GRAPH`/`DEFAULT_ZONE_COORDS`, 맵은 `docs/Traffic_node_graph.jpg` 참고.

---

## 8. 시나리오 상태 흐름

### 주문 (상품 2개)
```text
1. MOVE_TO_PRODUCT   RUNNING   PICKY=BUSY/MOVING_TO_PRODUCT
2. MOVE_TO_PRODUCT   SUCCESS   PICKY=BUSY/WAITING_FOR_COBOT, COBOT=IDLE/STANDBY
3. SORTING_AND_LOAD  RUNNING   COBOT=BUSY/SORTING|LOADING
4. SORTING_AND_LOAD  STOWING_ARM   Fleet이 다음 MOVE_TO_PRODUCT 선계획
5. SORTING_AND_LOAD  SUCCESS   둘 다 IDLE/STANDBY, 다음 MOVE 실행 가능
6. (남은 상품 반복)
7. 마지막 SORTING_AND_LOAD STOWING_ARM -> MOVE_TO_PICKUP/INSPECTION/UNLOAD 선계획
8. MOVE_TO_PICKUP    RUNNING
9. INSPECTION        RUNNING -> STOWING_ARM -> SUCCESS
10. UNLOAD           RUNNING -> STOWING_ARM -> SUCCESS   order=PICKUP_READY, slot=OCCUPIED
```

### 진열
```text
1. MOVE_TO_STOCK    RUNNING -> SUCCESS (PICKY WAITING_FOR_COBOT)
2. SORTING_AND_LOAD RUNNING -> STOWING_ARM -> SUCCESS   (display_item 기준, MOVE_TO_DISPLAY 선예약)
3. MOVE_TO_DISPLAY  RUNNING -> SUCCESS (PICKY WAITING_FOR_COBOT)
4. DISPLAY_SCAN     RUNNING -> STOWING_ARM -> SUCCESS
5. DISPLAY_PLACE    RUNNING -> STOWING_ARM -> SUCCESS   display_item=COMPLETED, stock_qty 반영(계획값)
```
