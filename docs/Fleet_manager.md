# Fleet Manager

Just Pick It의 중앙 제어 노드. 한 ROS2 프로세스 안에서 **DB 접근 + 작업 스케줄링 + 경로 관리 + 로봇 명령 + 웹 API**를 조립한다.

- 인터페이스 계약(Fleet API / Traffic / 로봇 연동): `docs/Fleet_manager_interface.md`
- 담당 분담 / 결정 / 작업 현황: `docs/Fleet_manager_TODO.md`

---

## 1. 개요

- **DB를 직접 만지는 주체는 Fleet Manager 하나뿐이다**(읽기·쓰기 모두). 다른 어떤 프로세스도 DB에 직접 붙지 않는다.
- 웹은 화면과 프록시만 담당하는 별도 프로세스(Web Gateway)이며, 읽기·쓰기 모두 Fleet Manager의 HTTP/WebSocket API를 경유한다.
- 로봇(PICKY/COBOT)과는 **ROS2(Action/Service/Topic)** 로만 통신한다. (System Architecture: Fleet Manager ↔ AMR/Cobot Controller = ROS2)

```text
Customer/Admin Browser
  --HTTP/WS-->  Web Gateway (:8000, 화면 + /api/* 프록시)
  --HTTP/WS-->  Fleet API (:8100, Fleet Manager 프로세스 내부)
  -->           FleetRepository  -->  just_pick_it_db  -->  PostgreSQL

FleetManagerNode  --ROS2-->  PICKY / COBOT State Manager
```

---

## 2. 구성요소 & 담당 경계

`fleet_manager_node.py`의 `FleetManagerNode`만 `rclpy.Node`를 상속한다. 나머지는 이 노드에 조립되는 일반 Python 클래스다.

```text
FleetManagerNode
  ├── FleetRepository      DB 접근 단일 계층
  ├── FleetApiServer       HTTP/WebSocket API (uvicorn, 데몬 스레드)
  ├── TrafficManager       PICKY 경로 탐색/예약/충돌 회피
  ├── RobotStateMonitor    로봇 텔레메트리 구독 -> DB + Traffic
  ├── RobotCommandGateway  task -> ROS2 Action/Service 명령
  └── TaskManager          주문/진열 polling, task 생성/전이/dispatch
```

| 모듈 | 파일 | 책임 | 담당 |
|---|---|---|---|
| `Web Service` | `web/` | 화면 렌더링 + `/api/*` 프록시 (DB 코드 없음) | 이명제 |

|---|---|---|---|
| `FleetManagerNode` | `fleet_manager_node.py` | 컴포넌트 생성·배선·타이머·명령 전파 | 공동 |
| `FleetRepository` | `fleet_repository.py` | `just_pick_it_db` 통한 DB 조회/쓰기, snapshot, 상태 전이 | 박서우 |
| `TrafficManager` | `traffic_manager.py` | zone 그래프 BFS, 점유 예약/해제, 도크 선정 | 박서우 |
| `RobotStateMonitor` | `robot_state_monitor.py` | picky_state/battery/pose 구독 -> DB 반영 + Traffic 전달 | 박서우 |
| `FleetApiServer` | `fleet_api_server.py` | REST/WebSocket endpoint 제공 | 이명제 |
| `TaskManager` | `task_manager.py` | 대기 작업 polling, task 생성/전이/dispatch, 재시작 복구 | 이명제 |
| `RobotCommandGateway` | `robot_command_gateway.py` | task -> PICKY/COBOT Action/Service 변환, 콜백 연결 | 이명제 |

|---|---|---|---|
| `State Manager` | `pinky_amr_1/.../state_manager.py` | 실제 로봇 주행/도킹/상태 발행 | 박서우 |
|---|---|---|---|

**수정 경계**: 위 표의 담당 외 파일은 직접 고치지 않고, 계약 변경이 필요하면 사유를 먼저 공유한다. 상태 전이 규칙(`just_pick_it_db/services/*`)은 양측 합의 영역.

읽는 순서: `fleet_manager_node.py` → `fleet_api_server.py` → `fleet_repository.py` → `task_manager.py` → `robot_command_gateway.py` → `traffic_manager.py` → `robot_state_monitor.py`.

---

## 3. 실행 / 기동

고정 실행 기준: **Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12 / FastAPI 0.101 / Pydantic 1.10**.

```bash
cd ~/just_pick_it
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./run_all.sh        # PostgreSQL 확인 -> Fleet Manager(+Fleet API :8100) -> Web Gateway(:8000)
```

Web만 단독 기동은 `web/scripts/run.sh`(단, 화면 데이터는 Fleet Manager가 떠 있어야 조회됨).

설정: `src/just_pick_it/fleet_manager/config/fleet_manager.yaml`

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `robot_ids` | PICKY1/2, COBOT1/2 | 관리 대상 로봇 |
| `waiting_work_poll_period_sec` | 5.0 | 대기 작업 polling 주기 |
| `robot_state_flush_period_sec` | 1.0 | 텔레메트리 -> DB 반영(coalesce) 주기 |
| `reconcile_delay_sec` | 2.0 | 기동 후 재시작 복구 1회 실행 지연 |
| `api_host` / `api_port` | 0.0.0.0 / 8100 | Fleet API 바인딩 |
| `api_push_interval_sec` | 1.0 | WebSocket 스냅샷 push 주기 |

기동 순서: FleetRepository → RobotCommandGateway → (zone 좌표 조회) → TrafficManager → TaskManager → RobotStateMonitor → poll 타이머 → FleetApiServer → **reconcile one-shot 타이머**(spin 이후 1회 재시작 복구).

---

## 4. 전체 워크플로

### 4.1 5초 polling의 의미

"무조건 5초마다 가져온다"가 아니라 "5초마다 확인하되, 새 작업을 받을 수 있는 PICKY가 IDLE/STANDBY일 때만 주문/진열 polling을 연다". 정상 task 진행은 polling을 기다리지 않고 Action result(`handle_task_result`)에서 즉시 다음 단계로 넘어간다. polling은 신규 작업과 막혀 있던 흐름을 다시 확인하는 보정 진입점이다.

`check_waiting_work()` 한 사이클: 재시작 복구 재동기 → 충전 완료 정리 → 기존 주문/진열 flow 다음 task → 신규 ORDER_WAIT/REQUESTED 처리 → 실행 가능 ASSIGNED dispatch.

### 4.2 주문 흐름

```text
POST /api/orders (Web Gateway 프록시)
  -> FleetRepository.create_order(): 재고 검증/차감, order/order_item 생성, status=ORDER_WAIT
  -> (polling) TaskManager._process_new_order()
       1. _select_available_unit(): 같은 unit_id의 PICKY/COBOT pair, 배터리 높은 unit 우선
       2. update_order_status(assigned_unit_id)
       3. get_order_work(): item/zone/robot 이름 정규화
       4. _create_next_product_tasks(): TrafficManager로 가장 가까운 PRODUCT_ZONE 예약
          -> MOVE_TO_PRODUCT(PICKY) + SORTING_AND_LOAD(COBOT) 생성
       5. _dispatch_ready_tasks()
```

상품 task가 모두 SUCCESS → 남은 WAITING item 있으면 다음 상품 task, 없으면 pickup task(MOVE_TO_PICKUP/INSPECTION/UNLOAD) 생성. `UNLOAD` SUCCESS 시 `order=PICKUP_READY`, `pickup_slot=OCCUPIED`. 고객 수령(`POST /api/orders/{id}/complete`) 시 `COMPLETED` + slot `EMPTY`. pickup까지 끝나면 RETURN_HOME/DOCK_IN/CHARGE housekeeping을 이어 만든다(다음 작업이 있거나 배터리 충분하면 복귀 생략).

### 4.3 진열 흐름

진열 요청(`display_item`)은 창고에서 상품을 꺼내 진열 구역에 채우는 **진열 task** 흐름으로 처리한다. 모든 task는 주문과 무관하므로 `order_id`/`order_item_id` 없이 `display_item_id`로만 연결된다. 창고에서 상품을 선별·적재하는 단계는 주문 흐름의 `SORTING_AND_LOAD`를 재사용한다(`display_item` 기준).

```text
POST /api/admin/display-items -> create_display_item(): status=REQUESTED
  -> (polling) TaskManager._process_new_display_item()
     MOVE_TO_STOCK(PICKY)                창고 구역 이동
     -> SORTING_AND_LOAD(COBOT)          창고 상품 선별 + PICKY 적재
     -> MOVE_TO_DISPLAY(PICKY)           진열 구역 이동
     -> DISPLAY_SCAN(COBOT)              진열대 빈자리 탐색
     -> DISPLAY_PLACE(COBOT)             진열 상품 진열
```

`DISPLAY_PLACE` SUCCESS 시 `apply_display_success`가 **계획값**(`display_item.stock_delta`)으로 `product.stock_qty`를 반영한다(결정 D4: 비전 실측 경로 미구현).

### 4.4 emergency / resume

```text
POST /api/admin/emergency-stop|resume
  -> FleetManagerNode.trigger_emergency_stop()
     - DB 전이: FleetRepository.apply_emergency_stop()/apply_resume()
       (모든 robot EMERGENCY_STOP / RUNNING task PAUSED, 또는 그 역)
     - 로봇 전파: RobotCommandGateway.set_emergency_stop() -> /{ns}/emergency_control
     - TaskManager.handle_emergency_stop()/handle_resume(): dispatch 게이트 제어
```

### 4.5 로봇 상태 반영 (ROS2 텔레메트리)

```text
PICKY State Manager
  -> /pickyX/picky_state (String)        -> RobotStateMonitor -> TrafficManager.notify_state() (즉시)
  -> /pickyX/battery/percent (Float32)   -> RobotStateMonitor (캐시)
  -> /pickyX/amcl_pose (PoseWithCov)     -> RobotStateMonitor (캐시)
  -> 1Hz coalesce -> FleetRepository.update_robot_state(picky_state, battery_level, pos_*)
  -> battery 30% 초과 진입 시 1회 -> TaskManager.handle_battery_update()
```

**`robot_status`는 task 전이(`workflow_service`)만 기록**한다(결정 D2). 텔레메트리는 `picky_state`/battery/pose만 갱신한다. 자세한 토픽 계약은 `Fleet_manager_interface.md` 참고.

---

## 5. 재시작 복구 (R1 / A'')

Fleet Manager가 RUNNING task 도중 재시작되면 in-memory 점유가 비고, ROS Action goal handle이 소실된다(재접속 불가). 복구 원칙: **DB/로봇을 진실로 보고, 로봇은 건드리지 않으며, 로봇 현재 위치 기준으로 점유만 다시 세운 뒤 텔레메트리로 완료를 재동기한다.**

```text
기동 후 reconcile one-shot 타이머 -> TaskManager.reconcile_on_startup() (1회):
  1. EMERGENCY 상태였으면 게이트 닫은 채 종료(재개는 admin resume)
  2. repo.list_recovery_tasks()로 RUNNING task + 로봇 현재 pose/state 조회
  3. RUNNING MOVE/DOCK: 로봇 현재 zone(nearest_zone(pose)) 기준으로 reserve_path 재예약
     CHARGE: nearest_dock으로 도크 점유(rebuild_dock)만 복원
     -> MOVE/DOCK/COBOT은 _recovering(도착 재동기 대상)으로 등록
  4. 게이트 해제 후 ASSIGNED dispatch

이후 poll의 _resync_recovering_tasks():
  - 로봇 도착(picky_state == 도착 상태 / 위치 일치) -> SUCCESS 처리 + 다음 단계 진행
  - 도착 신호 없이 타임아웃(기본 120s) -> FAILED + 재계획
```

재배차/재발행을 하지 않으므로 "stale source로 되돌아가 재주행"하거나 "예약 경로 ≠ 실제 경로"가 되는 충돌 위험이 없다. 설계 배경과 대안(A/A'/B) 비교는 git 이력 및 `Fleet_manager_TODO.md`의 R1 항목 참고.

---

## 6. 통합 배경 (왜 이 구조인가)

원래 Control Server(FastAPI 웹)가 DB를 단독 소유하고, Fleet Manager(ROS2)는 모든 읽기/쓰기를 HTTP로 처리했다(`/api/fleet/*` 브리지). 이 chatty 통신은 제어 루프에 지연·실패 지점을 늘리고 상태 전이 규칙을 HTTP 양쪽에서 이중 관리하게 만들었다.

통합 방향(확정):

- DB 소유권을 Fleet Manager로 단일화. 기존 SQLAlchemy 자산(`models`, `services/*`)을 `just_pick_it_db` 패키지로 이사해 재사용(재작성 아님).
- 웹은 DB 비접근 Web Gateway로 분리, 읽기·쓰기 모두 Fleet API(HTTP/REST + WebSocket) 경유.
- `MultiThreadedExecutor` + SQLAlchemy 안전을 위해 콜백/요청 단위 `session_scope`(스레드별 세션).
- 로봇 제어는 HTTP 스레드에서 rclpy를 직접 호출하지 않고 노드 메서드로 위임.

이 구조는 "API 서버만 DB를 만지고 프런트엔드는 DB에 직접 붙지 않는다"는 표준 3계층 패턴이다. 로봇이 수십 대로 늘면 메시지 큐 재검토 여지가 있으나 현재 2대 규모에선 HTTP/REST로 충분하다.

---

## 7. 검증

```bash
source /opt/ros/jazzy/setup.bash && source install/setup.bash
colcon build --packages-select fleet_manager --symlink-install
python3 -m pytest -q src/just_pick_it/fleet_manager/test/
web/.venv/bin/python -m compileall -q web/app
```

남은 실로봇 검증 항목과 작업 현황은 `Fleet_manager_TODO.md` 참고.
