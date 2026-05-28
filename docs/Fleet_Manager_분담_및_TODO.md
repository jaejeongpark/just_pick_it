# Fleet Manager 정리 — 담당 분담 및 TODO

작성일: 2026-05-28 (결정 D1·D2 및 S1 토픽 구성 확정 반영)

코드/문서 교차 점검 결과 발견한 항목을 담당 경계에 맞춰 사람별 할 일로 정리한다.

- 심각도: **S**(통합 환경 기능 결함) / **R**(견고성·운영) / **C**(명세·문서 정합) / **Q**(품질·테스트)
- 체크박스 `[ ]`는 아직 안 한 일, `[x]`는 완료.

---

## 0. 담당 경계

| 영역 | 파일 | 담당 |
|------|------|------|
| Fleet Repository | `fleet_manager/fleet_repository.py` | 박서우 |
| Traffic Manager | `fleet_manager/traffic_manager.py` | 박서우 |
| State Manager (PICKY) | `pinky_amr_1/.../state_manager.py` (picky2 동형) | 박서우 |
| Robot State Monitor | `fleet_manager/robot_state_monitor.py` | 박서우 |
| Web Service | `web/` | 이명제 |
| Task Manager | `fleet_manager/task_manager.py` | 이명제 |
| Robot Command Gateway | `fleet_manager/robot_command_gateway.py` | 이명제 |
| 회색지대(협의) | `fleet_manager_node.py`, `fleet_api_server.py`, `fleet_api_schemas.py`, `just_pick_it_db/services/*`, `just_pick_it_interfaces/*`, `docs/*` | 공동 |

---

## 1. 결정 현황

### 확정

| # | 결정 | 내용 |
|---|------|------|
| **D1** | 로봇 텔레메트리 단일 경로 = **ROS2 토픽** | State Manager 발행 → RobotStateMonitor 구독 → DB. HTTP 보고 경로 제거. |
| **D2** | `robot_status` 소유권 = **task 전이 전용** | `workflow_service`만 `robot_status`를 기록. 로봇 텔레메트리는 `picky_state`/battery/pose만 갱신. |

**D1 근거 (System Architecture 준수)**: `docs/3_System_Architecture.pdf`(ver_2.0)의 Software/System Architecture 다이어그램에서 **Fleet Manager ↔ AMR/Cobot Controller = ROS2(빨간색)**, HTTP(파란색)는 Browser ↔ Web Service ↔ Fleet Manager 구간 전용이다. 현재 State Manager의 HTTP `PATCH /api/fleet/robots/{id}` 보고는 이 구조를 거스르므로 ROS2 토픽 통일이 아키텍처 준수 요건이다.

### 미정 (회의 필요)

| # | 결정 | 선택지 | 관련 |
|---|------|--------|------|
| **D3** | `/api/fleet/*` 표면 유지 여부 | (유지+문서 갱신) vs (제거 후 정식 `/api/admin/*`로 대체) | C1, S1 회색지대 |
| **D4** | 입고 완료 시 재고 반영 출처 | 계획값(`stocking_item.stock_delta`, 현 동작) vs 비전 실측값(`complete_stocking`) | 입고 흐름 |

---

## 2. 박서우 TODO

### [S1] 로봇 상태 보고를 ROS2 토픽으로 통일 + `robot_status` 오염 제거  (최우선)

**문제**: State Manager가 1초마다 HTTP `PATCH /api/fleet/robots/{id}`로 `{"status": <picky_state 값>, battery, pos}`를 보고하고, Fleet API가 이 `status`를 `robot_status` 컬럼에 기록한다. 그 결과 `robot_status`가 비표준 값으로 매초 덮어써져 task 전이의 IDLE/BUSY와 충돌하고, `robot_status == "IDLE"` 조건이 깨져 새 작업 배정이 멈출 수 있다. 한편 `RobotStateMonitor`는 picky_state만 받아 traffic에만 전달하고 DB·battery·pose에는 반영하지 않는다.

**관련**: `state_manager.py:317-364`, `robot_state_monitor.py`(전체), `fleet_repository.py:600-652`(`update_robot_state`).

**토픽 구성 (확정)**

| 데이터 | 토픽 | 타입 | 발행 측 | 비고 |
|--------|------|------|---------|------|
| picky_state | `/pickyX/picky_state` | `std_msgs/String` | State Manager | 이미 구독 중 |
| battery | `/pickyX/battery/percent` | `std_msgs/Float32` | pinky_bringup `battery_publisher` | 이미 % 값 → int 반올림해 `battery_level`로 |
| pose | `/pickyX/amcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | AMCL | map frame. 우선 진행 확정 |

> - pose: `amcl_pose` 구독으로 우선 진행. robot별 namespaced TF(`map->base_link`) buffer fallback은 **실 로봇 테스트 후 결정(현재 보류)** — amcl_pose가 안 나오거나 갱신이 너무 드물면 도입.
> - battery: 드라이버는 `battery/percent`,`battery/voltage`(Float32)만 발행한다. State Manager가 구독하던 `battery_state`(BatteryState)는 발행 측이 없어 dead. **`battery/percent`를 권위 출처**로 쓴다.

**State Manager**
- [x] `_report_to_server` HTTP PATCH + `server_base_url` 파라미터 제거(아키텍처 위반 경로 제거). launch의 `server_base_url` arg도 제거.
- [x] 기존 `picky_state` 발행 유지. battery/pose는 재발행하지 않는다(소비자가 원시 토픽 직접 구독). `report_interval_sec` → `state_publish_interval_sec`로 개명(heartbeat publish 전용).
- [ ] (선택) HTTP 제거로 이제 write-only가 된 `_battery_pct`/`_pos_*`/`_update_pose`(10Hz TF)/`battery_state` 구독 정리 — 후속 cleanup.

**Robot State Monitor** — 구독 확장 + 1Hz coalesce 반영
- [x] picky_state / battery(`battery/percent`) / pose(`amcl_pose`) 구독 추가. robot별 최신값만 캐시.
- [x] picky_state 콜백은 **즉시** `traffic_manager.notify_state()` 호출(경로/도크 자동 해제는 지연되면 안 됨). 기존 동작 유지.
- [x] 1Hz 타이머로 캐시값을 `FleetRepository.update_robot_state(picky_state=..., battery_level=..., pos_x/y/theta=...)`로 변경분만 한 번에 반영(coalesce). **`robot_status`는 인자로 넘기지 않는다(D2).**
- [x] battery 값 변동 시 타이머에서 `task_manager.handle_battery_update(robot_name, level)` 호출.
- [ ] (선택) 일정 시간(예: 5s) telemetry 미수신 시 `robot_status=OFFLINE` 처리할지 — D2 예외로 별도 합의.

**Fleet Repository**
- [x] `update_robot_state`는 picky_state/battery/pos를 이미 지원 — 변경 없음. 1Hz 호출이라 세션 비용 문제 없음.

> 진행(2026-05-28): 위 코드 반영 완료. `fleet_manager_node`에서 TaskManager를 RobotStateMonitor보다 먼저 생성하도록 순서 변경 + battery hook 배선. 새 파라미터 `robot_state_flush_period_sec`(기본 1.0, `fleet_manager.yaml`) 추가. 빌드(fleet_manager·pinky_amr_1)·모듈 import·traffic 테스트 54개 통과. 실로봇 검증(amcl_pose 토픽 확인, robot_status가 IDLE로 유지되는지)은 남음.

**회색지대 (이명제와 협의, D3 연계)**
- [ ] `fleet_api_server`의 `PATCH /api/fleet/robots/{id}`를 로봇 보고 용도에서 제거. admin UI 수동 보정용으로 남길지는 별도 명시.

### [R1] 재시작 시 RUNNING task 경로 예약 복구 (Traffic 측)

재기동되면 `TrafficManager`의 예약 상태가 비어 있는데 DB엔 RUNNING task가 남는다. (이명제 [R1]과 한 쌍)

**관련**: `traffic_manager.py:165-180`.
- [ ] 시작 시 DB의 RUNNING MOVE/DOCK task로 예약을 재구성하는 진입점(예: `rebuild_reservation(robot_id, waypoints, task_id)`) 노출. 호출은 Task Manager reconcile에서 들어온다.

### [Q1] Fleet Repository / Traffic Manager 테스트

- [ ] `fleet_repository.py` 상태 전이·트랜잭션 테스트(주문 생성/완료, task 전이, emergency/resume) 추가. 현재 0.
- [ ] `traffic_manager` 테스트(621줄)에 재시작 복구([R1]) 케이스 추가.

### [점검] State Manager / Repository 정합 (확인만)

- [ ] State Manager가 발행하는 picky_state 값이 `MOVING_STATES`/`OCCUPYING_STATES`(`traffic_manager.py:76-88`)와 일치하는지 확인(불일치 시 도크/경로 자동 해제가 안 됨).
- [ ] `update_robot_state`의 picky_state/cobot_state 가드(`fleet_repository.py:623-631`)가 새 흐름과 맞는지 확인.

---

## 3. 이명제 TODO

### [S2] COBOT 디스패치 영구 스킵 버그

`_dispatch_cobot_task`는 send 실패 시 `_unsupported_task_warned`에 task_id를 영구 등록하고, 이후 호출에서 set에 있으면 send 시도조차 안 한다. 지금 `send_cobot_task`가 항상 False라 모든 COBOT task가 첫 디스패치에서 박혀, 나중에 `ExecuteTask` 서버가 떠도 재시도되지 않는다.

**관련**: `task_manager.py:2091-2104`.
- [ ] "서버 없음"을 task별 영구 상태로 기억하지 말 것. 로그 1회 억제는 robot_name/시간 기반으로 바꾸고, dispatch는 매 cycle 재시도 가능하게.

### [S3] 미연결 메서드 동작 확정

- [ ] `handle_battery_update`(`task_manager.py:1655`): 호출은 박서우(RobotStateMonitor)가 연결. 이명제는 호출 빈도에서 lock 경합·성능 확인.
- [ ] `preplan_after_cobot_stowing`(`task_manager.py:201`): 호출처는 COBOT 상태 감지(회색지대/미배정). 인터페이스 유지 여부 결정.

### [R1] 재시작 reconcile (Task Manager 측)

- [ ] 시작 시 DB RUNNING task를 읽어 (a) Traffic 예약 재구성(박서우 [R1] 인터페이스 호출) 또는 (b) 안전하게 FAILED 처리 후 재계획. 정책 선택.
- **관련**: `task_manager.py:1816-1841`(`_dispatch_ready_tasks`는 ASSIGNED만 처리).

### [R2] in-memory 자료구조 무한 증가 정리

- **관련**: `_completed_move_target_by_task`, `_unsupported_task_warned`, `_housekeeping_stopped_flows`(`task_manager.py:99-101`).
- [ ] 주문/입고 흐름 종료(완료/취소) 시 해당 키 제거.

### [Q2] Task Manager 정리/테스트

- [ ] 미사용 wrapper 제거: `_process_waiting_orders`(`:571`), `_process_requested_stocking_items`(`:1004`).
- [ ] task 생성/dispatch/housekeeping 흐름 단위 테스트 추가(현재 0).

### [Web] Web Service

- [ ] D3 결정에 따라 `admin.js`의 `/api/fleet/*` 호출 정리(유지/대체).
- [ ] LLM parser 실제 구현(`web/app/services/llm_client.py`).

---

## 4. 회색지대 (공동 결정 후 분담)

| 항목 | 내용 | 관련 파일 |
|------|------|-----------|
| [B3] emergency-stop가 HTTP 스레드에서 rclpy 직접 호출 | 통합계획 2.3/3.4 위배. executor로 위임 필요. 전파는 Gateway(이명제), 위임 mechanism은 node(회색) | `fleet_manager_node.py:164`, `robot_command_gateway.py:394` |
| [D4] 입고 완료 재고 반영 출처 | `complete_stocking`(박서우, Fleet Repo) 호출 여부 / 비전 detected_quantity 연결. 호출 지점은 Task Manager(이명제) 또는 비전 | `fleet_repository.py:1054`, `workflow_service.py:294` |
| [C1/D3] `/api/fleet/*` 제거 미이행 | 스펙 초안 §4·통합계획과 코드 불일치 | `fleet_api_server.py:172-283`, admin.js |
| [C2] 워크플로 문서 갱신 | `전체_워크플로.md` "Robot 상태 반영" 절이 실제 흐름과 다름 | `docs/전체_워크플로.md:454-474` |
| WebSocket push 방식 | 1초 전체 스냅샷 폴링 → 이벤트 기반 전환 여지 | `fleet_api_server.py:415` |
| `just_pick_it_db/services/*` | 상태 전이 규칙 변경 시 양측 합의 필요 | `workflow_service.py` 등 |

---

## 5. 권장 순서

1. ~~D1·D2 합의~~ **확정(ROS2 토픽 통일).** 남은 결정 **D3·D4는 회의 후 결정.**
2. 박서우 **[S1]**, 이명제 **[S2]** 동시 진행 (서로 독립).
3. **[S3] + battery hook 연결** — 박서우가 RobotStateMonitor에서 호출 연결, 이명제가 동작/시그니처 확인.
4. **[R1] 재시작 reconcile** — 박서우(Traffic 인터페이스) + 이명제(reconcile 정책) 합의.
5. [R2] / [B3] / [D4] / 문서([C1]·[C2]) / 테스트([Q1]·[Q2]).
