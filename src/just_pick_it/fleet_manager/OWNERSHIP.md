# Fleet Manager 담당 분배

`fleet_manager` 패키지 내부의 책임을 Traffic 담당과 Task 담당으로 명확히 나눠
충돌과 의사결정 지연을 줄이기 위한 문서.

## 단일 ROS2 Node 원칙

`fleet_manager_node.py` 의 `FleetManagerNode` 만이 `rclpy.Node` 를 상속한다.
나머지 클래스(`TrafficManager`, `TaskManager`, `ControlServerClient`,
`RobotCommandGateway`, `RobotStateMonitor`) 는 일반 Python 클래스이며,
필요한 경우 생성자에서 `node` 를 받아 publisher / subscription / timer 를 만든다.

```text
FleetManagerNode  # 유일한 rclpy.Node
  ├── ControlServerClient
  ├── TrafficManager
  ├── TaskManager
  ├── RobotCommandGateway
  └── RobotStateMonitor
```

## 모듈별 담당

| 모듈 | 파일 | 담당 | 비고 |
|---|---|---|---|
| `TrafficManager` | `fleet_manager/traffic_manager.py` | **Traffic** | 외부 의존 0. 순수 path engine |
| `RobotStateMonitor` | `fleet_manager/robot_state_monitor.py` | **Traffic** | TrafficManager 입력 어댑터 |
| `TaskManager` | `fleet_manager/task_manager.py` (예정) | **Task** | 주문/입고 → task 변환, 상태 전이 |
| `RobotCommandGateway` | `fleet_manager/robot_command_gateway.py` (예정) | **Task** | task → 로봇 Action goal 전송 |
| `ControlServerClient` | `fleet_manager/control_server_client.py` | **회색지대** | 양쪽이 메서드 추가 |
| `FleetManagerNode` | `fleet_manager/fleet_manager_node.py` | **회색지대** | 조립자. 모듈 추가 시 양쪽 수정 |

## Traffic 담당이 결정·구현하는 것

### `TrafficManager` 본체
- zone graph 정의 (`ZONE_GRAPH`)
- BFS 경로 탐색 로직
- 충돌 회피 정책 (`MOVING_STATES`, `OCCUPYING_STATES`)
- 도크 자원 선정 (`DOCK_PRIORITY`, `_robot_dock`)
- 경로/예약 등록 상태 관리 (`_robot_paths`, `_robot_reservations`)
- 외부 계약: `estimate_path` / `reserve_path` / `reserve_nearest_from` /
  `reserve_return_home_path` / `update_path_progress` / `release_path` /
  `notify_state` / `get_robot_state` / `get_all_states`
- 도메인 제약 반영 (예: zone 단일 점유)

자세한 계약은 `TRAFFIC_MANAGER_API.md` 참고.

### `RobotStateMonitor`
- `/picky{N}/picky_state` 토픽 구독
- 받은 상태를 `TrafficManager.notify_state(robot_id, state)` 로 전달

## Task 담당이 결정·구현하는 것

### `TaskManager` 본체
- ORDER_WAIT 주문 polling
- robot_unit 선택, `orders.assigned_unit_id` 갱신
- 주문 → task payload 변환, task sequence_no 부여
- 주문 task: `MOVE_TO_PRODUCT`, `SORTING_AND_LOAD`, `MOVE_TO_PICKUP`, `INSPECTION`, `UNLOAD`
- 입고 task: `MOVE_TO_STOCK`, `STOCKING_PICK`, `MOVE_TO_STORAGE`, `STOCKING_PLACE`
- 기존 task 있는 주문 skip (재시작 시 중복 생성 방지)
- 다음 실행 가능한 task 선정 및 상태 전이
- 이동 task 실행 시 TrafficManager 호출 (reserve_path 등)
- 상품 후보 선정: 남은 상품 zone 리스트를 `reserve_nearest_from` 에 전달
- 선택된 zone → 상품 역매핑 (TaskManager 가 자체 보유)
- `release_path` 호출 시점 관리 (SUCCESS/FAILED/CANCELLED/timeout)
- task tick 재진입 방지 (`threading.Lock` 또는 `_ticking` 플래그)

### `RobotCommandGateway`
- TaskManager 의 task 객체를 PICKY State Manager 의 Action goal 로 변환
- ROS Action client 보유
- 로봇 응답을 task 결과로 환산

## 회색지대 — 양쪽이 함께 만지는 모듈

### `ControlServerClient`
- 두 담당자가 자기 영역에 필요한 메서드를 직접 추가
- 공용 헬퍼 (`_get_json`, `_patch` 등) 가 필요해지면 PR 단위로 합의
- 현재 구현: `fetch_zone_coords()` (Traffic 측 사용)
- 추가 예정 (Task 측): `list_waiting_orders`, `list_order_tasks`,
  `create_tasks_bulk`, `update_task_status`, `update_robot_state`,
  `assign_order_unit`, `create_exception`

### `FleetManagerNode`
- 새 모듈 추가 시 import + 생성자 호출 한 줄만 추가
- parameter 추가 / executor 설정 변경 시 짧은 PR 로 처리
- 의존성 순서: `ControlServerClient` 먼저 → 이를 받는 `TrafficManager`,
  `TaskManager` → 콜백 연결 (`RobotStateMonitor`, `RobotCommandGateway`)

### 협업 규칙
1. 회색지대 파일을 수정할 때는 PR 설명에 영향 범위 명시
2. 같은 파일을 동시에 수정 중일 때는 Slack/메신저로 알림
3. 큰 구조 변경 (메서드 시그니처, import 추가 등) 은 사전 협의

## 외부 모듈 (이 패키지 밖)

| 모듈 | 위치 | 담당 |
|---|---|---|
| PICKY `state_manager.py` | `pinky_amr_1/` | **미정** |
| PICKY `aruco_parking.py` | `pinky_amr_1/` | **Traffic 담당** |
| PICKY `move_to_goal.py` | `pinky_amr_1/` | **Task 담당** (잠정) |
| AprilTag perception | `just_pick_it_perception/` | 본 분배 대상 외 |
| Control Server (FastAPI) | `web/` | 본 분배 대상 외 |

PICKY 측 `state_manager.py` 담당이 정해지면 다음 항목을 그 담당자와 합의한다.

- `picky_state` enum (`STANDBY`, `MOVING_TO_PRODUCT`, `WAITING_FOR_COBOT`, ...)
  의 정확한 집합과 전이 규칙
- `picky_state` 토픽 발행 주기
- 로봇 상태 (battery, pose) 보고 경로 (Fleet Manager 경유 여부)
- `MoveCommand.action` 인터페이스 형태

## 합의 필요 사항 (미해결)

| 항목 | 결정 주체 | 비고 |
|---|---|---|
| `task_id` 타입 / 발급 시점 | Task 담당 | DB auto-increment 가정. TrafficManager 는 `int` 로만 사용 |
| `picky_state` enum 집합 | State Manager + Traffic | 정해지면 공용 상수 모듈 추가 검토 |
| `MoveCommand.action` 인터페이스 | Task + State Manager | 현재 `task_type + waypoints` 구조 유지할지 확장할지 |
| RETURN_HOME 의 도킹 포함 여부 | Task 담당 | RETURN_HOME task 가 standby_zone 도착까지인지 도킹까지인지 |
| 도크 leak 대응 (`release_dock` API) | Traffic 담당 | RETURN_HOME 실패 시 도크 점유 해제 정책 |
| `cost` 단위 (hop 수 vs 유클리드 거리) | Traffic 담당 | 현재 hop 수. 필요 시 거리 합으로 교체 |
| Polling 주기 | Task 담당 | 기본 5~10 초 권장 |
| Fleet Manager 자체 HTTP/WebSocket 서버 | 양쪽 | v0 보류 |

## 단계별 우선순위 (현재 v0 진입 전)

**Phase 0** (완료): `robot_id` 통일 → PICKY1/PICKY2.

**Phase 1** (완료): TrafficManager 외부 의존 분리. `ControlServerClient`,
`RobotStateMonitor` 신설. `PathResult` + 신 계약 도입.

**Phase 2** (Task 담당 시작): `TaskManager.tick()` 골격, ORDER_WAIT polling,
주문 → task 변환, `POST /api/fleet/tasks/bulk` 호출. TrafficManager / Gateway
연결은 보류.

**Phase 3** (양쪽 협업): `RobotCommandGateway` 신설. `MOVE_TO_PRODUCT` 한 종류만
reserve_path → action goal → 결과 수신 → release_path 까지 end-to-end 검증.

**Phase 4+**: 나머지 task type 확장, COBOT 명령, WebSocket 이벤트,
pose/battery 보고를 Fleet Manager 경유로 이동.

## 변경 이력

- 2026-05-22: 초안. Phase 1 완료 시점 기준 담당 분배 정리.
