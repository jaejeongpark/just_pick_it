# Fleet Manager Code Review

이 문서는 Fleet Manager 구현 파일을 리뷰할 때 보는 기준 문서다.
PICKY/COBOT State Manager가 실제 로봇 쪽에서 지켜야 하는 실행 계약은
`PICKY_COBOT_상태관리_연동가이드.md`를 본다.

읽는 순서:

```text
1. 전체 책임 경계
2. 구현 상태 요약
3. 본인이 수정한 모듈 섹션
4. 다음 구현 시 확인할 것
```

대상 파일은 다음이다.

- `fleet_manager/control_server_client.py`
- `fleet_manager/task_manager.py`
- `fleet_manager/robot_command_gateway.py`
- `fleet_manager/fleet_manager_node.py`
- `fleet_manager/traffic_manager.py`
- `fleet_manager/robot_state_monitor.py`

관련 Control Server 상태 전이 파일은 다음이다.

- `web/app/services/workflow_service.py`

`traffic_manager.py`, `robot_state_monitor.py`는 Traffic 담당 영역이므로 이 문서에서는 상태와 계약을 중심으로 리뷰한다.

## 전체 책임 경계

```text
ControlServerClient
  -> Control Server HTTP API 호출과 응답 정규화

TaskManager
  -> 주문/입고 polling, task 생성, 상태 전이, STOWING_ARM 선계획, 실패 처리

TrafficManager
  -> zone graph 기반 경로 탐색, path 예약/해제

RobotCommandGateway
  -> task를 ROS2 Action/Service 명령으로 변환

RobotStateMonitor
  -> 로봇 상태 Topic 입력을 TrafficManager/TaskManager로 전달

workflow_service.py
  -> task RUNNING/SUCCESS/FAILED에 따른 robot/order/item 상태 전이
```

## 구현 상태 요약

| 모듈 | 현재 상태 | 판단 |
|---|---|---|
| `FleetManagerNode` | `ControlServerClient`, `TrafficManager`, `RobotStateMonitor`, `RobotCommandGateway`, `TaskManager` 조립, PICKY IDLE 상태에서만 대기 작업 polling timer 실행, fleet event WebSocket 수신 | 조립자 역할 구현됨 |
| `ControlServerClient` | HTTP helper, 주문/입고/task 조회, 상태 보고, task 생성, 정규화 helper | TaskManager 운용에 필요한 HTTP adapter 구현됨 |
| `TaskManager` | 주문/입고 통합 priority queue polling, unit 배정, 상품 반복 task 생성, pickup task 생성, PICKY 이동 dispatch, COBOT STOWING_ARM 선계획, 결과/실패 보상 처리 | PICKY 이동 중심 end-to-end 흐름 구현됨 |
| `RobotCommandGateway` | PICKY MoveCommand/DockCommand ActionClient, feedback/result callback, EmergencyControl ServiceClient | PICKY 이동/도킹과 emergency/resume 전파 구현됨, COBOT은 `ExecuteTask.action` 정의 대기 |
| `TrafficManager` | BFS path engine, reserve/release, nearest 선택, return/dock path | path engine 구현됨, 도크 실패 보상 정책은 별도 확정 필요 |
| `RobotStateMonitor` | PICKY picky_state String 구독 후 TrafficManager 전달 | Traffic 입력은 구현됨, battery/pose/task result 수신은 별도 확장 필요 |
| `workflow_service.py` | task 상태 변경 시 robot/order/order_item/stocking_item 상태 전이 | COBOT 작업 중 companion PICKY `WAITING_FOR_COBOT` 반영됨 |

## FleetManagerNode

### 역할

`FleetManagerNode`는 Fleet Manager 프로세스의 유일한 ROS2 Node다.
하위 모듈은 일반 Python class로 두고, 이 노드가 생성과 의존성 연결을 담당한다.

### 현재 구성

```text
FleetManagerNode
  ├── ControlServerClient
  ├── RobotCommandGateway
  ├── TrafficManager
  ├── RobotStateMonitor
  └── TaskManager
```

### 주요 파라미터

| 파라미터 | 의미 |
|---|---|
| `robot_ids` | Fleet Manager가 관리하는 전체 PICKY/COBOT 목록. `FleetManagerNode`가 이 중 PICKY만 골라 TrafficManager/RobotStateMonitor에 전달 |
| `server_base_url` | Control Server base URL |
| `waiting_work_poll_period_sec` | PICKY IDLE 상태에서 대기 주문/입고를 확인하는 polling 주기 |
| `fleet_event_ws_enabled` | Control Server fleet event WebSocket 수신 여부 |
| `fleet_event_reconnect_sec` | fleet event WebSocket 재연결 대기 시간 |

### 리뷰 포인트

- `FleetManagerNode`만 `rclpy.Node`를 상속하는가?
- 하위 매니저들이 각자 Node를 상속하지 않는가?
- Control Server URL과 대기 작업 polling 주기가 parameter로 빠져 있는가?
- TaskManager timer가 중복 생성되지 않는가?
- TrafficManager 상태 입력은 RobotStateMonitor callback을 통해 들어오는가?
- Control Server fleet event WebSocket이 끊겼을 때 재연결하는가?
- `EMERGENCY_STOP`/`RESUME` event가 RobotCommandGateway의 EmergencyControl service 호출로 변환되는가?

## ControlServerClient

### 역할

`ControlServerClient`는 Control Server와 통신하는 단일 HTTP adapter다.
TaskManager가 URL, HTTP method, JSON 파싱, timeout, status code 처리를 직접 알지 않도록 한다.

### 주요 섹션

```text
HTTP 공통 처리
Zone / Product 조회
Order 조회
Order / Robot / Task 상태 변경
Pickup Slot
Task 생성
Exception
Stocking
정규화 helpers
```

### 핵심 함수

| 함수 | 역할 |
|---|---|
| `_request_json()` | 모든 HTTP 요청의 timeout, 예외, status code, JSON 파싱 처리 |
| `get_snapshot()` | robot unit 배정을 위한 Fleet snapshot 조회 |
| `list_orders()` | 진행 중 주문 목록 조회 |
| `fetch_zone_coords()` | TrafficManager 초기화용 zone 좌표 dict 생성 |
| `list_waiting_orders()` | `ORDER_WAIT` 주문 polling |
| `list_order_tasks()` | 주문별 기존 task 조회, 중복 생성 방지 |
| `list_tasks()` | ASSIGNED/RUNNING task 조회, 실행 큐 판단 |
| `get_order_work()` | 주문 상세를 TaskManager용 dict로 정규화 |
| `create_tasks_bulk()` | task 목록을 Control Server에 일괄 생성 |
| `assign_pickup_slot()` | TrafficManager가 선택한 pickup zone에 맞는 pickup slot 배정 |
| `list_requested_stocking_items()` | `REQUESTED` 입고 item polling |
| `complete_stocking()` | 입고 완료와 재고 증가 보고 |

### 리뷰 포인트

- HTTP 실패가 조용히 성공처럼 처리되지 않는가?
- API 응답 schema가 틀렸을 때 로그가 남는가?
- `PRODUCT_SLOT_*`과 `PRODUCT_ZONE_*`을 구분하는가?
- pickup slot 자동 배정 API 대신 선택된 slot을 직접 배정하는가?
- TaskManager 판단 로직이 이 파일로 들어오지 않았는가?

## TaskManager

### 역할

`TaskManager`는 Fleet Manager의 작업 흐름 중심이다.
주문/입고 요청을 감지하고, robot unit을 배정하고, task row를 만들고,
TrafficManager 예약과 DB task_id를 연결한다.

### 주요 섹션

```text
Tick 진입점
COBOT STOWING_ARM lookahead planning
주문/입고 통합 polling / 중복 방지
Robot unit 배정
주문 상품 task 생성
입고 task 생성
Task payload / zone 변환
RobotCommandGateway callback
```

### 현재 구현 범위

현재 `TaskManager`는 task 생성에서 끝나지 않고, task result callback을 기준으로 다음 task 묶음 생성과 dispatch까지 즉시 이어간다.
`FleetManagerNode`는 주기마다 `TaskManager.has_idle_picky_for_waiting_work()`를 먼저 확인한다.
PICKY가 새 작업을 받을 수 있는 `IDLE/STANDBY` 상태일 때만 `check_waiting_work()`를 호출한다.
`check_waiting_work()`는 기존 task를 진행시키는 메인 루프가 아니라, 작업 가능한 unit이 있을 때 신규 주문/입고와 경로 차단 등으로 멈춰 있던 flow를 다시 확인하는 polling 진입점이다.

```text
구현됨:
- ORDER_WAIT 주문 polling
- REQUESTED stocking_item polling
- ORDER_WAIT/REQUESTED를 `WorkRequest` priority queue로 합쳐 처리
- 기존 task 있는 주문 skip
- IDLE robot unit 선택
- 같은 polling cycle에서 같은 robot unit 중복 배정 방지
- 주문에 assigned_unit_id 기록
- 상품 후보를 TrafficManager.reserve_nearest_from()으로 예약
- MOVE_TO_PRODUCT / SORTING_AND_LOAD task 반복 생성
- TrafficManager.attach_task_id() 연결
- 입고 task 4개 생성
- 기존 주문 task가 모두 SUCCESS이면 다음 상품 task 생성
- 모든 order_item이 SORTED이면 pickup slot 후보 조회
- TrafficManager가 가까운 pickup zone 선택
- MOVE_TO_PICKUP / INSPECTION / UNLOAD task 생성
- COBOT STOWING_ARM 진입 시 다음 이동 task 선계획
- STOWING_ARM 중 선계획한 task 실패 보상 처리
- 주문/입고 완료 후 다음 작업 가능 여부와 배터리를 판단해 housekeeping task 생성
- `RETURN_HOME -> DOCK_IN -> CHARGE` 단계적 생성
- `PARKING` 사유의 `RETURN_HOME`은 새 작업이 생기면 선점 취소 가능
- 기존 주문 advance가 새 MOVE/pickup task를 만들 때도 선점 가능한 `RETURN_HOME`을 먼저 취소
- `LOW_BATTERY` 사유의 `RETURN_HOME`은 `CHARGE SUCCESS` 전까지 신규 배정 차단
- `CHARGE` task는 별도 Action 없이 DB 상태를 `RUNNING`으로 두고 battery가 기준 초과 시 `SUCCESS` 처리
- RobotStateMonitor battery update hook으로 `CHARGE` task 즉시 SUCCESS 처리
- Fleet emergency stop 중 신규 polling/dispatch gate 차단
- Fleet resume 수신 시 가능한 task flow 즉시 보정/dispatch
- ASSIGNED task 순서 검증
- PICKY 이동 task RUNNING 전환 후 RobotCommandGateway dispatch
- MoveCommand feedback/result 처리 callback
- task SUCCESS result 수신 직후 해당 주문/입고 flow만 즉시 advance
- advance 직후 실행 가능한 ASSIGNED task 즉시 dispatch
- 이미 SUCCESS/FAILED/CANCELLED 된 task의 늦은 result는 stale result로 보고 무시

외부 인터페이스 대기 또는 별도 확정 필요:
- COBOT ExecuteTask.action 연결
- 실제 현재 zone 추적 고도화
```

### 핵심 함수

| 함수 | 역할 |
|---|---|
| `has_idle_picky_for_waiting_work()` | 대기 작업 polling을 열 수 있는 PICKY IDLE/STANDBY 상태가 있는지 확인 |
| `check_waiting_work()` | 신규 주문/입고 및 경로 차단으로 대기 중인 flow polling 진입점, lock으로 재진입 방지 |
| `handle_emergency_stop()` | emergency 중 신규 polling/dispatch를 막는 내부 gate 설정 |
| `handle_resume()` | resume 후 가능한 흐름을 즉시 보정하고 ASSIGNED task를 dispatch |
| `preplan_after_cobot_stowing()` | COBOT이 STOWING_ARM에 들어간 시점에 다음 이동 task를 미리 생성/예약 |
| `_preplan_after_sorting_and_load()` | 다음 상품 task 또는 pickup task를 STOWING_ARM 중 선생성 |
| `_pre_reserve_next_existing_move_task()` | 이미 존재하는 다음 MOVE task의 경로를 STOWING_ARM 중 선예약 |
| `_cancel_preplanned_after_cobot_failure()` | COBOT 실패 시 STOWING_ARM 중 만든 task와 path 예약을 정리 |
| `_process_waiting_work()` | `ORDER_WAIT` 주문과 `REQUESTED` 입고를 통합 priority queue로 처리 |
| `_collect_waiting_work()` | 주문/입고 대기 작업을 `WorkRequest` 목록으로 정규화하고 정렬 |
| `_process_waiting_orders()` | 호환용 wrapper. 신규 구현은 `_process_waiting_work()` 사용 |
| `_select_available_unit()` | 사용 가능한 PICKY/COBOT pair 선택 |
| `_create_next_product_tasks()` | 다음 상품 1개에 대한 이동/상차 task 생성 |
| `_advance_existing_orders()` | 기존 task 성공 여부를 보고 다음 상품/pickup task 생성 |
| `_advance_order_by_id_if_ready()` | result callback 직후 해당 주문만 다음 단계로 즉시 진행 |
| `_create_pickup_tasks()` | 빈 pickup slot 후보 중 TrafficManager 선택 결과로 pickup task 생성 |
| `_advance_existing_stocking_items()` | 입고 task 완료 후 housekeeping 흐름 진행 |
| `_advance_stocking_item_by_id_if_ready()` | result callback 직후 해당 입고 흐름만 다음 단계로 즉시 진행 |
| `_evaluate_housekeeping_decision()` | 대기 작업/배터리 기준으로 RETURN_HOME 필요 여부 판단 |
| `_create_next_housekeeping_task()` | RETURN_HOME / DOCK_IN / CHARGE를 단계적으로 생성 |
| `_complete_ready_charge_tasks()` | battery가 기준을 넘은 CHARGE task를 SUCCESS 처리 |
| `handle_battery_update()` | RobotStateMonitor 배터리 이벤트로 CHARGE 완료를 즉시 반영 |
| `_dispatch_charge_task()` | CHARGE task를 RUNNING으로 전환 |
| `create_stocking_tasks_for_item()` | 입고 item 1건을 task 4개로 변환 |
| `_dispatch_ready_tasks()` | 실행 가능한 ASSIGNED task 선택 |
| `_dispatch_move_task()` | PICKY 이동 task를 RUNNING으로 바꾸고 MoveCommand 전송 |
| `_build_task_payload()` | Control Server task bulk payload 생성 |
| `handle_move_feedback()` | Action feedback을 TrafficManager progress로 전달 |
| `handle_task_result()` | Action result를 SUCCESS/FAILED로 반영하고 성공 시 즉시 advance/dispatch 수행 |

### 리뷰 포인트

- `check_waiting_work()`와 `handle_task_result()`가 동시에 같은 flow를 수정하지 않도록 lock을 공유하는가?
- 정상 task 연결이 다음 polling 주기를 기다리지 않고 result callback에서 즉시 진행되는가?
- emergency stop 중 polling이나 result callback이 새 task를 dispatch하지 않는가?
- resume event 후 TaskManager가 가능한 ASSIGNED task를 즉시 dispatch하는가?
- CANCELLED task에 늦게 도착한 result가 최종 상태를 덮어쓰지 않는가?
- 기존 task가 있는 주문을 다시 생성하지 않는가?
- Traffic 예약 후 task 생성 실패 시 `release_path(robot, None)`을 호출하는가?
- `attach_task_id()` 실패 시 로그가 남는가?
- `TaskManager`가 HTTP path를 직접 알지 않는가?
- `TaskManager`가 ROS2 ActionClient를 직접 들고 있지 않는가?
- `STOWING_ARM` 선계획 task가 predecessor COBOT task SUCCESS 전에는 dispatch되지 않는가?
- predecessor COBOT task 실패 시 선계획 task가 `CANCELLED`되고 TrafficManager path가 해제되는가?
- PARKING 사유의 `RETURN_HOME`만 새 주문/입고로 선점 취소되는가?
- LOW_BATTERY 사유의 `RETURN_HOME`은 `CHARGE SUCCESS` 전까지 신규 배정이 막히는가?
- `HOUSEKEEPING_REASON` marker가 RUNNING/SUCCESS/FAILED 상태 변경 중에도 보존되는가?
- `RETURN_HOME` 마지막 waypoint가 DOCK_IN 시작 위치와 이어지는가?
- `DOCK_IN`은 waypoint 주행이 아니라 `dock_name/start_zone_name` 기반 로컬 도킹으로 처리되는가?
- RobotStateMonitor가 battery update 수신 시 `TaskManager.handle_battery_update(robot_name, battery_level)`를 호출하는가?

### STOWING_ARM 선계획 계약

`preplan_after_cobot_stowing(cobot_task_id)`는 COBOT 작업이 최종 SUCCESS 되기 전에
다음 이동 준비를 앞당기기 위한 hook이다.

```text
COBOT task RUNNING
  -> COBOT feedback/state: STOWING_ARM
  -> TaskManager.preplan_after_cobot_stowing(cobot_task_id)
  -> 다음 MOVE task 생성/예약 또는 기존 MOVE task 경로 선예약
  -> 현재 COBOT task SUCCESS 전까지는 sequence gate 때문에 실행 금지
  -> COBOT task SUCCESS 후 dispatch loop가 다음 MOVE task 실행
```

적용 대상:

| 현재 COBOT task | STOWING_ARM 중 하는 일 |
|---|---|
| `SORTING_AND_LOAD` | 남은 상품이 있으면 다음 `MOVE_TO_PRODUCT/SORTING_AND_LOAD` 생성. 없으면 `MOVE_TO_PICKUP/INSPECTION/UNLOAD` 생성 |
| `STOCKING_PICK` | 이미 생성된 다음 `MOVE_TO_STORAGE`의 TrafficManager path 선예약 |
| `INSPECTION` | 다음 task가 `UNLOAD`라 이동 preplan 없음 |
| `UNLOAD` | 주문 종료 task라 이동 preplan 없음 |
| `STOCKING_PLACE` | 입고 종료 task라 이동 preplan 없음 |

실제 로봇 연동에서는 COBOT Action feedback 또는 `RobotStateMonitor`가 `STOWING_ARM`
진입을 감지해 이 함수를 정확히 한 번 호출해야 한다.

## RobotCommandGateway

### 역할

`RobotCommandGateway`는 TaskManager의 task를 실제 ROS2 명령으로 바꾸는 출력 adapter다.
TaskManager가 action name, message type, PoseStamped 변환을 직접 알지 않도록 한다.

### 주요 섹션

```text
PICKY MoveCommand
COBOT ExecuteTask
EmergencyControl Service
Pose 변환 helpers
```

### 현재 구현 범위

```text
구현됨:
- /{picky_ns}/move_command ActionClient lazy 생성
- zone_name waypoint를 PoseStamped 배열로 변환
- MoveCommand goal 송신
- feedback callback을 TaskManager로 전달
- result callback을 TaskManager용 dict로 변환
- /{picky_ns}/dock_command ActionClient lazy 생성
- DOCK_IN task를 DockCommand goal로 송신
- DockCommand feedback 로그 기록
- DockCommand result를 TaskManager용 dict로 변환
- cancel_task()
- /{robot_ns}/emergency_control ServiceClient lazy 생성
- EmergencyControl request 전송
- response accepted/status/message 로그 기록

외부 인터페이스 대기 또는 추가 구현:
- ExecuteTask.action 파일 추가 후 COBOT ActionClient 구현
- COBOT feedback에서 `STOWING_ARM`을 감지해 `TaskManager.preplan_after_cobot_stowing()` 호출
```

### 핵심 함수

| 함수 | 역할 |
|---|---|
| `send_move_task()` | PICKY 이동 task를 MoveCommand.action goal로 전송 |
| `send_dock_task()` | PICKY DOCK_IN task를 DockCommand.action goal로 전송 |
| `cancel_task()` | active MoveCommand/DockCommand goal 취소 요청 |
| `_on_move_feedback()` | waypoint index feedback 전달 |
| `_on_move_result()` | action result를 dict로 변환해 TaskManager에 전달 |
| `_on_dock_feedback()` | DOCK_IN 진행 feedback을 로그로 기록 |
| `_on_dock_result()` | DOCK_IN action result를 dict로 변환해 TaskManager에 전달 |
| `set_emergency_stop()` | PICKY/COBOT State Manager에 emergency/resume service 요청 전파 |
| `_get_emergency_client()` | `/{robot_ns}/emergency_control` ServiceClient lazy 생성 |
| `_on_emergency_response()` | EmergencyControl response를 로그로 남김 |
| `_build_pose_waypoints()` | TrafficManager zone path를 PoseStamped 목록으로 변환 |
| `_robot_name_to_namespace()` | `PICKY1` -> `picky1` namespace 변환 |

### 리뷰 포인트

- `MoveCommand.action` goal에 없는 `task_id`를 억지로 싣지 않는가?
- `task_id`는 Gateway 내부 callback 매핑으로만 쓰는가?
- `DOCK_IN`을 MoveCommand가 아니라 DockCommand로 보내는가?
- `RETURN_HOME`은 standby/pre-dock zone 이동까지만 담당하는가?
- `DockCommand`가 DB zone pose를 요구하지 않고 `dock_name/start_zone_name`만 전달하는가?
- `DockCommand` action name이 `/{robot_ns}/dock_command`로 통일되어 있는가?
- zone pose가 없을 때 goal을 보내지 않는가?
- feedback index가 TrafficManager의 `update_path_progress()` 계약과 맞는가?
- COBOT 인터페이스 대기 상태가 silent fallback이 아니라 warn log로 드러나는가?
- COBOT `ExecuteTask.action` result는 `STOWING_ARM` 완료 후에만 SUCCESS를 반환하는가?
- COBOT `STOWING_ARM` feedback이 TaskManager 선계획 hook으로 연결되는가?
- emergency/resume은 표준 `SetBool`이 아니라 `EmergencyControl.srv`로 reason/task_id/request_id를 남기는가?
- service 이름이 `/{robot_ns}/emergency_control`로 통일되어 있는가?

## TrafficManager

### 역할

`TrafficManager`는 외부 I/O 없이 zone graph 기반 경로 탐색과 다중 PICKY path 예약을 담당한다.
TaskManager는 상품/주문 도메인을 알고, TrafficManager는 zone과 path만 안다.

### 현재 구현 상태

```text
완성에 가까운 부분:
- PathResult
- reserve_path()
- reserve_nearest_from()
- attach_task_id()
- reserve_return_home_path()
- reserve_dock_path()
- update_path_progress()
- release_path()
- notify_state()
- BFS 기반 차단 노드/엣지 계산

주의할 부분:
- DOCK_IN 실패 시 _robot_dock 예약 해제 정책이 아직 명시적으로 닫히지 않았다.
- update_path_progress()가 기대하는 waypoint index 의미와 MoveCommand feedback index 의미를 실제 주행에서 맞춰봐야 한다.
- 실제 현재 zone 추적은 아직 RobotStateMonitor/TaskManager 쪽과 완전히 연결되지 않았다.
```

### 리뷰 포인트

- TrafficManager가 Control Server API를 직접 호출하지 않는가?
- TrafficManager가 task/order/product 도메인을 알지 않는가?
- `reserve_nearest_from(task_id=None)` 후 `attach_task_id()` 흐름이 지켜지는가?
- task 생성 실패 시 TaskManager가 `release_path(robot_id, None)`을 호출하는가?
- DOCK_IN 실패 보상 정책이 별도 이슈로 관리되는가?

## RobotStateMonitor

### 역할

`RobotStateMonitor`는 로봇 상태 Topic을 구독하고, 필요한 모듈에 callback으로 전달하는 입력 adapter다.
현재 구현은 PICKY `picky_state`를 TrafficManager에 전달하는 데 집중되어 있다.

### 현재 구현 상태

```text
구현됨:
- /{picky_ns}/picky_state 구독
- std_msgs/msg/String 수신
- on_state_change(robot_id, state) callback 호출

확장 필요:
- battery 구독
- pose/zone 추적
- cobot_state 구독
- task result/event topic 수신
- COBOT `STOWING_ARM` 진입 감지 후 TaskManager preplan callback 호출
- Control Server robot 상태 보고 throttling
```

### 리뷰 포인트

- RobotStateMonitor가 TaskManager/TrafficManager 객체 전체를 강하게 잡지 않고 callback만 받는가?
- 상태 캐시가 필요하면 책임 위치가 명확한가?
- pose/battery를 매번 HTTP PATCH하지 않도록 throttle 정책이 있는가?

## Control Server workflow_service.py

### 역할

`workflow_service.py`는 Fleet Manager가 task 상태를 보고했을 때 Control Server DB의
runtime 상태를 갱신한다. 실제 데모와 Admin UI에서 보이는 robot/order/item 상태는
이 파일의 정책을 따른다.

### 현재 구현 상태

```text
구현됨:
- 주문 생성 시 ORDER_WAIT 전이
- task RUNNING 시 assigned robot BUSY 전이
- PICKY task RUNNING 시 picky_state 갱신
- COBOT task RUNNING 시 cobot_state 갱신
- COBOT task RUNNING 시 같은 unit의 PICKY를 BUSY / WAITING_FOR_COBOT으로 표시
- PICKY 이동 task SUCCESS 직후 다음 task가 COBOT이면 PICKY를 WAITING_FOR_COBOT으로 유지
- COBOT task SUCCESS/FAILED/CANCELLED 시 companion PICKY 대기 상태 해제
- SORTING_AND_LOAD SUCCESS 시 order_item SORTED
- INSPECTION SUCCESS 시 order_item INSPECTED
- UNLOAD SUCCESS 시 pickup_slot OCCUPIED, order PICKUP_READY
- STOCKING_PLACE SUCCESS 시 product.stock_qty 증가, stocking_item COMPLETED
```

### 리뷰 포인트

- COBOT 작업 중 PICKY가 `IDLE`로 풀리지 않는가?
- COBOT `STOWING_ARM` 중에도 PICKY가 `WAITING_FOR_COBOT`으로 유지되는가?
- COBOT 작업이 끝난 뒤 companion PICKY가 `IDLE / STANDBY`로 복구되는가?
- `STOCKING_PLACE` 성공이 중복 호출돼도 재고가 중복 증가하지 않는가?

## 다음 구현 시 확인할 것

1. COBOT Action 인터페이스
   - `ExecuteTask.action` 신규 정의 여부
   - goal/result/feedback 필드 확정
   - feedback에 `STOWING_ARM` 또는 동등한 phase 포함 필요
   - result SUCCESS는 `STOWING_ARM` 완료 후에만 반환

2. 현재 robot source zone
   - 현재 구현은 성공한 이동 task의 마지막 waypoint를 TaskManager 메모리에 보존해 후속 task 출발지로 사용한다.
   - Fleet Manager 재시작 후에는 DB target_zone 또는 standby zone fallback을 쓴다.
   - 실제 운용에서는 RobotStateMonitor의 현재 zone 보고를 연결하면 더 견고해진다.

3. COBOT result 처리
   - ExecuteTask.action result가 오면 `handle_task_result()`가 같은 방식으로 SUCCESS/FAILED를 반영한다.
   - STOCKING_PLACE 성공 시 `complete_stocking()` 호출 위치를 확정해야 한다.
   - STOWING_ARM 시작 시 `preplan_after_cobot_stowing()` 호출 경로를 확정해야 한다.

4. 긴급 정지/재개
   - `just_pick_it_interfaces/srv/EmergencyControl.srv`를 사용한다.
   - FleetManagerNode는 Control Server fleet event WebSocket의 `EMERGENCY_STOP`/`RESUME`을 수신한다.
   - RobotCommandGateway는 `/{robot_ns}/emergency_control` ServiceClient로 각 State Manager에 전파한다.
   - PICKY/COBOT State Manager는 같은 service server를 반드시 제공해야 한다.
