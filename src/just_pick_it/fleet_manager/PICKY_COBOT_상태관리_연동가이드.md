# PICKY/COBOT 상태관리 연동 가이드

이 문서는 PICKY/COBOT State Manager 담당자가 Fleet Manager와 맞춰야 하는
실행 계약을 정리한다.

코드 구조와 리뷰 기준은 `FLEET_MANAGER_CODE_REVIEW.md`를 보고,
이 문서는 실제 로봇 쪽에서 “언제 어떤 상태/feedback/result를 보내야 하는지”를 볼 때 사용한다.

읽는 대상:

```text
- PICKY State Manager 담당자
- COBOT State Manager 담당자
- RobotCommandGateway / RobotStateMonitor 연결 담당자
- 데모 시나리오를 확인하는 팀원
```

목표는 하나다.

```text
State Manager는 실제 로봇 동작을 수행한다.
Fleet Manager는 task 생성, 경로 예약, 상태 보고, 다음 task dispatch를 담당한다.
Control Server는 DB와 UI 상태를 저장/표시한다.
```

## 기본 원칙

- PICKY/COBOT은 Control Server API를 직접 호출하지 않는다.
- 로봇은 Fleet Manager가 내린 ROS2 Action/Service 명령만 수행한다.
- task 성공 result는 실제 물리 동작이 안전하게 끝난 뒤에만 보낸다.
- emergency/resume은 `/{robot_ns}/emergency_control` service로 받는다.
- COBOT task의 SUCCESS는 `STOWING_ARM`까지 끝난 최종 완료를 의미한다.
- PICKY는 COBOT 작업 중 움직이면 안 된다.
- COBOT이 `STOWING_ARM`에 들어가면 Fleet Manager는 다음 PICKY 이동 task를 미리 생성/예약할 수 있다.
- 미리 생성된 다음 MOVE task는 이전 COBOT task SUCCESS 전에는 실행되면 안 된다.

## Robot / Namespace

| robot_name | ROS namespace | 역할 |
|---|---|---|
| `PICKY1` | `/picky1` | 이동 로봇 |
| `COBOT1` | `/cobot1` | robot_unit 1의 로봇팔 |
| `PICKY2` | `/picky2` | 이동 로봇 |
| `COBOT2` | `/cobot2` | robot_unit 2의 로봇팔 |

## Emergency / Resume Service 계약

Fleet Manager는 Control Server의 emergency/resume 요청을 받으면 각 State Manager에 ROS2 service를 호출한다.

```text
service name: /{robot_ns}/emergency_control
service type: just_pick_it_interfaces/srv/EmergencyControl
```

대상 service:

| robot_name | service |
|---|---|
| `PICKY1` | `/picky1/emergency_control` |
| `PICKY2` | `/picky2/emergency_control` |
| `COBOT1` | `/cobot1/emergency_control` |
| `COBOT2` | `/cobot2/emergency_control` |

`EmergencyControl.srv`:

```srv
bool emergency_stop
string reason
int32 task_id
string request_id
---
bool accepted
string status
string message
```

요청 의미:

| 요청 | 의미 |
|---|---|
| `emergency_stop=true` | 즉시 안전 정지 또는 emergency 상태 진입 |
| `emergency_stop=false` | emergency 해제와 재개 가능 상태 전이 |

State Manager가 해야 할 일:

1. `/{robot_ns}/emergency_control` service server를 항상 띄운다.
2. `emergency_stop=true`를 받으면 주행/팔 동작을 안전하게 멈추고 내부 emergency flag를 켠다.
3. emergency 중인 action을 성공으로 속여 반환하지 않는다.
4. 정지 상태를 Fleet Manager가 알 수 있도록 상태 topic 또는 result/event로 보고한다.
5. `emergency_stop=false`를 받으면 emergency flag를 끄고 재개 가능 상태로 전이한다.
6. 재개할 수 없으면 `accepted=false`, `status`, `message`로 사유를 반환한다.

Fleet Manager는 service response의 `accepted/status/message`를 로그로 남긴다.

주의:

- 이 service는 표준 `std_srvs/SetBool`이 아니다.
- reason/task_id/request_id를 남겨 emergency 원인과 당시 task를 추적하기 위해 커스텀 service를 사용한다.
- emergency 상태에서 기존 action을 pause할지 cancel/fail할지는 State Manager 구현 정책으로 정하되, 성공하지 않은 물리 동작을 SUCCESS로 보고하면 안 된다.

## Fleet Manager 내부 호출 경로

State Manager가 직접 Python 함수를 호출하는 것은 아니다. 실제 연결은
`RobotCommandGateway`와 `RobotStateMonitor`가 담당한다.

```text
PICKY Action feedback
  -> RobotCommandGateway
  -> TaskManager.handle_move_feedback(robot_name, task_id, current_waypoint_index)
  -> TrafficManager.update_path_progress(...)

PICKY Action result
  -> RobotCommandGateway
  -> TaskManager.handle_task_result({...})
  -> Control Server PATCH /api/fleet/tasks/{task_id}

COBOT Action feedback: STOWING_ARM
  -> RobotCommandGateway 또는 RobotStateMonitor
  -> TaskManager.preplan_after_cobot_stowing(task_id)
  -> 다음 MOVE task 생성/예약

COBOT Action result
  -> RobotCommandGateway
  -> TaskManager.handle_task_result({...})
  -> Control Server PATCH /api/fleet/tasks/{task_id}
```

## PICKY State Manager 계약

### 담당 task_type

| task_type | PICKY 상태 |
|---|---|
| `MOVE_TO_PRODUCT` | `MOVING_TO_PRODUCT` |
| `MOVE_TO_PICKUP` | `MOVING_TO_PICKUP` |
| `MOVE_TO_STOCK` | `MOVING_TO_STOCK` |
| `MOVE_TO_STORAGE` | `MOVING_TO_STORAGE` |
| `RETURN_HOME` | `RETURNING` |
| `DOCK_IN` | `DOCKING` |

### MoveCommand action goal 수신 시

Fleet Manager는 PICKY의 일반 경로 이동 task에 MoveCommand action goal을 보낸다.

현재 구현 기준:

```text
/{picky_ns}/move_command
just_pick_it_interfaces/action/MoveCommand
```

MoveCommand 대상 task:

```text
MOVE_TO_PRODUCT
MOVE_TO_PICKUP
MOVE_TO_STOCK
MOVE_TO_STORAGE
RETURN_HOME
```

State Manager가 해야 할 일:

1. goal을 받으면 task_type에 맞는 `picky_state`를 발행한다.
2. waypoint를 순서대로 수행한다.
3. waypoint 통과마다 feedback으로 `current_waypoint_index`를 보낸다.
4. 최종 목적지에 도착하면 result `success=true`를 보낸다.
5. 실패하면 result `success=false`와 실패 메시지를 보낸다.

주의:

- `DOCK_IN`은 MoveCommand로 받지 않는다.
- `RETURN_HOME`은 standby/pre-dock zone까지 이동하는 task다.
- 정밀 도킹은 별도 `DockCommand.action`으로 수행한다.

### DockCommand action goal 수신 시

Fleet Manager는 `DOCK_IN` task에 DockCommand action goal을 보낸다.

현재 구현 기준:

```text
/{picky_ns}/dock_command
just_pick_it_interfaces/action/DockCommand
```

DockCommand 의미:

```text
RETURN_HOME SUCCESS
  -> PICKY가 standby/pre-dock zone에 있음
  -> Fleet Manager가 DOCK_IN task dispatch
  -> PICKY State Manager가 ArUco/라인 기반 로컬 도킹 수행
  -> 도킹 완료 후 result success=true
```

State Manager가 해야 할 일:

1. goal을 받으면 `picky_state=DOCKING`을 발행한다.
2. goal의 `start_zone_name`에서 `dock_name` 방향으로 로컬 도킹 루틴을 시작한다.
3. 라인트레이싱, PID, ArUco 보정, 후진 주차 같은 정밀 도킹은 State Manager 내부 구현으로 처리한다.
4. 도킹 단계가 바뀌면 feedback의 `phase`, `progress`, `message`를 보낸다.
5. 도킹이 물리적으로 끝나면 result `success=true`를 보낸다.
6. 실패하면 result `success=false`와 실패 메시지를 보낸다.

주의:

- DockCommand는 TrafficManager waypoint 주행용이 아니다.
- `CHARGING_DOCK_*`는 Control Server DB zone pose가 아니라 TrafficManager/State Manager가 공유하는 논리 도크 이름이다.
- Fleet Manager는 DOCK_IN 전에 TrafficManager로 도크 예약만 잡고, 실제 정밀 진입은 PICKY State Manager에 맡긴다.
- `CHARGE`는 별도 주행 action이 아니라 충전 상태/배터리 상태로 완료 판단하는 logical task다.

### PICKY feedback 규칙

`current_waypoint_index`는 로봇이 통과한 waypoint index다.

```text
waypoints = [A, B, C, D]

A 출발 직후       -> current_waypoint_index=0 또는 feedback 생략 가능
B 통과            -> current_waypoint_index=1
C 통과            -> current_waypoint_index=2
D 도착 직전/도착  -> current_waypoint_index=3
```

Fleet Manager는 이 값을 `TrafficManager.update_path_progress()`로 넘긴다.
index 의미가 어긋나면 TrafficManager가 지나간 path를 너무 빨리 풀거나 늦게 풀 수 있다.

### PICKY result 규칙

PICKY result는 이동이 실제로 끝난 뒤에만 보낸다.

```text
MOVE_TO_PRODUCT result SUCCESS
  -> PICKY는 물건 앞에 도착한 상태
  -> 다음 COBOT task가 있으면 Control Server/UI는 PICKY를 WAITING_FOR_COBOT으로 표시
  -> PICKY는 새 MoveCommand를 받을 때까지 움직이지 않는다
```

주의:

- PICKY가 임의로 다음 위치로 이동하면 안 된다.
- 다음 MOVE task는 Fleet Manager가 새 Action goal로 내려준다.
- COBOT 작업 중 PICKY는 `WAITING_FOR_COBOT` 상태로 유지되어야 한다.

## COBOT State Manager 계약

### 담당 task_type

| task_type | 작업 상태 | 팔 복귀 상태 |
|---|---|---|
| `SORTING_AND_LOAD` | `SORTING` 또는 `LOADING` | `STOWING_ARM` |
| `INSPECTION` | `INSPECTING` | `STOWING_ARM` |
| `UNLOAD` | `UNLOADING` | `STOWING_ARM` |
| `STOCKING_PICK` | `STOCKING_SORTING` 또는 `STOCKING_LOADING` | `STOWING_ARM` |
| `STOCKING_PLACE` | `STOCKING_PLACING` | `STOWING_ARM` |

### Action goal 수신 시

COBOT 담당자가 `ExecuteTask.action`을 정의하면 Fleet Manager의
`RobotCommandGateway.send_cobot_task()`에 연결한다.

State Manager가 해야 할 일:

1. goal을 받으면 task_type에 맞는 `cobot_state`로 전이한다.
2. 실제 선별/상차/검수/하차/입고 작업을 수행한다.
3. 작업 동작이 끝나면 바로 SUCCESS를 보내지 말고 `STOWING_ARM`으로 전이한다.
4. `STOWING_ARM` 시작 feedback을 Fleet Manager가 받을 수 있게 보낸다.
5. 팔이 안전한 기본 자세로 완전히 복귀한 뒤 result `success=true`를 보낸다.
6. 실패하면 result `success=false`를 보내고 실패 사유를 포함한다.

## COBOT STOWING_ARM 선계획

`STOWING_ARM`은 다음 이동을 준비할 수 있는 시점이다.
하지만 PICKY가 움직일 수 있는 시점은 아니다.

```text
COBOT 작업 본동작 완료
  -> COBOT state/feedback = STOWING_ARM
  -> Fleet Manager: TaskManager.preplan_after_cobot_stowing(task_id)
  -> 다음 MOVE task 생성/경로 예약
  -> COBOT 팔 복귀 완료
  -> COBOT result SUCCESS
  -> 다음 MOVE task dispatch 가능
```

### task_type별 선계획 동작

| STOWING_ARM trigger | Fleet Manager가 미리 하는 일 |
|---|---|
| `SORTING_AND_LOAD` | 남은 상품이 있으면 다음 `MOVE_TO_PRODUCT/SORTING_AND_LOAD` 생성 |
| `SORTING_AND_LOAD` | 남은 상품이 없으면 `MOVE_TO_PICKUP/INSPECTION/UNLOAD` 생성 |
| `STOCKING_PICK` | 이미 생성된 다음 `MOVE_TO_STORAGE` 경로 선예약 |
| `INSPECTION` | 다음은 `UNLOAD`라 이동 선계획 없음 |
| `UNLOAD` | 주문 종료 task라 이동 선계획 없음 |
| `STOCKING_PLACE` | 입고 종료 task라 이동 선계획 없음 |

### 실패 시 보상

COBOT이 `STOWING_ARM`까지 갔지만 최종 실패하면 Fleet Manager는 다음을 수행한다.

```text
1. STOWING_ARM 중 미리 생성한 후속 task CANCELLED
2. STOWING_ARM 중 미리 예약한 TrafficManager path release
3. 실패 task FAILED
4. exception 기록
```

따라서 COBOT은 실패를 숨기고 SUCCESS를 보내면 안 된다.

## 주문 시나리오 상태 흐름

상품 2개 주문 예시:

```text
1. MOVE_TO_PRODUCT RUNNING
   PICKY1 = BUSY / MOVING_TO_PRODUCT

2. MOVE_TO_PRODUCT SUCCESS
   PICKY1 = BUSY / WAITING_FOR_COBOT
   COBOT1 = IDLE / STANDBY

3. SORTING_AND_LOAD RUNNING
   PICKY1 = BUSY / WAITING_FOR_COBOT
   COBOT1 = BUSY / SORTING 또는 LOADING

4. SORTING_AND_LOAD STOWING_ARM
   PICKY1 = BUSY / WAITING_FOR_COBOT
   COBOT1 = BUSY / STOWING_ARM
   Fleet Manager가 다음 MOVE_TO_PRODUCT를 미리 생성/예약

5. SORTING_AND_LOAD SUCCESS
   PICKY1 = IDLE / STANDBY
   COBOT1 = IDLE / STANDBY
   이미 생성된 다음 MOVE_TO_PRODUCT 실행 가능

6. 남은 상품 반복

7. 마지막 SORTING_AND_LOAD STOWING_ARM
   Fleet Manager가 MOVE_TO_PICKUP / INSPECTION / UNLOAD를 미리 생성/예약

8. MOVE_TO_PICKUP RUNNING
   PICKY1 = BUSY / MOVING_TO_PICKUP

9. INSPECTION RUNNING -> STOWING_ARM -> SUCCESS
   PICKY1 = BUSY / WAITING_FOR_COBOT
   COBOT1 = BUSY / INSPECTING -> STOWING_ARM -> STANDBY

10. UNLOAD RUNNING -> STOWING_ARM -> SUCCESS
    PICKY1 = BUSY / WAITING_FOR_COBOT
    COBOT1 = BUSY / UNLOADING -> STOWING_ARM -> STANDBY
    order = PICKUP_READY
```

## 입고 시나리오 상태 흐름

```text
1. MOVE_TO_STOCK RUNNING
   PICKY1 = BUSY / MOVING_TO_STOCK

2. MOVE_TO_STOCK SUCCESS
   PICKY1 = BUSY / WAITING_FOR_COBOT

3. STOCKING_PICK RUNNING
   PICKY1 = BUSY / WAITING_FOR_COBOT
   COBOT1 = BUSY / STOCKING_SORTING 또는 STOCKING_LOADING

4. STOCKING_PICK STOWING_ARM
   COBOT1 = BUSY / STOWING_ARM
   Fleet Manager가 MOVE_TO_STORAGE 경로 선예약

5. STOCKING_PICK SUCCESS
   PICKY1 = IDLE / STANDBY
   COBOT1 = IDLE / STANDBY
   MOVE_TO_STORAGE 실행 가능

6. MOVE_TO_STORAGE RUNNING
   PICKY1 = BUSY / MOVING_TO_STORAGE

7. MOVE_TO_STORAGE SUCCESS
   PICKY1 = BUSY / WAITING_FOR_COBOT

8. STOCKING_PLACE RUNNING -> STOWING_ARM -> SUCCESS
   PICKY1 = BUSY / WAITING_FOR_COBOT
   COBOT1 = BUSY / STOCKING_PLACING -> STOWING_ARM -> STANDBY
   stocking_item = COMPLETED
```

## 데모 확인 방법

Control Server를 `web/scripts/run.sh`로 실행한 뒤 다음을 실행한다.

```bash
web/.venv/bin/python src/just_pick_it/fleet_manager/demo_test.py --reset --scenario order
```

입고까지 같이 확인:

```bash
web/.venv/bin/python src/just_pick_it/fleet_manager/demo_test.py --reset --scenario both
```

기본 지연 시간:

```text
일반 task RUNNING: 3초
COBOT STOWING_ARM: 3초
```

빠른 검증:

```bash
web/.venv/bin/python src/just_pick_it/fleet_manager/demo_test.py \
  --reset --scenario both --delay-sec 0.1 --stow-delay-sec 0.1
```

## 현재 구현상 남은 연결점

현재 저장소 기준으로 남은 실제 로봇 연결 작업은 다음이다.

```text
1. COBOT ExecuteTask.action 정의
2. RobotCommandGateway.send_cobot_task() 구현
3. COBOT feedback에서 STOWING_ARM 감지
4. STOWING_ARM 감지 시 TaskManager.preplan_after_cobot_stowing(task_id) 호출
5. COBOT result SUCCESS는 STOWING_ARM 완료 후에만 반환
6. RobotStateMonitor의 cobot_state / task event 수신 확장
```
