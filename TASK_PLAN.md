# TASK_PLAN

작성일: 2026-05-22

이 문서는 Fleet Manager 내부에서 Task 담당자가 위에서부터 순서대로 보며 구현하기 위한 작업 계획서다.
실제 수행 로그는 `STATUS.md`에 짧게 남긴다.

## 0. 읽는 방법

아래 순서대로 진행한다.

```text
1. ControlServerClient
2. TaskManager 골격
3. 주문 감지/robot unit 배정
4. 상품 dict 생성
5. TrafficManager로 다음 상품 선택
6. 선택된 상품 task 생성
7. 픽업 슬롯 배정과 픽업 task 생성
8. task 실행 상태 전이
9. RobotCommandGateway 연결
10. Emergency/Resume 전파
11. 작업 종료 후 복귀/충전
12. 입고 task 확장
```

각 단계의 성공 기준을 만족한 뒤 다음 단계로 넘어간다.

## 1. 작업 경계

직접 구현 대상:

- `src/just_pick_it/fleet_manager/fleet_manager/control_server_client.py`
- `src/just_pick_it/fleet_manager/fleet_manager/task_manager.py`
- `src/just_pick_it/fleet_manager/fleet_manager/robot_command_gateway.py`
- `src/just_pick_it/fleet_manager/fleet_manager/fleet_manager_node.py`의 조립/이벤트 연결부
- Task 담당 계획/문서

직접 수정 금지:

- `src/just_pick_it/fleet_manager/fleet_manager/traffic_manager.py`
- `src/just_pick_it/fleet_manager/fleet_manager/robot_state_monitor.py`
`fleet_manager_node.py`는 공동 조립 지점이다. 수정할 때는 Task/Traffic 양쪽 계약을 깨지 않는 범위에서만 변경한다.

## 2. 최종 운영 흐름

목표 흐름:

```text
주문 생성
  -> Control Server가 orders/order_item 저장
  -> Fleet Manager가 주문 감지
  -> TaskManager가 배정 가능한 robot unit 선택
  -> ControlServerClient가 주문 item을 dict로 정규화
  -> TaskManager가 남은 상품 후보를 `{zone_name: 수량}` dict로 집계해 TrafficManager에 전달
  -> TrafficManager가 reserve_nearest_from()으로 가장 가까운 상품 zone 선택
  -> TaskManager가 선택된 zone을 item dict와 매칭
  -> TaskManager가 해당 상품의 MOVE_TO_PRODUCT/SORTING_AND_LOAD task 생성
  -> 남은 item dict 갱신
  -> 모든 상품 상차 완료 후 EMPTY pickup slot 후보 조회
  -> TaskManager가 빈 pickup zone 후보를 `{zone_name: 1}` dict로 만들어 TrafficManager에 전달
  -> TrafficManager가 reserve_nearest_from()으로 가장 가까운 pickup zone 선택
  -> TaskManager가 선택된 pickup zone에 대응되는 pickup slot을 RESERVED로 배정
  -> MOVE_TO_PICKUP/INSPECTION/UNLOAD 처리
  -> RobotCommandGateway가 로봇에 실행 명령 송신
  -> TaskManager가 결과를 받아 task/order/robot 상태 보고
```

## 3. TrafficManager 계약 가정

최신 TrafficManager는 아래 방향으로 맞춰진다고 가정한다.

- `estimate_path()`는 사용하지 않는다.
- 상품 후보 선택은 `reserve_nearest_from()`을 사용한다.
- 첫 상품 선택 호출은 `task_id=None`으로 가능하다.
- `reserve_nearest_from()`으로 임시 예약한 뒤, task 생성 후 `attach_task_id(robot_id, task_id)`로 연결한다.
- task 생성 실패 시 `release_path(robot_id, None)`으로 임시 예약을 해제한다.
- `PathResult`에는 `selected_zone` 필드가 없다.
- 선택된 zone은 `result.waypoints[-1]`로 확인한다.
- TaskManager는 선택된 zone을 이용해 item dict를 역매핑한다.
- `reserve_return_home_path()`는 `RETURN_HOME` 전용이며 standby zone 복귀까지만 담당한다.
- `reserve_dock_path()`는 `DOCK_IN` 전용이며 충전 도크 선택과 경로 예약을 담당한다.

TaskManager가 TrafficManager에 넘기는 값:

```text
robot_id
source_zone
candidates = {"PRODUCT_ZONE_1": 2, "PRODUCT_ZONE_3": 1}
```

TrafficManager가 TaskManager에 알려줘야 하는 값:

```text
ok
waypoints
cost
reason
```

## 4. 구현 전 정합성 점검

코드 작성 전에 확인한다.

- `task_id`는 DB `task.task_id SERIAL` 기준 int auto-increment다.
- robot 이름은 `PICKY1`, `COBOT1`, `PICKY2`, `COBOT2`를 쓴다.
- ROS namespace는 `/picky1`, `/cobot1`, `/picky2`, `/cobot2`를 쓴다.
- `*_ZONE_*`은 PICKY 정차 위치다.
- `*_SLOT_*`은 COBOT 작업 위치다.
- `PRODUCT_SLOT_*`에서 `PRODUCT_ZONE_*`으로 매핑할 기준이 필요하다.
- pickup 위치는 현재 seed 기준 `PICKUP_ZONE_1~2`, `PICKUP_SLOT_1~2` 두 개다.
- `PICKUP_SLOT_1`은 `PICKUP_ZONE_1`, `PICKUP_SLOT_2`는 `PICKUP_ZONE_2`에 대응한다.
- `DOCK_IN`을 쓰면 `db/schema.sql`, `web/app/models.py`, `web/app/schemas.py` enum이 모두 같아야 한다.
- task 상태 변경만으로 Control Server가 order/order_item/robot/stocking_item 상태를 일부 자동 반영한다.
- 그래도 TaskManager는 어떤 task를 언제 `RUNNING`/`SUCCESS`/`FAILED`로 바꾸는지 명확히 책임진다.

## 5. 1단계: ControlServerClient 구현

목표:

TaskManager가 HTTP URL과 raw JSON 구조를 몰라도 되게 만든다.

### 5-1. 공통 HTTP helper 정리

먼저 내부 helper를 만든다.

```text
_get_json(path, params=None)
_post_json(path, payload=None)
_patch_json(path, payload)
```

원칙:

- timeout을 반드시 건다.
- 실패하면 endpoint, status code, payload 맥락을 로그에 남긴다.
- 조용히 성공한 것처럼 처리하지 않는다.

성공 기준:

- 기존 `fetch_zone_coords()`도 helper를 써서 동작할 수 있다.

### 5-2. 주문 조회 메서드

구현 순서:

```text
list_waiting_orders()
list_order_tasks(order_id)
get_order_detail(order_id)
```

사용 API:

```text
GET /api/fleet/orders?status=ORDER_WAIT
GET /api/fleet/orders/{order_id}/tasks
GET /api/orders/{order_id}
```

성공 기준:

- `ORDER_WAIT` 주문 목록을 가져온다.
- 주문별 기존 task 유무를 확인한다.
- 주문 상세에서 order item 목록을 가져온다.

### 5-3. 배정/상태 변경/이벤트 메서드

구현 순서:

```text
update_order_status(order_id, ...)
update_robot_state(robot_name, ...)
update_task_status(task_id, ...)
create_task_event(task_id, ...)
list_pickup_slots(status="EMPTY")
assign_pickup_slot(order_id, slot_id 또는 pickup_zone_name)
create_exception(...)
complete_stocking(task_id, ...)
```

사용 API:

```text
PATCH /api/fleet/orders/{order_id}
PATCH /api/fleet/robots/{robot_name}
PATCH /api/fleet/tasks/{task_id}
POST  /api/fleet/tasks/{task_id}/events
GET   /api/fleet/pickup-slots
POST  /api/fleet/orders/{order_id}/assign-pickup-slot
PATCH /api/fleet/orders/{order_id}
POST  /api/fleet/exceptions
POST  /api/fleet/stocking/complete
```

성공 기준:

- 주문/로봇/task 상태 변경 요청을 보낼 수 있다.
- task 진행 이벤트를 남길 수 있다.
- 상품 상차 완료 후 EMPTY pickup slot 후보를 조회할 수 있다.
- TrafficManager가 선택한 pickup zone과 같은 번호의 pickup slot을 주문에 배정할 수 있다.
- `STOCKING_PLACE` 성공 후 입고 완료/재고 반영을 요청할 수 있다.
- 실패 시 어떤 대상이 실패했는지 로그로 추적 가능하다.

### 5-4. 주문 작업 dict 정규화

구현 메서드:

```text
get_order_work(order_id)
```

반환 형태:

```python
{
    "order_id": 1,
    "order_no": "ORD-0001",
    "priority": 2,
    "assigned_unit_id": 1,
    "picky_name": "PICKY1",
    "cobot_name": "COBOT1",
    "pickup_slot_id": 1,
    "pickup_slot_name": "PICKUP_SLOT_1",
    "pickup_zone_name": "PICKUP_ZONE_1",
    "items": [
        {
            "order_item_id": 10,
            "product_id": 3,
            "product_name": "우유",
            "quantity": 2,
            "product_zone_name": "PRODUCT_ZONE_3",
            "product_slot_name": "PRODUCT_SLOT_3",
            "status": "WAITING",
        }
    ],
}
```

주의:

- ControlServerClient는 정규화까지만 한다.
- 상품 방문 순서와 task 생성 판단은 TaskManager가 한다.
- dict 필드는 TaskManager가 쓰기 쉽게 명확한 이름으로 둔다.

성공 기준:

- 주문 1건을 위 dict 형태로 만들 수 있다.
- `items`가 비어 있으면 실패로 처리하고 로그를 남긴다.
- 상품의 PICKY zone과 COBOT slot을 구분할 수 있다.

### 5-5. task 생성 메서드

구현 메서드:

```text
create_tasks_bulk(tasks)
```

사용 API:

```text
POST /api/fleet/tasks/bulk
```

성공 기준:

- task payload 목록을 보내고 `task_ids`를 받을 수 있다.
- 생성 개수와 요청 개수가 다르면 로그로 남긴다.

## 6. 2단계: TaskManager 골격 구현

목표:

아직 로봇 명령을 보내지 않고, 주문을 감지하고 처리할 수 있는 구조만 만든다.

구현 순서:

```text
TaskManager 클래스 생성
__init__(node, control_server, traffic_manager, robot_gateway=None)
_scheduler_lock 추가
check_waiting_work()
_process_waiting_work()
_collect_waiting_work()
_process_order(order_summary)
_process_stocking_item(stocking_summary)
```

`check_waiting_work()` 기본 형태:

```python
def check_waiting_work(self):
    if not self._scheduler_lock.acquire(blocking=False):
        self._node.get_logger().debug("[TaskManager] scheduler/preplan already running")
        return
    try:
        self._process_waiting_work()
    finally:
        self._scheduler_lock.release()
```

주문과 입고는 별도 루프가 아니라 하나의 대기 작업 큐로 합친다.

```python
@dataclass(frozen=True)
class WorkRequest:
    kind: str        # "ORDER" 또는 "STOCKING"
    work_id: int     # order_id 또는 stocking_item_id
    priority: int
    payload: dict
```

정렬 기준:

```text
priority 낮은 값 우선
같은 priority면 work_id 낮은 값 우선
같은 work_id면 kind 문자열 순서
```

성공 기준:

- `check_waiting_work()`가 중복 진입하지 않는다.
- `ORDER_WAIT` 주문과 `REQUESTED` 입고가 없으면 조용히 끝난다.
- 주문/입고가 동시에 있어도 같은 robot unit에 중복 배정하지 않는다.
- 예외가 나도 lock이 풀린다.

## 7. 3단계: 주문 감지와 중복 방지

구현 순서:

```text
orders = control_server.list_waiting_orders()
for order in orders:
    existing_tasks = control_server.list_order_tasks(order_id)
    if existing_tasks:
        skip
    else:
        _process_order(order)
```

성공 기준:

- 이미 task가 있는 주문은 다시 task를 만들지 않는다.
- Fleet Manager를 재시작해도 중복 task가 생기지 않는다.
- skip 사유가 로그로 남는다.

## 8. 4단계: robot unit 배정

구현 순서:

```text
사용 가능한 robot unit 찾기
orders.assigned_unit_id 갱신
PICKY/COBOT 이름 결정
필요 시 로봇 상태 BUSY 갱신
```

초기 규칙:

- `PICKY1/COBOT1`, `PICKY2/COBOT2`를 한 unit으로 본다.
- PICKY와 COBOT이 모두 신규 작업 가능해야 unit 배정 가능으로 본다.
- PICKY 후보는 `robot_status=IDLE` 또는 문서상 작업 가능 상태인 `CHARGING`/`RETURNING`까지 포함할지 정책을 정한다.
- 후보가 여러 개면 배터리 잔량이 높은 PICKY의 unit을 우선 배정한다.
- 배정 직후 로봇을 바로 `BUSY`로 바꿀지, 첫 task가 `RUNNING`이 될 때 바꿀지 정책을 정한다.
- 초기는 첫 task `RUNNING` 보고로 Control Server가 robot `BUSY/current_task_id`를 반영하게 두는 편이 단순하다.
- pickup slot은 이 단계에서 배정하지 않는다.
- pickup slot은 상품 상차가 끝난 뒤 `MOVE_TO_PICKUP` target을 정할 때 배정한다.

성공 기준:

- 주문 하나가 하나의 robot unit에 배정된다.
- PICKY/COBOT 이름을 확정할 수 있다.
- 사용 가능한 unit이 없으면 task를 만들지 않고 다음 `check_waiting_work()` 호출에서 재시도한다.

## 9. 5단계: 주문 item dict 관리

구현 순서:

```text
order_work = control_server.get_order_work(order_id)
remaining_items = order_work["items"]
zone_to_items = group remaining_items by item["product_zone_name"]
candidates = {zone_name: total_quantity for zone_name, items in zone_to_items.items()}
```

item 상태 후보:

```text
WAITING
RESERVED
MOVE_TASK_CREATED
SORTING_TASK_CREATED
DONE
FAILED
```

주의:

- 처음부터 item을 바로 삭제하지 말고 status를 바꿔 추적한다.
- 완전히 성공한 뒤에만 `DONE` 또는 목록 제거를 한다.
- 실패 복구가 필요하면 `WAITING`으로 되돌릴 수 있어야 한다.

성공 기준:

- 남은 상품 후보를 `{zone_name: 수량}` dict로 만들 수 있다.
- 선택된 zone으로 item을 정확히 찾을 수 있다.

## 10. 6단계: TrafficManager로 다음 상품 선택

구현 순서:

```text
candidates = {zone_name: 수량} 생성
traffic_manager.reserve_nearest_from(task_id=None, ...) 호출
실패하면 주문을 대기 상태로 두고 다음 `check_waiting_work()` 호출에서 재시도
성공하면 selected_zone = result.waypoints[-1] 확인
selected_item = zone_to_items[selected_zone]에서 이번에 처리할 item 선택
item["status"] = "RESERVED"
```

호출 형태는 Traffic 담당 최신 구현에 맞춘다.

```python
result = traffic_manager.reserve_nearest_from(
    robot_id=picky_name,
    task_id=None,
    source_zone=current_zone,
    candidates=candidates,
)
```

반환값에서 확인할 것:

```text
result.ok
result.waypoints[-1]
result.waypoints
result.reason
```

성공 기준:

- 후보 중 하나가 선택된다.
- 선택된 상품을 item dict에서 찾는다.
- 경로 실패 시 task를 만들지 않고 재시도 가능 상태로 남긴다.

## 11. 7단계: 선택된 상품 task 생성

구현 순서:

```text
selected_item 기준 MOVE_TO_PRODUCT payload 생성
같은 selected_item 기준 SORTING_AND_LOAD payload 생성
create_tasks_bulk([move_task, sorting_task])
응답 task_ids 저장
traffic_manager.attach_task_id(picky_name, move_task_id)
item status 갱신
remaining_items 갱신
```

task 생성 실패 보상:

```text
reserve_nearest_from(task_id=None) 성공 후 create_tasks_bulk 실패
  -> traffic_manager.release_path(picky_name, None)
  -> item status를 WAITING으로 되돌림
  -> 다음 `check_waiting_work()` 호출에서 재시도
```

`attach_task_id()` 실패 보상:

```text
create_tasks_bulk 성공 후 attach_task_id 실패
  -> 생성된 task를 취소/FAILED 처리하거나 보상 삭제 API가 필요함
  -> Traffic 예약 상태와 DB task 상태가 어긋난 것이므로 error 로그를 남김
```

task payload 기준:

```text
MOVE_TO_PRODUCT
  assigned_robot_name = PICKY*
  order_item_id = selected_item["order_item_id"]
  source_zone_id/name = current_zone
  target_zone_id/name = selected_item["product_zone_name"]

SORTING_AND_LOAD
  assigned_robot_name = COBOT*
  order_item_id = selected_item["order_item_id"]
  source_zone_id/name = selected_item["product_slot_name"]
  target_zone_id/name = PICKY가 대기 중인 product zone 또는 필요한 작업 위치
```

주의:

- source/target이 DB에서는 zone_id로 필요할 수 있으므로 name -> id 변환 경로를 ControlServerClient에 둔다.
- `sequence_no`는 TaskManager가 증가시킨다.
- task 생성 실패 시 Traffic 예약을 해제하거나 다음 `check_waiting_work()` 호출에서 정리할 수 있어야 한다.
- `MOVE_TO_PRODUCT` task 생성 후에는 반드시 `attach_task_id()` 성공 여부를 확인한다.

성공 기준:

- 선택된 상품에 대해 MOVE/SORTING task가 생성된다.
- 생성된 task_id를 item dict에 저장한다.
- TrafficManager 임시 예약이 `move_task_id`와 연결된다.
- 같은 상품 task가 중복 생성되지 않는다.

## 12. 8단계: 픽업 슬롯 배정과 픽업 task 생성

조건:

```text
remaining_items가 모두 DONE 또는 상차 task 생성 완료 상태
```

문서 기준으로 pickup slot은 주문 초반이 아니라 상품 상차가 끝난 뒤 배정한다.
현재 pickup 위치는 seed 기준 2개다.

```text
PICKUP_ZONE_1 <-> PICKUP_SLOT_1
PICKUP_ZONE_2 <-> PICKUP_SLOT_2
```

실행 구현에서는 모든 `SORTING_AND_LOAD` 성공 후 `MOVE_TO_PICKUP`을 실행하기 전에 배정한다.

구현 순서:

```text
모든 상품 상차 완료 확인
control_server.list_pickup_slots(status="EMPTY")
EMPTY slot을 `{pickup_zone_name: 1}` candidates dict로 변환
TrafficManager.reserve_nearest_from(robot_id, task_id=None, current_zone, candidates)
selected_pickup_zone = result.waypoints[-1] 확인
selected_pickup_zone에 대응되는 pickup_slot 확인
MOVE_TO_PICKUP target = selected_pickup_zone
MOVE_TO_PICKUP / INSPECTION / UNLOAD task payload 생성
create_tasks_bulk([...])
traffic_manager.attach_task_id(picky_name, move_to_pickup_task_id)
control_server.assign_pickup_slot(order_id, selected_slot_id)
```

생성할 task:

```text
MOVE_TO_PICKUP
INSPECTION
UNLOAD
```

주의:

- pickup slot이 없으면 `MOVE_TO_PICKUP`을 만들거나 실행하지 않는다.
- TrafficManager는 DB의 pickup slot 상태를 직접 알지 않는다.
- TaskManager가 Control Server에서 EMPTY pickup slot을 조회하고, 대응되는 pickup zone 후보 dict만 TrafficManager에 넘긴다.
- TrafficManager는 후보 pickup zone 중 경로 가능한 가장 가까운 zone을 고른다.
- TaskManager는 TrafficManager가 고른 pickup zone과 같은 번호의 pickup slot을 주문에 배정한다.
- `PICKUP_ZONE_2`를 골랐으면 `PICKUP_SLOT_2`가 `RESERVED` 되어야 한다.
- `reserve_nearest_from(task_id=None)`으로 `MOVE_TO_PICKUP` 경로를 먼저 잡았다면, task 생성 후 `attach_task_id()`를 호출한다.
- task 생성 또는 slot 배정 실패 시 `release_path(picky_name, None)` 또는 `release_path(picky_name, move_to_pickup_task_id)`로 예약 정리를 고려한다.
- 현재 `POST /api/fleet/orders/{order_id}/assign-pickup-slot`이 특정 slot 지정 없이 낮은 번호 EMPTY slot을 자동 배정한다면, selected slot을 지정하는 경로가 필요하다.
- 대안은 `PATCH /api/fleet/orders/{order_id}`로 `pickup_slot_id`를 직접 지정하거나, assign API를 selected slot 지정 가능하게 확장하는 것이다.
- Control Server는 `INSPECTION`이 `RUNNING`일 때 pickup slot 예약을 안전망으로 한 번 더 처리할 수 있지만, TaskManager가 선택한 zone/slot과 어긋나면 안 된다.

성공 기준:

- 모든 상품 task 뒤에 픽업/검수/하차 task가 이어진다.
- TrafficManager가 선택한 pickup zone과 같은 번호의 pickup slot이 `RESERVED`가 된다.
- `sequence_no`가 끊기지 않는다.
- order 상태를 `SORTING`, `DELIVERING`, `INSPECTING`, `PICKUP_READY`로 바꿀 지점을 구분한다.

## 13. 9단계: task 실행 상태 전이

목표:

DB에 생성된 task를 실제 실행 상태로 넘긴다.

구현 순서:

```text
다음 실행 가능한 task 선택
task ASSIGNED -> RUNNING
필요 시 create_task_event(TASK_STARTED)
robot current_task_id 갱신
RobotCommandGateway 호출
성공 result 수신
task RUNNING -> SUCCESS
필요 시 create_task_event(TASK_SUCCESS)
실패 result 수신
task RUNNING -> FAILED
필요 시 create_task_event(TASK_FAILED)
TrafficManager release 호출
다음 task로 진행
```

성공 기준:

- task 상태가 역행하지 않는다.
- `SUCCESS`, `FAILED`, `CANCELLED`, timeout 모두 path release 기준이 있다.
- task/order/robot 상태가 서로 어긋나지 않는다.
- task 시작/완료/실패 원인이 `task_event` 또는 task `result_message`로 추적 가능하다.

## 14. 10단계: RobotCommandGateway 구현

우선 구현:

```text
send_move_task(robot_name, task_id, task_type, waypoints)
cancel_task(robot_name, task_id)
set_emergency_stop(robot_names, enabled, reason, task_id, request_id)
```

나중에 구현:

```text
send_cobot_task(robot_name, task_id, task_type, payload)
```

콜백 흐름:

```text
Action feedback
  -> TrafficManager.update_path_progress(...)

Action result
  -> TaskManager.handle_task_result(...)
```

성공 기준:

- `MOVE_TO_PRODUCT` 한 종류를 end-to-end로 실행할 수 있다.
- action 실패/timeout이 task `FAILED`로 이어진다.
- 이동 중 waypoint progress를 TrafficManager에 전달할 수 있다.

## 14-1. Emergency/Resume 전파

목표:

Control Server의 emergency/resume 요청을 Fleet Manager가 받아 모든 PICKY/COBOT State Manager에 ROS2 service로 전파한다.

ROS2 service 계약:

```text
service type: just_pick_it_interfaces/srv/EmergencyControl
service name: /{robot_ns}/emergency_control
대상 namespace: /picky1, /picky2, /cobot1, /cobot2
```

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

흐름:

```text
Admin UI emergency/resume
  -> Control Server가 DB 상태 갱신
  -> Control Server fleet event websocket에 EMERGENCY_STOP 또는 RESUME push
  -> FleetManagerNode가 event 수신
  -> RobotCommandGateway.set_emergency_stop(...)
  -> 각 State Manager의 /{robot_ns}/emergency_control 호출
```

State Manager 책임:

```text
emergency_stop=true
  -> 즉시 안전 정지 또는 안전 정지 상태 진입
  -> 진행 중 action을 성공으로 속여 반환하지 않음
  -> robot 상태를 EMERGENCY_STOP 계열로 보고

emergency_stop=false
  -> emergency flag 해제
  -> 기존 action 재개 또는 새 action 수신 가능 상태로 전이
  -> 재개 불가하면 accepted=false와 reason message 반환
```

성공 기준:

- `EmergencyControl.srv`가 `just_pick_it_interfaces`에 정의되어 있다.
- `RobotCommandGateway`가 `/picky1`, `/picky2`, `/cobot1`, `/cobot2` service client를 lazy 생성한다.
- `FleetManagerNode`가 Control Server fleet event websocket을 통해 emergency/resume을 수신한다.
- service response의 `accepted/status/message`가 로그로 남는다.

## 15. 11단계: 작업 종료 후 복귀/충전

목표:

주문 또는 입고 작업이 끝난 뒤 같은 robot unit이 다음 작업을 받을지, 복귀/도킹/충전을 할지 결정한다.

문서 기준:

```text
UNLOAD 또는 STOCKING_PLACE 완료
  -> 대기 주문/입고 존재 여부 확인
  -> PICKY battery_level 확인
  -> 대기 작업이 있고 battery_level > 40 이면 다음 작업 반복
  -> 대기 작업이 없으면 RETURN_HOME(reason=PARKING)
  -> battery_level <= 40 이면 RETURN_HOME(reason=LOW_BATTERY)
  -> standby zone 도착
  -> PARKING이면 다시 대기 작업/배터리 확인
  -> LOW_BATTERY이거나 계속 복귀 필요하면 DOCK_IN
  -> CHARGE 상태 진입
  -> battery_level > 40 이면 CHARGE SUCCESS 후 신규 작업 배정 가능
```

task 후보:

```text
RETURN_HOME
DOCK_IN
CHARGE
```

TrafficManager 호출 기준:

```text
RETURN_HOME
  -> traffic_manager.reserve_return_home_path(robot_id, task_id, current_zone)
  -> result.waypoints[-1]은 STANDBY_ZONE_1 또는 STANDBY_ZONE_2

DOCK_IN
  -> traffic_manager.reserve_dock_path(robot_id, task_id, current_zone)
  -> result.waypoints[-1]은 CHARGING_DOCK_1 또는 CHARGING_DOCK_2
  -> DockCommand에는 dock_name과 start_zone_name만 전달한다
```

주의:

- `RETURN_HOME`은 standby zone 복귀까지로 본다.
- `RETURN_HOME`에는 `PARKING`과 `LOW_BATTERY` 사유를 구분해서 남긴다.
- `PARKING` 사유의 `RETURN_HOME`은 새 작업이 생기고 배터리가 충분하면 선점 취소 가능하다.
- `LOW_BATTERY` 사유의 `RETURN_HOME`은 `CHARGE SUCCESS` 전까지 신규 작업 배정 대상이 아니다.
- `DOCK_IN`은 ArUco 기반 정밀 도킹으로 분리한다.
- `CHARGING_DOCK_*`는 Control Server DB zone pose가 아니라 TrafficManager/State Manager가 공유하는 논리 도크 이름이다.
- 실제 도킹 이동은 STANDBY_ZONE에서 라인트레이싱/PID/ArUco 보정 같은 PICKY 로컬 제어로 처리한다.
- `CHARGE`는 별도 이동 Action이 아니라 충전 상태 유지 task로 본다.
- `CHARGE`는 battery_level이 40%를 넘으면 `SUCCESS` 처리한다.
- `reserve_dock_path()` 성공 후 `DOCK_IN` task가 실패하면 도크 점유가 남을 수 있다.
- TrafficManager 문서상 `release_dock` API는 아직 없으므로, DOCK_IN 실패 보상 정책을 Traffic 담당자와 확정한다.
- 배터리 임계값은 `40% 이하`면 충전 복귀, `40% 초과`면 신규 작업 가능으로 본다.

성공 기준:

- 작업 완료 후 다음 작업이 있으면 바로 이어서 수행할 수 있다.
- 다음 작업이 없으면 복귀/도킹/충전 흐름으로 빠질 수 있다.
- 복귀/도킹 실패가 `FAILED`와 exception으로 추적 가능하다.

## 16. 12단계: 입고 task 확장

LLM 메시지 파싱은 다른 담당자가 구현한다. TaskManager는 LLM 메시지를 직접 파싱하지 않는다.

권장 흐름:

```text
LLM 담당 모듈
  -> Control Server에 stocking_item 생성

TaskManager
  -> REQUESTED stocking_item 감지
  -> ControlServerClient.get_stocking_work(stocking_item_id)
  -> create_stocking_tasks_for_item(stocking_work)
  -> 입고 task 4개 생성
```

입고 task 기본 흐름:

```text
MOVE_TO_STOCK
STOCKING_PICK
MOVE_TO_STORAGE
STOCKING_PLACE
```

외부 모듈이 task 세부값을 직접 넣게 하지 않는다.

좋지 않은 방향:

```text
LLM 담당자가 task_type, source_zone, target_zone, robot_name을 직접 넘김
```

권장 방향:

```text
LLM 담당자는 stocking_item만 만들고,
TaskManager가 stocking_item을 읽어 정해진 규칙으로 4개 task를 만든다.
```

TaskManager 내부 함수 후보:

```python
def create_stocking_tasks_for_item(self, stocking_work: dict) -> list[int]:
    ...
```

`stocking_work` dict 후보:

```python
{
    "stocking_item_id": 7,
    "product_id": 1,
    "product_name": "우유",
    "requested_quantity": 5,
    "stocking_policy": "REQUESTED_QUANTITY",
    "priority": 2,
    "assigned_unit_id": 1,
    "picky_name": "PICKY1",
    "cobot_name": "COBOT1",
    "stock_zone_name": "STOCK_ZONE",
    "stock_slot_name": "STOCK_SLOT",
    "product_zone_name": "PRODUCT_ZONE_1",
    "product_slot_name": "PRODUCT_SLOT_1",
}
```

내부 helper 후보:

```python
def _build_task_payload(
    *,
    sequence_no: int,
    task_type: str,
    assigned_robot_name: str,
    stocking_item_id: int | None = None,
    order_id: int | None = None,
    order_item_id: int | None = None,
    source_zone_name: str | None = None,
    target_zone_name: str | None = None,
    priority: int = 2,
    status: str = "ASSIGNED",
    result_message: str | None = None,
) -> dict:
    ...
```

입고 task 생성 규칙:

```text
MOVE_TO_STOCK
  assigned_robot_name = PICKY*
  target = STOCK_ZONE

STOCKING_PICK
  assigned_robot_name = COBOT*
  source/target = STOCK_SLOT 또는 입고 작업 규칙에 맞는 위치

MOVE_TO_STORAGE
  assigned_robot_name = PICKY*
  target = product_zone_name

STOCKING_PLACE
  assigned_robot_name = COBOT*
  target = product_slot_name
```

성공 기준:

- LLM 담당 모듈은 task 세부 payload를 몰라도 된다.
- `REQUESTED` stocking item 하나가 입고 task 4개로 변환된다.
- `stocking_item_id` 기준으로 task가 생성된다.
- 주문 task와 입고 task의 상태 처리가 섞이지 않는다.
- `STOCKING_PLACE` 성공 후 `POST /api/fleet/stocking/complete`로 재고 반영을 완료할 수 있다.

## 17. 실패 처리 기준

반드시 로그와 상태로 남길 실패:

- Control Server 연결 실패
- API 응답 schema 불일치
- 사용 가능한 robot unit 없음
- pickup slot 없음
- 주문에 상품이 없음
- product zone/slot 매핑 실패
- TrafficManager 경로 선택 실패
- task 생성 실패
- reservation/task 연결 실패
- task_event 기록 실패
- 입고 완료/재고 반영 실패
- task status conflict
- action timeout
- sorting/inspection 실패

원칙:

- silent fallback 금지
- 재시도 가능한 실패와 즉시 실패를 구분한다.
- task 실패, order ERROR, exception report 중 어느 계층에 남길지 명확히 한다.

## 18. 첫 손코딩 체크리스트

처음 손으로 칠 때는 여기까지만 한다.

```text
[ ] ControlServerClient helper
[ ] list_waiting_orders()
[ ] list_order_tasks(order_id)
[ ] get_order_detail(order_id)
[ ] get_order_work(order_id)
[ ] update_order_status(order_id, ...)
[ ] list_pickup_slots(status="EMPTY")
[ ] create_tasks_bulk(tasks)
[ ] TaskManager 클래스
[ ] `check_waiting_work()` 재진입 방지 lock
[ ] ORDER_WAIT 주문과 REQUESTED 입고를 WorkRequest 큐로 합치기
[ ] 기존 task 있는 주문/입고 skip 처리
[ ] robot unit 선택과 assigned_unit_id 갱신
[ ] item dict 생성
[ ] candidates = {zone_name: 수량} dict 생성
[ ] reserve_nearest_from(task_id=None, candidates=...) 호출부
[ ] 선택된 상품 MOVE_TO_PRODUCT/SORTING_AND_LOAD task 생성
[ ] attach_task_id(picky_name, move_task_id) 연결
[ ] task 생성 실패 시 release_path(picky_name, None) 보상 처리
[ ] 중복 생성 방지 확인
```

첫 단계에서 아직 하지 않는다.

- 실제 로봇 Action 송신
- COBOT Action 송신
- retry queue 고도화
- `traffic_manager.py` 직접 수정
- `robot_state_monitor.py` 직접 수정
