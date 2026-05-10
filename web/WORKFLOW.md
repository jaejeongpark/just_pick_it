# Just Pick It 주문-로봇 Workflow

이 문서는 주문이 들어온 뒤 어떤 상태가 어디에서 바뀌는지 한눈에 보기 위한 기준 문서입니다.

## 기준점

전체 흐름의 중심축은 **주문 상태(`orders.status`)**로 잡습니다.

이유:

- 고객은 주문 상태만 이해하면 됩니다.
- 관리자도 주문이 어디까지 갔는지를 먼저 보고, 필요할 때 task/robot을 확인합니다.
- Fleet Manager는 task와 robot을 실행하지만, 최종 목적은 주문을 다음 상태로 보내는 것입니다.

따라서 화면과 문서에서는 아래 순서로 봅니다.

```text
주문 상태(order)
  -> 작업 상태(task)
      -> 로봇 상태(robot)
          -> 픽업 슬롯 상태(pickup_slot)
```

## 역할 분리

```text
Customer UI
  - 상품 선택
  - 주문 생성
  - 주문 상태 확인
  - 픽업 완료 처리

Control Server / Web API
  - 주문, task, robot, pickup slot 상태를 DB에 저장
  - Fleet Manager가 보낸 상태 업데이트를 DB에 반영
  - DB 변경 API commit 후 고객/관리자 UI에 WebSocket broadcast

Fleet Manager / ROS2
  - 실제 로봇 작업을 실행
  - task queue, robot 배정, 상태 전이를 판단
  - 판단한 상태를 Control Server로 전달

Admin UI
  - 현재 주문, 로봇, 픽업 슬롯, 예외 상태 확인
  - 개발/시연 중 수동 상태 수정
  - 예외 처리, 긴급정지, 재개
```

## 실시간 반영 원칙

정식 연동 경로에서는 DB를 직접 수정하지 않고 Control Server API를 통해 수정합니다.

```text
Control Server API
  -> DB commit
  -> Admin WebSocket broadcast
  -> Customer WebSocket broadcast
```

`psql` 등으로 DB를 직접 수정하는 것은 개발/복구 전용입니다. 이 경우 WebSocket broadcast가 자동으로 발생하지 않으므로 새로고침이 필요할 수 있습니다.

## 외부 시스템 연동 원칙

Fleet Manager, Control Bridge, Vision Server, LLM Server는 DB에 직접 접근하지 않습니다.

```text
외부 시스템
  -> Control Server API 호출
      -> Control Server가 DB 변경
          -> WebSocket으로 Admin/Customer UI 갱신
```

역할 기준:

```text
Fleet Manager
  - task queue 판단
  - AMR/COBOT 배정
  - 주문/작업/로봇/픽업슬롯 상태 전이 결정
  - 결정 결과를 /api/fleet/* 로 보고

Vision Server
  - 인식/검수/감지 결과 생성
  - 단순 예외 알림은 /api/fleet/exceptions 로 보고 가능
  - 작업 성공/실패 판단이 필요한 결과는 Fleet Manager가 받아서 task/order 상태로 보고

LLM Server
  - 자연어 명령을 구조화
  - Control Server가 LLM 응답을 받아 Admin UI에 표시하거나 Fleet에서 사용할 명령 형태로 전달

Control Server
  - 외부 시스템이 보낸 결과를 저장
  - DB 변경 후 WebSocket broadcast
  - Fleet의 판단을 대신하지 않음
```

## 정상 주문 시나리오

예시 주문: `ORD-0007`

상품: Cola 1개, Snack 1개

사용 로봇:

```text
COBOT1 : 상품 선별
AMR1   : 상품 운반 + 픽업 슬롯 하차
COBOT2 : 상품 검수
```

AMR 배정 원칙:

```text
주문 1건당 AMR 1대를 배정한다.
같은 주문의 DELIVERY task와 UNLOAD task는 같은 AMR이 담당한다.
다른 주문은 Fleet Manager 판단에 따라 다른 AMR을 배정할 수 있다.
```

### 1. 고객이 주문하기 클릭

호출:

```text
POST /api/orders
```

고객 주문 직후 DB 변화:

```text
orders
  status: ORDER_RECEIVED

order_item
  status: WAITING

pickup_slot
  변화 없음
```

이후 Fleet Manager가 주문을 접수하고 task를 생성/시작합니다.

호출 예:

```text
POST  /api/fleet/tasks                 SORTING task 생성
POST  /api/fleet/tasks                 DELIVERY task 생성
POST  /api/fleet/tasks                 INSPECTION task 생성
POST  /api/fleet/tasks                 UNLOAD task 생성
PATCH /api/fleet/tasks/{sorting_id}    {"status":"RUNNING","assigned_robot_id":"COBOT1"}
POST  /api/fleet/tasks/{sorting_id}/events
PATCH /api/fleet/robots/COBOT1         {"status":"SORTING","current_task_id":sorting_id}
PATCH /api/fleet/orders/7              {"status":"SORTING"}
```

Fleet 접수 후 DB 변화:

```text
orders
  status: ORDER_RECEIVED -> SORTING

order_item
  status: WAITING

task
  SORTING    : RUNNING, assigned_robot_id=COBOT1
  DELIVERY   : QUEUED,  assigned_robot_id=AMR1
  INSPECTION : QUEUED,  assigned_robot_id=COBOT2
  UNLOAD     : QUEUED,  assigned_robot_id=AMR1

robot
  COBOT1.status: IDLE -> SORTING
  COBOT1.current_task_id: SORTING task id

pickup_slot
  변화 없음
```

화면:

```text
Customer UI
  ORD-0007 / 상품 선별 중

Admin UI
  Orders: ORD-0007 / 선별 중
  Robots: COBOT1 / 작업 #... 선별 중
```

### 2. Fleet Manager가 선별 완료 후 상태 업데이트

호출:

```text
PATCH /api/fleet/tasks/101      {"status":"SUCCESS"}
PATCH /api/fleet/robots/COBOT1  {"status":"IDLE"}
PATCH /api/fleet/tasks/102      {"status":"RUNNING","assigned_robot_id":"AMR1"}
PATCH /api/fleet/robots/AMR1    {"status":"DELIVERING","current_task_id":102}
PATCH /api/fleet/orders/7       {"status":"DELIVERING"}
```

DB 변화:

```text
orders
  status: SORTING -> DELIVERING

order_item
  status: WAITING -> SORTED

task
  SORTING  : RUNNING -> SUCCESS
  DELIVERY : QUEUED  -> RUNNING

robot
  COBOT1.status: SORTING -> IDLE
  COBOT1.current_task_id: null

  AMR1.status: IDLE -> DELIVERING
  AMR1.current_task_id: DELIVERY task id
```

화면:

```text
Customer UI
  ORD-0007 / 상품 운반 중

Admin UI
  Orders: ORD-0007 / 운반 중
  Robots: AMR1 / 배송 task 진행 중
```

### 3. Fleet Manager가 운반 완료 후 검수 시작

호출:

```text
PATCH /api/fleet/tasks/102      {"status":"SUCCESS"}
PATCH /api/fleet/robots/AMR1    {"status":"IDLE"}
POST  /api/fleet/orders/7/assign-pickup-slot
PATCH /api/fleet/tasks/103      {"status":"RUNNING","assigned_robot_id":"COBOT2"}
PATCH /api/fleet/robots/COBOT2  {"status":"INSPECTING","current_task_id":103}
PATCH /api/fleet/orders/7       {"status":"INSPECTING"}
```

DB 변화:

```text
orders
  status: DELIVERING -> INSPECTING

task
  DELIVERY   : RUNNING -> SUCCESS
  INSPECTION : QUEUED  -> RUNNING

robot
  AMR1.status: DELIVERING -> IDLE
  COBOT2.status: IDLE -> INSPECTING

pickup_slot
  첫 번째 EMPTY 슬롯 선택
  status: EMPTY -> RESERVED
  orders.pickup_slot_id에 slot_id 저장
```

화면:

```text
Customer UI
  ORD-0007 / 상품 검수 중

Admin UI
  Robots: COBOT2 / 검수 task 진행 중
```

### 4. Fleet Manager가 검수 완료 후 하차 시작

호출:

```text
PATCH /api/fleet/tasks/103           {"status":"SUCCESS"}
PATCH /api/fleet/robots/COBOT2       {"status":"IDLE"}
PATCH /api/fleet/tasks/104           {"status":"RUNNING","assigned_robot_id":"AMR1"}
PATCH /api/fleet/robots/AMR1         {"status":"UNLOADING","current_task_id":104}
PATCH /api/fleet/orders/7            {"status":"DELIVERING"}
```

DB 변화:

```text
orders
  status: INSPECTING -> DELIVERING

order_item
  status: SORTED -> INSPECTED

task
  INSPECTION : RUNNING -> SUCCESS
  UNLOAD     : QUEUED  -> RUNNING

robot
  COBOT2.status: INSPECTING -> IDLE
  AMR1.status: IDLE -> UNLOADING
  AMR1.current_task_id: UNLOAD task id
```

화면:

```text
Customer UI
  ORD-0007 / 상품 운반 중
  아직 픽업 칸 번호는 보여주지 않음

Admin UI
  Pickup Slots: 예약됨
```

### 5. Fleet Manager가 하차 완료 후 상태 업데이트

호출:

```text
PATCH /api/fleet/tasks/104       {"status":"SUCCESS"}
PATCH /api/fleet/robots/AMR1     {"status":"IDLE","current_task_id":null}
PATCH /api/fleet/pickup-slots/1  {"status":"OCCUPIED"}
PATCH /api/fleet/orders/7        {"status":"PICKUP_READY"}
```

DB 변화:

```text
orders
  status: DELIVERING -> PICKUP_READY

task
  UNLOAD: RUNNING -> SUCCESS

robot
  AMR1.status: UNLOADING -> IDLE
  AMR1.current_task_id: null

pickup_slot
  status: RESERVED -> OCCUPIED
```

화면:

```text
Customer UI
  ORD-0007 / 픽업 준비 완료
  픽업 칸: 1번
  픽업 완료 버튼 활성화

Admin UI
  Orders: ORD-0007 / 픽업 준비
  Pickup Slots: 픽업 대기
```

### 6. 고객이 픽업 완료 클릭

호출:

```text
POST /api/orders/{order_id}/complete
```

DB 변화:

```text
orders
  status: PICKUP_READY -> COMPLETED

pickup_slot
  status: OCCUPIED -> EMPTY

task
  남은 task가 있으면 SUCCESS로 정리

robot
  해당 주문과 연결된 current_task_id 정리
```

화면:

```text
Customer UI
  진행 중 주문 목록에서 ORD-0007 제거

Admin UI
  Orders 목록에서 제거
  Order History에 표시
  Pickup Slots: 비어 있음
```

## 상태 전이 요약

### 주문 상태

```text
ORDER_RECEIVED
  -> SORTING
  -> DELIVERING
  -> INSPECTING
  -> DELIVERING
  -> PICKUP_READY
  -> COMPLETED
```

예외 상황:

```text
어느 단계에서든
  -> ERROR
```

로봇이나 task가 바로 시작되지 못하면:

```text
ORDER_WAIT
```

### Task 상태

```text
QUEUED
  -> RUNNING
  -> SUCCESS
```

대기 또는 수동 배정 상태:

```text
ASSIGNED
```

예외 상황:

```text
RUNNING -> FAILED
RUNNING -> CANCELLED
RUNNING -> PAUSED
```

### Robot 상태

```text
IDLE        : 작업 가능 대기
SORTING     : 상품 선별 중
DELIVERING  : 배송/운반 중
INSPECTING  : 검수 중
UNLOADING   : 픽업 슬롯 하차 중
RETURNING   : 복귀 중
PARKING     : 파킹 위치로 이동 또는 파킹 동작 중
CHARGING    : 충전 중
ERROR       : 장애 상태
OFFLINE     : 연결 끊김
EMERGENCY_STOP : 긴급정지
```

정리 기준:

```text
IDLE
  - 다음 task를 받을 수 있는 상태

RETURNING
  - 작업 이후 홈/대기 구역으로 복귀 중

PARKING
  - 지정된 파킹 위치에 들어가는 동작 중
  - 복귀와 구분해서 UI에 표시하고 싶을 때 사용
```

## Pickup Slot 상태

```text
EMPTY
  - 비어 있음
  - 새 주문에 배정 가능

RESERVED
  - 특정 주문에 배정됨
  - 아직 상품이 들어가지는 않음

OCCUPIED
  - 상품이 들어 있음
  - 고객 픽업 가능

BLOCKED
  - 사용 불가
  - 장애, 수동 잠금, 점검 등
```

고객 UI에서는 `PICKUP_READY`가 되기 전까지 픽업 칸을 보여주지 않습니다.

## Fleet / Bridge 연동 API

실제 운영 기준에서는 Fleet Manager가 상태를 판단합니다.
Control Server는 Fleet Manager가 보낸 상태를 검증한 뒤 DB에 반영하고 UI에 보여줍니다.

```text
GET   /api/fleet/tasks
POST  /api/fleet/tasks
GET   /api/fleet/orders/{order_id}/tasks
PATCH /api/fleet/orders/{order_id}
POST  /api/fleet/orders/{order_id}/assign-pickup-slot
PATCH /api/fleet/tasks/{task_id}
POST  /api/fleet/tasks/{task_id}/events
GET   /api/fleet/tasks/{task_id}/events
PATCH /api/fleet/robots/{robot_id}
GET   /api/fleet/pickup-slots
PATCH /api/fleet/pickup-slots/{slot_id}
POST  /api/fleet/exceptions
```

조회 API는 각 로봇 담당 노드나 Control Bridge가 현재 DB 기준 task queue/pickup slot 상태를 확인할 때 사용합니다.
픽업 슬롯은 `assign-pickup-slot` API로 Control Server가 예약까지 한 번에 처리합니다.
상태 판단과 다음 동작 결정은 외부 로봇/Fleet 쪽에서 하고, Control Server는 API로 받은 결과를 저장하는 역할로 둡니다.

예시:

```json
POST /api/fleet/tasks
{
  "order_id": 7,
  "task_type": "DELIVERY",
  "status": "QUEUED",
  "assigned_robot_id": "AMR1",
  "source_zone_id": 1,
  "target_zone_id": 2
}
```

```json
POST /api/fleet/tasks/101/events
{
  "robot_id": "COBOT1",
  "to_status": "SUCCESS",
  "event_name": "SORTING_DONE",
  "reason": "item picked successfully"
}
```

```json
PATCH /api/fleet/tasks/101
{
  "status": "SUCCESS",
  "assigned_robot_id": "COBOT1",
  "result_message": "sorting done"
}
```

```json
PATCH /api/fleet/robots/AMR1
{
  "status": "DELIVERING",
  "current_task_id": 102,
  "battery_level": 84,
  "pos_x": 1.2,
  "pos_y": 0.4,
  "pos_theta": 3.14
}
```

```json
PATCH /api/fleet/orders/7
{
  "status": "DELIVERING",
  "pickup_slot_id": 1
}
```

```json
POST /api/fleet/exceptions
{
  "exception_type": "INSPECTION_FAIL",
  "robot_id": "COBOT2",
  "task_id": 103,
  "order_id": 7,
  "detail": "검수 결과 주문 상품과 실제 상품이 일치하지 않음"
}
```

이 구조에서는 다음 판단을 Fleet Manager가 담당합니다.

```text
다음 task가 무엇인지
어느 로봇이 어떤 task를 맡는지
주문 상태가 어디로 넘어가는지
pickup slot을 언제 예약/점유할지
예외 발생 후 재시도/취소/수동처리 정책
```

## 관리자 수동 조작과 실제 연동 차이

관리자 UI의 상태 수정 기능은 개발/시연 중 상태를 직접 바꿔 보면서 확인하기 위한 보조 기능입니다.

```text
Admin 수동 수정
  - 사람이 UI에서 주문/task/robot/pickup slot 상태를 변경
  - DB와 WebSocket 반영을 빠르게 확인
  - Fleet Manager 없이도 화면 테스트 가능

Fleet / Bridge API
  - 실제 Fleet Manager/ROS2가 호출
  - Fleet이 판단한 상태를 Control Server가 DB에 반영
  - 실제 연동 기준
```

## 예외 처리 흐름

예시: 선별 실패

```text
COBOT1 선별 중 gripper 실패
  -> Fleet Manager가 TASK_FAILED 전송
  -> task.status = FAILED
  -> orders.status = ERROR
  -> robot.status = ERROR
  -> exception_log 기록
  -> Admin UI Exceptions에 표시
```

관리자가 예외를 확인하고 처리 버튼을 누르면:

```text
exception_log.is_resolved = true
Exceptions 목록에서 제거
Exception History로 이동
```

예외 처리 이후 주문을 재시도할지, 취소할지, 수동 완료할지는 아직 정책 미정입니다.

## 확정/미정 정책

### 확정: PICKUP_SLOT 배정 시점

```text
1. 주문 접수 시점에는 픽업 칸을 배정하지 않는다.
2. 검수 시작 시점에 픽업 칸을 배정한다.
3. EMPTY 슬롯 중 낮은 번호를 선택하고 RESERVED로 변경한다.
4. orders.pickup_slot_id에 선택한 slot_id를 저장한다.
5. 하차 완료 시 pickup_slot.status를 OCCUPIED로 변경한다.
6. orders.status를 PICKUP_READY로 변경한다.
7. 고객 UI는 PICKUP_READY가 되기 전까지 픽업 칸 번호를 보여주지 않는다.
8. 고객 픽업 완료 시 pickup_slot.status를 EMPTY로 되돌린다.
```

### 미정

아래 내용은 Fleet Manager 구현 전에 확정해야 합니다.

```text

1. RETURNING과 PARKING 구분
   - RETURNING: 홈/대기 위치로 복귀 중
   - PARKING: 지정 파킹 위치에 들어가는 중
   - 둘 중 하나만 쓸지, 둘 다 쓸지 Fleet 동작 기준으로 확정 필요

2. 실패 주문 재처리 정책
   - ERROR 주문을 다시 QUEUED로 보낼지
   - 새 task를 만들지
   - 관리자 수동 처리로 끝낼지 결정 필요

3. task_id와 queue 순서
   - task_id는 DB 식별자
   - 실제 우선순위/큐 순서는 Fleet Manager가 별도로 판단
```

## 명칭 기준

```text
STANBY
  - 오타이므로 사용하지 않는다.

STANDBY
  - 현재 DB task_type에는 넣지 않는다.
  - 로봇 대기는 robot.status = IDLE 또는 WAITING으로 표현한다.
  - 위치 대기는 zone_name으로 표현한다.

현재 task_type
  - SORTING
  - DELIVERY
  - INSPECTION
  - UNLOAD
  - PATROL
  - CHARGE
  - RETURN_HOME
```
