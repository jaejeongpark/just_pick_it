# TrafficManager API 계약

Fleet Manager 내부의 `TrafficManager`가 외부(주로 `TaskManager`)에 제공하는 인터페이스 명세.
구현은 `fleet_manager/traffic_manager.py` 참고.

## 개요

`TrafficManager`는 zone 기반 BFS 경로 탐색과 다중 PICKY 간 충돌 회피를 담당한다.
외부 의존(HTTP, ROS 토픽 구독)은 보유하지 않으며, 모든 입력은 메서드 호출 또는
`RobotStateMonitor` 콜백으로 들어온다.

### 책임 경계

| | TrafficManager | TaskManager |
|---|---|---|
| zone 그래프 / 경로 탐색 | O | X |
| 다른 로봇 점유 고려한 충돌 회피 | O | X |
| 도크 자원 선정 | O | X |
| task 순서 결정 | X | O |
| 상품 후보 선정 | X | O |
| HTTP API / 로봇 명령 전송 | X | X (각각 Client/Gateway 담당) |
| DB / 외부 state 수정 | X | X (Control Server 책임) |

`TrafficManager`는 in-memory 레지스트리(`_robot_paths`, `_robot_reservations`,
`_robot_dock`, `_robot_states`) 외에는 어떤 외부 상태도 변경하지 않는다.
DB 정합성은 Control Server가, ROS 송신은 RobotCommandGateway가, polling/dispatch는
TaskManager가 각각 담당한다. 특히 **dock 점유 정보(`_robot_dock`)는 TrafficManager
단독 보유**이며 Control Server / DB / TaskManager 어디에도 노출하거나 통지하지
않는다 — 경로 탐색·dock 선정 모두 TrafficManager 책임이므로 외부가 알 필요가 없다.

## PathResult

```python
@dataclass(frozen=True)
class PathResult:
    ok: bool
    waypoints: tuple[str, ...] = ()      # ok=False 이면 ()
    cost: float | None = None             # 현재 구현은 hop 수 (len(waypoints) - 1)
    reason: str | None = None             # ok=True 이면 None
```

- `waypoints`는 시작 zone 부터 목적지 zone 까지의 zone_name 순서.
- `cost`는 현재 hop 수. 추후 유클리드 거리 합 등으로 확장 가능.
- 실패 사유 종류: `no path: {source} -> {target}`, `no available charging dock`.

## 메서드 명세

### `reserve_path(robot_id, task_id, source_zone, target_zone) -> PathResult`

목적지가 단일 zone 으로 정해진 이동에 사용. 경로를 계산하고 내부 예약
레지스트리에 등록한다 (예: MOVE_TO_PICKUP, MOVE_TO_STORAGE).

- **원자성**: BFS + 등록을 단일 lock 안에서 수행.
- **중복 예약**: 한 로봇이 이미 예약을 보유 중이면 그 예약을 덮어쓴다.
  일반적으로는 `release_path` 후 호출하는 것이 안전.
- **실패 시**: `PathResult(ok=False, reason=...)` 반환. 호출자는 잠시 대기 후 재시도.

### `reserve_nearest_from(robot_id, task_id, source_zone, candidates) -> PathResult`

`candidates` 중 reserve 가능하고 cost 가 가장 낮은 zone 을 atomic 하게 예약한다.
후보가 여럿인 이동(예: MOVE_TO_PRODUCT) 에서 평가 + 선정 + 예약을 한 호출에 끝낸다.

- **`task_id` 타입**: `int | None`. MOVE_TO_PRODUCT 의 첫 호출처럼 "path 가
  먼저 결정돼야 task INSERT 가 가능 (= task_id 도 그때 발행)" 인 흐름에서는
  `None` 으로 호출해 path 만 받고, INSERT 후 발행된 task_id 를
  `attach_task_id()` 로 사후 연결한다.
- **`candidates` 형식**: `dict[str, int]` — `{zone_name: 상품 수량}` 매핑.
  TaskManager 가 보유한 매핑을 변환 없이 그대로 넘길 수 있도록 dict 형태로 받는다.
  같은 zone 의 상품이 여러 개여도 픽업은 한 번이므로 TrafficManager 는 key 만
  사용하고 value (수량) 는 참고하지 않는다.
- **원자성**: 평가와 예약을 단일 lock 안에서 수행 → 평가 결과를 보고 예약하는
  사이에 다른 로봇이 점유하는 race 를 차단한다.
- **선택된 zone 확인**: `result.waypoints[-1]`.
- **상품 도메인은 모름**: 호출자(TaskManager) 가 zone → 도메인 객체(상품) 매핑을
  자체적으로 보유한다.
- **모든 후보 차단 시**: `PathResult(ok=False, reason='all candidates blocked')`.

### `attach_task_id(robot_id, task_id) -> bool`

`reserve_nearest_from(task_id=None)` 으로 등록된 임시 예약 path 에 task INSERT
후 발행된 `task_id` 를 사후 연결한다.

- **전제**: `_robot_reservations[robot_id]` 가 `None` 이고 path 가 등록되어
  있는 상태 (즉 직전에 `reserve_nearest_from(task_id=None, ...)` 가 성공한 직후).
- **반환**: 성공 시 `True`, 실패 시 `False` (warn 로그).
- **실패 케이스**: 이미 다른 `task_id` 가 연결되어 있거나 (실수 호출) 임시
  예약된 path 자체가 없는 경우.
- **호출 시점**: TaskManager 가 task DB INSERT 로 `task_id` 를 받은 직후.

### `reserve_return_home_path(robot_id, task_id, source_zone) -> PathResult`

RETURN_HOME 전용. `STANDBY_ZONE_1` / `STANDBY_ZONE_2` 중 BFS 비용이 가장 낮은 곳으로 경로를 예약한다.

- **도크 예약 없음**: 도크 점유는 하지 않는다. 도크 예약은 이후 DOCK_IN task 시작 시
  `reserve_dock_path()` 로 별도 수행한다.
- **반환 의미**: `waypoints[-1]`이 도착 STANDBY_ZONE.
- **모든 standby zone 차단 시**: `PathResult(ok=False, reason='no path to standby zone')`.

### `reserve_dock_path(robot_id, task_id, source_zone) -> PathResult`

DOCK_IN 전용. 빈 충전 도크를 안쪽 우선으로 선택해 경로 + 도크를 예약한다.

- **도크 우선순위**: `CHARGING_DOCK_1` (안쪽) → `CHARGING_DOCK_2` (바깥쪽).
- **원자성**: 도크 선정과 경로/도크 예약을 단일 lock 안에서 수행.
- **반환 의미**: `waypoints[-1]`이 목적지 `CHARGING_DOCK`.
  실제 도킹 동작(ArUco 등)은 State Manager가 별도로 수행한다.
- **모든 도크 점유 또는 경로 없음**: `PathResult(ok=False, reason='no available charging dock')`.

### `update_path_progress(robot_id, task_id, current_waypoint_index) -> None`

로봇이 waypoint를 통과할 때마다 호출. 지나온 구간을 점유 해제하여 다른 로봇이
사용할 수 있게 한다.

- **호출 시점**: Task Manager가 Action 피드백(`current_waypoint_index`)을 받을 때.
- **stale 호출**: `task_id`가 현재 예약과 다르면 무시하고 warn 로그.
- **인덱스 의미**: "방금 도달한 waypoint의 인덱스". 0은 시작점이므로 무시,
  1 이상부터 잘라낸다.

### `release_path(robot_id, task_id) -> None`

task 종료 시 경로 예약을 해제. `task_id` 타입은 `int | None`.

- **호출 시점**: task가 SUCCESS / FAILED / CANCELLED / timeout 으로 종료될 때.
- **`task_id` 가 int 인 경우 (정상)**: 현재 예약의 task_id 와 일치해야 해제.
  다르면 stale release 로 간주하고 warn 로그 후 무시.
- **`task_id` 가 None 인 경우**: 현재 예약이 task_id 미배정 상태일 때만 임시
  예약 path 를 해제. `reserve_nearest_from(task_id=None)` 후 `attach_task_id`
  호출 전에 task DB INSERT 가 실패한 경우 같은 임시 예약 정리용. 이미 task_id
  가 연결된 상태면 warn 로그 후 무시.
- **도크 점유**: 별도. picky_state 변화로 자동 해제됨 (아래 참고).

### `notify_state(robot_id, state) -> None`

`RobotStateMonitor` 전용. TaskManager는 직접 호출하지 않는다.

- **동작**: 내부 `_robot_states` 갱신.
- **안전망**: `MOVING_STATES`도 `OCCUPYING_STATES`도 아닌 상태로 바뀌면
  path와 reservation을 자동 해제 (`release_path` 누락 대비).
- **도크 해제**: `CHARGING` → 타상태 전환 시점에 `_robot_dock` 해제.

### `get_robot_state(robot_id) -> str | None`, `get_all_states() -> dict[str, str]`

읽기 전용 조회. lock 안에서 안전하게 복사 반환.

## 상태 분류 (내부 참고)

```python
MOVING_STATES = {
    'MOVING_TO_PRODUCT', 'MOVING_TO_PICKUP', 'MOVING_TO_STOCK',
    'MOVING_TO_STORAGE', 'RETURNING', 'DOCKING',
}
OCCUPYING_STATES = {
    'WAITING_FOR_COBOT',
}
```

- **path 가 등록되어 있고 state ∈ `OCCUPYING_STATES`**: 마지막 노드(현재 머무는
  노드) 만 차단됨. 경유 노드는 풀어줌.
- **path 가 등록되어 있고 그 외 모든 state**: 경로 전체 노드 + 엣지가 다른
  로봇에게 차단됨. 여기에는 `STANDBY` 같은 idle 상태도 포함된다 — reserve_\*
  성공 자체가 "이 로봇이 곧 path 를 점유" 라는 약속이므로, picky_state 가
  MOVING 으로 갱신되기 전 짧은 윈도우에도 path 가 차단된다.
- **path 가 등록되지 않은 경우**: 어떤 노드도 차단하지 않음.

### 도메인 제약: 노드 점유 단일성

같은 노드에 두 로봇이 동시에 머무를 수 있는 공간이 없다.
따라서 차단된 노드는 **목적지여도 도달 불가**다.
예: 한 로봇이 `WAITING_FOR_COBOT` 상태로 `PRODUCT_ZONE_3` 에 있다면,
다른 로봇은 그 노드를 target 으로 reserve 할 수 없다.
호출자(TaskManager)는 점유 해제 또는 다른 후보 선택을 결정해야 한다.

## 표준 호출 순서

### 패턴 1. 목적지가 정해진 단일 이동 (MOVE_TO_PICKUP 등)

```python
result = traffic.reserve_path('PICKY1', task_id=42, current_zone, 'PICKUP_ZONE_2')
if not result.ok:
    # 잠시 대기 후 재시도
    ...

# 이동 중 피드백마다
traffic.update_path_progress('PICKY1', task_id=42, current_waypoint_index=i)

# 종료 시
traffic.release_path('PICKY1', task_id=42)
```

### 패턴 2. 후보 중 최적 선정 + task_id 사후 발행 (MOVE_TO_PRODUCT)

TaskManager 의 흐름: 주문 큐에서 배정받은 직후, task 가 아직 INSERT 되지
않아 `task_id` 가 없다. MOVE_TO_PRODUCT 를 실행하려면 어느 PD 로 갈지 결정해야
하는데 그건 `reserve_nearest_from` 결과로 정해진다. 즉 reserve → INSERT 순서.

```python
# TaskManager 가 보유한 zone -> 상품 매핑 (도메인 객체)
zone_to_item = {
    'PRODUCT_ZONE_2': item_A,
    'PRODUCT_ZONE_3': item_C,
    'PRODUCT_ZONE_5': item_B,
}

# TrafficManager 에 넘길 후보 dict: zone -> 상품 수량
candidates = {
    'PRODUCT_ZONE_2': 1,
    'PRODUCT_ZONE_3': 2,
    'PRODUCT_ZONE_5': 1,
}

# 1. task_id 가 아직 없는 상태로 임시 예약 (path 만 결정)
result = traffic.reserve_nearest_from(
    'PICKY1', None, current_zone, candidates
)
if not result.ok:
    # 모든 후보 차단 — 잠시 대기 후 재시도
    return

selected_zone = result.waypoints[-1]
selected_item = zone_to_item[selected_zone]

# 2. selected_zone 으로 task DB INSERT → task_id 발행
task_id = control_server.create_move_to_product(
    unit_id='PICKY1', target_zone=selected_zone, item=selected_item, ...
)

# 3. 발행된 task_id 를 임시 예약에 사후 연결
if not traffic.attach_task_id('PICKY1', task_id):
    # 매우 드문 경우: 임시 예약이 사라졌거나 다른 task 가 연결됨
    # → control_server.delete_task(task_id) 등 보상 처리
    return

# 4. 이후 update_path_progress / release_path 는 task_id 로 호출 (패턴 1 과 동일)
```

INSERT 가 실패해서 task_id 가 발행되지 못한 경우:

```python
result = traffic.reserve_nearest_from('PICKY1', None, current_zone, candidates)
try:
    task_id = control_server.create_move_to_product(...)
except DBError:
    # 임시 예약 정리 (task_id 없이 해제)
    traffic.release_path('PICKY1', None)
    raise
```

### 패턴 3. RETURN_HOME

```python
result = traffic.reserve_return_home_path('PICKY1', task_id=99, current_zone)
if not result.ok:
    # 모든 standby zone 차단 — 잠시 대기 후 재시도
    ...
target_standby = result.waypoints[-1]
# Task 종료 시
traffic.release_path('PICKY1', task_id=99)
```

### 패턴 4. DOCK_IN

```python
result = traffic.reserve_dock_path('PICKY1', task_id=100, current_zone)
if not result.ok:
    # 모든 도크 점유 또는 경로 없음 — 잠시 대기 후 재시도
    ...
target_dock = result.waypoints[-1]  # CHARGING_DOCK_1 또는 CHARGING_DOCK_2
# Task 종료 시 (도킹 성공/실패 무관)
traffic.release_path('PICKY1', task_id=100)
# 도크 점유는 picky_state 가 CHARGING 에서 이탈할 때 notify_state() 가 자동 해제
```

## 동시성 보장

- 모든 BFS, 예약, 해제는 단일 `self._lock` 안에서 원자적으로 수행됨.
- 두 로봇이 동시에 `reserve_path` / `reserve_nearest_from` /
  `reserve_return_home_path` 를 호출해도 한 쪽만 성공하고 다른 쪽은 갱신된
  점유 상태를 본다.

## 알려진 한계

1. **DOCK_IN 실패 시 도크 점유 해제 정책 (의도된 한계)**
   - 로봇이 CHARGING_DOCK 도달 전에 task 가 FAILED/CANCELLED 되면
     `_robot_dock` 예약은 `release_path` 만으로는 해제되지 않는다.
   - 도크 점유 해제는 오직 `notify_state()` 안전망 — picky_state 가
     `CHARGING` 에서 다른 상태로 전이될 때 자동 — 으로만 일어난다.
   - **`release_dock` 같은 명시적 해제 API 는 의도적으로 추가하지 않는다**.
     이유: TrafficManager 에 DB / 외부 state 동기화 책임을 부여하지 않기 위함.
     다음 MOVE task 가 dispatch 되면 PICKY 가 도크에서 이탈하면서 picky_state
     전이가 발생하고, 그 시점에 자연스럽게 도크가 해제된다고 가정한다.
   - dock 해제 이벤트도 외부에 통지하지 않는다 — Control Server 는 state_manager 가
     별도로 보내는 `picky_state` 전이 보고로 "PICKY 가 CHARGING 을 떠났다 = 도크
     자리가 비었다" 를 충분히 파악할 수 있으므로 dock_name 단위 통지는 불필요.
   - 운영상 leak 이 관측되면 안전망의 트리거 조건(`prev == 'CHARGING'`) 을
     확장하거나 Control Server / TaskManager 가 상위 흐름에서 정리한다.

2. **`cost` 의 단위**
   - 현재는 hop 수. zone 간 실제 거리가 다르면 부정확.
   - 필요 시 유클리드 거리 합으로 교체 (`_zone_coords` 활용).

## 변경 이력

- 2026-05-21: 초안. `find_path` 계열 제거, `estimate_path` / `reserve_path` /
  `reserve_return_home_path` / `release_path` / `update_path_progress` (task_id 추가)
  도입.
- 2026-05-21: `reserve_nearest_from(robot_id, task_id, source_zone, candidates)`
  추가. 후보 평가 + 선정 + 예약을 atomic 하게 처리. TaskManager 가 zone 리스트만
  넘기면 되므로 race 와 호출 횟수 모두 감소.
- 2026-05-21: BFS 의 "목적지 차단 무시" 정책 제거. 도메인 제약상 같은 노드에
  두 로봇이 동시에 머무를 수 없으므로 차단된 노드는 목적지여도 도달 불가.
- 2026-05-22: `estimate_path` 제거. 후보 평가+선정+예약을 `reserve_nearest_from`
  이 atomic 하게 처리하므로 평가 전용 API 가 더 이상 필요하지 않다.
  "estimate 결과로 선정 후 reserve 사이에 다른 로봇이 점유" race 도 함께 사라짐.
- 2026-05-22: `reserve_nearest_from` 의 `candidates` 타입을 `list[str]` 에서
  `dict[str, int]` (`{zone_name: 상품 수량}`) 로 변경. TaskManager 측 자료구조를
  변환 없이 그대로 넘기기 위함. TrafficManager 는 key 만 사용한다.
- 2026-05-22: `docs/Traffic_node_graph.jpg` 의 새 맵 토폴로지 반영. 좌측 수직 복도
  `TRAFFIC_L1`/`L2`/`L3` 추가, 우측 수직 복도 `TRAFFIC_R1~R4` 와
  `PICKUP_ZONE_3`/`_4` 제거. `STOCK_ZONE` 진출이 `TRAFFIC_L1` 으로 변경되고
  `STANDBY_ZONE_2` 와 `PRODUCT_ZONE_1`/`_4` 사이에 `TRAFFIC_L2` 가 끼어 hop 1
  거리가 hop 2 로 늘어남. `PICKUP_ZONE_1` 은 `TRAFFIC_T3`, `PICKUP_ZONE_2` 는
  `TRAFFIC_B3` 와 직접 인접.
- 2026-05-22: `reserve_nearest_from` 의 `task_id` 를 `int | None` 으로 확장.
  task_id 가 task INSERT 시점에 비로소 발행되는 흐름(MOVE_TO_PRODUCT) 을
  지원. 새 메서드 `attach_task_id(robot_id, task_id)` 신설 — 임시 예약 path 에
  사후 발행된 task_id 를 연결. `release_path` 에서도 task_id=None 허용 (임시
  예약 정리용).
- 2026-05-22: `_build_blocked_sets` 정책 변경. picky_state 와 무관하게 path 가
  등록되어 있으면 차단 (단 OCCUPYING_STATES 면 path[-1] 만). reserve 직후
  picky_state 가 MOVING 으로 갱신되기 전의 race window 가 닫힘.
- 2026-05-23: DOCK_IN task 분리 반영. `reserve_return_home_path` 에서 도크 예약 제거
  (STANDBY_ZONE 도착까지만 담당). `reserve_dock_path` 신설 — DOCK_IN 시작 시
  TaskManager 가 호출하여 도크 예약 + CHARGING_DOCK 까지의 경로를 원자적으로 수행.
  `STANDBY_ZONES` 상수 추가.
- 2026-05-24: 책임 경계 명문화 — TrafficManager 는 DB / 외부 state 를 절대 수정하지
  않고 in-memory 레지스트리만 다룬다. `release_dock` 같은 명시적 도크 해제 API 는
  추가하지 않기로 결정. 도크 점유 해제는 오직 `notify_state` 안전망에서 picky_state
  의 CHARGING 이탈 시점에만 일어나며, 운영상 leak 이 관측되면 안전망 트리거 조건을
  확장하거나 상위 흐름에서 정리한다. "알려진 한계" 1번 항목을 의도된 정책으로 갱신.
- 2026-05-24: dock 점유 정보(`_robot_dock`)는 TrafficManager 단독 보유로 확정.
  경로 탐색·dock 선정 모두 TrafficManager 책임이므로 Control Server / DB /
  TaskManager 는 dock 점유를 모른다. dock 해제 이벤트도 callback / HTTP / publisher
  어디로도 통지하지 않는다 (PICKY 의 picky_state 보고로 충분히 외부에 전달됨).
