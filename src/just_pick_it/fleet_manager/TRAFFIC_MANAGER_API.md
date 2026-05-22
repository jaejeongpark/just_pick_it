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

### `estimate_path(robot_id, source_zone, target_zone) -> PathResult`

예약 없이 현재 traffic 기준 경로 가능 여부와 비용만 계산.

- **사용처**: 상품 후보 선정 등 평가 단계.
- **보장 수준**: 다음 순간 다른 로봇이 점유할 수 있어 100% 보장이 아님.
  실제 실행 직전에는 반드시 `reserve_path`로 재확정.
- **부수효과**: 없음 (read-only).

### `reserve_path(robot_id, task_id, source_zone, target_zone) -> PathResult`

경로를 계산하고 내부 예약 레지스트리에 등록.

- **원자성**: BFS + 등록을 단일 lock 안에서 수행.
- **중복 예약**: 한 로봇이 이미 예약을 보유 중이면 그 예약을 덮어쓴다.
  일반적으로는 `release_path` 후 호출하는 것이 안전.
- **실패 시**: `PathResult(ok=False, reason=...)` 반환. 호출자는 다른 후보를
  시도하거나 잠시 대기 후 재시도.

### `reserve_nearest_from(robot_id, task_id, source_zone, candidates) -> PathResult`

`candidates` 중 reserve 가능하고 cost가 가장 낮은 zone을 atomic하게 예약한다.

- **사용처**: TaskManager가 주문에 남은 상품 zone 리스트를 그대로 넘기는 경우.
  평가 + 선정 + 예약을 한 호출에서 끝낸다.
- **원자성**: 평가와 예약을 단일 lock 안에서 수행 → estimate 결과를 보고 reserve
  하는 사이에 다른 로봇이 점유하는 race를 차단한다.
- **선택된 zone 확인**: `result.waypoints[-1]`.
- **상품 도메인은 모름**: `candidates`는 단순 zone_name 리스트. 호출자(TaskManager)가
  zone → 도메인 객체(상품 등) 매핑을 자체적으로 보유한다.
- **모든 후보 차단 시**: `PathResult(ok=False, reason='all candidates blocked')`.

### `reserve_return_home_path(robot_id, task_id, source_zone) -> PathResult`

RETURN_HOME 전용. 비어있는 충전 도크를 안쪽 우선으로 선택해 예약.

- **도크 우선순위**: `CHARGING_DOCK_1` (안쪽) → `CHARGING_DOCK_2` (바깥쪽).
- **반환 의미**: `waypoints[-1]`이 도착 STANDBY_ZONE.
  도킹 자체는 State Manager가 별도로 수행한다.
- **모든 도크 점유 시**: `PathResult(ok=False, reason='no available charging dock')`.

### `update_path_progress(robot_id, task_id, current_waypoint_index) -> None`

로봇이 waypoint를 통과할 때마다 호출. 지나온 구간을 점유 해제하여 다른 로봇이
사용할 수 있게 한다.

- **호출 시점**: Task Manager가 Action 피드백(`current_waypoint_index`)을 받을 때.
- **stale 호출**: `task_id`가 현재 예약과 다르면 무시하고 warn 로그.
- **인덱스 의미**: "방금 도달한 waypoint의 인덱스". 0은 시작점이므로 무시,
  1 이상부터 잘라낸다.

### `release_path(robot_id, task_id) -> None`

task 종료 시 경로 예약을 해제.

- **호출 시점**: task가 SUCCESS / FAILED / CANCELLED / timeout 으로 종료될 때.
- **stale 호출**: `task_id`가 현재 예약과 다르면 무시하고 warn 로그.
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

- `MOVING_STATES`: 해당 로봇의 경로 전체 노드 + 엣지가 다른 로봇에게 차단됨.
- `OCCUPYING_STATES`: 마지막 노드(목적지)만 차단됨.
- 그 외 상태(`STANDBY`, `CHARGING`, `IDLE`, `ERROR_RECOVERY` 등): 어떤 노드도 차단하지 않음.

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

### 패턴 2. 후보 중 최적 선정 (MOVE_TO_PRODUCT 등)

```python
# TaskManager 가 보유한 zone -> 상품 매핑
zone_to_item = {
    'PRODUCT_ZONE_2': item_A,
    'PRODUCT_ZONE_3': item_C,
    'PRODUCT_ZONE_5': item_B,
}

# 한 호출로 평가 + 선정 + 예약
result = traffic.reserve_nearest_from(
    'PICKY1', task_id=42, current_zone, list(zone_to_item.keys())
)
if not result.ok:
    # 모든 후보 차단 — 잠시 대기 후 재시도
    ...

# 선택된 zone 으로부터 상품 역추적
selected_zone = result.waypoints[-1]
selected_item = zone_to_item[selected_zone]

# 이후는 패턴 1 과 동일하게 update_path_progress / release_path
```

### 패턴 3. 예약 없이 평가만 (우선순위 정렬 등)

```python
# 후보들의 cost 만 알고 싶을 때
costs = {}
for zone in candidates:
    r = traffic.estimate_path('PICKY1', current_zone, zone)
    if r.ok:
        costs[zone] = r.cost
# costs 를 다른 휴리스틱과 결합해서 최종 결정
```

> 평가 후 실제 실행 시점에는 반드시 `reserve_path` 또는 `reserve_nearest_from`
> 으로 재확정해야 한다. estimate 결과는 다음 순간 무효해질 수 있다.

### 패턴 4. RETURN_HOME

```python
result = traffic.reserve_return_home_path('PICKY1', task_id=99, current_zone)
if not result.ok:
    # 모든 도크 점유 — 잠시 대기 또는 STANDBY 유지
    ...
target_standby = result.waypoints[-1]
# Task 종료 시
traffic.release_path('PICKY1', task_id=99)
```

## 동시성 보장

- 모든 BFS, 예약, 해제는 단일 `self._lock` 안에서 원자적으로 수행됨.
- 두 로봇이 동시에 `reserve_path`를 호출해도 한 쪽만 성공하고
  다른 쪽은 갱신된 점유 상태를 본다.
- `estimate_path`는 read-only이므로 동시 호출 가능.

## 알려진 한계

1. **RETURN_HOME 실패 시 도크 leak 가능성**
   - 로봇이 STANDBY_ZONE 도달 전에 task가 FAILED/CANCELLED 되면
     `_robot_dock` 예약은 `release_path` 만으로는 해제되지 않는다.
   - 현재는 picky_state가 CHARGING 으로 전환되지 않으면 도크 점유가 남는다.
   - 대응: 명시적 `release_dock` API 필요 시 후속 추가.

2. **`cost` 의 단위**
   - 현재는 hop 수. zone 간 실제 거리가 다르면 부정확.
   - 필요 시 유클리드 거리 합으로 교체 (`_zone_coords` 활용).

3. **상품 후보 evaluation race**
   - `estimate_path` 결과로 선정한 후보를 `reserve_path` 호출 시점에
     다른 로봇이 점유해버릴 수 있다.
   - 호출자(TaskManager)가 실패 시 다음 후보로 재시도하는 로직 필수.

## 변경 이력

- 2026-05-21: 초안. `find_path` 계열 제거, `estimate_path` / `reserve_path` /
  `reserve_return_home_path` / `release_path` / `update_path_progress` (task_id 추가)
  도입.
- 2026-05-21: `reserve_nearest_from(robot_id, task_id, source_zone, candidates)`
  추가. 후보 평가 + 선정 + 예약을 atomic 하게 처리. TaskManager 가 zone 리스트만
  넘기면 되므로 race 와 호출 횟수 모두 감소.
- 2026-05-21: BFS 의 "목적지 차단 무시" 정책 제거. 도메인 제약상 같은 노드에
  두 로봇이 동시에 머무를 수 없으므로 차단된 노드는 목적지여도 도달 불가.
