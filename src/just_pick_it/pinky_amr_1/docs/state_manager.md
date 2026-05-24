# State Manager

PICKY 측 상태기계 노드. Fleet Manager 의 명령을 받아 `picky_state` 를 전환하고
실제 주행 / 도킹은 `move_to_goal` 과 `reverse_docking` 에 위임한다.

구현: `pinky_amr_1/pinky_amr_1/state_manager.py`

---

## 개요

이 노드의 단일 책임:

- Fleet Manager 의 MoveCommand / DockCommand Action 수신
- `picky_state` 전환 publish + Control Server HTTP 보고
- 도크 이탈 short 모션
- (위임) waypoint 주행 → `MoveToGoal.move_to_goal(x, y, theta)`
- (위임) ArUco 후진 도킹 → `ReverseDocking.reverse_dock(marker_id, x, y, yaw)`

같은 프로세스 안에 `state_manager`, `move_to_goal`, `reverse_docking` 세 노드가
`ReentrantCallbackGroup` 으로 띄워진다. 모든 토픽 / 액션은 launch namespace
기준 상대경로다.

---

## 외부 인터페이스

### Action Server

| Action | 인터페이스 | 트리거 task | 종료 |
|---|---|---|---|
| `move_command` | `just_pick_it_interfaces/MoveCommand` | MOVE_TO_PRODUCT / MOVE_TO_PICKUP / MOVE_TO_STOCK / MOVE_TO_STORAGE / RETURN_HOME | success / abort / canceled |
| `dock_command` | `just_pick_it_interfaces/DockCommand` | DOCK_IN | success / abort |

### Publisher

| Topic | 메시지 | 설명 |
|---|---|---|
| `picky_state` | `std_msgs/String` | Traffic Manager 가 구독 |
| `cmd_vel` | `geometry_msgs/Twist` | 도크 이탈 시 short 모션 (그 외 주행은 move_to_goal 위임) |

### Subscription

| Topic | 메시지 | 용도 |
|---|---|---|
| `battery/voltage` | `std_msgs/Float32` | 배터리 % 환산 |
| `battery_state` | `sensor_msgs/BatteryState` | 동일. 들어오는 쪽 사용 |
| `/tf`, `/tf_static` | (글로벌) | map → base_link pose 추정 |

### HTTP 보고

`server_base_url` + `/api/fleet/robots/{robot_id}` 로 `report_interval_sec` 주기 PATCH:

```json
{"status": "<picky_state>", "battery_level": <int>,
 "pos_x": ..., "pos_y": ..., "pos_theta": ...}
```

---

## 상태 전이

DB `picky_state` enum 과 1:1 매핑.

```
CHARGING  --(MOVE_* | RETURN_HOME accept)--> _depart_from_dock --> MOVING_* / RETURNING
MOVING_*  --(모든 waypoint 도착)-----------> WAITING_FOR_COBOT   (ARRIVAL_STATE[task_type])
RETURNING --(STANDBY_ZONE 도착)-------------> STANDBY            (ARRIVAL_STATE['RETURN_HOME'])
STANDBY   --(DOCK_IN accept)----------------> DOCKING
DOCKING   --(reverse_dock 성공)-------------> CHARGING
DOCKING   --(reverse_dock 실패)-------------> ERROR_RECOVERY
MOVING_*  --(navigation 실패)---------------> ERROR_RECOVERY
```

- `TASK_TO_MOVING_STATE` — 이동 시작 시 진입 상태.
- `ARRIVAL_STATE` — 모든 waypoint 통과 후 진입 상태. RETURN_HOME 은 STANDBY 로
  종료한다. 후진 도킹은 별도 DOCK_IN task 가 수행하므로 state_manager 가
  RETURN_HOME 끝에서 자동 도킹하지 않는다.

---

## DockCommand 처리

`DockCommand.Goal`:

```
int32  task_id
string dock_name          # CHARGING_DOCK_1 또는 CHARGING_DOCK_2
string start_zone_name    # 시작 STANDBY zone (로그/추적용)
```

처리 흐름:

1. `_on_dock_goal` 이 `dock_name` 을 `_dock_pose_by_name` 에서 조회.
   미등록 dock 이면 REJECT.
2. `_execute_dock` 에서 `picky_state` → `DOCKING` 전이 + `DockCommand.Feedback`
   1회 publish.
3. `self._reverse_docking.reverse_dock(marker_id, map_x, map_y, map_yaw)` 호출
   (blocking, 내부에서 Phase 0 ~ 3 진행).
4. 성공: `picky_state` → `CHARGING`, Result success=True.
5. 실패: `picky_state` → `ERROR_RECOVERY`, `goal_handle.abort()`, Result success=False.

cancel 요청은 ACCEPT 되지만 phase 도중 중단은 지원하지 않는다 (`reverse_dock`
이 sync blocking call).

---

## MoveCommand 처리

`MoveCommand.Goal`:

```
string                      task_type   # MOVE_TO_* | RETURN_HOME
geometry_msgs/PoseStamped[] waypoints
```

처리 흐름:

1. `_on_move_goal` 이 `task_type` 을 `TASK_TO_MOVING_STATE` 에서 조회.
   미등록이면 REJECT.
2. 현재 상태가 `CHARGING` 이면 `_depart_from_dock()` 으로 도크 이탈.
3. `picky_state` → `MOVING_* / RETURNING`.
4. waypoint 별로 `feedback.current_waypoint_index = i` publish 후
   `move_to_goal.move_to_goal(x, y, theta)` 호출.
5. 모든 waypoint 성공 시 `picky_state` → `ARRIVAL_STATE[task_type]`,
   Result success=True.
6. 도중 실패 시 `picky_state` → `ERROR_RECOVERY`, abort.

---

## 파라미터

| 이름 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `server_base_url` | str | `http://192.168.4.1:8000` | Control Server HTTP base |
| `robot_id` | str | `PICKY1` | DB `robots.robot_id` 값 |
| `report_interval_sec` | float | `1.0` | picky_state publish + HTTP 보고 주기 (s) |
| `dock_departure_distance` | float | `0.08` | CHARGING 이탈 시 전진 거리 (m) |
| `battery_full_voltage` | float | `8.4` | 100% 환산 기준 |
| `battery_empty_voltage` | float | `6.8` | 0% 환산 기준 |
| `charging_dock_1.marker_id` | int | `0` | CHARGING_DOCK_1 의 ArUco 마커 ID |
| `charging_dock_1.map_x` | float | `0.10` | CHARGING_DOCK_1 도킹 완료 시 base_link x (map frame, m) |
| `charging_dock_1.map_y` | float | `0.10` | y |
| `charging_dock_1.map_yaw` | float | `0.0` | yaw (rad) |
| `charging_dock_2.marker_id` | int | `1` | CHARGING_DOCK_2 동일 |
| `charging_dock_2.map_x` | float | `0.27` |  |
| `charging_dock_2.map_y` | float | `0.10` |  |
| `charging_dock_2.map_yaw` | float | `0.0` |  |

dock pose 4개 값의 의미:

- `marker_id` — 도크 벽에 부착된 ArUco 마커 ID. reverse_docking 이 카메라로
  탐지·추적하는 대상 마커.
- `map_x` / `map_y` / `map_yaw` — **도킹 완료 후 로봇 base_link 가 정지하는 위치**
  (마커 위치 아님). 도킹 종료 시 `reverse_docking` 이 이 값을 `/initialpose`
  로 publish 하여 AMCL 파티클 필터를 재수렴시킨다.

> `traffic_manager` 의 `DEFAULT_ZONE_COORDS['CHARGING_DOCK_*']` 와 같은 값이어야
> 한다. 두 곳을 별도로 관리하면 BFS path 종점과 AMCL 재초기화 좌표가 어긋난다.
> 향후 Control Server 의 `/api/fleet/zones` 응답에서 단일 소스로 받아오도록
> 일원화하는 것이 권장.

---

## namespace 운용

state_manager / move_to_goal / reverse_docking 모두 launch namespace 안에서
띄워지는 것을 전제로 한다. 모든 토픽 / 액션이 상대경로이므로 launch 의
`namespace` 인자만 바꾸면 다른 PICKY 로 그대로 재사용된다.

### PICKY1 실행

```bash
# AMR 하드웨어 bringup (pinky_bringup 의 전역 토픽을 /picky1 으로 remap)
ros2 launch pinky_amr_1 picky1_bringup.launch.py

# State Manager + reverse_docking + move_to_goal
ros2 launch pinky_amr_1 picky1_state_manager.launch.py \
  server_base_url:=http://192.168.4.1:8000
```

실행 후 토픽 / 액션:

- `/picky1/picky_state`, `/picky1/move_command`, `/picky1/dock_command`
- `/picky1/cmd_vel`, `/picky1/odom`, `/picky1/scan`
- `/picky1/battery/voltage`, `/picky1/battery_state`
- `/picky1/initialpose`, `/picky1/camera/image_raw`
- `/picky1/navigate_to_pose`

### PICKY2

`pinky_amr_2` 패키지가 같은 패턴의 `picky2_*` launch 를 별도로 관리한다.
amr_2 폴더는 이명제 담당이며 amr_1 측에서는 수정하지 않는다.

---

## 책임 경계

| | state_manager | move_to_goal | reverse_docking |
|---|---|---|---|
| picky_state 전이 | O | X | X |
| MoveCommand goal accept/result | O | X | X |
| DockCommand goal accept/result | O | X | X |
| waypoint 1개 주행 `(x, y, θ)` | X | O | X |
| ArUco 검출 + Phase 0 ~ 3 | X | X | O |
| AMCL `/initialpose` publish | X | X | O |
| Control Server HTTP 보고 | O | X | X |

`state_manager` 는 세션 상태만 다루고 실제 모터 제어는 다른 두 노드에 위임한다.
새 주행 동작이 필요하면 state_manager 가 직접 만들지 말고 별도 노드를 신설해
위임하는 패턴을 유지한다.

---

## 변경 이력

### 2026-05-24

**DockCommand Action Server 신설**

- RETURN_HOME 과 DOCK_IN 흐름을 인터페이스 수준에서 분리. RETURN_HOME 은
  STANDBY_ZONE 도착으로 종료, 후진 도킹은 별도 DOCK_IN task 로 dispatch.
- `/{ns}/dock_command` Action Server 추가 (`_on_dock_goal`, `_on_dock_cancel`,
  `_execute_dock`). MoveCommand 콜백은 `_on_move_goal` / `_on_move_cancel`
  로 명칭 정리.
- `_execute_move` 의 RETURN_HOME 분기 단순화. `_do_docking()` 메서드 삭제.
- `ARRIVAL_STATE` 에 `'RETURN_HOME': 'STANDBY'` 추가.

**reverse_docking 명명 통일**

- `ReverseDocking.dock(...)` → `ReverseDocking.reverse_dock(...)` 메서드명 변경
  (메서드 / 로그 / 진입점 문서 모두 일관화).
- state_manager 내부 변수 `self._aruco` → `self._reverse_docking`,
  `aruco_node` → `reverse_docking_node`.
- 파라미터 정리: `aruco_marker_id` / `standby_x` / `standby_y` / `standby_theta`
  제거. 대신 dock 별 `charging_dock_{1,2}.{marker_id, map_x, map_y, map_yaw}`
  파라미터 신설 (DockCommand goal 의 `dock_name` 으로 lookup).

**PICKY1 namespace 인프라**

- `launch/picky1_bringup.launch.py` 신설 — `pinky_bringup` 의 전역 토픽
  (`/cmd_vel`, `/odom`, `/scan`, `/joint_states`, `/battery/*`) 을 `/picky1/...`
  로 remap.
- `launch/picky1_state_manager.launch.py` 신설 — `namespace='picky1'` 에서
  `state_manager` exec 실행. `reverse_docking.yaml` 파라미터 주입 + `robot_id`,
  `server_base_url` 도 launch arg 로 전달.
- `setup.py` 에 `launch/*.launch.py` glob 등록.

**토픽 상대경로화**

- `state_manager.py` 의 `robot_namespace` 파라미터 제거. `f'/{self._ns}/...'`
  형태로 코드에서 직접 namespace prefix 를 붙이던 절대경로 토픽을 모두
  상대경로로 변경 (`picky_state`, `move_command`, `dock_command`,
  `battery/voltage`, `battery_state`).
- `reverse_docking.py` 의 `/initialpose` → `initialpose`, `camera_topic`
  기본값 `/camera/image_raw` → `camera/image_raw`.
- `params/reverse_docking.yaml` 의 `camera_topic` 도 동일 정정. yaml top-level
  키를 `reverse_docking:` → `/**/reverse_docking:` 으로 변경해 namespace
  prefix (`/picky1/reverse_docking` 등) 와 무관하게 파라미터가 매칭되도록 함.
- 효과: launch namespace 만 다르게 띄우면 PICKY1 / PICKY2 가 같은 ROS_DOMAIN_ID
  에서도 토픽 충돌 없이 공존. 노드 코드는 어느 PICKY 인지 모르게 작성된다.

**관련 외부 변경**

같은 날 `fleet_manager/TRAFFIC_MANAGER_API.md` 도 갱신:

- 책임 경계 표에 "DB / 외부 state 수정 — TrafficManager / TaskManager 모두 X"
  명문화.
- 알려진 한계 1번 항목을 "DOCK_IN 도크 점유 해제 정책 (의도된 한계)" 으로 갱신.
  `release_dock` 같은 명시적 해제 API 는 의도적으로 추가하지 않으며, 도크 해제는
  `notify_state` 안전망 (picky_state 가 CHARGING 에서 이탈할 때 자동) 에만
  의존한다.
