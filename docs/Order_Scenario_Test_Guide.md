# 주문 시나리오 통합 테스트 가이드

처음 하는 사람이 **UI에서 주문 → PICKY가 주문 task 수행 → 픽업 완료 → 도크 복귀**까지를
끝까지 따라 할 수 있도록 정리한 문서다. 명령어는 그대로 복사해서 쓸 수 있다.

관련 설계 문서: `docs/Fleet_manager.md`(동작), `docs/Fleet_manager_interface.md`(인터페이스 계약), `docs/Fleet_manager_TODO.md`(현황).

---

## 0. 먼저 꼭 알아야 할 것 (현재 한계)

이 테스트는 **반자동**이다. 이유는 다음과 같다.

- **PICKY(주행 로봇) 이동/도킹은 실제로 동작한다.** `MOVE_TO_PRODUCT`, `MOVE_TO_PICKUP`,
  `RETURN_HOME`, `DOCK_IN` 같은 PICKY task는 Fleet Manager가 로봇에 명령을 내리고 로봇이 진짜 움직인다.
- **COBOT(로봇팔) 작업은 아직 자동으로 실행되지 않는다.** COBOT 실행 인터페이스
  (`ExecuteTask.action`)가 아직 정의/연결되지 않았다(`robot_command_gateway.py`의
  `send_cobot_task()`가 `False` 반환). 그래서 `SORTING_AND_LOAD`, `INSPECTION`, `UNLOAD`
  같은 COBOT task는 로봇에 전달되지 못하고 **`ASSIGNED` 상태에서 멈춘다.**
- 따라서 주문 흐름을 끝까지 진행하려면, **COBOT task는 테스터가 API로 직접 "완료" 처리**해 줘야 한다.
  (이 문서 §5.3 참고)

> 한 줄 요약: PICKY는 진짜 움직이고, COBOT 차례가 오면 사람이 손으로 "다 했다"고 눌러주는 테스트다.

> **로봇/시뮬을 전혀 띄우지 않고 Fleet Manager 로직만 보고 싶다면** §3.3~3.5(T3/T4/T5)를
> 건너뛰지 말고 **Mock PICKY 액션 서버**로 대체해야 한다. 로봇 스택이 하나도 없으면 PICKY task는
> 멈추는 게 아니라 게이트웨이가 `MoveCommand action server 없음`으로 **약 2초 만에 FAILED 처리**해
> 주문이 ERROR로 빠진다. 자세한 방법은 §부록 A를 참고한다.

흐름 한눈에 보기 (상품 1개 주문 기준):

```text
[UI 주문]
  -> MOVE_TO_PRODUCT   (PICKY, 자동 주행)        상품 보관 구역으로 이동
  -> SORTING_AND_LOAD  (COBOT, 수동 완료 필요)   상품 선별/PICKY에 상차
  -> MOVE_TO_PICKUP    (PICKY, 자동 주행)        픽업 구역으로 이동
  -> INSPECTION        (COBOT, 수동 완료 필요)   주문 검수
  -> UNLOAD            (COBOT, 수동 완료 필요)   픽업 슬롯에 하차  => 주문 PICKUP_READY
  -> RETURN_HOME       (PICKY, 자동 주행)        대기 구역 복귀
  -> DOCK_IN           (PICKY, 자동 도킹)        충전 도크 진입
  -> CHARGE            (논리 task, 자동 완료)     충전
```

상품이 2개 이상이면 `MOVE_TO_PRODUCT` + `SORTING_AND_LOAD` 쌍이 상품 수만큼 반복된다.

---

## 1. 사전 준비 (최초 1회)

워크스페이스가 빌드되어 있고 DB가 준비되어 있어야 한다. 처음이거나 환경이 꼬였으면:

```bash
cd ~/just_pick_it
./reset_ws.sh
```

`reset_ws.sh`가 하는 일: `web/.venv` 세팅 / PostgreSQL role·DB·schema·seed / rosdep / `colcon build`.

확인 사항:

- ROS 2 Jazzy 설치됨 (`/opt/ros/jazzy/setup.bash` 존재)
- PostgreSQL 동작 중 (`pg_isready` 통과)
- `jq`, `curl` 설치 권장 (API 응답 보기 편함): `sudo apt install -y jq curl`

이미 한 번 세팅했다면 §2로 바로 간다.

---

## 2. 무엇을 어디서 띄우는가 (터미널 구성)

각 항목을 **별도 터미널**에서 띄운다. 모든 ROS 터미널은 먼저 아래 2줄을 실행해야 한다.

```bash
cd ~/just_pick_it
source /opt/ros/jazzy/setup.bash && source install/setup.bash
```

| 터미널 | 역할 | 비고 |
|---|---|---|
| T1 | DB 초기화(테스트 시작 전 데이터 리셋) | 한 번 실행 후 닫아도 됨 |
| T2 | Fleet Manager(:8100) + Web Gateway(:8000) | `./run_all.sh` |
| T3 | PICKY1 하드웨어 bringup | 실로봇/시뮬 |
| T4 | PICKY1 내비게이션(Nav2) | `navigate_to_pose` 제공 + 위치추정(localize) |
| T5 | PICKY1 State Manager | 이동/도킹 Action 서버 |
| T6 | 모니터링/수동 명령 (curl, ros2 topic echo) | 작업용 |

> COBOT 노드는 이번 테스트에서 띄우지 않는다(아직 명령을 못 받으므로). COBOT 차례는 §5.3으로 처리한다.

---

## 3. 기동 절차

### 3.1 T1 — DB 초기화

테스트는 항상 깨끗한 데이터에서 시작하는 것이 헷갈리지 않는다.

```bash
cd ~/just_pick_it
./reset_demo_data.sh
```

이 명령은 schema는 유지하고 주문/task/로봇 등 데모 테이블을 비운 뒤 `db/seed.sql`을 다시 넣는다.
seed에는 PICKY1/PICKY2, COBOT1/COBOT2, 상품 6종, zone, 픽업 슬롯 2개가 들어간다(PICKY 배터리 100%).

### 3.2 T2 — Fleet Manager + Web

```bash
cd ~/just_pick_it
./run_all.sh
```

성공 시 로그에 다음이 보인다.

- `Fleet API ready`
- `HTTP API 서버 시작: http://0.0.0.0:8100`
- Web Gateway가 `:8000`에서 실행

별도 터미널에서 헬스 체크:

```bash
curl -s http://localhost:8100/api/health/db
# {"status":"ok"}
```

### 3.3 T3 — PICKY1 bringup (로봇 하드웨어)

실제 로봇이면 PICKY1 본체에서:

```bash
ros2 launch pinky_amr_1 picky1_bringup.launch.py
```

이 launch는 원본 `pinky_bringup`을 `/picky1` 네임스페이스로 remap해서 `/picky1/cmd_vel`,
`/picky1/scan`, `/picky1/odom`, `/picky1/battery/percent`, `/picky1/tf` 등을 만든다.

> 시뮬레이션으로 할 경우 `scripts/navigation/sim_navigation.sh` 기반으로 Gazebo + Nav2를 띄운다(아래 3.4 참고).

### 3.4 T4 — PICKY1 내비게이션 (Nav2)

PICKY가 실제로 움직이려면 **Nav2의 `navigate_to_pose` 액션이 `/picky1` 네임스페이스로 제공되고,
로봇이 맵에서 위치추정(localize)되어 있어야 한다.** State Manager 내부의 `move_to_goal`이
`/picky1/navigate_to_pose`로 목표를 보내기 때문이다.

- 실로봇: `bash scripts/navigation/real_navigation.sh`
- 시뮬레이션: `bash scripts/navigation/sim_navigation.sh`

기동 후 RViz의 **2D Pose Estimate**로 초기 위치를 맞춰 `/picky1/amcl_pose`가 나오도록 한다.

> 알려진 주의점(메모리/현황): **Nav2 멀티로봇 네임스페이스화는 아직 정리되지 않았다.**
> 단일 로봇(PICKY1)만 테스트할 때는 `navigate_to_pose`가 `/picky1/navigate_to_pose`로
> 노출되는지 반드시 확인한다. 아래로 확인:
>
> ```bash
> ros2 action list | grep navigate_to_pose
> # /picky1/navigate_to_pose 가 보여야 한다
> ```

### 3.5 T5 — PICKY1 State Manager

```bash
ros2 launch pinky_amr_1 picky1_state_manager.launch.py
```

이 launch는 한 프로세스에 `state_manager` + `move_to_goal` + `reverse_docking`를
`/picky1` 네임스페이스로 띄운다. 시작 로그: `[StateManager] 시작 — robot_id=PICKY1`.

### 3.6 기동 검증 (T6에서)

```bash
# PICKY1 이동 명령 Action 서버가 떴는지
ros2 action list | grep picky1
#   /picky1/move_command
#   /picky1/dock_command

# picky_state가 발행되는지 (Ctrl+C로 종료)
ros2 topic echo /picky1/picky_state
#   data: CHARGING  (또는 STANDBY)
```

여기까지 보이면 준비 완료다.

---

## 4. 주문 시작 (UI)

### 4.1 웹 화면에서

1. 브라우저에서 **고객 페이지**를 연다: `http://localhost:8000/customer`
2. 상품을 1개 담고 주문을 넣는다(처음엔 상품 1개로 시작하는 걸 권장).
3. **관제 페이지**를 다른 탭에 연다: `http://localhost:8000/admin/orders`
   - 주문/작업/픽업 슬롯 상태가 실시간(WebSocket)으로 갱신된다.
   - 로봇 위치는 `http://localhost:8000/admin/map`, 로봇 상태는 `http://localhost:8000/admin/robots`.

### 4.2 curl로 주문 (화면 없이 테스트할 때)

상품 id 1번을 1개 주문:

```bash
curl -s -X POST http://localhost:8000/api/orders \
  -H 'Content-Type: application/json' \
  -d '{"items":[{"product_id":1,"quantity":1}]}' | jq
```

> `:8000`은 Web Gateway, 내부적으로 `:8100`(Fleet API)로 프록시된다. `:8100`로 직접 호출해도 된다.

주문이 들어가면 주문 상태는 `ORDER_WAIT`가 되고, Fleet Manager가 **최대 5초 polling 주기**
(`waiting_work_poll_period_sec`) 안에 로봇 unit(PICKY1+COBOT1)을 배정하고 첫 task를 만든다.

---

## 5. 흐름 따라가기 + COBOT 수동 완료

### 5.1 현재 task 보기

주문 id가 1이라고 가정(주문 응답의 `order_id` 사용):

```bash
# 이 주문의 task 목록 (순서/타입/상태/담당로봇)
curl -s http://localhost:8100/api/fleet/orders/1/tasks | jq '.[] | {sequence_no, task_type, status, assigned_robot_name}'
```

또는 관제 페이지 `http://localhost:8000/admin/orders`에서 표로 본다.

### 5.2 PICKY 이동은 그냥 지켜본다 (자동)

`MOVE_TO_PRODUCT`, `MOVE_TO_PICKUP`, `RETURN_HOME`, `DOCK_IN`은 자동이다. 진행 확인:

```bash
ros2 topic echo /picky1/picky_state
#   MOVING_TO_PRODUCT -> (도착) WAITING_FOR_COBOT
```

- 이동 시작 시 `MOVING_TO_*`로 바뀐다.
- 목적지 도착 시 `WAITING_FOR_COBOT`으로 바뀌고, 그 task는 `SUCCESS`가 된다.
- `WAITING_FOR_COBOT`이 되면 = "이제 COBOT 차례"라는 뜻 → §5.3으로 넘어간다.

### 5.3 COBOT task 수동 완료 (핵심)

COBOT task(`SORTING_AND_LOAD` / `INSPECTION` / `UNLOAD`)는 `ASSIGNED`에서 멈춘다.
Fleet Manager 로그에도 `COBOT 실행 인터페이스 대기 중, 상태는 ASSIGNED 유지 후 재시도`가 주기적으로 찍힌다.

**1) 지금 멈춰 있는 COBOT task의 id 찾기:**

```bash
curl -s 'http://localhost:8100/api/fleet/tasks?status=ASSIGNED' \
  | jq '.[] | {task_id, task_type, status, assigned_robot_name}'
```

COBOT task(`SORTING_AND_LOAD` 등)의 `task_id`를 확인한다(예: 2).

**2) 그 task를 실제 로봇이 한 것처럼 RUNNING → SUCCESS로 전이:**

```bash
TASK_ID=2   # 위에서 찾은 COBOT task id

# (1) 작업 시작: ASSIGNED -> RUNNING
curl -s -X PATCH http://localhost:8100/api/fleet/tasks/$TASK_ID \
  -H 'Content-Type: application/json' \
  -d '{"current_status":"ASSIGNED","status":"RUNNING"}' | jq

# (2) 작업 완료: RUNNING -> SUCCESS
curl -s -X PATCH http://localhost:8100/api/fleet/tasks/$TASK_ID \
  -H 'Content-Type: application/json' \
  -d '{"current_status":"RUNNING","status":"SUCCESS"}' | jq
```

> **왜 두 단계(RUNNING → SUCCESS)인가?** Fleet Manager는 RUNNING 진입 시점에 일부 처리를 한다.
> 특히 `INSPECTION`이 RUNNING이 되면 픽업 슬롯을 예약(`RESERVED`)한다. 한 번에 SUCCESS로 건너뛰면
> 이 단계가 빠져 픽업 슬롯 상태가 어긋날 수 있다. 그래서 실제 로봇 수명주기와 똑같이 두 번 호출한다.
> `current_status`는 낙관적 잠금(현재 상태가 기대와 다르면 거부)이라 정확히 맞춰야 한다.

**3) 다음 task로 진행:** COBOT task를 SUCCESS로 만든 뒤, Fleet Manager polling(최대 5초)이
다음 task를 만들어 PICKY에 보낸다. `ros2 topic echo /picky1/picky_state`로 다음 이동이
시작되는지 보거나, §5.1로 task 목록을 다시 확인한다.

이 과정을 시퀀스대로 반복한다:

```text
MOVE_TO_PRODUCT(자동) -> SORTING_AND_LOAD(수동) -> MOVE_TO_PICKUP(자동)
  -> INSPECTION(수동) -> UNLOAD(수동) -> (주문 PICKUP_READY)
```

---

## 6. 픽업 완료 + 도크 복귀 확인

### 6.1 주문 PICKUP_READY

`UNLOAD`가 SUCCESS가 되면:

- 주문 상태가 `PICKUP_READY`로 바뀐다.
- 픽업 슬롯이 `OCCUPIED`가 된다.

```bash
curl -s http://localhost:8100/api/orders/1 | jq '{order_id, status, pickup_slot_id}'
```

### 6.2 도크 복귀는 자동으로 이어진다

주문의 모든 task가 SUCCESS이고, **다른 대기 주문이 없고 배터리가 40% 초과**면
Fleet Manager가 복귀 체인을 자동으로 만든다(`PARKING` 사유).

```text
RETURN_HOME -> DOCK_IN -> CHARGE
```

- `RETURN_HOME`: PICKY가 대기 구역(STANDBY_ZONE)으로 복귀(`picky_state=RETURNING` → 도착 시 `STANDBY`).
- `DOCK_IN`: ArUco 기반 후진 도킹(`picky_state=DOCKING` → 완료 시 `CHARGING`).
- `CHARGE`: 논리 task. 배터리가 기준 초과면 자동으로 `SUCCESS`.

확인:

```bash
ros2 topic echo /picky1/picky_state
#   RETURNING -> STANDBY -> DOCKING -> CHARGING
```

`picky_state=CHARGING`이고 도크에 들어가 있으면 **"도크로 돌아오는 것까지" 성공**이다.

> 만약 복귀가 안 생긴다면: 처리되지 않은 다른 주문이 큐에 있거나(`_has_assignable_waiting_work`),
> 같은 로봇에 다른 열린 task가 있으면 복귀를 생략한다. 또는 배터리가 40% 이하인지 확인한다.

### 6.3 (선택) 고객 수령 처리

도크 복귀와 별개로, 고객이 물건을 가져가는 것은 UI/별도 호출이다. 이걸 하면 주문이 `COMPLETED`가 되고
픽업 슬롯이 다시 `EMPTY`가 된다(로봇 움직임 없음).

```bash
curl -s -X POST http://localhost:8100/api/orders/1/complete | jq
```

---

## 7. 성공 판정 체크리스트

- [ ] 주문 생성 직후 주문 상태 `ORDER_WAIT` → 잠시 후 task가 생성됨
- [ ] PICKY가 상품 구역으로 실제 이동(`MOVING_TO_PRODUCT` → `WAITING_FOR_COBOT`)
- [ ] `SORTING_AND_LOAD` 수동 SUCCESS 후 다음 이동 자동 진행
- [ ] PICKY가 픽업 구역으로 이동(`MOVING_TO_PICKUP` → `WAITING_FOR_COBOT`)
- [ ] `INSPECTION`, `UNLOAD` 수동 SUCCESS 후 주문 `PICKUP_READY`, 슬롯 `OCCUPIED`
- [ ] `RETURN_HOME` → `DOCK_IN` 자동 생성·수행, 최종 `picky_state=CHARGING`

---

## 8. 트러블슈팅

| 증상 | 원인 / 확인 | 해결 |
|---|---|---|
| `curl .../health/db` 실패 | Fleet Manager 미기동 또는 DB 미연결 | T2 `./run_all.sh` 로그 확인, `pg_isready` |
| 주문해도 task가 안 생김 | 배정 가능한 로봇 unit 없음 / polling 대기 | 최대 5초 대기. 로봇 `robot_status`가 `IDLE`인지(`admin/robots`) 확인 |
| COBOT task가 계속 `ASSIGNED` | 정상(아직 COBOT 미연동) | §5.3으로 수동 완료 |
| PICKY가 안 움직임 | `navigate_to_pose` 없음 / 위치추정 안 됨 | `ros2 action list \| grep picky1/navigate_to_pose`, RViz에서 2D Pose Estimate |
| `MoveCommand action server 없음` 경고 | State Manager 미기동 | T5 `picky1_state_manager.launch.py` 확인 |
| `task status conflict` | `current_status`가 실제 상태와 다름 | `GET /api/fleet/tasks`로 현재 status 확인 후 맞춰서 PATCH |
| 다른 로봇/구역과 충돌해 멈춤 | zone 단일 점유 제약(한 zone에 로봇 1대) | 단일 로봇으로 테스트, 경로상 다른 로봇 정리 |
| 도크 복귀가 안 생김 | 대기 주문 있음 / 배터리 충분+다음 작업 있음 | 큐 비우거나 다음 주문 없이 테스트 |
| 처음부터 다시 | 데이터 꼬임 | T2 중지 → T1 `./reset_demo_data.sh` → T2 재기동 |

로그 보기:

- Fleet Manager: T2 콘솔(`[TaskManager]`, `[RobotCommandGateway]`, `[TrafficManager]` 태그)
- State Manager: T5 콘솔(`[StateManager]` 태그)

---

## 9. 빠른 치트시트

```bash
# (1) 초기화
./reset_demo_data.sh

# (2) 인프라
./run_all.sh                                   # T2

# (3) 로봇 (각각 별도 터미널, 먼저 source 2줄)
ros2 launch pinky_amr_1 picky1_bringup.launch.py          # T3
bash scripts/navigation/real_navigation.sh               # T4 (시뮬은 sim_navigation.sh)
ros2 launch pinky_amr_1 picky1_state_manager.launch.py    # T5

# (4) 주문
curl -s -X POST http://localhost:8000/api/orders -H 'Content-Type: application/json' \
  -d '{"items":[{"product_id":1,"quantity":1}]}' | jq

# (5) 멈춘 COBOT task 찾기
curl -s 'http://localhost:8100/api/fleet/tasks?status=ASSIGNED' \
  | jq '.[] | {task_id, task_type, assigned_robot_name}'

# (6) COBOT task 수동 완료 (TASK_ID 교체)
curl -s -X PATCH http://localhost:8100/api/fleet/tasks/TASK_ID -H 'Content-Type: application/json' -d '{"current_status":"ASSIGNED","status":"RUNNING"}'
curl -s -X PATCH http://localhost:8100/api/fleet/tasks/TASK_ID -H 'Content-Type: application/json' -d '{"current_status":"RUNNING","status":"SUCCESS"}'

# (7) 상태 보기
ros2 topic echo /picky1/picky_state
curl -s http://localhost:8100/api/fleet/orders/1/tasks | jq '.[] | {sequence_no, task_type, status}'
```

---

## 부록 A. 로봇 없이 로직만 테스트 (Mock PICKY 액션 서버)

Gazebo/실로봇을 띄우기 어려운 환경에서 **Fleet Manager 오케스트레이션 로직만** 검증하고 싶을 때 쓴다.
이 방식은 실제 주행/도킹/Nav2/위치추정은 검증하지 않는다. 검증 대상은 task 흐름, 상태 전이,
복귀 체인, 픽업 슬롯 로직이다.

### A.1 왜 필요한가

Fleet Manager의 `RobotCommandGateway`는 PICKY 이동/도킹 task를 보낼 때
`/picky1/move_command`(MoveCommand), `/picky1/dock_command`(DockCommand) 액션 서버를 찾는다
(namespace는 robot_name 소문자, 예: PICKY1 은 picky1). 액션 서버가 없으면
`wait_for_server` 타임아웃(기본 2초) 뒤 task를 **FAILED**로 만들고 주문이 ERROR로 빠진다.

따라서 로봇 스택을 안 띄울 때는, 들어온 goal을 즉시 success로 돌려주는 가짜 액션 서버가 필요하다.
이 Mock만 띄우면 PICKY task는 자동으로 SUCCESS가 되고, COBOT task는 §5.3대로 수동 완료하면
주문 흐름이 끝까지 진행된다.

### A.2 Mock 액션 서버 스크립트

아래를 `/tmp/mock_picky.py` 등에 저장한다(레포에 둘 필요는 없다).

```python
#!/usr/bin/env python3
"""주문 시나리오 로직 워크스루용 Mock PICKY 액션 서버.

실제 주행/도킹 없이 MoveCommand / DockCommand 액션을 즉시 success로 반환한다.
"""
import time

import rclpy
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from just_pick_it_interfaces.action import DockCommand, MoveCommand


class MockRobot(Node):
    def __init__(self, ns: str):
        super().__init__(f"mock_{ns}")
        self.ns = ns
        self._move = ActionServer(self, MoveCommand, f"/{ns}/move_command", self.move_cb)
        self._dock = ActionServer(self, DockCommand, f"/{ns}/dock_command", self.dock_cb)
        self.get_logger().info(f"[MockRobot] {ns} move_command/dock_command 액션 서버 시작")

    def move_cb(self, goal_handle):
        g = goal_handle.request
        self.get_logger().info(f"[{self.ns}] MOVE 수신: task_type={g.task_type} waypoints={len(g.waypoints)}")
        time.sleep(0.5)
        goal_handle.succeed()
        res = MoveCommand.Result()
        res.success = True
        res.message = f"mock move {g.task_type} done"
        return res

    def dock_cb(self, goal_handle):
        g = goal_handle.request
        self.get_logger().info(f"[{self.ns}] DOCK 수신: task_id={g.task_id} dock={g.dock_name}")
        time.sleep(0.5)
        goal_handle.succeed()
        res = DockCommand.Result()
        res.success = True
        res.message = "mock dock done"
        return res


def main():
    rclpy.init()
    ex = MultiThreadedExecutor()
    nodes = [MockRobot("picky1"), MockRobot("picky2")]
    for n in nodes:
        ex.add_node(n)
    try:
        ex.spin()
    finally:
        for n in nodes:
            n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
```

### A.3 실행 절차

§3.3~3.5(T3/T4/T5 로봇 스택) 대신 아래 한 줄로 Mock을 띄운다. 나머지(§3.1 DB 초기화, §3.2
`run_all.sh`)는 동일하다.

```bash
cd ~/just_pick_it
source /opt/ros/jazzy/setup.bash && source install/setup.bash
python3 /tmp/mock_picky.py
```

기동 확인:

```bash
ros2 action list | grep -E 'picky1|picky2'
#   /picky1/dock_command
#   /picky1/move_command
#   /picky2/dock_command
#   /picky2/move_command
```

이후 §4(주문)~§6(복귀/수령)을 그대로 따라가면 된다. 차이점은 단 하나, PICKY 이동/도킹이
`ros2 topic echo /picky1/picky_state`로 보이지 않고(텔레메트리 미발행), Mock 로그
`[MockRobot] picky1 MOVE 수신 ...`으로만 확인된다는 점이다. task가 SUCCESS로 넘어가는지는
§5.1처럼 task 목록으로 본다.

### A.4 한계

- 실제 로봇 위치/배터리 텔레메트리(`picky_state`/`battery`/`amcl_pose`)는 발행되지 않으므로,
  이에 의존하는 화면(`admin/map`, `admin/robots`)은 갱신되지 않을 수 있다.
- zone 단일 점유, 경로 예약 같은 TrafficManager 로직은 동작하지만, 실제 충돌 회피는 검증되지 않는다.
- 이 테스트가 통과해도 실주행/도킹/Nav2 연동은 별도로 §3.3~3.6으로 검증해야 한다.
