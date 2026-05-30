# 주문 시나리오 통합 테스트 가이드 (헤드리스 실로봇 PICKY1)

UI에서 **주문 → PICKY 주행 → 픽업 완료 → 도크 복귀**까지 처음 하는 사람도 따라 할 수 있게
정리한 문서다. 명령은 그대로 복사해서 쓴다.

구성은 **모니터 없는 보드(raspi, SSH 접속) + 관제 PC** 두 대다. PICKY1 한 대로 테스트한다.
**모든 셸은 `ROS_DOMAIN_ID=25`** 여야 통신된다(`~/.bashrc` 에 넣어두면 편하다).

관련 설계 문서: `docs/Fleet_manager.md`(동작), `docs/Fleet_manager_interface.md`(인터페이스 계약),
`docs/Fleet_manager_TODO.md`(현황).

---

## 0. 먼저 알아둘 것 (현재 한계)

이 테스트는 **반자동**이다.

- **PICKY(주행) 이동/도킹은 실제로 동작한다.** `MOVE_TO_PRODUCT`, `MOVE_TO_PICKUP`,
  `RETURN_HOME`, `DOCK_IN` 은 Fleet Manager가 명령하고 로봇이 진짜 움직인다.
- **COBOT(로봇팔) 작업은 action 메시지와 action server가 확정되어야 자동 실행된다.**
  Fleet 쪽 연결 코드는 준비되어 있지만 `ExecuteTask.action` 생성 또는 COBOT State Manager의
  `/cobot1/execute_task` 서버가 없으면 `SORTING_AND_LOAD`, `INSPECTION`, `UNLOAD` 는
  **`ASSIGNED` 상태에서 멈춘다.**
- 따라서 COBOT 차례가 오면 **테스터가 API로 직접 "완료" 처리**해 줘야 한다(§3.2).

> 한 줄 요약: PICKY는 진짜 움직이고, COBOT 차례엔 사람이 손으로 "다 했다"고 눌러주는 테스트다.

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

> 로봇/시뮬 없이 Fleet Manager 로직만 보고 싶으면 §1.2 대신 **부록 A(Mock)** 를 쓴다.

---

## 1. 기동

### 1.1 빌드 (최초 1회 또는 코드 갱신 시) — 보드

```bash
cd ~/just_pick_it
git pull origin dev
./build_amr.sh                 # AMR 보드용 (로봇팔 패키지 제외)
source install/setup.bash
```

`.py`/`.xml`(launch)/`.yaml`(params) 가 바뀌었을 때만 빌드하면 된다. `.sh`/`.md` 만 받았으면
빌드 없이 바로 실행한다.

### 1.2 보드 — 주행 스택 (명령 하나)

```bash
bash scripts/navigation/run_picky1_all.sh
```

tmux 세션 `picky1` 에 **bringup / nav / state** 3개 창이 뜬다.

- 창 전환 `Ctrl+b` 0/1/2 | 떼기(노드 유지) `Ctrl+b d` | 다시 붙기 `tmux attach -t picky1`
- 전체 종료 `tmux kill-session -t picky1` | tmux 없으면 `sudo apt install -y tmux`
- SSH 가 끊겨도 노드는 살아있다.

### 1.3 관제 PC — DB 초기화 + Fleet/Web

```bash
cd ~/just_pick_it
./reset_demo_data.sh           # 데모 데이터 리셋 (PICKY/COBOT, 상품 6종, 픽업 슬롯 2개 seed)
./run_all.sh                   # Fleet Manager(:8100) + Web Gateway(:8000), 도메인 25 자동
```

`Fleet API ready` 가 보이면 성공. 헬스 체크: `curl -s http://localhost:8100/api/health/db` → `{"status":"ok"}`

### 1.4 관제 PC — RViz

```bash
bash scripts/navigation/rviz_picky1.sh
```

RViz 의 `/tf`,`/tf_static`,`/initialpose`,`/goal_pose` 를 `/picky1/...` 로 remap 해서 띄운다.
그냥 `rviz2` 로 띄우면 TF 를 글로벌 `/tf` 에서 찾다가 아무것도 안 그려진다.

### 1.5 기동 검증 (관제 PC)

```bash
export ROS_DOMAIN_ID=25
ros2 node list | grep map_server      # /picky1/map_server (picky1 한 번이어야 정상, 이중 아님)
ros2 topic hz /picky1/scan            # 라이다 약 10Hz
ros2 action list | grep -E 'picky1/(navigate_to_pose|move_command|dock_command)'
```

세 액션이 다 보이면 Fleet 가 PICKY1 에 명령을 보낼 수 있는 상태다.

---

## 2. 위치추정 (RViz)

`navigate_to_pose` 서버가 떠 있어도 **amcl 이 로봇 위치를 모르면** 주행이 실패한다. RViz 에서
초기 위치를 잡아준다.

1. `Fixed Frame` 을 `odom` 으로 두고 `Add > By topic` 으로 **`/picky1/scan`**(LaserScan) 추가
   → 라이다 빨간 점이 보이면 TF/데이터 정상.
2. **`/picky1/map`**(Map) 추가. 맵이 안 뜨면 Map display 의 `Topic > Durability Policy` 를
   **`Transient Local`** 로 바꾼다(맵은 latched 토픽이라 QoS 가 안 맞으면 안 들어온다).
3. amcl 을 한 번 깨운다(관제 PC):
   ```bash
   export ROS_DOMAIN_ID=25
   ros2 topic pub --once /picky1/initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
   "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, orientation: {z: 0.0, w: 1.0}}}}"
   ```
4. `Fixed Frame` 을 `map` 으로 바꾸고, 상단 **2D Pose Estimate** 로 로봇 실제 위치·방향을 클릭.
   라이다 점이 맵 벽과 겹치면 성공. 로봇을 살짝 주행시키면 amcl 이 수렴해 더 정확해진다.
   ```bash
   ros2 topic echo /picky1/amcl_pose --once    # 값이 나오면 위치추정 완료
   ```

> **배터리 확인 (주행 전 필수).** PICKY1 배터리가 0 이면 Fleet 가 배터리 높은 PICKY2 에 주문을
> 배정하고, PICKY2 노드가 없어 주문이 ERROR 가 된다. 배터리 기준은 7.6V=100%, 6.8V=0%(2셀)다.
> ```bash
> ros2 topic echo /picky1/battery/percent --once   # 0 보다 커야 함
> ros2 topic echo /picky1/battery/voltage --once   # 복귀(40%)까지 보려면 약 7.12V, 권장 7.3V 이상
> ```
> 충전이 필요하면 충전기를 꽂아 충전한 뒤 **뺀다**. 충전기를 **꽂는 순간 보드 노드가 죽으므로**,
> 충전 후 충전기를 빼고 §1.2 부터 다시 기동한다.

---

## 3. 주문 + COBOT 수동 완료

### 3.1 주문 (관제 PC)

상품 id 1번 1개 주문(처음엔 1개 권장):

```bash
curl -s -X POST http://localhost:8000/api/orders -H 'Content-Type: application/json' \
  -d '{"items":[{"product_id":1,"quantity":1}]}' | jq
```

응답의 **`order_id`** 를 기억한다. 주문이 들어가면 Fleet 가 최대 5초 polling 안에 로봇
unit(PICKY1+COBOT1)을 배정하고 첫 task 를 만든다.

PICKY 이동 관찰:

```bash
export ROS_DOMAIN_ID=25
ros2 topic echo /picky1/picky_state
#   MOVING_TO_PRODUCT -> (도착) WAITING_FOR_COBOT
```

`WAITING_FOR_COBOT` 이 뜨면 = "COBOT 차례" 다 → §3.2.

현재 task 목록은 언제든:

```bash
curl -s http://localhost:8100/api/fleet/orders/<order_id>/tasks | jq '.[] | {sequence_no, task_type, status}'
```

### 3.2 COBOT task 수동 완료 (핵심)

COBOT task 는 `ASSIGNED` 에서 멈춘다. 실제 로봇이 한 것처럼 RUNNING → SUCCESS 로 전이한다.

```bash
# 멈춰 있는 COBOT task 찾기
curl -s 'http://localhost:8100/api/fleet/tasks?status=ASSIGNED' \
  | jq '.[] | {task_id, task_type, assigned_robot_name}'

# 찾은 task_id 로 두 단계 전이 (TASK_ID 교체)
TASK_ID=2
curl -s -X PATCH http://localhost:8100/api/fleet/tasks/$TASK_ID -H 'Content-Type: application/json' \
  -d '{"current_status":"ASSIGNED","status":"RUNNING"}' | jq
curl -s -X PATCH http://localhost:8100/api/fleet/tasks/$TASK_ID -H 'Content-Type: application/json' \
  -d '{"current_status":"RUNNING","status":"SUCCESS"}' | jq
```

> **왜 두 단계인가.** `INSPECTION` 이 RUNNING 이 될 때 픽업 슬롯을 예약(`RESERVED`)한다. 한 번에
> SUCCESS 로 건너뛰면 슬롯 상태가 어긋난다. `current_status` 는 낙관적 잠금이라 현재 상태와
> 정확히 맞춰야 한다(다르면 `task status conflict`).

### 3.3 반복

```text
MOVE_TO_PRODUCT(자동) -> SORTING_AND_LOAD(수동) -> MOVE_TO_PICKUP(자동)
  -> INSPECTION(수동) -> UNLOAD(수동) -> (주문 PICKUP_READY)
```

COBOT task 를 SUCCESS 로 만들면 Fleet polling(최대 5초)이 다음 PICKY task 를 만들어 보낸다.

---

## 4. 픽업 완료 + 도크 복귀

`UNLOAD` 가 SUCCESS 가 되면 주문이 `PICKUP_READY`, 픽업 슬롯이 `OCCUPIED` 가 된다.

```bash
curl -s http://localhost:8100/api/orders/<order_id> | jq '{order_id, status, pickup_slot_id}'
```

이후 **다른 대기 주문이 없고 배터리 40% 초과**면 복귀 체인이 자동 생성된다.

```text
RETURN_HOME -> DOCK_IN -> CHARGE
```

```bash
ros2 topic echo /picky1/picky_state
#   RETURNING -> STANDBY -> DOCKING -> CHARGING
```

`picky_state=CHARGING` 이고 도크에 들어가 있으면 **"도크 복귀까지" 성공**이다.

(선택) 고객 수령 처리 — 주문이 `COMPLETED` 가 되고 슬롯이 `EMPTY` 로 돌아간다(로봇 움직임 없음):

```bash
curl -s -X POST http://localhost:8100/api/orders/<order_id>/complete | jq
```

---

## 5. 성공 판정 체크리스트

- [ ] 주문 생성 직후 `ORDER_WAIT` → 잠시 후 task 생성
- [ ] PICKY 가 상품 구역으로 실제 이동(`MOVING_TO_PRODUCT` → `WAITING_FOR_COBOT`)
- [ ] `SORTING_AND_LOAD` 수동 SUCCESS 후 다음 이동 자동 진행
- [ ] PICKY 가 픽업 구역으로 이동(`MOVING_TO_PICKUP` → `WAITING_FOR_COBOT`)
- [ ] `INSPECTION`, `UNLOAD` 수동 SUCCESS 후 주문 `PICKUP_READY`, 슬롯 `OCCUPIED`
- [ ] `RETURN_HOME` → `DOCK_IN` 자동 수행, 최종 `picky_state=CHARGING`

---

## 6. 트러블슈팅

| 증상 | 원인 | 해결 |
|---|---|---|
| `urdf_tutorial`/`moveit_msgs` 못 찾아 빌드 실패 | AMR 보드가 로봇팔 패키지까지 빌드 | `./build_amr.sh` 로 로봇팔 계열 제외 |
| `curl .../health/db` 실패 | Fleet 미기동 또는 DB 미연결 | `./run_all.sh` 로그, `pg_isready` 확인 |
| Nav2 가 `nav2_container` 만 뜨고 map_server 없음 | ARM 에서 composition 로드 실패 | `headless_picky1_nav.sh` 가 `use_composition:=False` 로 띄움(기본) |
| `/picky1/picky1/...` 이중 네임스페이스 | bringup_launch 의 namespace 누수 | 자식 include 에 namespace 빈 값 전달(수정 반영됨), 재빌드 |
| RViz 에 아무것도 안 뜸 | 도메인 불일치 또는 TF 글로벌 구독 | 도메인 25 확인 + `rviz_picky1.sh`(TF remap) 로 실행 |
| 맵만 안 뜨고 라이다는 뜸 | Map display QoS 불일치 | Durability 를 `Transient Local` 로 |
| 맵은 떴는데 라이다가 안 맞음 | 위치추정 부정확 | 2D Pose Estimate 다시, 로봇 살짝 주행시켜 수렴 |
| PICKY 가 안 움직임 | `navigate_to_pose` 없음 / 위치추정 안 됨 | §1.5 액션 확인, §2 위치추정 |
| COBOT task 가 계속 `ASSIGNED` | 정상(아직 COBOT 미연동) | §3.2 로 수동 완료 |
| 주문이 ERROR / PICKY2 로 배정됨 | PICKY1 배터리 0 → Fleet 가 PICKY2 선택, PICKY2 노드 없음 | PICKY1 배터리 충전(§2) |
| `task status conflict` | `current_status` 가 실제와 다름 | `GET /api/fleet/tasks` 로 현재 status 확인 후 맞춰 PATCH |
| 도크 복귀가 안 생김 | 대기 주문 있음 / 배터리 40% 이하 | 큐 비우거나 배터리 확인 |
| 충전기 꽂은 뒤 노드 전멸 | 충전기 연결 시 전원 순단 | 충전 후 충전기 빼고 §1.2 재기동 |
| 처음부터 다시 | 데이터 꼬임 | `./run_all.sh` 중지 → `./reset_demo_data.sh` → 재기동 |

로그: Fleet Manager 는 `run_all.sh` 콘솔(`[TaskManager]`, `[RobotCommandGateway]`, `[TrafficManager]`),
State Manager 는 tmux `picky1` 의 state 창(`[StateManager]`).

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
이 Mock만 띄우면 PICKY task는 자동으로 SUCCESS가 되고, COBOT task는 §3.2대로 수동 완료하면
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

§1.2(보드 주행 스택) 대신 아래 한 줄로 Mock 을 띄운다. 나머지(§1.3 DB 초기화 + `run_all.sh`)는
동일하다.

```bash
cd ~/just_pick_it
source /opt/ros/jazzy/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=25
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

이후 §3(주문)~§4(복귀/수령)을 그대로 따라간다. 차이는 PICKY 이동/도킹이
`ros2 topic echo /picky1/picky_state` 로 안 보이고(텔레메트리 미발행) Mock 로그
`[MockRobot] picky1 MOVE 수신 ...` 로만 확인된다는 점이다. task 진행은 §3.1 처럼 task 목록으로 본다.

### A.4 한계

- 실제 로봇 위치/배터리 텔레메트리(`picky_state`/`battery`/`amcl_pose`)는 발행되지 않으므로,
  이에 의존하는 화면(`admin/map`, `admin/robots`)은 갱신되지 않을 수 있다.
- zone 단일 점유, 경로 예약 같은 TrafficManager 로직은 동작하지만, 실제 충돌 회피는 검증되지 않는다.
- 이 테스트가 통과해도 실주행/도킹/Nav2 연동은 §1~§2 로 별도 검증해야 한다.
