# ROS 2 자율주행 A-to-Z 입문 학습서

> 대상: ROS 2와 자율주행을 처음 시작하는 개발자  
> 기준일: 2026-05-14  
> 기본 실습 기준: **Ubuntu 24.04 + ROS 2 Jazzy Jalisco LTS**  
> 보조 기준: Ubuntu 22.04 사용자는 **ROS 2 Humble Hawksbill LTS**  
> 작성 목적: ROS 기본기 → Pinky Pro 하드웨어 경계 이해 → Control Server 연동 → 직접 AMR 자율주행 스택 구현까지 연결하기

---

## 0. 먼저 읽어야 할 범위 설명

이 문서의 중심은 **자율주행에 필요한 기본 지식**입니다.
프로젝트 설명은 그 지식을 `autonomous_sys_ws`에 어떻게 적용할지 보여주는 예시입니다.

이 문서는 두 가지 역할을 동시에 합니다.

1. ROS 2와 자율주행을 처음 배우는 사람을 위한 기본 개념 설명서.
2. `autonomous_sys_ws` 프로젝트에서 AMR 주행을 구현할 때 보는 프로젝트 적용 가이드.

자율주행은 한 문서에 “진짜로 하나도 빠짐없이” 담기에는 너무 큰 분야입니다. 실제 현업에서는 다음이 모두 별도 전문 분야입니다.

- ROS 2 미들웨어
- 로봇 좌표계 / 시간 동기화 / 센서 드라이버
- SLAM / Localization
- Nav2 기반 모바일 로봇 주행
- 차량형 자율주행: 인지, 예측, 판단, 계획, 제어
- 시뮬레이션, 데이터셋, 검증, 안전, 배포

따라서 이 문서는 **ROS 2, TF, odom, SLAM, AMCL, Nav2, planner, controller 같은 기본 지식을 먼저 잡고, 그다음 프로젝트에 적용하는 것**을 목표로 합니다.

`AMR_PLAN.md`와의 관계는 이렇게 보면 됩니다.

| 문서 | 역할 | 보는 타이밍 |
|---|---|---|
| `ros2_driving_beginner_guide.md` | ROS 2, SLAM, AMCL, Nav2, Planning/Control 기본 지식서 | 기본 개념을 배울 때 먼저 읽고, 막힐 때 다시 참고 |
| `AMR_PLAN.md` | 실제로 패키지와 노드를 만들 때 따라가는 작업 순서표 | 손코딩할 때 항상 열어두고 단계별로 진행 |

즉 공부할 때는 이 문서에서 기본 지식을 먼저 보고, 실제 구현을 시작할 때는 `AMR_PLAN.md`를 기준으로 진행합니다. 이 문서 전체를 처음부터 끝까지 외운 뒤 코딩하려고 하면 너무 오래 걸립니다.

자율주행 기본 지식은 아래 순서로 봅니다.

| 순서 | 기본 지식 | 왜 필요한가 |
|---|---|---|
| 1 | ROS 2 Node/Topic/Service/Action/Parameter/Launch | 로봇 기능을 여러 노드로 나누고 통신시키기 위해 필요 |
| 2 | TF2, `map`, `odom`, `base_link` | 모든 센서, 지도, goal pose를 같은 좌표계로 해석하기 위해 필요 |
| 3 | `cmd_vel`, odometry, differential drive | 로봇이 어떻게 움직이고 현재 위치를 어떻게 추정하는지 이해하기 위해 필요 |
| 4 | SLAM, OccupancyGrid, SLAM Toolbox | 주행할 지도를 만들고 map이 어떻게 생기는지 이해하기 위해 필요 |
| 5 | Localization, AMCL, particle filter | 만든 지도 안에서 로봇 위치를 찾는 원리를 이해하기 위해 필요 |
| 6 | Nav2 구조, planner, controller, costmap, recovery | 상용 navigation stack이 어떤 구조로 움직이는지 이해하기 위해 필요 |
| 7 | Dijkstra, A*, Pure Pursuit, obstacle stop | 직접 planner/controller/safety를 구현하기 위해 필요 |
| 8 | 디버깅, rosbag, RViz, 로그 | 안 움직일 때 원인을 좁히기 위해 필요 |

우리 프로젝트에서는 특히 아래를 목표로 봅니다.

- Pinky Pro가 `cmd_vel`을 받아 움직이고 `odom`/TF를 발행하는 흐름 이해.
- Pinky Pro에서 제공하는 Nav2/SLAM 코드는 정답으로 쓰기보다 참고 구현과 비교 기준으로 활용.
- 실제 AMR 주행 로직은 가능한 한 직접 구현.
- 로봇 제어 코드와 웹/API/DB 상태 전이를 섞지 않고 분리.

[정확도: 높음]  
ROS 2 / Nav2 / SLAM Toolbox / Autoware / CARLA에 대한 구조 설명은 공식 문서와 공개 문서에 근거했습니다.  
[정확도: 중간]  
알고리즘 선택 기준과 실무 튜닝 조언은 일반적인 로보틱스 실무 패턴에 근거한 요약입니다. 로봇 하드웨어, 센서, 환경에 따라 달라질 수 있습니다.

---

## 0.1 전체 읽는 순서

이 문서는 기본 지식을 먼저 잡고, 그다음 프로젝트 적용으로 넘어가는 흐름으로 봅니다.

| 순서 | 읽을 범위 | 목적 |
|---|---|---|
| 1 | Part A | ROS 2 Node/Topic/Service/Action/Parameter/Launch |
| 2 | Part B | TF2, `map`, `odom`, `base_link`, URDF |
| 3 | Part C | 이동 로봇, differential drive, `cmd_vel`, odometry, EKF |
| 4 | Part D | SLAM, OccupancyGrid, SLAM Toolbox, Mapping/Localization 차이 |
| 5 | Part D 24.1, Part F 38장 | AMCL, particle filter, localization 알고리즘 |
| 6 | Part E | Nav2 구조, AMCL/Nav2 연결, costmap, planner, controller, recovery |
| 7 | Part F | Dijkstra, A*, Pure Pursuit, obstacle avoidance 같은 직접 구현 알고리즘 |
| 8 | Part I, L, M, N | 디버깅, 체크리스트, 명령어, 용어 |
| 9 | 0.2장부터 0.10장 | 위 지식을 이 프로젝트에 어떻게 적용할지 확인 |
| 10 | `AMR_PLAN.md` | 실제 손코딩 순서대로 구현 |

처음 공부할 때는 프로젝트 설명보다 Part A~F를 먼저 보는 것이 맞습니다.
프로젝트 설명은 "이 지식을 우리 코드에 어디에 붙일지"를 확인하는 적용 파트입니다.

### 0.1.1 진짜 먼저 알아야 하는 기본 개념

아래 단어들은 AMR 자율주행을 하면서 계속 나옵니다.
정확한 수식보다 먼저 “무슨 역할인지”를 잡는 것이 중요합니다.

| 용어 | 먼저 이해할 뜻 |
|---|---|
| 좌표계 / frame | 위치와 방향을 표현하는 기준입니다. 지도 기준인지, 로봇 기준인지, 센서 기준인지가 다릅니다. |
| pose | 위치 `x, y, z`와 방향 `roll, pitch, yaw`를 합친 로봇의 자세입니다. |
| yaw | 평면에서 로봇이 바라보는 방향입니다. AMR 회전 제어에서 가장 자주 씁니다. |
| quaternion | ROS에서 3D 방향을 표현하는 방식입니다. 초반에는 yaw로 변환해서 이해합니다. |
| TF | 서로 다른 좌표계 사이의 변환 관계입니다. 예: `map -> odom -> base_link`. |
| `map` | 지도 기준 좌표계입니다. 저장된 지도에서 절대적인 기준처럼 씁니다. |
| `odom` | 로봇이 출발한 뒤 바퀴 이동량으로 이어지는 연속 좌표계입니다. 부드럽지만 시간이 지나면 오차가 쌓입니다. |
| `base_link` | 로봇 몸체 기준 좌표계입니다. 로봇 중심을 나타내는 경우가 많습니다. |
| `base_footprint` | 로봇을 바닥 평면에 투영한 기준 좌표계입니다. 2D 주행에서 자주 씁니다. |
| odometry / odom | 바퀴 encoder, IMU 등으로 “내가 얼마나 움직였는지” 추정한 값입니다. |
| `cmd_vel` | 로봇에게 주는 속도 명령입니다. `linear.x`는 전진 속도, `angular.z`는 회전 속도입니다. |
| `/scan` | 2D LiDAR가 본 거리 배열입니다. 장애물 정지와 SLAM에 자주 씁니다. |
| OccupancyGrid | 지도를 격자로 나눈 표현입니다. 보통 `0`은 빈 공간, `100`은 장애물, `-1`은 모름입니다. |
| SLAM | 지도를 만들면서 동시에 내 위치도 추정하는 기술입니다. 처음 환경을 탐색할 때 씁니다. |
| Localization | 이미 있는 지도 안에서 현재 로봇 위치를 찾는 기술입니다. |
| AMCL | 저장된 map과 현재 scan을 비교해서 로봇 위치를 찾는 particle filter 기반 localization입니다. |
| Costmap | 장애물과 위험도를 숫자로 표현한 지도입니다. planner/controller가 참고합니다. |
| Planner | 현재 위치에서 목표까지 갈 경로를 계산합니다. 예: Dijkstra, A*. |
| Controller | planner가 만든 경로를 실제 `cmd_vel`로 바꿉니다. 예: Pure Pursuit. |
| Recovery | 막혔을 때 정지, 재시도, 회전, 재계획 같은 회복 행동을 합니다. |
| Nav2 | ROS 2의 대표적인 navigation framework입니다. planner, controller, costmap, recovery를 묶은 시스템입니다. |

이 프로젝트에서 직접 구현할 때 가장 중요한 연결은 아래입니다.

```text
현재 pose를 odom/TF에서 읽는다.
  ↓
goal pose와 비교해서 거리와 yaw 오차를 계산한다.
  ↓
planner가 path를 만든다.
  ↓
controller가 path를 cmd_vel로 바꾼다.
  ↓
scan safety가 위험하면 cmd_vel을 제한하거나 0으로 만든다.
  ↓
pinky_bringup이 cmd_vel을 바퀴 명령으로 바꾼다.
```

## 0.2 이 프로젝트에서 먼저 잡아야 하는 그림

`autonomous_sys_ws`는 단순한 ROS 실습 워크스페이스가 아닙니다. 웹 Control Server, Pinky Pro AMR, myCobot 작업 로봇을 한 흐름으로 묶는 프로젝트입니다.

```text
고객/관리자 UI
  ↓ HTTP/WebSocket
web Control Server
  ↓ Fleet event/API
Fleet Manager
  ↓ State Manager / ROS2 command
PICKY runner
  ↓ 직접 만든 navigation API 또는 cmd_vel
Custom AMR Navigation Stack
  ↓ cmd_vel
pinky_bringup
  ↓ Dynamixel wheel
실제 로봇 이동
```

각 계층의 책임은 분리해서 봐야 합니다.

| 계층 | 이 프로젝트의 위치 | 책임 |
|---|---|---|
| 웹/관제 | `web/app` | 주문/재고/로봇/task 상태 저장, 관리자 UI 표시 |
| Fleet Manager | 앞으로 만들 후보: 별도 ROS/Fleet 패키지 | task 생성/배정, 경로 판단, State Manager 명령, Control Server 상태 보고 |
| PICKY runner | 앞으로 만들 후보: `src/just_pick_amr` 또는 별도 ROS 패키지 | Fleet Manager 명령 수신, task 실행 결과 보고, 직접 만든 주행 모듈 호출 |
| Custom navigation | 앞으로 직접 구현 | pose/scan/map/goal을 받아 planner/controller 계산 후 `cmd_vel` 발행 |
| 로봇 bringup | `src/pinky_pro/pinky_bringup` | 최소 하드웨어 adapter: `cmd_vel` 구독, 모터 제어, `odom`/TF 발행 |
| 제공 Navigation | `src/pinky_pro/pinky_navigation` | 참고 구현: SLAM, AMCL, Nav2, costmap, web bridge |
| Gazebo simulation | `src/pinky_pro/pinky_gz_sim` | 직접 만든 알고리즘 검증용 simulation 입출력 |

중요한 기준:

```text
Control Server는 "업무 데이터와 상태를 저장한다."
Fleet Manager는 "무슨 일을 어떤 순서로 할지"를 결정한다.
PICKY runner는 "받은 주행 명령을 실제로 수행한다."
Custom navigation은 "목표까지 어떻게 움직일지"를 직접 계산한다.
pinky_bringup은 "cmd_vel을 실제 바퀴 명령으로 바꾸는 일"만 맡긴다.
```

이 프로젝트에서는 Pinky에서 제공하는 Nav2 흐름을 그대로 쓰는 것이 최종 목표가 아닙니다. 제공 코드는 아래처럼 봅니다.

| 구분 | 사용 방식 |
|---|---|
| `pinky_bringup` | 우선 사용. 모터 드라이버, `cmd_vel`, `odom`, 배터리 경계로 활용 |
| `pinky_gz_sim` | 우선 사용. 직접 만든 알고리즘을 Gazebo에서 검증 |
| `pinky_navigation` Nav2/SLAM | 참고/비교/백업 용도. 메인 구현은 직접 작성 |
| `nav2_web_server.py` | TF 조회, action goal 전송 코드 패턴 참고 |
| `nav2_params.yaml` | costmap/controller 파라미터가 어떤 의미인지 학습 자료로 활용 |

직접 구현 우선 순서는 다음입니다.

```text
1. ROS 2 기본 입출력 확인: cmd_vel, odom, scan, tf
2. Control Server와 분리된 standalone custom navigation 노드 작성
3. 단순 go-to-goal controller 구현
4. scan 기반 장애물 정지/회피 구현
5. 직접 map 표현 또는 OccupancyGrid 사용
6. A* 같은 global planner 구현
7. Pure Pursuit/DWA 계열 local controller 구현
8. PICKY runner와 연결해서 Fleet Manager가 내려준 goal pose 수행
9. Gazebo 검증 후 실로봇 적용
```

Nav2는 “정답으로 가져다 쓰는 것”이 아니라, 직접 만든 결과가 이상할 때 비교하는 기준선입니다.

---

## 0.3 현재 코드 기준 핵심 파일

### Pinky Pro 실제 구동

| 파일 | 봐야 하는 이유 |
|---|---|
| `src/pinky_pro/pinky_bringup/pinky_bringup/bringup.py` | `cmd_vel`을 받아 Dynamixel wheel RPM으로 변환하고, `odom`, `joint_states`, `odom -> base_footprint` TF를 발행 |
| `src/pinky_pro/pinky_bringup/pinky_bringup/battery_publisher.py` | `battery/percent`, `battery/voltage` 발행 |
| `src/pinky_pro/pinky_bringup/launch/bringup_robot.launch.xml` | 실제 로봇 bringup 시작점 |

현재 `bringup.py`에서 중요한 토픽/프레임:

| 항목 | 값 |
|---|---|
| 입력 속도 명령 | `cmd_vel` |
| 오도메트리 | `odom` |
| joint state | `joint_states` |
| odom frame | `odom` |
| child frame | `base_footprint` |
| 배터리 전압 | `battery/voltage` |
| 배터리 퍼센트 | `battery/percent` |

즉 커스텀 자율주행을 하더라도 최종 출력은 우선 `geometry_msgs/msg/Twist` 형태의 `cmd_vel`이면 됩니다.

### Pinky Pro Navigation

| 파일 | 봐야 하는 이유 |
|---|---|
| `src/pinky_pro/pinky_navigation/launch/navigation_launch.xml` | Nav2가 어떤 노드 조합으로 주행하는지 참고 |
| `src/pinky_pro/pinky_navigation/launch/bringup_launch.xml` | map server, AMCL, navigation stack 구성을 참고 |
| `src/pinky_pro/pinky_navigation/launch/web_nav2.launch.xml` | 실제 로봇에서 제공 Nav2를 띄우는 방식 참고 |
| `src/pinky_pro/pinky_navigation/launch/gz_web_nav2.launch.xml` | Gazebo에서 제공 Nav2를 띄우는 방식 참고 |
| `src/pinky_pro/pinky_navigation/params/nav2_params.yaml` | AMCL, controller, costmap, planner 파라미터 학습 자료 |
| `src/pinky_pro/pinky_navigation/scripts/nav2_web_server.py` | TF pose 조회, map/path/costmap snapshot 코드 참고 |

현재 Nav2 흐름에서 특히 중요한 remap:

```text
controller_server / bt_navigator
  cmd_vel -> cmd_vel_nav

velocity_smoother
  cmd_vel_nav -> cmd_vel
```

즉 Nav2가 바로 `cmd_vel`을 내는 것이 아니라, 먼저 `cmd_vel_nav`를 만들고 `velocity_smoother`가 최종 `cmd_vel`로 바꿉니다. 커스텀 주행 노드를 만들 때 Nav2와 동시에 `cmd_vel`을 내보내면 충돌할 수 있으므로 실행 모드를 분리해야 합니다.

우리 방향에서는 이 패키지를 “사용할 기능”보다 “분석할 예시”로 봅니다. 예를 들어 직접 controller를 만들 때는 Nav2의 Regulated Pure Pursuit 설정을 보고 속도, goal tolerance, collision check가 왜 필요한지 이해합니다. 하지만 최종 실행 노드는 직접 만든 패키지에서 띄우는 것을 목표로 합니다.

### Gazebo Simulation

| 파일 | 봐야 하는 이유 |
|---|---|
| `src/pinky_pro/pinky_gz_sim/params/pinky_bridge.yaml` | Gazebo와 ROS 2 토픽 bridge |
| `src/pinky_pro/pinky_gz_sim/launch/launch_sim.launch.xml` | 시뮬레이션 시작 |
| `src/pinky_pro/pinky_gz_sim/worlds/*.world` | 테스트 환경 |

bridge 기준:

| ROS topic | 방향 | 의미 |
|---|---|---|
| `clock` | Gazebo -> ROS | simulation time |
| `tf` | Gazebo -> ROS | 시뮬레이션 TF |
| `scan` | Gazebo -> ROS | LiDAR |
| `odom` | Gazebo -> ROS | 오도메트리 |
| `cmd_vel` | ROS -> Gazebo | 로봇 속도 명령 |
| `joint_states` | Gazebo -> ROS | joint state |

실로봇 적용 전에는 여기서 먼저 검증하는 것이 안전합니다.

### Control Server

| 파일 | 봐야 하는 이유 |
|---|---|
| `web/WORKFLOW.md` | 주문/task 상태 전이 기준 문서 |
| `web/app/routers/fleet_router.py` | Fleet Manager가 호출할 Fleet API |
| `web/app/services/workflow_service.py` | task 상태 결과가 order/robot/stocking 상태에 반영되는 정책 |

Fleet Manager가 우선 써야 하는 API:

| 목적 | API |
|---|---|
| 전체 상태 복구 | `GET /api/fleet/snapshot` |
| PICKY 주차/상품/픽업/슬롯 좌표 조회 | `GET /api/fleet/zones` |
| task bulk 생성 | `POST /api/fleet/tasks/bulk` |
| 로봇 상태/pose/battery 보고 | `PATCH /api/fleet/robots/{robot_id}` |
| task 시작/완료/실패 보고 | `PATCH /api/fleet/tasks/{task_id}` |
| 예외 보고 | `POST /api/fleet/exceptions` |
| 다른 PICKY의 실행 task 확인 | `GET /api/fleet/robots/{robot_id}/running-task` |

---

## 0.4 직접 만들 것과 빌려 쓸 것

“다 직접 만든다”는 말은 현실적으로 두 단계로 나누는 것이 좋습니다.

1. 로봇 하드웨어와 통신하는 최소 adapter는 우선 빌려 쓴다.
2. 자율주행 판단/계획/제어는 직접 만든다.

처음부터 Dynamixel 통신, encoder odometry, LiDAR driver, battery driver까지 모두 다시 쓰면 자율주행 알고리즘을 만들기 전에 하드웨어 디버깅에 시간을 대부분 쓰게 됩니다. 그래서 1차 목표는 아래처럼 경계를 잡습니다.

| 영역 | 1차 전략 | 이유 |
|---|---|---|
| Dynamixel motor 제어 | `pinky_bringup` 사용 | `cmd_vel`만 내면 로봇이 움직이는 경계를 확보 |
| Encoder odometry | `pinky_bringup` 사용 | 현재 pose 추정의 최소 입력 확보 |
| Battery publish | `battery_publisher.py` 사용 | Fleet Manager 상태 보고에 필요 |
| Gazebo bridge | `pinky_gz_sim` 사용 | 직접 만든 알고리즘을 안전하게 반복 테스트 |
| SLAM | 처음엔 제공 launch로 개념 확인, 이후 직접 구현/대체 검토 | map 품질 문제와 주행 문제를 분리 |
| Localization | 처음엔 odom 기반, 이후 AMCL/직접 localization 비교 | 처음부터 particle filter까지 가면 범위가 너무 커짐 |
| Global planner | 직접 구현 | A*, Dijkstra부터 시작하기 좋음 |
| Local controller | 직접 구현 | go-to-goal, Pure Pursuit, DWA 순서로 확장 |
| Obstacle avoidance | 직접 구현 | `/scan` 기반 정지/회피부터 시작 |
| Fleet task runner | 직접 구현 | Control Server API와 PICKY/COBOT 실행부를 이어주는 핵심 |

최종적으로 더 깊게 가고 싶으면 `pinky_bringup`까지 대체할 수 있습니다. 하지만 프로젝트 목표가 AMR 자율주행이라면 1차로는 `cmd_vel`, `odom`, `scan`, `tf` 경계를 안정적으로 쓰고 그 위의 주행 스택을 직접 만드는 편이 좋습니다.

---

## 0.5 우리 프로젝트의 PICKY 실행 흐름

Fleet Manager 기준 task 흐름은 다음입니다.

```text
주문 생성
  ↓
Control Server가 ORDER_CREATED 이벤트 전송
  ↓
Fleet Manager가 주문/상품/zone/robot snapshot 조회
  ↓
Fleet Manager가 task 생성/배정 후 POST /api/fleet/tasks/bulk
  ↓
Fleet Manager가 PICKY runner에 주행 명령
  ↓
Fleet Manager가 task RUNNING 및 robot BUSY 상태 보고
  ↓
Custom navigation이 목표 지점으로 이동
  ↓
Fleet Manager가 성공/실패 결과를 Control Server에 보고
```

PICKY 주행 task:

| task_type | 의미 | 1차 주행 처리 |
|---|---|---|
| `MOVE_TO_PRODUCT` | 주문 상품 주차존 이동 | target zone pose로 이동 |
| `MOVE_TO_PICKUP` | 픽업존 이동 | target zone pose로 이동 |
| `MOVE_TO_STOCK` | 창고(입고)존 이동 | target zone pose로 이동 |
| `MOVE_TO_DISPLAY` | 진열 구역 이동 | target zone pose로 이동 |
| `RETURN_HOME` | 대기/충전 구역 복귀 | standby 또는 dock 앞 pose로 이동 |
| `CHARGE` | 충전 도킹 | 도킹 보정 루틴 실행 |

COBOT task(`SORTING_AND_LOAD`, `INSPECTION`, `UNLOAD`, `DISPLAY_SCAN`, `DISPLAY_PLACE`)는 PICKY 주행 runner가 직접 실행하지 않고 Fleet Manager/State Manager가 COBOT 쪽으로 분리해서 내려준다.

로봇 쪽에서 처음 구현할 runner는 DB를 직접 보면 안 됩니다. Fleet Manager가 내려주는 명령만 실행하고 결과를 Fleet Manager에 돌려주는 구조로 둡니다.

좋은 구조:

```text
PickyTaskRunner
  - Fleet Manager 명령 수신
  - 이동 시작/완료/실패 결과 보고
  - battery/pose report
  - NavigationAdapter 호출

NavigationAdapter
  - run(task) -> NavigationResult

CustomNavAdapter
  - 직접 planner/controller 사용
  - 최종 출력은 cmd_vel

Nav2Adapter
  - 선택 사항
  - 1차 구현 대상 아님
  - 직접 구현 결과와 비교하거나 긴급 fallback이 필요할 때만 검토
```

---

## 0.6 초보자용 학습 순서를 이 프로젝트에 맞게 바꾸면

범용 ROS 2 학습 순서는 아래 Part A 이후에 자세히 설명합니다. 하지만 이 프로젝트에서는 다음 순서로 보면 덜 헤맵니다.

### 1단계: 워크스페이스와 패키지 확인

```bash
cd /home/ane/autonomous_sys_ws
source /opt/ros/jazzy/setup.bash
colcon list --names-only | sort
```

빌드가 되어 있다면:

```bash
source install/setup.bash
ros2 pkg list | grep -E '^(pinky_|mycobot_)'
```

현재 워크스페이스에는 `build/`, `install/`, `log/`가 있고 `install/setup.bash`도 생성되어 있습니다. 새 터미널에서는 `source install/setup.bash`를 다시 해야 프로젝트 패키지가 보입니다.

### 2단계: ROS 2 기본 도구 익히기

처음에는 코드를 고치기보다 아래 명령으로 “지금 뭐가 떠 있는지” 보는 연습이 먼저입니다.

```bash
ros2 node list
ros2 topic list
ros2 topic echo /odom
ros2 topic echo /cmd_vel
ros2 topic echo /scan
ros2 run tf2_tools view_frames
ros2 action list
```

### 3단계: Pinky bringup 이해

실제 로봇:

```bash
ros2 launch pinky_bringup bringup_robot.launch.xml
```

확인:

```bash
ros2 topic list | grep -E 'cmd_vel|odom|joint_states|battery'
ros2 topic echo /battery/percent
ros2 topic echo /odom
```

여기서 `odom`이 안 나오면 Nav2나 커스텀 자율주행 이전 문제입니다.

### 4단계: 제공 SLAM/Nav2는 참고 기준으로만 실행해보기

직접 구현을 하더라도 제공 예제를 한 번은 띄워보는 것이 좋습니다. 이유는 “정답으로 쓰기 위해서”가 아니라, 정상적인 ROS topic/TF/map 흐름이 어떤 모습인지 눈으로 익히기 위해서입니다.

시뮬레이션 SLAM:

```bash
bash scripts/mapping/sim_map_building.sh
```

실로봇 SLAM:

```bash
bash scripts/mapping/real_map_building.sh
```

직접 launch할 때:

```bash
ros2 launch pinky_navigation map_building.launch.xml
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run nav2_map_server map_saver_cli -f my_map
```

여기서 얻을 것:

```text
/scan이 어떻게 생겼는지
/odom이 얼마나 흔들리는지
map이 어떤 형식으로 저장되는지
map -> odom -> base_footprint TF가 어떻게 이어지는지
```

### 5단계: 직접 만든 최소 주행 노드부터 시작

처음 직접 만들 노드는 완전한 자율주행이 아니어도 됩니다. 목표는 “내 코드가 pose와 goal을 보고 `cmd_vel`을 낼 수 있는가”입니다.

첫 노드의 기능:

```text
입력:
  /odom 또는 TF
  goal x, y, theta

출력:
  /cmd_vel

동작:
  1. goal 방향으로 회전
  2. goal까지 천천히 직진
  3. 가까워지면 정지
```

검증:

```bash
ros2 topic echo /odom
ros2 topic echo /cmd_vel
```

이 단계에서는 map, SLAM, Nav2, 장애물 회피를 넣지 않습니다. 먼저 가장 작은 폐루프를 완성합니다.

```text
현재 pose -> goal 오차 계산 -> cmd_vel publish -> 로봇 이동 -> odom 변화
```

### 6단계: scan 기반 안전 정지/회피 추가

그 다음에 `/scan`을 읽어서 앞에 장애물이 가까우면 멈추게 합니다.

```text
if front_obstacle_distance < threshold:
  cmd_vel = 0
else:
  go_to_goal
```

처음부터 예쁜 회피 경로를 만들 필요는 없습니다. “부딪히지 않고 멈춤”이 먼저입니다.

### 7단계: Fleet Manager 명령과 연결

Fleet Manager가 켜진 상태에서 PICKY runner가 할 일:

```text
1. battery/percent 구독
2. TF 또는 odom으로 현재 pose 추출
3. Fleet Manager로 현재 pose/battery 주기 보고
4. Fleet Manager에서 주행 goal 수신
5. target_zone_pose를 직접 만든 navigation goal로 전달
6. 결과를 SUCCESS 또는 FAILED로 보고
```

이 단계에서도 runner와 navigation은 분리합니다. runner는 Fleet Manager 명령과 실행 결과만 알고, navigation은 goal 이동만 알아야 합니다.

### 8단계: map과 planner 추가

goal controller는 장애물이 없거나 단순한 환경에서만 됩니다. 그 다음 단계가 map 기반 planner입니다.

추천 순서:

```text
1. OccupancyGrid 구조 이해
2. grid 좌표와 map 좌표 변환
3. Dijkstra 또는 A* 구현
4. path를 waypoint list로 변환
5. Pure Pursuit로 waypoint 추종
6. scan 기반 local obstacle avoidance 추가
```

이제부터가 진짜 “직접 자율주행” 영역입니다.

---

## 0.7 이 프로젝트에서 헷갈리면 안 되는 경계

### `cmd_vel`은 최종 구동 명령이다

`cmd_vel`은 “로봇이 얼마의 선속도/각속도로 움직일지”를 나타냅니다.

```text
linear.x  = 앞으로 가는 속도
angular.z = 제자리 회전 속도
```

Pinky Pro에서는 `pinky_bringup`이 이 값을 받아 양쪽 바퀴 RPM으로 바꿉니다. 따라서 직접 만든 주행 노드도 최종적으로는 `cmd_vel`을 잘 내보내면 실제 로봇이 움직입니다.

주의:

```text
제공 Nav2와 직접 만든 주행 노드가 동시에 cmd_vel을 publish하면 안 된다.
```

### `odom`은 로봇이 계산한 이동 추정이다

`odom`은 바퀴 encoder 기반 이동 추정입니다. 연속적이지만 시간이 지나면 누적 오차가 생길 수 있습니다.

### `map -> odom`은 Localization이 맞춰준다

AMCL 또는 SLAM이 “지도에서 로봇이 어디 있는지”를 추정해서 `map -> odom` 관계를 맞춥니다.

### `base_link`와 `base_footprint`를 섞어 쓰지 않는다

현재 `bringup.py`는 `odom -> base_footprint`를 publish합니다. Nav2 설정은 일부에서 `base_link`, 일부 costmap에서 `base_footprint`를 씁니다. TF tree가 실제로 어떻게 연결되는지 반드시 확인해야 합니다.

```bash
ros2 run tf2_tools view_frames
ros2 run tf2_ros tf2_echo map base_footprint
ros2 run tf2_ros tf2_echo odom base_footprint
```

TF가 안 맞으면 제공 Nav2도 실패하고, 직접 만든 자율주행도 pose를 틀리게 읽습니다.

---

## 0.8 프로젝트 구현 로드맵

### Milestone 1: 관측만 하는 AMR reporter

목표:

- `battery/percent` 구독
- TF 또는 `odom`에서 pose 읽기
- `PATCH /api/fleet/robots/{robot_id}` 주기적 호출
- 서버가 꺼져 있어도 죽지 않고 재시도

검증:

- Admin UI에서 AMR battery/pose가 갱신됨.
- 네트워크 실패 로그가 남고 다음 주기에 회복됨.

### Milestone 2: dry-run task runner

목표:

- `GET /api/fleet/tasks?robot_name=PICKY1&status=ASSIGNED` 조회
- 배정 task를 `RUNNING`으로 변경
- 실제 주행 없이 몇 초 후 `SUCCESS` 보고

검증:

- Fleet Manager가 생성한 task가 UI와 `GET /api/fleet/tasks`에서 진행 상태로 보임.
- 실패 시 어느 API에서 실패했는지 로그로 추적 가능.

### Milestone 3: standalone go-to-goal controller

목표:

- Control Server와 분리된 상태에서 직접 만든 ROS 2 노드 실행.
- `/odom` 또는 TF로 현재 pose 읽기.
- 파라미터 또는 service로 goal pose 입력.
- goal 방향 회전, 직진, 도착 정지.
- 최종 출력은 `/cmd_vel`.

검증:

- Gazebo에서 단순 목표 지점까지 이동.
- 목표 근처에서 `cmd_vel` 0으로 정지.
- 실로봇에서는 낮은 속도 제한으로 짧은 거리만 테스트.

### Milestone 4: scan 기반 안전 정지/회피

목표:

- `/scan` 구독.
- 전방 일정 거리 안에 장애물이 있으면 정지.
- 이후 단순 회피 정책 추가.

검증:

- 장애물 앞에서 정지.
- 장애물이 사라지면 다시 goal 방향 진행.
- 정지/회피 상태가 로그로 추적 가능.

### Milestone 5: PICKY runner와 custom navigation 연결

목표:

- Fleet Manager 명령의 `target_zone_pose`를 직접 만든 navigation goal로 변환.
- 이동 성공/실패 결과를 Fleet Manager에 보고.
- timeout, obstacle blocked, no pose 같은 실패 원인을 `result_message`로 남김.

검증:

- UI 주문 흐름에서 AMR task가 직접 만든 navigation으로 진행.
- 실패 시 서버와 로봇 로그만 보고 원인 추적 가능.

### Milestone 6: map 기반 global planner

목표:

- OccupancyGrid 또는 직접 만든 grid map 사용.
- world 좌표와 grid 좌표 변환.
- Dijkstra 또는 A*로 path 생성.
- path를 waypoint list로 변환.

검증:

- 장애물을 돌아가는 path 생성.
- RViz 또는 자체 로그로 path 확인.

### Milestone 7: path tracking controller

목표:

- waypoint list 추종.
- Pure Pursuit 또는 간단한 carrot following 구현.
- 속도 제한, 회전 제한, goal tolerance 적용.

검증:

- path를 따라가며 흔들림 없이 이동.
- goal 근처에서 안정적으로 정지.

### Milestone 8: Display(진열) / Return Home 연결

목표:

- `MOVE_TO_STOCK`, `MOVE_TO_DISPLAY`, `RETURN_HOME` task를 주행 runner에 연결.
- 진열 작업 중 COBOT 작업 대기 상태를 Fleet Manager에 보고.
- 예외 상황 발생 시 `/api/fleet/exceptions` 보고.

검증:

- 순찰 task가 주문 task보다 먼저 배정됨.
- 회전 완료 후 task가 `SUCCESS`.
- 예외가 Admin UI에 표시됨.

---

## 0.9 이 프로젝트에서 자세히 구현할 범위

이 문서는 자율주행 전체 분야를 모두 같은 깊이로 다루지 않습니다.
대신 `autonomous_sys_ws`에서 직접 만들 AMR stack에 필요한 부분은 구현 관점까지 내려가서 봅니다.

깊게 볼 범위:

| 범위 | 깊게 봐야 하는 내용 | 구현 결과물 |
|---|---|---|
| ROS 2 패키지/노드 기본 | `ament_python`, node 생성, publisher/subscriber, timer, parameter, launch | `just_pick_amr` 패키지와 실행 가능한 최소 노드 |
| Robot I/O 경계 | `/cmd_vel`, `/odom`, `/scan`, `/tf`, `/battery/percent`의 의미와 확인 명령 | Pinky/Gazebo에서 입출력 경계 확인 |
| Pose 처리 | odom pose 읽기, yaw 추출, goal까지 거리/각도 오차 계산 | `pose_utils.py` |
| Go-to-goal 제어 | 목표 방향 정렬, 전진, 도착 정지, 속도 제한, goal tolerance | `go_to_goal_controller.py` |
| Safety layer | LaserScan 전방 sector 추출, 유효 range 필터링, 정지 거리, blocked timeout | `safety_layer.py` |
| Task runner | Fleet Manager 명령 수신, RUNNING/SUCCESS/FAILED 결과 보고, retry/timeout 로그 | `picky_task_runner.py` |
| Navigation 인터페이스 | runner와 navigation 사이 action goal/result/feedback 분리 | `NavigateTask.action` |
| Global planner | OccupancyGrid 해석, world-grid 변환, A*/Dijkstra, path 생성 | `grid_planner.py` |
| Path tracking | waypoint list 추종, lookahead, Pure Pursuit 또는 carrot following | `path_tracker.py` |
| 검증/디버깅 | topic echo/hz/info, TF 확인, RViz path/debug marker, server API 로그 | 단계별 검증 기록 |

얕게만 볼 범위:

| 범위 | 지금은 어디까지 보면 되는가 |
|---|---|
| Dynamixel driver | 직접 재작성하지 않고 `pinky_bringup`이 `cmd_vel`을 wheel RPM으로 바꾸는 경계만 이해 |
| LiDAR driver | 직접 재작성하지 않고 `/scan`이 정상 발행되는지 확인 |
| Battery driver | 직접 재작성하지 않고 `battery/percent`를 Control Server 보고에 사용 |
| SLAM 전체 구현 | 처음에는 제공 SLAM/Nav2를 참고하고, 직접 planner를 만들 때 map 입력으로 활용 |
| Nav2 전체 대체 | 구조와 파라미터는 참고하되 메인 주행은 직접 만든 상용급 지향 custom stack으로 진행 |
| Autoware/CARLA | 모바일 AMR 구현 이후 확장 지식으로만 참고 |

직접 구현 범위의 핵심 흐름은 아래와 같습니다.

```text
odom/TF로 현재 pose 읽기
  ↓
goal pose와 현재 pose의 거리/각도 오차 계산
  ↓
go-to-goal controller가 기본 cmd_vel 계산
  ↓
scan safety layer가 위험하면 cmd_vel 제한 또는 정지
  ↓
cmd_vel publish
  ↓
odom 변화 확인
  ↓
도착/실패/timeout 판단
  ↓
Control Server task 상태 보고
```

이 흐름을 먼저 완성한 뒤에 map 기반 planner와 path tracker를 추가합니다.
즉 처음부터 모든 기능을 동시에 만들지는 않지만, 최종 목표는 planner, path tracking, safety, recovery, Control Server 연동, 실로봇 검증까지 포함한 상용급 지향 AMR stack입니다.

## 0.10 이후 Part를 읽는 방법

처음부터 끝까지 외우려고 하면 오래 걸립니다. 지금 프로젝트에 필요한 순서는 이렇습니다.

| 지금 할 일 | 먼저 읽을 파트 |
|---|---|
| ROS 2가 뭔지 모르겠음 | Part A |
| TF/odom/map이 헷갈림 | Part B, Part C |
| SLAM으로 지도 만들기 | Part D |
| 제공 Nav2 구조를 참고하고 싶음 | Part E |
| 직접 알고리즘 만들기 | Part F의 Planning/Control/Obstacle Avoidance |
| 안 움직일 때 디버깅 | Part I |
| runner 구조 설계 | Part J |

우리 프로젝트에서 가장 먼저 봐야 하는 질문:

```text
1. /cmd_vel을 누가 내고 있는가?
2. /odom은 정상인가?
3. map -> odom -> base_footprint TF가 이어지는가?
4. /scan이 정상 주기로 들어오는가?
5. 직접 만든 navigation 노드가 goal을 받고 있는가?
6. 직접 만든 navigation 노드가 /cmd_vel을 내고 있는가?
7. Control Server task 상태가 ASSIGNED/RUNNING/SUCCESS로 맞게 바뀌는가?
```

이 질문들이 잡히면 제공 Nav2 없이도 직접 만든 주행 흐름을 확장할 수 있습니다.

---

## 1. 전체 로드맵

### 1.1 자율주행 시스템의 큰 그림

자율주행은 보통 아래 파이프라인으로 봅니다.

```text
센서 입력
  ↓
전처리 / 시간 동기화 / 좌표 변환
  ↓
위치 추정 Localization
  ↓
지도 Mapping 또는 기존 지도 사용
  ↓
인지 Perception
  ↓
예측 Prediction
  ↓
경로 계획 Planning
  ↓
제어 Control
  ↓
차량 / 로봇 구동 Actuation
  ↓
모니터링 / 안전 / 로그 / 재시작
```

게임 엔진 비유로 보면 다음과 같습니다.

| 자율주행 요소 | 게임 엔진 비유 | 설명 |
|---|---|---|
| 센서 | 카메라, 레이캐스트, 콜리전 센서 | 세상을 관측합니다. |
| TF 좌표계 | Transform 계층 구조 | `map`, `odom`, `base_link`, `laser_link` 사이 위치 관계입니다. |
| SLAM | 맵 생성 + 자기 위치 갱신 | 던전을 탐험하면서 미니맵을 그리는 기능입니다. |
| Localization | 이미 있는 맵에서 내 위치 찾기 | 미니맵 위 플레이어 아이콘 위치를 맞추는 기능입니다. |
| Planner | 경로 탐색 AI | 네비게이션 메시 위에서 목표까지 길을 찾습니다. |
| Controller | 이동 컴포넌트 | 찾은 길을 실제 속도 명령으로 바꿉니다. |
| Costmap | 위험도 맵 | 벽, 장애물, 좁은 곳에 비용을 부여합니다. |
| Behavior Tree | AI 행동 트리 | “길 찾기 → 따라가기 → 실패하면 회복 행동”을 관리합니다. |

---

## 2. 학습 순서

초보자는 아래 순서를 지키는 것이 좋습니다.

```text
1. Linux / 터미널 / Git
2. ROS 2 기본 개념
3. TF2 좌표계
4. URDF / robot_state_publisher
5. 센서 토픽 / rosbag / RViz
6. 오도메트리와 센서 융합
7. SLAM
8. Nav2
9. 자율주행 알고리즘
10. 시뮬레이션
11. 실로봇 적용
12. 디버깅 / 안정화
```

핵심은 **SLAM이나 Nav2부터 바로 외우지 않는 것**입니다.  
Nav2가 실패하는 대부분의 원인은 알고리즘이 아니라 다음 중 하나입니다.

- TF가 틀렸습니다.
- 시간 동기화가 틀렸습니다.
- 센서 frame이 틀렸습니다.
- QoS가 맞지 않습니다.
- costmap 설정이 부적절합니다.
- 오도메트리가 튑니다.
- 로봇 footprint가 실제와 다릅니다.
- map과 odom 관계가 깨졌습니다.

---

# Part A. ROS 2 기본

---

## 3. ROS 2란 무엇인가?

ROS는 이름에 Operating System이 들어가지만, 엄밀히 말하면 Windows/Linux 같은 OS가 아닙니다.  
ROS는 **로봇 프로그램을 여러 노드로 나누고, 노드끼리 메시지를 주고받게 해주는 프레임워크**입니다.

### 3.1 왜 ROS를 쓰는가?

로봇은 보통 이런 부품이 동시에 돌아갑니다.

- LiDAR 드라이버
- 카메라 드라이버
- IMU 드라이버
- 모터 컨트롤러
- SLAM
- Localization
- Planner
- Controller
- RViz 시각화
- 로그 기록
- 안전 노드

이걸 하나의 거대한 프로그램으로 만들면 유지보수가 지옥이 됩니다.  
ROS는 각 기능을 **노드**로 분리합니다.

```text
/lidar_node  ── /scan ──▶ /slam_toolbox
/odom_node   ── /odom ──▶ /robot_localization
/nav2        ── /cmd_vel ──▶ /motor_driver
```

즉 ROS는 로봇의 **이벤트 버스 + 패키지 생태계 + 디버깅 도구**라고 보면 됩니다.

---

## 4. ROS 2 핵심 개념

### 4.1 Node

Node는 하나의 실행 단위입니다.

예시:

| 노드 | 역할 |
|---|---|
| `lidar_node` | 라이다 데이터를 읽어 `/scan` 발행 |
| `camera_node` | 카메라 이미지를 `/image_raw`로 발행 |
| `slam_toolbox` | `/scan`, `/odom`을 받아 맵 생성 |
| `controller_server` | path를 따라가기 위한 `/cmd_vel` 생성 |
| `robot_state_publisher` | URDF 기반 TF 발행 |

좋은 설계 원칙:

```text
한 노드는 한 가지 책임만 가진다.
```

나쁜 예:

```text
sensor_driver_and_slam_and_motor_controller_node
```

좋은 예:

```text
lidar_driver_node
odom_fusion_node
mapping_node
navigation_node
motor_bridge_node
```

---

### 4.2 Topic

Topic은 지속적으로 흐르는 데이터 통로입니다.

예시:

| Topic | 메시지 타입 | 의미 |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` | 2D LiDAR |
| `/pointcloud` | `sensor_msgs/PointCloud2` | 3D LiDAR |
| `/image_raw` | `sensor_msgs/Image` | 카메라 이미지 |
| `/odom` | `nav_msgs/Odometry` | 오도메트리 |
| `/cmd_vel` | `geometry_msgs/Twist` | 속도 명령 |
| `/tf` | `tf2_msgs/TFMessage` | 좌표 변환 |
| `/map` | `nav_msgs/OccupancyGrid` | 2D 점유 격자 지도 |

Topic은 “방송”입니다.

```text
Publisher: 내 데이터를 계속 뿌립니다.
Subscriber: 필요한 노드가 구독합니다.
```

게임 비유:

```text
Publisher = 이벤트 브로드캐스터
Subscriber = 이벤트 리스너
Topic = 이벤트 채널
```

---

### 4.3 Service

Service는 요청-응답입니다.

예시:

```text
"맵 저장해줘" → "저장 완료"
"현재 파라미터 알려줘" → "값 반환"
```

Topic과 차이:

| 구분 | Topic | Service |
|---|---|---|
| 흐름 | 계속 발행 | 한 번 요청, 한 번 응답 |
| 예시 | LiDAR, 카메라, cmd_vel | map 저장, 모드 변경 |
| 성격 | 스트리밍 | 함수 호출 |

---

### 4.4 Action

Action은 오래 걸리는 작업을 처리합니다.

예시:

```text
목표 지점까지 이동해라.
  ↓
진행률 feedback
  ↓
성공 / 실패 result
```

Nav2의 `NavigateToPose`가 대표적인 Action입니다.

Service로 하면 중간 취소나 진행률 확인이 어렵습니다.  
Action은 다음을 지원합니다.

- 목표 전송
- 진행 상태 feedback
- 취소 cancel
- 결과 result

---

### 4.5 Parameter

Parameter는 노드 설정값입니다.

예시:

```yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    min_x_velocity_threshold: 0.001
```

실무 기준:

- 코드 안에 튜닝값을 박지 않습니다.
- YAML 파라미터로 뺍니다.
- 환경별 파일을 분리합니다.

```text
config/
  nav2_sim.yaml
  nav2_real_robot.yaml
  ekf_sim.yaml
  ekf_real.yaml
```

---

### 4.6 Launch

Launch 파일은 여러 노드를 한 번에 실행합니다.

예시:

```text
로봇 실행에 필요한 것:
- robot_state_publisher
- lidar_driver
- ekf_node
- slam_toolbox
- nav2_bringup
- rviz2
```

이걸 터미널 6개로 실행하면 관리가 어렵습니다.  
Launch 파일은 실행 순서, 파라미터, namespace, remap을 관리합니다.

---

### 4.7 Package

Package는 ROS 프로젝트 단위입니다.

예시 구조:

```text
my_robot_ws/
  src/
    my_robot_description/
    my_robot_bringup/
    my_robot_navigation/
    my_robot_slam/
    my_robot_control/
```

권장 구조:

| 패키지 | 책임 |
|---|---|
| `my_robot_description` | URDF, mesh, xacro |
| `my_robot_bringup` | 전체 launch |
| `my_robot_navigation` | Nav2 config |
| `my_robot_slam` | SLAM config |
| `my_robot_control` | motor bridge, controller |
| `my_robot_perception` | sensor processing |

나쁜 구조:

```text
my_robot_everything
```

이런 패키지는 금방 유지보수가 무너집니다.

---

## 5. ROS 2 설치 기준

### 5.1 추천 배포판

2026년 5월 기준으로 입문자는 보통 아래를 권장합니다.

| Ubuntu | 추천 ROS 2 | 이유 |
|---|---|---|
| Ubuntu 24.04 | Jazzy Jalisco | LTS, 2029년까지 지원 |
| Ubuntu 22.04 | Humble Hawksbill | LTS, 2027년까지 지원 |
| 최신 실험 | Kilted Kaiju | 최신 일반 릴리스지만 지원 기간 짧음 |
| 개발 추적 | Rolling | 깨질 수 있음. 입문자 비추천 |

이 문서의 명령은 기본적으로 `Jazzy`를 기준으로 작성합니다.

```bash
# ROS 배포판 환경 변수 예시
export ROS_DISTRO=jazzy
```

---

## 6. ROS 2 필수 명령어

### 6.1 환경 설정

```bash
source /opt/ros/jazzy/setup.bash
```

매번 입력하기 싫으면:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

워크스페이스를 빌드한 뒤에는:

```bash
source ~/ros2_ws/install/setup.bash
```

---

### 6.2 워크스페이스 생성

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

### 6.3 패키지 생성

Python 패키지:

```bash
cd ~/ros2_ws/src
ros2 pkg create my_robot_py --build-type ament_python --dependencies rclpy std_msgs
```

C++ 패키지:

```bash
cd ~/ros2_ws/src
ros2 pkg create my_robot_cpp --build-type ament_cmake --dependencies rclcpp std_msgs
```

---

### 6.4 노드 확인

```bash
ros2 node list
ros2 node info /node_name
```

---

### 6.5 토픽 확인

```bash
ros2 topic list
ros2 topic echo /scan
ros2 topic info /cmd_vel
ros2 topic hz /scan
ros2 topic bw /image_raw
```

`topic hz`는 주파수 확인입니다.  
LiDAR가 10Hz로 나와야 하는데 2Hz라면 센서, QoS, CPU, 네트워크 문제를 의심해야 합니다.

---

### 6.6 메시지 타입 확인

```bash
ros2 interface show geometry_msgs/msg/Twist
ros2 interface show sensor_msgs/msg/LaserScan
ros2 interface show nav_msgs/msg/Odometry
```

---

### 6.7 서비스 확인

```bash
ros2 service list
ros2 service type /service_name
ros2 service call /service_name package/srv/Type "{}"
```

---

### 6.8 액션 확인

```bash
ros2 action list
ros2 action info /navigate_to_pose
```

---

### 6.9 파라미터 확인

```bash
ros2 param list
ros2 param get /controller_server controller_frequency
ros2 param set /controller_server controller_frequency 10.0
```

---

### 6.10 rosbag 기록과 재생

기록:

```bash
ros2 bag record /scan /odom /tf /tf_static
```

전체 기록:

```bash
ros2 bag record -a
```

재생:

```bash
ros2 bag play bag_name
```

자율주행에서 rosbag은 매우 중요합니다.

```text
실패 상황을 rosbag으로 저장
  ↓
실험실에서 반복 재생
  ↓
SLAM / Localization / Nav2 파라미터 수정
  ↓
동일 데이터로 다시 검증
```

실로봇 디버깅 시간을 줄이는 핵심 도구입니다.

---

## 7. QoS

ROS 2는 DDS 기반이므로 QoS가 중요합니다.

QoS는 쉽게 말해 **통신 품질 정책**입니다.

| QoS 항목 | 의미 |
|---|---|
| Reliability | 반드시 전달할지, 최신 데이터 우선인지 |
| Durability | 늦게 구독한 노드에게 과거 데이터를 줄지 |
| History | 메시지 큐를 얼마나 저장할지 |
| Depth | 큐 크기 |
| Deadline | 주기 제한 |
| Lifespan | 메시지 유효 시간 |

### 7.1 Best Effort vs Reliable

| 설정 | 의미 | 사용 예 |
|---|---|---|
| Best Effort | 유실 가능, 최신성 우선 | LiDAR, 카메라 |
| Reliable | 재전송, 전달 보장 | 서비스성 데이터, 중요 상태 |

센서 데이터는 보통 “예전 데이터를 완벽히 받는 것”보다 “최신 데이터를 빨리 받는 것”이 중요합니다.  
그래서 센서 토픽은 Best Effort인 경우가 많습니다.

문제 예시:

```text
LiDAR publisher: best_effort
Subscriber: reliable
```

이러면 연결이 안 되거나 데이터가 안 들어올 수 있습니다.  
Nav2, SLAM, RViz에서 `/scan`이 안 보이면 QoS를 반드시 확인해야 합니다.

```bash
ros2 topic info /scan --verbose
```

---

# Part B. 좌표계와 로봇 모델

---

## 8. TF2

자율주행 초보자가 가장 많이 막히는 부분이 TF입니다.

TF는 여러 좌표계의 관계를 시간에 따라 관리합니다.

```text
map
 └── odom
      └── base_link
           ├── laser_link
           ├── camera_link
           └── imu_link
```

게임 엔진의 Transform 계층과 거의 같습니다.

```text
World
 └── Player
      ├── Camera
      ├── WeaponSocket
      └── SensorSocket
```

로봇에서는 좌표계가 틀리면 센서 데이터가 엉뚱한 위치에 찍힙니다.

---

## 9. REP-105 핵심 프레임

ROS 모바일 로봇에서 기본 frame은 다음입니다.

| Frame | 의미 |
|---|---|
| `map` | 전역 지도 좌표계 |
| `odom` | 시작 위치 기준의 연속적인 로컬 좌표계 |
| `base_link` | 로봇 본체 중심 좌표계 |
| `base_footprint` | 바닥 투영 좌표계 |
| `laser_link` | LiDAR 좌표계 |
| `camera_link` | 카메라 좌표계 |
| `imu_link` | IMU 좌표계 |

### 9.1 map

`map`은 전역 기준입니다.  
SLAM 또는 AMCL이 로봇이 지도에서 어디 있는지 추정합니다.

### 9.2 odom

`odom`은 연속성이 중요합니다.  
바퀴 엔코더나 IMU 기반 오도메트리는 부드럽지만 시간이 지나면 드리프트합니다.

### 9.3 base_link

`base_link`는 로봇 몸체에 붙은 기준점입니다.  
보통 로봇 회전 중심에 둡니다.

---

## 10. 중요한 TF 관계

Nav2에서 필수적으로 필요한 관계는 다음입니다.

```text
map -> odom
odom -> base_link
base_link -> sensor_frames
```

누가 발행하는가?

| Transform | 보통 발행하는 노드 |
|---|---|
| `map -> odom` | SLAM Toolbox 또는 AMCL |
| `odom -> base_link` | wheel odometry, robot_localization |
| `base_link -> laser_link` | robot_state_publisher 또는 static_transform_publisher |
| `base_link -> camera_link` | robot_state_publisher |
| `base_link -> imu_link` | robot_state_publisher |

중요 규칙:

```text
같은 TF를 두 노드가 동시에 발행하면 안 됩니다.
```

예를 들어 `odom -> base_link`를 Gazebo와 EKF가 동시에 발행하면 TF 충돌이 납니다.

---

## 11. TF 디버깅 명령어

```bash
ros2 run tf2_tools view_frames
```

PDF로 TF 트리를 생성합니다.

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_link
```

RViz에서 Fixed Frame을 `map` 또는 `odom`으로 설정합니다.

증상별 의심:

| 증상 | 의심 |
|---|---|
| RViz에 센서가 안 보임 | Fixed Frame 오류, TF 누락 |
| scan이 로봇 뒤에 찍힘 | sensor frame 방향 오류 |
| 로봇 모델이 공중에 뜸 | URDF origin 오류 |
| Nav2가 goal을 못 받음 | `map -> base_link` 변환 실패 |
| costmap이 안 움직임 | `odom -> base_link` 누락 |
| 로봇 위치가 순간이동 | `map -> odom` 업데이트 문제 |

---

## 12. URDF와 robot_state_publisher

URDF는 로봇의 링크와 조인트를 표현하는 XML 파일입니다.

예시:

```xml
<robot name="simple_bot">
  <link name="base_link"/>
  <link name="laser_link"/>

  <joint name="base_to_laser" type="fixed">
    <parent link="base_link"/>
    <child link="laser_link"/>
    <origin xyz="0.2 0 0.15" rpy="0 0 0"/>
  </joint>
</robot>
```

`robot_state_publisher`는 URDF를 읽고 TF를 발행합니다.

```text
URDF
  ↓
robot_state_publisher
  ↓
/tf_static, /tf
```

### 12.1 Xacro

Xacro는 URDF를 더 깔끔하게 작성하기 위한 매크로 시스템입니다.

나쁜 방식:

```xml
<origin xyz="0.123 0.0 0.456"/>
```

좋은 방식:

```xml
<xacro:property name="lidar_x" value="0.123"/>
<xacro:property name="lidar_z" value="0.456"/>
```

반복되는 바퀴, 센서, 링크를 함수처럼 재사용할 수 있습니다.

---

# Part C. 이동 로봇 기본

---

## 13. 로봇 형태

자율주행 알고리즘은 로봇 형태에 따라 달라집니다.

| 형태 | 예시 | 특징 |
|---|---|---|
| Differential Drive | TurtleBot, 청소로봇 | 좌우 바퀴 속도 차로 회전 |
| Ackermann Steering | 자동차 | 앞바퀴 조향, 회전 반경 제한 |
| Omnidirectional | 메카넘, 옴니휠 | x/y/회전 자유도 높음 |
| Legged Robot | 4족, 휴머노이드 | 보행 제어 필요 |

Nav2는 주로 모바일 로봇에 많이 쓰입니다.  
자동차형 자율주행은 Autoware, Apollo 같은 스택을 같이 봐야 합니다.

---

## 14. Differential Drive 기초

좌우 바퀴 속도:

```text
v_left
v_right
```

로봇의 선속도와 각속도:

```text
v = (v_right + v_left) / 2
w = (v_right - v_left) / wheel_base
```

ROS에서 속도 명령은 보통 `/cmd_vel`입니다.

```text
geometry_msgs/Twist
  linear.x   전진 속도
  angular.z  회전 속도
```

예시:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.2}, angular: {z: 0.0}}"
```

회전:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0}, angular: {z: 0.5}}"
```

실로봇에서는 반드시 낮은 속도부터 테스트해야 합니다.

---

## 15. Odometry

Odometry는 로봇이 “내가 얼마나 움직였는지” 추정한 값입니다.

출처:

- wheel encoder
- IMU
- visual odometry
- LiDAR odometry
- GPS
- sensor fusion

오도메트리 메시지:

```text
nav_msgs/Odometry
  header.frame_id = odom
  child_frame_id = base_link
  pose
  twist
  covariance
```

중요한 점:

```text
Odometry는 부드러워야 합니다.
하지만 장기적으로 정확하다고 보장되지는 않습니다.
```

바퀴가 미끄러지면 odom은 틀어집니다.  
그래서 `map -> odom`을 SLAM/AMCL이 보정합니다.

---

## 16. IMU

IMU는 보통 다음을 제공합니다.

- angular velocity
- linear acceleration
- orientation

주의:

- 저가 IMU의 yaw는 쉽게 드리프트합니다.
- IMU frame 방향이 틀리면 EKF가 망가집니다.
- covariance가 말이 안 되면 필터가 센서를 과신하거나 무시합니다.

---

## 17. robot_localization

`robot_localization`은 여러 센서의 위치 추정 정보를 EKF/UKF로 융합합니다.

대표 입력:

- `nav_msgs/Odometry`
- `sensor_msgs/Imu`
- `geometry_msgs/PoseWithCovarianceStamped`
- `geometry_msgs/TwistWithCovarianceStamped`

대표 출력:

- `/odometry/filtered`
- `odom -> base_link` TF

예시 구조:

```text
wheel odom ─┐
            ├── ekf_node ──▶ /odometry/filtered
IMU ────────┘                 └── odom -> base_link
```

### 17.1 EKF 개념

EKF는 대략 이렇게 동작합니다.

```text
1. 이전 상태에서 현재 상태를 예측한다.
2. 센서 측정값을 받는다.
3. 예측과 측정을 적당히 섞는다.
4. 더 그럴듯한 현재 상태를 만든다.
```

비유:

```text
바퀴 엔코더: "앞으로 1m 갔어"
IMU: "조금 회전했어"
EKF: "둘 다 고려하면 이 위치가 제일 그럴듯해"
```

### 17.2 단순 EKF YAML 예시

```yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true

    map_frame: map
    odom_frame: odom
    base_link_frame: base_link
    world_frame: odom

    odom0: /wheel/odom
    # x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az
    odom0_config: [true,  true,  false,
                   false, false, true,
                   true,  false, false,
                   false, false, true,
                   false, false, false]

    imu0: /imu/data
    # IMU에서 yaw rate를 쓸지 정해라
    imu0_config: [false, false, false,
                  false, false, true,
                  false, false, false,
                  false, false, true,
                  false, false, false]
```

실무 주의:

- `world_frame: odom`이면 EKF가 `odom -> base_link`를 담당합니다.
- SLAM/AMCL은 `map -> odom`을 담당합니다.
- 두 노드가 같은 TF를 발행하지 않게 해야 합니다.

---

# Part D. SLAM

---

## 18. SLAM이란?

SLAM은 **Simultaneous Localization and Mapping**입니다.

한국어로는:

```text
동시에 위치 추정과 지도 작성을 하는 기술
```

로봇은 처음 환경을 모릅니다.  
그래서 움직이면서 다음을 동시에 합니다.

```text
1. 내가 어디 있는지 추정한다.
2. 주변 장애물을 지도에 추가한다.
3. 다시 내 위치를 더 정확히 고친다.
4. 루프를 닫으면 전체 지도를 보정한다.
```

게임 비유:

```text
플레이어가 던전을 탐험하면서 미니맵을 자동으로 그린다.
그런데 플레이어 위치도 확실하지 않아서,
벽과 이동 기록을 보고 플레이어 위치까지 계속 보정한다.
```

---

## 19. SLAM의 핵심 구성

### 19.1 Sensor

SLAM에 쓰는 센서:

| 센서 | 용도 |
|---|---|
| 2D LiDAR | 실내 2D SLAM |
| 3D LiDAR | 3D mapping, 자율주행 |
| Camera | Visual SLAM |
| IMU | 회전/가속도 보조 |
| Wheel encoder | 이동량 추정 |
| GPS/GNSS | 실외 전역 위치 |

---

### 19.2 Scan Matching

LiDAR scan을 이전 scan 또는 map과 맞춥니다.

```text
현재 scan을 조금씩 이동/회전해본다.
기존 map과 가장 잘 겹치는 위치를 찾는다.
```

대표 알고리즘:

- ICP
- NDT
- Correlative Scan Matching
- GICP

---

### 19.3 Occupancy Grid

2D 지도는 보통 격자입니다.

```text
0   = free
100 = occupied
-1  = unknown
```

예시:

```text
? ? ? ? ?
? . . # ?
? . R # ?
? . . . ?
? ? ? ? ?
```

- `?`: 모름
- `.`: 비어 있음
- `#`: 장애물
- `R`: 로봇

---

### 19.4 Pose Graph

SLAM은 로봇의 이동 경로를 그래프로 봅니다.

```text
pose1 -- pose2 -- pose3 -- pose4
  \                    /
   ---- loop closure ---
```

- 노드: 로봇 pose
- 엣지: odometry, scan matching, loop closure constraint

---

### 19.5 Loop Closure

로봇이 예전에 왔던 장소를 다시 인식하는 것입니다.

```text
"어? 여기 아까 지나간 복도네."
```

그러면 누적된 오차를 전체적으로 보정합니다.

loop closure가 좋으면 지도 품질이 좋아집니다.  
하지만 잘못된 loop closure는 지도를 망가뜨립니다.

---

## 20. SLAM 종류

| 종류 | 대표 입력 | 용도 |
|---|---|---|
| 2D LiDAR SLAM | LaserScan, odom | 실내 모바일 로봇 |
| 3D LiDAR SLAM | PointCloud2, IMU | 실외, 자율주행 |
| Visual SLAM | Camera, IMU | 드론, AR, 카메라 기반 |
| LiDAR-Inertial SLAM | LiDAR + IMU | 고정밀 3D 위치 추정 |
| Visual-Inertial SLAM | Camera + IMU | 카메라 기반 고속 추정 |

초보자는 2D LiDAR SLAM부터 시작하는 것이 맞습니다.  
이유는 RViz에서 결과가 직관적이고, Nav2와 연결하기 쉽기 때문입니다.

---

## 21. SLAM Toolbox

ROS 2에서 2D LiDAR 기반 SLAM 실습에 많이 쓰는 패키지가 `slam_toolbox`입니다.

SLAM Toolbox는 보통 다음을 사용합니다.

입력:

- `/scan`
- `/odom`
- `/tf`

출력:

- `/map`
- `map -> odom`
- pose graph

### 21.1 Mapping 모드

새로운 맵을 만드는 모드입니다.

```bash
ros2 launch slam_toolbox online_async_launch.py
```

또는 특정 파라미터:

```bash
ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:=/path/to/slam_toolbox.yaml
```

### 21.2 지도 저장

Nav2 map server 도구를 사용할 수 있습니다.

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

결과:

```text
my_map.yaml
my_map.pgm
```

YAML 예시:

```yaml
image: my_map.pgm
mode: trinary
resolution: 0.05
origin: [-2.0, -3.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
```

---

## 22. SLAM 실습 순서

### 22.1 시뮬레이션 기준

1. 로봇 시뮬레이션 실행
2. `/scan` 확인
3. `/odom` 확인
4. TF 확인
5. SLAM 실행
6. RViz에서 `/map` 확인
7. 로봇을 천천히 조작
8. loop closure 확인
9. map 저장

### 22.2 필수 확인 명령

```bash
ros2 topic list
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser_link
```

---

## 23. SLAM 품질 체크리스트

| 항목 | 정상 기준 |
|---|---|
| `/scan` | 일정한 Hz로 들어옴 |
| `/odom` | 부드럽게 변화 |
| `base_link -> laser_link` | 정확한 위치 |
| `odom -> base_link` | 끊기지 않음 |
| `map -> odom` | SLAM이 발행 |
| RViz Fixed Frame | `map` |
| 지도 벽 | 실제와 비슷 |
| loop closure | 지도가 접히지 않음 |
| 로봇 속도 | 너무 빠르지 않음 |

### 23.1 맵이 찢어지는 원인

| 증상 | 원인 |
|---|---|
| 벽이 두 겹으로 생김 | odom 부정확, scan matching 실패 |
| 코너가 휘어짐 | 회전 오도메트리 문제 |
| 지도 전체가 밀림 | TF 시간 문제 |
| 갑자기 맵이 접힘 | 잘못된 loop closure |
| 좁은 복도에서 흔들림 | 특징 부족, LiDAR 노이즈 |

---

## 24. Mapping과 Localization의 차이

| 구분 | Mapping | Localization |
|---|---|---|
| 목적 | 새 지도 작성 | 기존 지도에서 위치 찾기 |
| 출력 | map 생성 | pose 추정 |
| 대표 패키지 | SLAM Toolbox | AMCL, SLAM Toolbox localization |
| 사용 시점 | 처음 환경 탐색 | 실제 운용 |

실무 흐름:

```text
1. Mapping 모드로 지도 생성
2. 지도 저장
3. Localization 모드로 전환
4. Nav2로 자율주행
```

### 24.1 AMCL

AMCL은 **Adaptive Monte Carlo Localization**입니다.
이미 만들어진 지도 위에서 로봇이 현재 어디 있는지 추정하는 ROS의 대표적인 localization 방식입니다.

핵심 아이디어:

```text
지도 위에 "로봇이 있을 법한 위치 후보"를 입자로 많이 뿌린다.
  ↓
로봇의 LiDAR scan이 지도와 얼마나 잘 맞는지 각 입자를 평가한다.
  ↓
잘 맞는 입자는 살아남고, 안 맞는 입자는 사라진다.
  ↓
살아남은 입자 주변에 다시 후보를 뿌린다.
  ↓
입자들이 모인 위치를 현재 로봇 위치로 본다.
```

AMCL이 쓰는 입력:

| 입력 | 의미 |
|---|---|
| `/map` | 이미 저장된 occupancy grid 지도 |
| `/scan` | 현재 LiDAR 관측 |
| `odom -> base_link` 또는 `odom -> base_footprint` | 바퀴 odom 기반 이동 추정 |
| `base_link -> laser_link` | LiDAR가 로봇 어디에 붙어 있는지 |
| initial pose | 처음 로봇이 지도 위 어디쯤 있는지 |

AMCL이 내보내는 출력:

| 출력 | 의미 |
|---|---|
| `map -> odom` | 지도 좌표계와 odom 좌표계의 관계 |
| `/amcl_pose` | 지도 기준 로봇 pose |
| particle cloud | 로봇 위치 후보 입자들 |

왜 `map -> odom`을 내보내는가:

```text
odom -> base_link
```

는 바퀴 odom이 계속 계산합니다. 하지만 odom은 시간이 지나면 조금씩 틀어집니다.
AMCL은 저장된 map과 현재 scan을 비교해서 이 오차를 보정하고,

```text
map -> odom
```

을 발행해서 전체 TF tree를 맞춥니다.

즉 전체 pose는 보통 이렇게 이어집니다.

```text
map -> odom -> base_link -> laser_link
```

AMCL이 필요한 시점:

| 상황 | 필요한가 |
|---|---|
| 처음 지도를 만드는 중 | 보통 SLAM 사용 |
| 이미 지도가 있고 그 안에서 주행 | AMCL 사용 |
| odom만으로 짧게 움직이는 테스트 | 없어도 가능 |
| Nav2로 저장된 map 기반 주행 | 거의 필수 |

초보자가 헷갈리기 쉬운 점:

- AMCL은 지도를 만들지 않습니다. 이미 있는 지도에서 위치만 찾습니다.
- AMCL은 바퀴 odom을 대체하지 않습니다. odom을 보정하는 역할입니다.
- initial pose가 너무 틀리면 AMCL이 엉뚱한 위치에 수렴할 수 있습니다.
- LiDAR 위치 TF가 틀리면 scan과 map이 안 맞아서 localization이 흔들립니다.
- 유리, 반사체, 특징 없는 긴 복도에서는 입자 분포가 흔들릴 수 있습니다.

AMCL이 틀어졌을 때 확인 순서:

```text
1. /map이 실제 환경과 맞는가?
2. /scan이 RViz에서 map 벽과 겹치는가?
3. odom -> base_link가 끊기지 않는가?
4. base_link -> laser_link가 실제 장착 위치와 맞는가?
5. initial pose를 제대로 줬는가?
6. 로봇을 너무 빠르게 움직이지 않았는가?
```

우리 프로젝트에서의 의미:

- 직접 만든 custom navigation을 처음 구현할 때는 odom 기반으로 짧은 이동부터 검증한다.
- map 기반 planner와 실제 구역 이동으로 넘어가면 `map`, `odom`, `base_footprint` 기준이 반드시 정리되어야 한다.
- 제공 `pinky_navigation`의 AMCL/Nav2 설정은 "정답으로 가져다 쓰기"보다 localization 구조를 이해하고 비교하기 위한 기준선으로 본다.

---

# Part E. Nav2

---

## 25. Nav2란?

Nav2는 ROS 2의 대표적인 navigation framework입니다.

목표:

```text
로봇이 현재 위치에서 목표 위치까지 안전하게 이동하도록 한다.
```

Nav2는 단일 알고리즘이 아닙니다.  
여러 서버와 플러그인을 조합한 시스템입니다.

입력:

- TF
- map
- odometry
- sensor data
- goal pose
- robot footprint
- parameters
- behavior tree

출력:

- `/cmd_vel`

---

## 26. Nav2 전체 구조

```text
RViz / Client
   │
   ▼
BT Navigator
   │
   ├── Planner Server ── global path
   │
   ├── Smoother Server ── smoother path
   │
   ├── Controller Server ── cmd_vel
   │
   ├── Behavior Server ── recovery
   │
   ├── Map Server
   │
   ├── AMCL or SLAM Toolbox
   │
   └── Lifecycle Manager
```

### 26.1 BT Navigator

Behavior Tree로 전체 navigation logic을 관리합니다.

예시 흐름:

```text
목표 수신
  ↓
경로 계산
  ↓
경로 추종
  ↓
장애물 때문에 실패?
  ↓
회복 행동
  ↓
다시 경로 계산
```

---

### 26.2 Planner Server

목표까지 global path를 계산합니다.

입력:

- 현재 pose
- goal pose
- global costmap

출력:

- `nav_msgs/Path`

대표 planner:

| Planner | 특징 |
|---|---|
| NavFn | 고전적인 grid 기반 planner |
| Smac 2D | 2D grid 기반 |
| Smac Hybrid-A* | 차량형 / 비홀로노믹 제약 고려 |
| Smac Lattice | motion primitive 기반 |
| Theta* | 직선성이 좋은 any-angle planning |

---

### 26.3 Controller Server

global path를 따라가기 위해 속도 명령을 계산합니다.

입력:

- path
- local costmap
- odometry
- robot pose

출력:

- `/cmd_vel`

대표 controller:

| Controller | 특징 |
|---|---|
| DWB | 여러 속도 후보를 평가하는 방식 |
| Regulated Pure Pursuit | Pure Pursuit를 안전하게 확장 |
| MPPI | 샘플링 기반 모델 예측 제어 |
| Rotation Shim | 방향 정렬 보조 |

---

### 26.4 Smoother Server

Planner가 만든 path를 더 부드럽게 만듭니다.

필요한 이유:

```text
Grid path는 계단처럼 꺾일 수 있습니다.
Controller가 따라가기 쉽게 path를 정리합니다.
```

---

### 26.5 Behavior Server

실패 상황에서 회복 행동을 수행합니다.

예시:

- rotate
- backup
- wait
- assisted teleop
- drive on heading
- custom recovery

---

### 26.6 Lifecycle Manager

Nav2 노드는 Lifecycle Node로 관리됩니다.

상태:

```text
unconfigured
  ↓
inactive
  ↓
active
  ↓
finalized
```

Lifecycle을 쓰는 이유:

- 초기화 순서 관리
- bringup 안정화
- 노드 죽음 감지
- 시스템 전체 shutdown 관리

---

## 27. Nav2에 필요한 필수 입력

| 입력 | 설명 |
|---|---|
| `map -> odom` | AMCL 또는 SLAM |
| `odom -> base_link` | odometry / EKF |
| `base_link -> sensor` | URDF / static TF |
| `/scan` 또는 point cloud | 장애물 감지 |
| `/map` | global costmap용 |
| `/odom` | controller용 |
| robot footprint | 충돌 검사 |
| Nav2 params | 전체 설정 |
| BT XML | navigation behavior |

---

## 28. Costmap

Costmap은 “갈 수 있는 곳과 위험한 곳”을 숫자로 표현한 지도입니다.

```text
낮은 비용 = 지나가기 좋음
높은 비용 = 위험
치명 비용 = 충돌
```

### 28.1 Global Costmap

Global planner가 사용합니다.

특징:

- 전체 map 기준
- 긴 경로 계산
- static layer 중요
- 보통 update frequency 낮음

### 28.2 Local Costmap

Controller가 사용합니다.

특징:

- 로봇 주변만 봄
- 실시간 장애물 반영
- update frequency 높음
- obstacle / voxel / inflation 중요

---

## 29. Costmap Layer

### 29.1 Static Layer

기존 지도(`/map`)를 costmap에 반영합니다.

```text
벽, 고정 장애물
```

### 29.2 Obstacle Layer

LiDAR 같은 실시간 센서로 장애물을 반영합니다.

```text
사람, 박스, 의자
```

### 29.3 Voxel Layer

3D 센서 데이터를 사용해 3D 장애물을 2D costmap으로 투영합니다.

```text
Depth camera, 3D LiDAR
```

### 29.4 Inflation Layer

장애물 주변에 안전 마진을 둡니다.

```text
벽 바로 옆은 지나갈 수는 있어도 위험합니다.
그래서 cost를 부풀립니다.
```

게임 비유:

```text
NavMesh에서 벽과 너무 가까운 영역에 penalty를 주는 것과 비슷합니다.
```

---

## 30. Footprint

로봇의 실제 충돌 영역입니다.

원형 로봇:

```yaml
robot_radius: 0.22
```

사각형 로봇:

```yaml
footprint: "[[0.3, 0.2], [0.3, -0.2], [-0.3, -0.2], [-0.3, 0.2]]"
```

실무 기준:

```text
실제 로봇보다 살짝 크게 잡는 것이 안전합니다.
```

하지만 너무 크게 잡으면 좁은 곳을 못 지나갑니다.

---

## 31. Nav2 기본 실행 흐름

### 31.1 SLAM과 함께 주행

```text
로봇 실행
  ↓
SLAM Toolbox 실행
  ↓
Nav2 실행(use_sim_time, slam mode)
  ↓
RViz에서 goal 지정
  ↓
주행하면서 map 생성
```

공식 Nav2 튜토리얼에서도 SLAM Toolbox와 Nav2를 함께 사용하는 흐름을 다룹니다.

---

### 31.2 저장된 map으로 주행

```text
map_server
  ↓
AMCL
  ↓
Nav2
  ↓
goal navigation
```

이 모드는 실제 운용에서 더 일반적입니다.

```text
1. 지도를 미리 잘 만든다.
2. 운용 시에는 AMCL로 위치만 잡는다.
3. Nav2로 주행한다.
```

---

## 32. Nav2 설치 예시

```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-navigation2
sudo apt install ros-$ROS_DISTRO-nav2-bringup
sudo apt install ros-$ROS_DISTRO-slam-toolbox
sudo apt install ros-$ROS_DISTRO-robot-localization
```

TurtleBot 계열 예시:

```bash
sudo apt install ros-$ROS_DISTRO-nav2-minimal-tb*
```

---

## 33. Nav2 예제 실행 흐름

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```

RViz에서:

1. `2D Pose Estimate`로 초기 위치 지정
2. `Nav2 Goal`로 목표 지정
3. global path 확인
4. local costmap 확인
5. `/cmd_vel` 확인

---

## 34. Nav2 파라미터 구조 예시

아래는 학습용 축약 예시입니다. 실제 프로젝트에서는 로봇에 맞게 세부 튜닝이 필요합니다.

```yaml
amcl:
  ros__parameters:
    use_sim_time: true
    base_frame_id: base_link
    odom_frame_id: odom
    global_frame_id: map
    scan_topic: scan

map_server:
  ros__parameters:
    use_sim_time: true
    yaml_filename: "map.yaml"

planner_server:
  ros__parameters:
    use_sim_time: true
    expected_planner_frequency: 5.0
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
      tolerance: 0.5
      use_astar: false
      allow_unknown: true

controller_server:
  ros__parameters:
    use_sim_time: true
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    progress_checker_plugins: ["progress_checker"]
    goal_checker_plugins: ["goal_checker"]

    progress_checker:
      plugin: "nav2_controller::SimpleProgressChecker"
      required_movement_radius: 0.5
      movement_time_allowance: 10.0

    goal_checker:
      plugin: "nav2_controller::SimpleGoalChecker"
      xy_goal_tolerance: 0.25
      yaw_goal_tolerance: 0.25
      stateful: true

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.26
      max_vel_theta: 1.0
      acc_lim_x: 2.5
      acc_lim_theta: 3.2
      # 초반에는 너무 빠르게 잡지 마라
      vx_samples: 20
      vtheta_samples: 20
      sim_time: 1.7

global_costmap:
  global_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: map
      robot_base_frame: base_link
      update_frequency: 1.0
      publish_frequency: 1.0
      resolution: 0.05
      robot_radius: 0.22
      plugins: ["static_layer", "obstacle_layer", "inflation_layer"]

      static_layer:
        plugin: "nav2_costmap_2d::StaticLayer"
        map_subscribe_transient_local: true

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan
        scan:
          topic: /scan
          data_type: "LaserScan"
          marking: true
          clearing: true

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.55
        cost_scaling_factor: 3.0

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: true
      global_frame: odom
      robot_base_frame: base_link
      rolling_window: true
      width: 3
      height: 3
      resolution: 0.05
      update_frequency: 5.0
      publish_frequency: 2.0
      robot_radius: 0.22
      plugins: ["obstacle_layer", "inflation_layer"]

      obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: scan
        scan:
          topic: /scan
          data_type: "LaserScan"
          marking: true
          clearing: true

      inflation_layer:
        plugin: "nav2_costmap_2d::InflationLayer"
        inflation_radius: 0.45
        cost_scaling_factor: 3.0
```

---

## 35. Nav2 튜닝 순서

무작정 YAML을 바꾸면 안 됩니다.  
아래 순서로 튜닝해야 합니다.

```text
1. TF 정상 확인
2. odom 정상 확인
3. sensor 정상 확인
4. map 정상 확인
5. localization 정상 확인
6. costmap 정상 확인
7. planner 정상 확인
8. controller 정상 확인
9. recovery 정상 확인
10. 속도 / 가속도 / tolerance 조정
```

### 35.1 Controller가 흔들릴 때

의심:

- 속도가 너무 높음
- angular velocity 제한이 부적절
- acceleration limit이 실제와 다름
- local costmap이 너무 작음
- path가 너무 각짐
- goal tolerance가 너무 빡빡함
- odom이 튐

### 35.2 Planner가 경로를 못 만들 때

의심:

- global costmap이 막힘
- footprint가 너무 큼
- inflation radius가 너무 큼
- map이 잘못됨
- unknown space 정책 문제
- goal이 장애물 위에 있음

### 35.3 로봇이 장애물에 너무 가까이 갈 때

조정:

- `inflation_radius` 증가
- `cost_scaling_factor` 조정
- footprint 확대
- obstacle layer clearing/marking 확인
- 센서 range 확인

---

## 36. Behavior Tree

Nav2는 Behavior Tree로 navigation 흐름을 제어합니다.

기본 개념:

| 노드 | 의미 |
|---|---|
| Sequence | 순서대로 실행, 하나 실패하면 실패 |
| Fallback | 하나 성공하면 성공 |
| Decorator | 자식 노드에 조건/반복 부여 |
| Action | 실제 작업 수행 |
| Condition | 조건 검사 |

예시:

```text
Fallback
 ├── NavigateNormally
 └── RecoverySequence
      ├── ClearCostmap
      ├── Spin
      └── RetryNavigate
```

FSM보다 BT가 유리한 이유:

- 복잡한 행동을 트리로 읽기 쉽습니다.
- recovery를 구조화하기 좋습니다.
- action server와 결합하기 좋습니다.
- 조건과 반복을 명확히 표현합니다.

---

# Part F. 실제 자율주행 알고리즘

---

## 37. 자율주행 알고리즘 전체 분류

```text
Localization
Mapping
Perception
Tracking
Prediction
Planning
Control
Decision / Behavior
Safety
```

실내 모바일 로봇과 자동차형 자율주행은 겹치는 부분이 있지만, 차이도 큽니다.

| 항목 | 실내 모바일 로봇 | 자동차형 자율주행 |
|---|---|---|
| 지도 | 2D occupancy grid | HD map, lane map |
| 위치추정 | AMCL, SLAM | GNSS/IMU, LiDAR NDT, HD map matching |
| 인지 | 장애물 중심 | 차량, 보행자, 차선, 신호등 |
| 예측 | 단순 회피 가능 | 주변 객체 미래 궤적 예측 필수 |
| 계획 | goal까지 path | route, behavior, motion planning 분리 |
| 제어 | `cmd_vel` | steering, throttle, brake |
| 운동 모델 | differential/omni | Ackermann vehicle model |

---

## 38. Localization 알고리즘

### 38.1 Dead Reckoning

바퀴 엔코더와 IMU로 이동량을 누적합니다.

장점:

- 빠름
- 구현 쉬움
- 짧은 시간에는 안정적

단점:

- 드리프트 누적
- 바퀴 미끄러짐에 약함

---

### 38.2 Kalman Filter

센서 융합의 기본입니다.

종류:

| 알고리즘 | 특징 |
|---|---|
| KF | 선형 시스템 |
| EKF | 비선형을 선형화 |
| UKF | sigma point 사용, EKF보다 안정적인 경우 있음 |

사용 예:

```text
wheel odom + IMU → smoother odometry
GNSS + IMU + wheel odom → outdoor localization
```

---

### 38.3 Particle Filter

여러 후보 위치 입자를 뿌리고, 센서 관측과 맞는 입자를 살립니다.

AMCL이 대표적입니다.

```text
입자 1000개를 맵 위에 뿌린다.
LiDAR scan과 map이 잘 맞는 입자의 가중치를 높인다.
안 맞는 입자는 줄인다.
최종적으로 가장 그럴듯한 위치를 얻는다.
```

장점:

- global localization 가능
- 비선형/다봉 분포에 강함

단점:

- 계산량
- 초기 위치가 너무 틀리면 수렴이 늦음

---

### 38.4 ICP

ICP는 두 point cloud를 정렬합니다.

```text
현재 LiDAR 포인트
기존 map 포인트
  ↓
가장 가까운 점끼리 매칭
  ↓
오차가 최소가 되도록 회전/이동 계산
```

단점:

- 초기 guess가 중요
- 반복 계산 필요
- 특징 없는 환경에 약함

---

### 38.5 NDT

NDT는 point cloud를 정규분포 격자로 표현하고 정렬합니다.  
자동차형 자율주행에서 LiDAR map localization에 많이 쓰입니다.

장점:

- 대규모 point cloud map과 잘 맞음
- ICP보다 안정적인 경우가 많음

단점:

- 좋은 초기 pose 필요
- map 품질 중요
- 계산량 있음

---

### 38.6 GNSS/IMU Localization

실외에서는 GNSS가 중요합니다.

하지만 GNSS만으로는 부족합니다.

문제:

- 터널
- 빌딩 숲
- multipath
- 신호 끊김
- 고주파 제어에 부족한 update rate

그래서 보통:

```text
GNSS + IMU + wheel odom + LiDAR localization
```

형태로 융합합니다.

---

## 39. Mapping 알고리즘

### 39.1 Occupancy Grid Mapping

2D 셀마다 점유 확률을 저장합니다.

```text
P(occupied)
```

LiDAR ray를 쏘면:

```text
ray가 지나간 곳 = free
ray 끝점 = occupied
```

---

### 39.2 TSDF / ESDF

3D 로봇이나 드론에서 자주 쓰입니다.

| 지도 | 의미 |
|---|---|
| TSDF | 표면까지의 signed distance를 누적 |
| ESDF | 장애물까지의 거리장 |

ESDF는 motion planning에서 유용합니다.

```text
장애물까지 얼마나 떨어져 있는지 바로 알 수 있음
```

---

### 39.3 HD Map

자동차형 자율주행은 HD map을 많이 씁니다.

HD map 정보:

- 차선
- 정지선
- 신호등 위치
- 횡단보도
- 제한속도
- 교차로 구조
- 도로 경계
- 주행 가능 영역

HD map은 단순한 그림이 아니라 **도로 의미 정보가 포함된 데이터 구조**입니다.

---

## 40. Perception 알고리즘

Perception은 주변 환경을 이해하는 단계입니다.

### 40.1 Detection

객체를 찾습니다.

예시:

- 차량
- 보행자
- 자전거
- 콘
- 신호등
- 표지판

입력:

- camera image
- LiDAR point cloud
- radar
- sensor fusion

대표 모델:

- YOLO 계열
- Faster R-CNN 계열
- CenterPoint
- PointPillars
- BEVFusion 계열

초보자는 모델 이름보다 파이프라인을 먼저 이해해야 합니다.

```text
sensor
  ↓
preprocess
  ↓
model inference
  ↓
postprocess
  ↓
tracked objects
```

---

### 40.2 Segmentation

픽셀 또는 포인트 단위로 분류합니다.

예시:

- 도로
- 인도
- 차선
- 차량
- 보행자
- 장애물

카메라 segmentation은 drivable area 판단에 유용합니다.

---

### 40.3 Lane Detection

차선을 인식합니다.

방식:

- 전통적 영상 처리
- 딥러닝 기반 차선 검출
- HD map 기반 lane association
- BEV 기반 lane detection

차량형 자율주행에서는 차선이 행동 계획의 핵심입니다.

---

### 40.4 Sensor Fusion

센서 융합 방식:

| 방식 | 설명 |
|---|---|
| Early Fusion | raw data 단계에서 결합 |
| Middle Fusion | feature 단계에서 결합 |
| Late Fusion | detection 결과 단계에서 결합 |

예시:

```text
Camera: 물체 종류를 잘 봄
LiDAR: 거리와 3D 위치를 잘 봄
Radar: 속도와 악천후에 강함
```

좋은 시스템은 각 센서의 장점을 조합합니다.

---

## 41. Tracking 알고리즘

Detection은 한 프레임의 결과입니다.  
Tracking은 시간에 따라 같은 객체를 이어 붙입니다.

```text
frame 1: car id 7
frame 2: car id 7
frame 3: car id 7
```

대표 방법:

- Kalman Filter
- Hungarian Matching
- JPDA
- MHT
- SORT
- DeepSORT
- AB3DMOT

Tracking 출력:

```text
object id
position
velocity
acceleration
heading
classification
covariance
```

Planning은 단순 detection보다 tracking 결과를 더 선호합니다.

---

## 42. Prediction 알고리즘

Prediction은 주변 객체가 앞으로 어떻게 움직일지 예측합니다.

예시:

```text
앞 차량이 계속 직진할까?
보행자가 횡단보도로 들어올까?
옆 차가 차선 변경할까?
```

### 42.1 단순 예측

- constant velocity
- constant acceleration
- constant turn rate

장점:

- 빠름
- 설명 가능
- baseline으로 좋음

단점:

- 복잡한 교통 상황에 약함

---

### 42.2 Map-based Prediction

HD map과 lane 정보를 사용합니다.

```text
차량은 보통 차선을 따라간다.
교차로에서는 가능한 lane path 후보가 있다.
```

---

### 42.3 Learning-based Prediction

딥러닝으로 미래 궤적을 예측합니다.

입력:

- 주변 객체 history
- ego vehicle state
- lane graph
- traffic light
- crosswalk
- map context

출력:

- 다중 미래 궤적
- 각 궤적 확률

---

## 43. Planning 계층

자율주행 Planning은 보통 3단계입니다.

```text
Route Planning
  ↓
Behavior Planning
  ↓
Motion Planning
```

### 43.1 Route Planning

큰 길을 정합니다.

```text
서울역 → 강남역
```

알고리즘:

- Dijkstra
- A*
- graph search

---

### 43.2 Behavior Planning

주행 행동을 결정합니다.

예시:

- 차선 유지
- 차선 변경
- 정지
- 추월
- 양보
- 교차로 진입
- 유턴
- 주차

Behavior planning은 단순 최단경로보다 어렵습니다.  
교통 규칙과 주변 객체를 같이 봐야 합니다.

---

### 43.3 Motion Planning

실제로 따라갈 수 있는 trajectory를 만듭니다.

출력:

```text
시간이 포함된 경로
x, y, yaw, velocity, acceleration, time
```

단순 path와 trajectory 차이:

| 구분 | Path | Trajectory |
|---|---|---|
| 시간 포함 | 없음 | 있음 |
| 속도 포함 | 보통 없음 | 있음 |
| 제어 입력 | 부족 | 가능 |
| 예시 | 점들의 선 | 언제 어디를 얼마나 빠르게 갈지 |

---

## 44. Planning 알고리즘

### 44.1 Dijkstra

모든 방향으로 비용이 낮은 경로를 탐색합니다.

장점:

- 최단 경로 보장
- 이해 쉬움

단점:

- 느릴 수 있음
- heuristic 없음

---

### 44.2 A*

Dijkstra에 heuristic을 추가합니다.

```text
f(n) = g(n) + h(n)
```

- `g(n)`: 시작점에서 현재까지 비용
- `h(n)`: 현재에서 목표까지 예상 비용

장점:

- 빠름
- grid map에서 많이 사용

단점:

- 차량 운동 제약을 직접 반영하기 어려움

---

### 44.3 Hybrid A*

A*에 차량의 방향과 회전 반경을 포함합니다.

상태:

```text
x, y, yaw
```

자동차형 로봇에 유용합니다.

장점:

- 비홀로노믹 제약 반영
- 주차, 차량형 경로에 적합

단점:

- 일반 A*보다 복잡
- 파라미터 영향 큼

---

### 44.4 RRT

무작위 샘플링 기반 탐색입니다.

```text
공간에 랜덤 점을 찍고 트리를 확장한다.
```

종류:

- RRT
- RRT*
- Informed RRT*
- RRT-Connect

장점:

- 고차원 공간에 적합
- 복잡한 제약에 응용 가능

단점:

- 결과가 들쭉날쭉
- smoothing 필요
- 실시간 안정성 관리 필요

---

### 44.5 PRM

미리 랜덤 그래프를 만들고 경로를 찾습니다.

장점:

- 정적인 환경에서 재사용 가능

단점:

- 동적 장애물에는 별도 처리 필요

---

### 44.6 Lattice Planner

motion primitive를 미리 정의하고 조합합니다.

예:

```text
조금 전진
왼쪽으로 부드럽게 회전
오른쪽으로 부드럽게 회전
정지
```

장점:

- 차량 동역학 반영 쉬움
- 부드러운 trajectory 가능

단점:

- primitive 설계가 중요
- 계산량

---

### 44.7 Optimization-based Planning

목적함수를 최소화합니다.

예:

```text
목표:
- 장애물과 멀리
- 차선 중앙 유지
- 속도 부드럽게
- 가속도 작게
- jerk 작게
- 목표 지점 도달
```

비용 함수:

```text
cost = obstacle_cost
     + lane_cost
     + smoothness_cost
     + speed_cost
     + goal_cost
```

장점:

- 부드러운 trajectory
- 여러 제약을 같이 반영 가능

단점:

- 초기값 중요
- local minimum 가능
- 계산량

---

## 45. Control 알고리즘

Planning이 만든 path/trajectory를 실제 명령으로 바꾸는 단계입니다.

---

### 45.1 PID

가장 기본적인 제어기입니다.

```text
error = target - current
control = P + I + D
```

| 항 | 역할 |
|---|---|
| P | 현재 오차에 반응 |
| I | 누적 오차 보정 |
| D | 변화율에 반응 |

장점:

- 단순
- 빠름
- 현장 튜닝 가능

단점:

- 복잡한 차량 동역학에는 한계
- 속도/곡률 변화에 약할 수 있음

---

### 45.2 Pure Pursuit

경로 위의 lookahead point를 따라갑니다.

```text
현재 위치에서 일정 거리 앞의 점을 본다.
그 점을 향해 조향한다.
```

장점:

- 구현 쉬움
- 안정적
- 모바일 로봇과 차량 모두에 응용 가능

단점:

- lookahead 튜닝 중요
- 고속/급커브에서 성능 제한

---

### 45.3 Stanley Controller

차량형 lateral control에 많이 알려진 방식입니다.

고려:

- heading error
- cross-track error

장점:

- 차선 추종에 직관적
- 차량형 모델에 적합

단점:

- 저속/정지 근처 처리 필요
- gain 튜닝 필요

---

### 45.4 LQR

선형화된 시스템에서 비용을 최소화하는 제어입니다.

장점:

- 수학적으로 깔끔
- 안정성 분석 가능

단점:

- 모델 필요
- 선형화 오차

---

### 45.5 MPC

Model Predictive Control입니다.

```text
미래 몇 초를 예측한다.
제약 조건을 고려해 최적 제어 입력을 계산한다.
첫 입력만 적용한다.
다음 순간 다시 계산한다.
```

장점:

- 제약 조건 반영 가능
- 차량 동역학 반영 가능
- 고급 제어에 적합

단점:

- 계산량
- 모델 품질 중요
- 튜닝 어려움

---

## 46. Obstacle Avoidance

장애물 회피는 planning과 control 사이에 걸쳐 있습니다.

대표 접근:

| 알고리즘 | 특징 |
|---|---|
| DWA | 속도 후보 샘플링 |
| TEB | 시간 탄성 밴드 최적화 |
| VFH | 장애물 히스토그램 기반 |
| MPC | 제약 최적화 기반 |
| ORCA | 다중 에이전트 회피 |

Nav2의 local controller도 실시간 costmap을 보고 장애물을 피합니다.

---

# Part G. Autoware / CARLA / 실제 차량형 자율주행

---

## 47. Autoware 개요

Autoware는 ROS 기반의 오픈소스 자율주행 프레임워크입니다.

주요 모듈:

```text
Sensing
Localization
Perception
Planning
Control
Vehicle Interface
Map
System Monitoring
```

Nav2가 “모바일 로봇이 목표점까지 가는 navigation”에 강하다면,  
Autoware는 “차량형 자율주행 전체 스택”을 다룹니다.

---

## 48. Autoware와 Nav2 차이

| 항목 | Nav2 | Autoware |
|---|---|---|
| 주 대상 | 모바일 로봇 | 자동차형 자율주행 |
| 지도 | 2D occupancy grid | HD map, point cloud map |
| 위치 추정 | AMCL, SLAM | GNSS/IMU, NDT, map matching |
| 제어 출력 | `/cmd_vel` | steering/throttle/brake |
| 인지 | costmap 장애물 중심 | 객체 인식/추적/신호등/차선 |
| 예측 | 제한적 | 핵심 모듈 |
| 행동 계획 | BT 기반 navigation | lane change, intersection 등 |

입문 순서:

```text
ROS 2 → SLAM → Nav2 → robot_localization → CARLA/Gazebo → Autoware
```

바로 Autoware부터 시작하면 너무 많은 개념이 동시에 들어옵니다.

---

## 49. CARLA

CARLA는 자율주행 연구용 오픈소스 시뮬레이터입니다.

가능한 것:

- 차량 시뮬레이션
- 카메라 / LiDAR / radar / GNSS / IMU 센서
- 날씨, 시간대, 교통 상황
- ROS bridge
- scenario runner
- 자율주행 agent 테스트

학습 용도:

```text
ROS 2 topic으로 센서 데이터를 받고,
자율주행 알고리즘을 테스트하고,
차량 제어 명령을 보낸다.
```

---

## 50. 시뮬레이터 선택

| 시뮬레이터 | 적합한 용도 |
|---|---|
| Gazebo | ROS 2 모바일 로봇, Nav2 실습 |
| Isaac Sim | 고품질 물리/센서, GPU, 로보틱스 |
| CARLA | 자동차형 자율주행 |
| AWSIM | Autoware 연동 |
| Webots | 교육/간단한 로봇 시뮬레이션 |

명제님 기준으로는 다음 흐름이 좋습니다.

```text
Gazebo + Nav2
  ↓
Isaac Sim + ROS 2 bridge
  ↓
CARLA 또는 AWSIM + Autoware
  ↓
Isaac Lab / 강화학습 기반 제어
```

---

# Part H. 실습 프로젝트

---

## 51. 프로젝트 1: ROS 2 기본 통신

목표:

```text
publisher/subscriber/service/action/parameter를 직접 작성
```

해야 할 것:

- `/hello_count` 토픽 발행
- subscriber로 출력
- parameter로 주기 조정
- launch 파일 작성
- rosbag 기록

완료 기준:

```bash
ros2 topic echo /hello_count
ros2 param set /hello_node publish_rate 5.0
ros2 bag record /hello_count
```

---

## 52. 프로젝트 2: TF와 URDF

목표:

```text
간단한 2륜 로봇 URDF 작성
```

구성:

```text
base_link
left_wheel_link
right_wheel_link
laser_link
camera_link
imu_link
```

완료 기준:

```bash
ros2 run tf2_tools view_frames
rviz2
```

RViz에서 RobotModel과 TF가 정상 표시되어야 합니다.

---

## 53. 프로젝트 3: SLAM

목표:

```text
2D LiDAR 기반 map 생성
```

절차:

```bash
sudo apt install ros-$ROS_DISTRO-slam-toolbox
ros2 launch slam_toolbox online_async_launch.py
```

확인:

```bash
ros2 topic echo /map --once
ros2 run tf2_ros tf2_echo map base_link
```

저장:

```bash
ros2 run nav2_map_server map_saver_cli -f ~/maps/test_map
```

완료 기준:

- RViz에서 map이 보입니다.
- 벽이 크게 틀어지지 않습니다.
- map 저장 파일이 생성됩니다.

---

## 54. 프로젝트 4: Nav2로 목표 이동

목표:

```text
저장한 map + AMCL + Nav2로 goal navigation 수행
```

절차:

1. map server 실행
2. AMCL 실행
3. Nav2 bringup
4. RViz에서 initial pose 지정
5. goal 지정

완료 기준:

- global path가 생성됩니다.
- local costmap이 움직입니다.
- `/cmd_vel`이 발행됩니다.
- 로봇이 goal에 도달합니다.

---

## 55. 프로젝트 5: 장애물 정지 노드 작성

목표:

```text
/scan을 보고 가까운 장애물이 있으면 /cmd_vel을 0으로 제한
```

구조:

```text
/nav2/cmd_vel_raw
  ↓
safety_filter_node
  ↓
/ /cmd_vel
```

Python 예시:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class SimpleSafetyStop(Node):
    def __init__(self):
        super().__init__("simple_safety_stop")

        self.declare_parameter("stop_distance", 0.35)
        self.stop_distance = self.get_parameter("stop_distance").value

        self.latest_scan_too_close = False

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.on_scan,
            10,
        )

        self.cmd_sub = self.create_subscription(
            Twist,
            "/cmd_vel_raw",
            self.on_cmd,
            10,
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

    def on_scan(self, msg: LaserScan):
        valid_ranges = [
            r for r in msg.ranges
            if msg.range_min < r < msg.range_max
        ]

        if not valid_ranges:
            self.latest_scan_too_close = False
            return

        # 가장 가까운 장애물 거리를 확인해라
        self.latest_scan_too_close = min(valid_ranges) < self.stop_distance

    def on_cmd(self, msg: Twist):
        if self.latest_scan_too_close and msg.linear.x > 0.0:
            safe_cmd = Twist()
            # 앞에 장애물이 있으면 전진하지 마라
            safe_cmd.linear.x = 0.0
            safe_cmd.angular.z = 0.0
            self.cmd_pub.publish(safe_cmd)
            return

        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = SimpleSafetyStop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

이건 실무용 최종 safety controller는 아닙니다.  
하지만 safety filter의 구조를 이해하기에 좋습니다.

---

## 56. 프로젝트 6: Pure Pursuit 구현

목표:

```text
nav_msgs/Path를 받아 lookahead point를 추종하는 controller 작성
```

핵심 공식:

```text
curvature = 2 * y_local / lookahead_distance^2
angular_z = linear_velocity * curvature
```

구조:

```text
/path
/odom
  ↓
pure_pursuit_node
  ↓
/cmd_vel
```

구현 체크:

- path를 base_link 기준으로 변환합니다.
- lookahead distance보다 먼 점을 찾습니다.
- local 좌표의 y값으로 곡률을 계산합니다.
- 속도 제한을 적용합니다.
- goal 근처에서는 정지합니다.

---

## 57. 프로젝트 7: Nav2 Controller Plugin 분석

목표:

```text
DWB / Regulated Pure Pursuit / MPPI 중 하나를 선택해 내부 구조 분석
```

분석 포인트:

- 입력 topic/action
- 사용하는 costmap
- 속도 샘플링 방식
- collision check 방식
- 파라미터
- 실패 조건
- goal checker와 progress checker

---

## 58. 프로젝트 8: Autoware 맛보기

목표:

```text
Autoware의 모듈 구조와 topic 흐름 이해
```

처음부터 실차 주행을 목표로 하지 마십시오.

추천 순서:

```text
1. Autoware 문서 읽기
2. planning simulator 실행
3. rosbag replay
4. localization/perception/planning/control topic 확인
5. CARLA 또는 AWSIM 연동 구조 확인
```

---

# Part I. 디버깅 플레이북

---

## 59. ROS 2 공통 디버깅

### 59.1 노드가 안 보임

```bash
ros2 node list
```

의심:

- source 안 함
- ROS_DOMAIN_ID 다름
- 노드 실행 실패
- namespace 다름

---

### 59.2 토픽이 안 들어옴

```bash
ros2 topic list
ros2 topic info /topic_name --verbose
ros2 topic echo /topic_name
```

의심:

- topic 이름 다름
- message type 다름
- QoS 불일치
- publisher 없음
- namespace/remap 문제

---

### 59.3 RViz에서 안 보임

의심:

- Fixed Frame 오류
- TF 누락
- message frame_id 오류
- use_sim_time 문제
- display topic 잘못 선택

---

## 60. TF 디버깅

### 60.1 map에서 base_link까지 변환 확인

```bash
ros2 run tf2_ros tf2_echo map base_link
```

실패하면:

```text
map -> odom 없음
odom -> base_link 없음
base_link -> sensor 없음
시간 timestamp 문제
```

### 60.2 TF tree 생성

```bash
ros2 run tf2_tools view_frames
```

확인:

- tree가 하나로 연결되어 있는가?
- 중복 parent가 없는가?
- 같은 transform을 두 노드가 발행하지 않는가?

---

## 61. SLAM 디버깅

| 증상 | 원인 | 대응 |
|---|---|---|
| map이 안 나옴 | `/scan` 없음 | scan topic 확인 |
| map이 흔들림 | odom 불안정 | EKF/encoder 확인 |
| 벽이 두꺼움 | scan matching 실패 | 속도 줄이기 |
| loop closure 후 망가짐 | 잘못된 loop closure | 파라미터 조정 |
| 로봇 pose가 튐 | TF 시간 문제 | `/clock`, use_sim_time 확인 |

---

## 62. Nav2 디버깅

### 62.1 goal을 줘도 안 감

확인:

```bash
ros2 action list
ros2 topic echo /cmd_vel
ros2 run tf2_ros tf2_echo map base_link
```

의심:

- Nav2 lifecycle inactive
- initial pose 미설정
- AMCL pose 불안정
- global costmap 막힘
- planner 실패
- controller 실패

---

### 62.2 global path는 있는데 움직이지 않음

의심:

- controller server 문제
- local costmap 장애물로 막힘
- progress checker 실패
- `/cmd_vel` remap 문제
- motor driver가 `/cmd_vel`을 안 받음

---

### 62.3 local costmap이 이상함

확인:

- sensor topic
- sensor frame
- obstacle layer
- clearing/marking
- transform tolerance
- inflation radius
- robot footprint

---

### 62.4 AMCL 위치가 틀림

대응:

- RViz에서 initial pose 다시 지정
- map과 실제 환경 일치 확인
- scan이 map 벽과 맞는지 확인
- odom 품질 확인
- AMCL particle 수 조정
- LiDAR range와 laser model 확인

---

# Part J. 설계 원칙

---

## 63. 좋은 ROS 패키지 구조

권장:

```text
my_robot/
  my_robot_description/
    urdf/
    meshes/
    launch/
  my_robot_bringup/
    launch/
    config/
  my_robot_navigation/
    config/
    launch/
  my_robot_slam/
    config/
    launch/
  my_robot_control/
    src/
    include/
  my_robot_perception/
    src/
    launch/
```

각 패키지의 책임을 분리합니다.

---

## 64. 좋은 노드 설계

좋은 노드:

```text
입력 명확
출력 명확
파라미터 명확
frame_id 명확
예외 처리 있음
로그 있음
테스트 가능
```

나쁜 노드:

```text
여러 일을 다 함
topic 이름이 코드에 박혀 있음
frame_id 하드코딩
파라미터 없음
실패 로그 없음
```

---

## 65. 인터페이스 중심 설계

자율주행 시스템은 모듈이 많습니다.  
따라서 인터페이스가 중요합니다.

예:

```text
Perception output = TrackedObjects
Prediction output = PredictedObjects
Planning output = Trajectory
Control output = ControlCommand
```

모듈이 서로 내부 구현을 몰라도 메시지 계약만 지키면 교체 가능합니다.

객체지향 비유:

```text
interface IPlanner
  plan(current_state, goal, map) -> trajectory
```

ROS에서는 이 interface가 message/action/service입니다.

---

## 66. Strategy Pattern과 Plugin

Nav2는 플러그인 구조를 많이 씁니다.

```text
Planner Server
  ├── NavFn
  ├── Smac 2D
  ├── Smac Hybrid-A*
  └── Theta*
```

이건 디자인 패턴으로 보면 Strategy Pattern입니다.

장점:

- 알고리즘 교체 가능
- 서버 구조 재사용
- 설정 파일로 선택 가능

---

## 67. Behavior Tree와 상태 관리

복잡한 주행 시스템을 단순 if문으로 만들면 망가집니다.

나쁜 예:

```python
if obstacle:
    stop()
elif goal:
    go()
elif fail:
    recover()
...
```

좋은 구조:

```text
Behavior Tree
  ├── Condition
  ├── Action
  ├── Recovery
  └── Retry
```

상태가 많아질수록 BT 또는 명확한 FSM이 필요합니다.

---

## 68. 실무 안전 원칙

실로봇 자율주행에서는 반드시 아래가 필요합니다.

- emergency stop
- velocity limit
- acceleration limit
- watchdog
- command timeout
- obstacle stop
- bumper handling
- battery monitoring
- motor fault handling
- localization confidence check
- stale sensor data check
- manual override

최소 safety filter:

```text
Nav2 /cmd_vel
  ↓
safety supervisor
  ↓
motor driver
```

motor driver가 Nav2 명령을 직접 받는 구조는 위험합니다.

---

# Part K. 12주 학습 계획

---

## 69. 1~2주차: ROS 2 기본

목표:

- node/topic/service/action 이해
- parameter/launch 이해
- rosbag 사용
- RViz 사용

실습:

- publisher/subscriber 작성
- service 작성
- action client/server 예제 실행
- rosbag 기록/재생

완료 기준:

```text
ROS 2 topic graph를 보고 데이터 흐름을 설명할 수 있다.
```

---

## 70. 3주차: TF / URDF

목표:

- `map`, `odom`, `base_link` 이해
- URDF 작성
- robot_state_publisher 사용

실습:

- 2륜 로봇 URDF
- LiDAR frame 추가
- RViz RobotModel 표시
- view_frames 확인

완료 기준:

```text
TF tree를 보고 어느 노드가 어떤 transform을 발행하는지 설명할 수 있다.
```

---

## 71. 4주차: Odometry / EKF

목표:

- wheel odom 이해
- IMU 이해
- robot_localization 사용

실습:

- `/odom` echo
- `/imu` echo
- EKF로 `/odometry/filtered` 생성
- TF 충돌 제거

완료 기준:

```text
odom -> base_link를 누가 발행해야 하는지 판단할 수 있다.
```

---

## 72. 5~6주차: SLAM

목표:

- SLAM 개념 이해
- SLAM Toolbox 사용
- map 저장

실습:

- Gazebo 또는 실제 LiDAR로 mapping
- RViz에서 map 확인
- map_saver 사용
- 품질 문제 분석

완료 기준:

```text
좋은 map과 나쁜 map의 원인을 설명할 수 있다.
```

---

## 73. 7~8주차: Nav2

목표:

- Nav2 architecture 이해
- map + AMCL + Nav2 실행
- costmap 튜닝
- planner/controller 차이 이해

실습:

- TurtleBot Nav2 예제
- own map으로 navigation
- inflation radius 변경 실험
- controller frequency 변경 실험

완료 기준:

```text
Nav2 실패 시 TF, map, localization, costmap, planner, controller 순서로 디버깅할 수 있다.
```

---

## 74. 9주차: Planning 알고리즘

목표:

- A*, Dijkstra, RRT, Hybrid A* 이해
- grid planner 직접 구현

실습:

- Python으로 A* 구현
- occupancy grid에서 path 생성
- path smoothing 적용

완료 기준:

```text
A*의 g/h/f cost를 설명할 수 있다.
```

---

## 75. 10주차: Control 알고리즘

목표:

- PID
- Pure Pursuit
- Stanley
- MPC 개념 이해

실습:

- Pure Pursuit node 작성
- lookahead 튜닝
- 속도에 따른 추종 성능 비교

완료 기준:

```text
path와 trajectory의 차이를 설명할 수 있다.
```

---

## 76. 11주차: Perception / Prediction

목표:

- detection/tracking/prediction 개념 이해
- camera/LiDAR/radar 역할 이해

실습:

- 카메라 object detection 예제
- LiDAR clustering 예제
- tracking baseline 구현

완료 기준:

```text
Detection 결과만으로 planning하면 왜 부족한지 설명할 수 있다.
```

---

## 77. 12주차: Autoware / CARLA / 통합

목표:

- 차량형 자율주행 stack 이해
- Autoware module 구조 이해
- CARLA ROS bridge 이해

실습:

- CARLA 센서 topic 확인
- Autoware planning simulator 문서 읽기
- ROS graph 분석

완료 기준:

```text
Nav2와 Autoware의 차이를 구조적으로 설명할 수 있다.
```

---

# Part L. 체크리스트

---

## 78. ROS 2 체크리스트

- [ ] `source /opt/ros/$ROS_DISTRO/setup.bash`를 이해했습니다.
- [ ] workspace를 만들고 `colcon build`를 실행할 수 있습니다.
- [ ] node/topic/service/action 차이를 설명할 수 있습니다.
- [ ] `ros2 topic echo`, `ros2 topic hz`를 사용할 수 있습니다.
- [ ] parameter YAML을 읽을 수 있습니다.
- [ ] launch 파일의 역할을 이해했습니다.
- [ ] rosbag을 기록하고 재생할 수 있습니다.
- [ ] QoS 문제를 의심할 수 있습니다.

---

## 79. TF 체크리스트

- [ ] `map`, `odom`, `base_link` 차이를 설명할 수 있습니다.
- [ ] `map -> odom`을 누가 발행하는지 압니다.
- [ ] `odom -> base_link`를 누가 발행하는지 압니다.
- [ ] `base_link -> sensor`는 URDF/static TF로 설정합니다.
- [ ] `tf2_echo`를 사용할 수 있습니다.
- [ ] `view_frames` 결과를 읽을 수 있습니다.
- [ ] 같은 TF를 두 노드가 발행하면 안 된다는 것을 압니다.

---

## 80. SLAM 체크리스트

- [ ] `/scan`이 정상입니다.
- [ ] `/odom`이 정상입니다.
- [ ] `base_link -> laser_link`가 정확합니다.
- [ ] SLAM이 `/map`을 발행합니다.
- [ ] SLAM이 `map -> odom`을 발행합니다.
- [ ] map을 저장할 수 있습니다.
- [ ] loop closure 실패 증상을 구분할 수 있습니다.

---

## 81. Nav2 체크리스트

- [ ] map server를 이해했습니다.
- [ ] AMCL을 이해했습니다.
- [ ] BT Navigator 역할을 이해했습니다.
- [ ] Planner Server와 Controller Server 차이를 압니다.
- [ ] global costmap과 local costmap 차이를 압니다.
- [ ] inflation layer를 튜닝할 수 있습니다.
- [ ] footprint를 설정할 수 있습니다.
- [ ] recovery behavior를 이해했습니다.
- [ ] `/cmd_vel`이 어디서 나오는지 압니다.

---

## 82. 자율주행 알고리즘 체크리스트

- [ ] Localization과 SLAM의 차이를 압니다.
- [ ] EKF/UKF/Particle Filter 차이를 압니다.
- [ ] A*/Dijkstra/RRT/Hybrid A* 차이를 압니다.
- [ ] Path와 Trajectory 차이를 압니다.
- [ ] PID/Pure Pursuit/Stanley/MPC 차이를 압니다.
- [ ] Detection/Tracking/Prediction 차이를 압니다.
- [ ] Nav2와 Autoware 차이를 압니다.

---

# Part M. 명령어 치트시트

---

## 83. ROS 기본

```bash
ros2 node list
ros2 topic list
ros2 service list
ros2 action list
ros2 param list
```

```bash
ros2 topic echo /topic
ros2 topic hz /topic
ros2 topic info /topic --verbose
```

```bash
ros2 interface show geometry_msgs/msg/Twist
ros2 interface show sensor_msgs/msg/LaserScan
```

---

## 84. TF

```bash
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_tools view_frames
```

---

## 85. SLAM

```bash
ros2 launch slam_toolbox online_async_launch.py
ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map
```

---

## 86. Nav2

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
ros2 lifecycle nodes
ros2 lifecycle get /controller_server
```

---

## 87. rosbag

```bash
ros2 bag record /scan /odom /tf /tf_static
ros2 bag record -a
ros2 bag play bag_name
```

---

# Part N. 용어집

---

| 용어 | 의미 |
|---|---|
| ROS | 로봇 소프트웨어 프레임워크 |
| Node | 실행 단위 |
| Topic | 지속 데이터 통로 |
| Service | 요청-응답 |
| Action | 오래 걸리는 작업 |
| Parameter | 노드 설정값 |
| Launch | 여러 노드 실행 파일 |
| TF | 좌표 변환 시스템 |
| URDF | 로봇 모델 XML |
| Odometry | 이동량 추정 |
| SLAM | 위치 추정과 지도 작성을 동시에 수행 |
| Localization | 지도 안에서 자기 위치 추정 |
| Costmap | 위험도 지도 |
| Planner | 경로 생성 |
| Controller | 속도/제어 명령 생성 |
| Behavior Tree | 행동 흐름 제어 트리 |
| AMCL | Particle filter 기반 localization |
| EKF | 확장 칼만 필터 |
| NDT | point cloud map matching 방식 |
| ICP | point cloud 정합 |
| HD Map | 고정밀 도로 지도 |
| Prediction | 주변 객체 미래 예측 |
| MPC | 모델 예측 제어 |

---

# Part O. 공부할 때의 핵심 결론

---

## 88. 가장 중요한 순서

```text
TF를 모르면 SLAM이 안 됩니다.
SLAM을 모르면 map이 안 됩니다.
map과 localization을 모르면 Nav2가 안 됩니다.
Nav2를 모르면 자율주행 stack 구조가 안 잡힙니다.
Planning과 Control을 모르면 실제 알고리즘을 바꿀 수 없습니다.
Perception과 Prediction을 모르면 차량형 자율주행으로 못 넘어갑니다.
```

---

## 89. 초보자에게 가장 위험한 실수

```text
바로 딥러닝 자율주행부터 시작하는 것
```

이러면 다음 문제가 생깁니다.

- `/tf`가 뭔지 모릅니다.
- `/odom`이 뭔지 모릅니다.
- frame_id 오류를 못 잡습니다.
- LiDAR가 왜 RViz에 안 뜨는지 모릅니다.
- costmap이 왜 막혔는지 모릅니다.
- controller가 왜 흔들리는지 모릅니다.
- 실로봇에서 위험합니다.

따라서 기초 순서는 반드시 지키는 것이 좋습니다.

---

## 90. 실무형 사고방식

자율주행은 “알고리즘 하나”가 아닙니다.

```text
좋은 자율주행 시스템 =
  안정적인 센서 입력
+ 정확한 시간 동기화
+ 올바른 좌표계
+ 신뢰 가능한 localization
+ 깨끗한 map
+ 보수적인 costmap
+ 상황에 맞는 planner
+ 안정적인 controller
+ safety supervisor
+ 로그와 재현 가능한 디버깅
```

알고리즘을 바꾸기 전에 먼저 시스템 입력이 정상인지 봐야 합니다.

---

# Part P. 참고 자료

아래 자료를 기준으로 구조와 최신 패키지 흐름을 확인했습니다.

## ROS 2

- ROS 2 공식 문서: https://docs.ros.org/
- ROS 2 Jazzy 배포판 문서: https://docs.ros.org/en/jazzy/Releases.html
- ROS 2 QoS 설계 문서: https://design.ros2.org/articles/qos.html
- REP-105 Coordinate Frames for Mobile Platforms: https://www.ros.org/reps/rep-0105.html

## Nav2

- Nav2 공식 문서: https://docs.nav2.org/
- Nav2 Getting Started: https://docs.nav2.org/getting_started/index.html
- Nav2 Navigation Concepts: https://docs.nav2.org/concepts/index.html
- Nav2 Transform Setup: https://docs.nav2.org/setup_guides/transformation/setup_transforms.html
- Nav2 Costmap Configuration: https://docs.nav2.org/configuration/packages/configuring-costmaps.html
- Nav2 Planner Server: https://docs.nav2.org/configuration/packages/configuring-planner-server.html
- Nav2 Controller Server: https://docs.nav2.org/configuration/packages/configuring-controller-server.html
- Nav2 AMCL: https://docs.nav2.org/configuration/packages/configuring-amcl.html
- Nav2 SLAM Tutorial: https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html

## SLAM / Localization

- SLAM Toolbox GitHub: https://github.com/SteveMacenski/slam_toolbox
- robot_localization 문서: https://docs.ros.org/en/melodic/api/robot_localization/html/index.html
- Nav2 robot_localization setup guide: https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html

## Autoware / CARLA

- Autoware Documentation: https://autowarefoundation.github.io/autoware-documentation/main/home/
- Autoware Overview: https://autoware.org/autoware-overview/
- CARLA 공식 사이트: https://carla.org/
- CARLA ROS Bridge 문서: https://carla.readthedocs.io/projects/ros-bridge/en/latest/

---

# 마지막 요약

입문자는 아래 순서를 따르면 됩니다.

```text
ROS 2 기본
  ↓
TF / URDF
  ↓
Odometry / EKF
  ↓
SLAM Toolbox
  ↓
Map 저장
  ↓
AMCL
  ↓
Nav2
  ↓
Planning / Control 알고리즘
  ↓
Perception / Prediction
  ↓
Autoware / CARLA
  ↓
실로봇 안전 / 디버깅 / 배포
```

가장 중요한 실무 기준은 이것입니다.

```text
자율주행은 알고리즘보다 시스템 연결이 먼저입니다.
TF, 시간, 센서, odom, map이 정상이어야 알고리즘이 의미를 가집니다.
```
