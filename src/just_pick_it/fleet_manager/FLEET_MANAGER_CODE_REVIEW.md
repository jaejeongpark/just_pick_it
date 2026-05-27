# Fleet Manager Code Review

이 문서는 팀원이 빠르게 구조를 맞추기 위한 전체 요약 코드리뷰다.
함수 단위의 상세 분석은 루트 `reviews/fleet_manager/` 아래 문서를 본다.

Fleet Manager는 ROS2 노드, Fleet API 서버, DB Repository, 작업 스케줄러를 한 프로세스 안에서 조립한다.
현재 기준에서 Web Gateway는 화면과 프록시만 담당하고, DB 접근은 Fleet Manager의 `FleetRepository`만 수행한다.

## 읽는 순서

```text
1. fleet_manager_node.py
2. fleet_api_server.py
3. fleet_repository.py
4. task_manager.py
5. robot_command_gateway.py
6. traffic_manager.py
7. robot_state_monitor.py
```

상세 리뷰 문서:

```text
reviews/fleet_manager/
├── README.md
├── fleet_manager_node_review.md
├── fleet_api_server_review.md
├── fleet_repository_review.md
├── task_manager_review.md
├── robot_command_gateway_review.md
├── traffic_manager_review.md
└── robot_state_monitor_review.md
```

## 전체 구조

```text
Browser
  -> Web Gateway (:8000)
  -> Fleet API (:8100)
  -> FleetRepository
  -> just_pick_it_db
  -> PostgreSQL

FleetManagerNode
  ├── FleetApiServer
  ├── FleetRepository
  ├── TaskManager
  ├── RobotCommandGateway
  ├── TrafficManager
  └── RobotStateMonitor
```

## 고정 실행 기준

```text
Ubuntu 24.04
ROS 2 Jazzy
Python 3.12
FastAPI 0.101.0
Pydantic 1.10.14
colcon build --symlink-install
```

## 파일별 책임

| 파일 | 책임 |
|---|---|
| `fleet_manager_node.py` | ROS2 Node 조립자, timer/API server/명령 전파 연결 |
| `fleet_api_server.py` | FastAPI/uvicorn 서버, REST/WebSocket endpoint 제공 |
| `fleet_repository.py` | DB 조회/쓰기, 상태 전이, snapshot 생성 |
| `task_manager.py` | 대기 주문/입고 확인, task 생성, dispatch/result 처리 |
| `robot_command_gateway.py` | task를 PICKY action/service 요청으로 변환, COBOT action 연결 대기 |
| `traffic_manager.py` | PICKY 경로 탐색, 점유 예약, 경로 해제 |
| `robot_state_monitor.py` | PICKY 상태 topic을 받아 TrafficManager에 전달 |

## 핵심 검토 포인트

```text
- Web Gateway가 DB/session/model을 import하지 않는가?
- Fleet API handler가 DB 작업을 FleetRepository로만 위임하는가?
- FleetRepository의 각 public method가 session_scope() 경계 안에서 DB를 다루는가?
- TaskManager가 HTTP URL이나 SQLAlchemy Session을 직접 알지 않는가?
- TrafficManager가 DB/HTTP/ROS Action을 직접 다루지 않는가?
- RobotCommandGateway가 Action/Service 변환과 callback 연결만 담당하는가?
- Emergency/Resume이 DB 상태 전이와 robot service 전파를 함께 수행하는가?
- Pydantic v1 문법(Field min_items 등)과 requirements/setup.py 버전이 일치하는가?
```

## 현재 연동 대기

| 항목 | 담당 연동 |
|---|---|
| PICKY `MoveCommand` action server | PICKY State Manager |
| PICKY `DockCommand` action server | PICKY State Manager |
| COBOT `ExecuteTask` action 정의/서버 | COBOT State Manager |
| `EmergencyControl` service server | 각 robot State Manager |
| Vision 결과 반영 | Vision/State Manager 담당 |
| LLM 실제 parser | `web/app/services/llm_client.py` 담당 구현 |

## 검증 명령

```bash
web/.venv/bin/python -m compileall -q web/app
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3.12 -m compileall -q src/just_pick_it/fleet_manager/fleet_manager
python3.12 -m pytest -q src/just_pick_it/fleet_manager/test/test_traffic_manager.py
colcon build --packages-select fleet_manager
```
