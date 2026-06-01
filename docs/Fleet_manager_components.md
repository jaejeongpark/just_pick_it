# Fleet Manager 구성요소 설명 (발표용)

Just Pick It의 중앙 제어 노드인 **Fleet Manager**는 하나의 ROS2 프로세스 안에서
DB 접근 / 작업 스케줄링 / 경로 관리 / 로봇 명령 / 웹 API를 조립한다.
`fleet_manager_node.py`의 `FleetManagerNode`만 `rclpy.Node`를 상속하고,
나머지 기능 클래스는 별도 노드가 아니라 이 노드에 조립되는 일반 Python 객체로 동작한다.

```text
FleetManagerNode
  ├── FleetRepository      DB 접근 단일 계층
  ├── FleetApiServer       HTTP/WebSocket API (uvicorn, 데몬 스레드)
  ├── TrafficManager       PICKY 경로 탐색/예약/충돌 회피
  ├── RobotStateMonitor    로봇 텔레메트리 구독 → DB + Traffic
  ├── RobotCommandGateway  task → ROS2 Action/Service 명령
  └── TaskManager          주문/진열 polling, task 생성/전이/dispatch
```

---

## 1. 컴포넌트별 상세 설명

### Fleet Manager Node — `fleet_manager_node.py`
- Fleet Manager의 메인 ROS2 노드. 아래 기능 클래스들은 각각 별도 노드가 아니라 이 노드에 조립되는 Python 객체로 동작한다.
- 노드 시작 시 설정을 읽고, 6개 컴포넌트를 생성·연결하며, 주기 타이머(대기 작업 polling, 재시작 복구)를 건다.

### Fleet API Schemas — `fleet_api_schemas.py`
- HTTP 명령 엔드포인트(POST/PATCH)의 입력 검증용 Pydantic 모델 모음. 주문·상품·작업 등 요청 본문의 형식을 검사한다.
- 응답은 Repository가 만든 dict를 그대로 쓰므로 응답 모델은 따로 두지 않는다.

### Fleet API Server — `fleet_api_server.py`
- 웹 프런트에 노출하는 HTTP/REST + WebSocket API 서버. ROS2 executor와 같은 프로세스 안에서 별도 스레드의 uvicorn으로 돈다.
- DB 접근은 Repository로, 로봇 전파가 필요한 명령만 메인 노드로 위임한다.

### Fleet Repository — `fleet_repository.py`
- Fleet Manager의 단일 DB 접근 계층. 주문·작업·로봇·zone·진열 데이터를 읽고 쓰는 유일한 진입점이다.
- PostgreSQL(`just_pick_it_db`)에 직접 접근하고, 상태 전이 등 비즈니스 로직은 DB services를 재사용한다. (`RepoError`는 not-found/검증 실패 예외)

### Robot Command Gateway — `robot_command_gateway.py`
- Fleet의 task를 ROS2 Action/Service 명령으로 변환해 로봇에 보내는 출력 어댑터. TaskManager가 topic/message 구조를 직접 몰라도 되게 한다.
- PICKY 이동(MoveCommand)·도킹(DockCommand)·비상정지(EmergencyControl)를 지원하고, Action 피드백/결과를 다시 콜백으로 돌려준다.

### Task Manager — `task_manager.py`
- Fleet Manager 내부 task의 생성·배정·상태 전이 담당. DB를 polling해 대기 중인 주문/진열 요청을 찾고, 가용 로봇에 배정하며, task로 변환한다.
- TrafficManager와 협업해 PICKY 경로를 예약하고, task 진행/실패를 DB에 보고한다. (ROS2 노드가 아니며 메인 노드의 logger만 사용)

### Traffic Manager — `traffic_manager.py`
- BFS 기반 경로 계획 + 다중 AMR 충돌 회피 모듈. zone 인접 그래프 위에서 경로를 탐색·예약한다.
- 외부 I/O가 없다: 로봇 상태는 RobotStateMonitor가 `notify_state()`로 전달하고, zone 좌표는 생성자에서 주입받는다. (`PathResult`는 탐색/예약 결과 구조체)

### (참고) Robot State Monitor — `robot_state_monitor.py`
- 목록 외 항목. PICKY 텔레메트리(picky_state / battery / pose)를 ROS2 토픽으로 구독해 DB와 TrafficManager에 반영하는 클래스.

---

## 2. 한 문장 요약 (슬라이드용)

| 구성요소 | 한 문장 설명 |
|---|---|
| **Fleet Manager Node** | 6개 기능 클래스를 한 프로세스에 조립·배선하고 주기 타이머를 돌리는 메인 ROS2 노드. |
| **Fleet API Schemas** | HTTP 요청 본문을 검증하는 Pydantic 입력 모델 모음. |
| **Fleet API Server** | 웹 프런트에 HTTP/REST + WebSocket을 노출하는 API 서버(별도 스레드 uvicorn). |
| **Fleet Repository** | 모든 DB 읽기·쓰기를 책임지는 단일 DB 접근 계층. |
| **Robot Command Gateway** | task를 ROS2 Action/Service 명령으로 바꿔 로봇에 보내는 출력 어댑터. |
| **Task Manager** | 주문/진열을 polling해 로봇에 배정하고 task 상태를 전이시키는 스케줄러. |
| **Traffic Manager** | zone 그래프 BFS로 경로를 계획하고 AMR 충돌을 회피하는 모듈. |
| *(참고)* **Robot State Monitor** | 로봇 텔레메트리를 구독해 DB와 Traffic에 반영하는 모듈. |
