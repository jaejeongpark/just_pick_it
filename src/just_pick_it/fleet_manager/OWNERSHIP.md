# Fleet Manager 담당 분배

`fleet_manager` 패키지 내부 책임과 수정 경계를 현재 Fleet API 통합 구조 기준으로 정리한다.

## 단일 ROS2 Node 원칙

`fleet_manager_node.py`의 `FleetManagerNode`만 `rclpy.Node`를 상속한다.
나머지는 이 노드에 조립되는 일반 Python 클래스다.

```text
FleetManagerNode
  ├── FleetRepository
  ├── FleetApiServer
  ├── TrafficManager
  ├── TaskManager
  ├── RobotCommandGateway
  └── RobotStateMonitor
```

## 모듈별 책임

| 모듈 | 파일 | 책임 |
|---|---|---|
| `FleetRepository` | `fleet_manager/fleet_repository.py` | `just_pick_it_db`를 통한 DB 접근, snapshot/query/write |
| `FleetApiServer` | `fleet_manager/fleet_api_server.py` | Web Gateway가 호출하는 HTTP/WebSocket API |
| `TaskManager` | `fleet_manager/task_manager.py` | 주문/입고 polling, task 생성, task 상태 전이 |
| `RobotCommandGateway` | `fleet_manager/robot_command_gateway.py` | task를 PICKY/COBOT Action/Service 명령으로 변환 |
| `TrafficManager` | `fleet_manager/traffic_manager.py` | PICKY 경로 탐색, 충돌 회피, path 예약/해제 |
| `RobotStateMonitor` | `fleet_manager/robot_state_monitor.py` | PICKY 상태 토픽 구독 후 TrafficManager에 전달 |
| `FleetManagerNode` | `fleet_manager/fleet_manager_node.py` | 위 컴포넌트를 생성하고 연결 |

## 수정 경계

- `TrafficManager`와 `RobotStateMonitor`는 다른 담당자 영역이므로 직접 수정하지 않는다.
- 위 두 파일 변경이 필요하면 필요한 계약 변경과 이유를 문서/메시지로 먼저 공유한다.
- `TaskManager`, `FleetRepository`, `FleetApiServer`, `RobotCommandGateway`, `FleetManagerNode`는 Fleet API 통합 작업 범위 안에서 수정 가능하다.
- Web Gateway는 `web/` 폴더 안에서 화면 제공과 `/api/*` 프록시만 담당한다. DB 접근 코드는 두지 않는다.

## 현재 API 경계

```text
Browser
  -> Web Gateway (:8000)
  -> Fleet API (:8100)
  -> FleetRepository
  -> just_pick_it_db
  -> PostgreSQL
```

- 브라우저는 계속 같은 origin인 `http://localhost:8000/api/*`를 호출한다.
- Web Gateway는 해당 요청을 Fleet API로 전달한다.
- DB 읽기/쓰기는 Fleet Manager 프로세스 내부의 `FleetRepository`만 수행한다.
- Emergency/Resume은 Fleet API `POST /api/admin/emergency-stop`, `POST /api/admin/resume`으로 들어와 DB 전이와 로봇 전파를 함께 수행한다.

## LLM 입고 명령 경계

- LLM 담당자는 `web/app/services/llm_client.py`에서 자연어 파싱만 구현한다.
- 파싱 결과가 `action="STOCKING"`이면 `web/app/routers/llm_router.py`가 Fleet API `POST /api/admin/stocking-items`를 호출한다.
- 입고 task 생성과 실행은 `TaskManager`가 `REQUESTED` stocking item을 polling해서 처리한다.

## 변경 이력

- 2026-05-22: 초안. Traffic/Task 분리 기준 정리.
- 2026-05-27: 기존 HTTP client 구조를 Fleet API + FleetRepository 구조로 갱신.
