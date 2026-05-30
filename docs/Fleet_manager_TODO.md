# Fleet Manager 작업 현황 (담당 · 결정 · TODO)

갱신: 2026-05-29. 설계·동작은 `docs/Fleet_manager.md`, 인터페이스 계약은 `docs/Fleet_manager_interface.md`.

심각도: **S**(기능 결함) / **R**(견고성) / **C**(문서·정합) / **Q**(테스트).

---

## 1. 담당 경계

| 영역(파일) | 담당 |
|---|---|
| FleetRepository / TrafficManager / RobotStateMonitor / State Manager(PICKY) | 박서우 |
| Web Service / FleetApiServer / TaskManager / RobotCommandGateway | 이명제 |
| FleetManagerNode / `just_pick_it_db/services/*` / interfaces / docs | 공동 |

상세 책임표는 `Fleet_manager.md` §2.

---

## 2. 결정 현황 (확정)

| # | 결정 | 요지 |
|---|---|---|
| D1 | 로봇 텔레메트리 = **ROS2 토픽** | State Manager 발행 → RobotStateMonitor 구독 → DB. HTTP 보고 폐기. (System Architecture 준수) |
| D2 | `robot_status` = **task 전이 전용** | `workflow_service`만 기록. 텔레메트리는 picky_state/battery/pose만 갱신 |
| D3 | `/api/fleet/*` = **유지** | admin UI 검증/디버그용. Fleet API→Repository→DB라 정책 위배 아님 |
| D4 | 진열 재고 = **계획값** | `display_item.stock_delta` 기반. 비전 실측(rack check) 미구현 → legacy 완료 API 제거 |
| D5 | 진열 처리 = **DISPLAY 흐름** | 창고 선별·적재는 주문용 `SORTING_AND_LOAD`를 재사용하고, `display_item` 기준으로 `MOVE_TO_STOCK` → `SORTING_AND_LOAD` → `MOVE_TO_DISPLAY` → `DISPLAY_SCAN` → `DISPLAY_PLACE` 순서로 처리한다. `DISPLAY_PLACE` SUCCESS 시 `stock_qty`를 반영한다. 기준: `6_Data_Structure.pdf` / `5_Picky_State_Diagram.drawio.png` ver2.0 |

---

## 3. 진행 현황

### 완료

| 항목 | 내용 | 담당 |
|---|---|---|
| S1 | 로봇 텔레메트리 ROS2 통일 + `robot_status` 오염 제거 (picky_state/battery percent/amcl_pose 구독 → 1Hz coalesce DB 반영) | 박서우 |
| — | State Manager HTTP 보고/TF/battery 죽은 코드 정리 | 박서우 |
| — | battery 임계 초과 구간 robot별 1회 `handle_battery_update` 게이팅 | 박서우 |
| D4 | 미사용 legacy 완료 API 제거 및 진열 재고 반영 정책 정리 | 박서우 |
| R1 | 재시작 복구 A'' (현재 위치 기준 점유 복원 + 텔레메트리 완료 재동기 + 타임아웃). 설계는 `Fleet_manager.md` §5 | 박서우 |
| S2 | COBOT 디스패치 영구 스킵 버그 수정(매 cycle 재시도, 경고만 rate-limit) | 이명제 |
| R2 | flow 종료 시 in-memory 임시 메모리 정리 | 이명제 |
| C1/C2 | Fleet Manager 문서 3종으로 통합·최신화 (이 문서 포함) | 공동 |
| C3 | D5(진열 흐름) 설계 문서 반영 | 공동 |
| D5 | 진열 흐름 코드 반영(enum/API/TaskManager/Web/테스트) | 이명제 |

C3 수정 문서:
- `docs/Fleet_manager.md` §4.3 진열 흐름 — 새 task 시퀀스, `stock_qty` 트리거를 `DISPLAY_PLACE`로
- `docs/Fleet_manager_interface.md` §4 COBOT 명령 task / §5 PICKY 계약 / §6 COBOT 계약표·STOWING_ARM 선계획표 / §7 MOVING_STATES / §8 시나리오
- `docs/Fleet_manager_TODO.md` D5 결정·이 항목 추가
- `src/just_pick_it/pinky_amr_1/docs/state_manager.md` move_command 트리거 task 목록
- `docs/ros2_driving_beginner_guide.md` 주행 task 표 / COBOT task 목록 / Milestone 8

### 남은 작업

**박서우**
- [ ] 실로봇 검증: `/pickyX/amcl_pose` namespaced 발행 확인 / 텔레메트리 들어와도 `robot_status` IDLE 유지 / battery 게이팅 실측 / R1 reconcile 실동작.
- [ ] pose TF fallback 도입 여부 — amcl_pose가 안 나오거나 갱신이 너무 드물면 robot별 namespaced TF buffer로. (실 테스트 후 결정, 현재 보류)
- [ ] Q: `fleet_repository`(상태 전이/트랜잭션) 단위 테스트 추가. recovery/traffic 테스트는 `test/test_recovery.py`에 존재.

**이명제**
- [x] Fleet 측 COBOT `send_cobot_task` 연결 준비 + STOWING_ARM 감지 → `preplan_after_cobot_stowing` 호출.
- [ ] `just_pick_it_interfaces` COBOT action 메시지 최종 정의 반영.
- [ ] COBOT State Manager가 `/{cobot_ns}/execute_task` action server를 실제 패키지에서 제공하는지 통합 확인.
- [ ] Web LLM parser 실제 구현(`web/app/services/llm_client.py`).
- [ ] Q: `task_manager` 추가 흐름 테스트, `fleet_api_server` 엔드포인트 테스트.

**공동 / 회색지대**
- [ ] B3: emergency-stop이 HTTP 스레드에서 rclpy를 직접 호출(`trigger_emergency_stop` → gateway). 통합계획 2.3/3.4대로 executor로 위임 검토.
- [ ] 관리자 엔드포인트 인증/인가.
- [ ] WebSocket push를 전체 스냅샷 → 변경분(delta) 전환 검토.
- [ ] 전체 E2E(주문/진열/emergency) 실로봇 검증.

---

## 4. 현재 로봇 연결 대기점

`Fleet_manager_interface.md` §4~6 계약 기준, 실로봇 전체 성공까지 필요한 서버:

```text
- PICKY MoveCommand / DockCommand action server
- 각 robot EmergencyControl service server
- COBOT ExecuteTask.action 정의 및 action server
```

없으면 주문/진열 생성·자동 task 생성까지는 확인되고, dispatch 단계에서 MOVE는 FAILED, COBOT은 ASSIGNED 유지(매 cycle 재시도)된다. 이는 Fleet Manager 오류가 아니라 로봇 실행 서버 부재를 의미한다.
