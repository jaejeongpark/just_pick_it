# Fleet Manager 작업 현황 (담당 · 결정 · TODO)

갱신: 2026-06-01. 설계·동작은 `docs/Fleet_manager.md`, 인터페이스 계약은 `docs/Fleet_manager_interface.md`.

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

### 3-A. 박서우 완료 작업 (단계별)

내가 맡은 영역(FleetRepository / TrafficManager / RobotStateMonitor / State Manager)에서
작업한 순서다. 각 단계는 dev 에 작업별로 커밋되어 있고, 모두 **완료(박서우)** 다.

1. **로봇 텔레메트리 ROS2 통일 (S1)** — ✅ 완료 (박서우)
   picky_state / battery percent / amcl_pose 를 토픽으로 구독해 1Hz coalesce 로 DB 반영.
   기존 HTTP 보고 경로를 폐기하고 `robot_status` 오염(텔레메트리가 task 상태를 덮어쓰던 것)을 제거.
2. **State Manager 죽은 코드 정리** — ✅ 완료 (박서우)
   HTTP 보고 / TF / battery 관련 미사용 코드 제거.
3. **배터리 게이팅** — ✅ 완료 (박서우)
   배터리가 임계값 초과 구간에 진입할 때 robot 별 1회만 `handle_battery_update` 호출(충전 완료 트리거 중복 방지).
4. **진열 재고 반영 정책 정리 (D4)** — ✅ 완료 (박서우)
   미사용 legacy 완료 API 제거. 진열 재고는 계획값(`display_item.stock_delta`) 기준으로 반영.
5. **재시작 복구 A'' (R1)** — ✅ 완료 (박서우)
   재시작 시 로봇 현재 위치 기준으로 점유만 복원 + 텔레메트리로 완료 재동기 + 타임아웃 처리.
   설계는 `Fleet_manager.md` §5.
6. **실로봇 주행 통합 (2026-06-01, picky1 단일로봇)** — ✅ 완료 (박서우)
   주문 흐름을 실주행으로 끝까지 돌리며 발견·수정. 세부는 아래 6단계.
   1. **맵 재생성** — 좌측벽 과포착으로 도크 출구가 막혀 재맵핑. 좌하단 코너를 원점(`[0,0,0]`)으로
      정렬하고 가짜 기둥/노이즈 정리. SLAM launch(`picky1_slam.launch.py`) 추가.
   2. **Nav2 namespace 적용** — `pinky_amr_1/picky1_nav.launch.py` 에서 RewrittenYaml 로 `/picky1`
      노드에 nav2_params 적용(기존 XML 은 namespace 에서 params 가 안 먹어 controller 가 기본
      DWB(critics 없음)로 죽던 버그). 소형 아레나용 nav 튜닝(inflation 0.03,
      use_cost_regulated / use_collision_detection off 로 RPP 크롤링 해소).
   3. **zone / traffic 정합** — zone 좌표를 맵 좌하단 원점 레이아웃에 맞추고 로봇 홈 갱신.
      `TRAFFIC_*` / `CHARGING_DOCK_*` zone 을 DB 에 시드(경유지 리스트 전송용 pose).
   4. **Fleet MoveCommand 경유지 전송** — MoveCommand 에 TrafficManager 예약 전체 경로(경유지
      리스트)를 전송하고, Action 피드백 인덱스(+1)로 traffic 점유 단계를 해제.
   5. **PICKY2 오배정 수정 + gateway 견고화** — config `robot_ids` 외 로봇은 배정 후보에서 제외
      (`_robot_available`). DB stale 배터리 때문에 PICKY2 로 잘못 배정되던 문제 해결. gateway action
      클라이언트에 reentrant 콜백그룹 + 기동 prewarm + wait timeout 2→8s(크로스머신 첫 주문
      discovery 실패 방지). 배터리 작업배정/충전복귀 임계 40→30.
   6. **State Manager 도착·도킹 정리** — 배터리>30% 면 `CHARGING`→`STANDBY`(상태만 전이, 이동 없음,
      Fleet 배정 게이트 충족). undock 은 `_at_dock`+`STANDBY` 일 때만(물리 도크 여부를 picky_state
      와 분리). 목적지 도착 후 가장 가까운 90°(축 정렬)로 정지 회전(zone theta 미사용, 회전 최소).
      `move_to_goal` 은 위치 도착 전담(경유지 통과·회전 제거), nav_timeout 120s.
7. **실주행 부분 검증 (2026-06-01)** — ✅ 완료 (박서우)
   06-01 주행 중 확인된 항목: `/picky1/amcl_pose` namespaced 발행 + 텔레메트리가 들어와도
   `robot_status` 가 task 전이로만 바뀌고 IDLE 유지, battery 게이팅(30% 임계) 실측.
   (E2E 완주·R1 reconcile 실검증은 배터리 충전 후로 잔존 — §3-D)

> 박서우 미완(다음 기동·배터리 충전 후): 주문 E2E 실주행 완주, R1 재시작 복구 실동작, `_at_dock` 부팅 가정 견고화(낮음). 상세는 §3-D.

### 3-B. 이명제 완료 작업

| 항목 | 내용 |
|---|---|
| S2 | COBOT 디스패치 영구 스킵 버그 수정(매 cycle 재시도, 경고만 rate-limit) |
| R2 | flow 종료 시 in-memory 임시 메모리 정리 |
| D5 | 진열 흐름 코드 반영(enum/API/TaskManager/Web/테스트) |
| — | Fleet 측 COBOT `send_cobot_task` 연결 준비 + STOWING_ARM 감지 -> `preplan_after_cobot_stowing` 호출 |

### 3-C. 공동 완료 (문서)

| 항목 | 내용 |
|---|---|
| C1/C2 | Fleet Manager 문서 3종으로 통합·최신화 (이 문서 포함). 발표용 구성요소 설명은 `Fleet_manager_interface.md` §0 으로 통합(별도 `Fleet_manager_components.md` 삭제) |
| C3 | D5(진열 흐름) 설계 문서 반영 |

C3 수정 문서:
- `docs/Fleet_manager.md` §4.3 진열 흐름 — 새 task 시퀀스, `stock_qty` 트리거를 `DISPLAY_PLACE`로
- `docs/Fleet_manager_interface.md` §4 COBOT 명령 task / §5 PICKY 계약 / §6 COBOT 계약표·STOWING_ARM 선계획표 / §7 MOVING_STATES / §8 시나리오
- `docs/Fleet_manager_TODO.md` D5 결정·이 항목 추가
- `src/just_pick_it/pinky_amr_1/docs/state_manager.md` move_command 트리거 task 목록
- `docs/ros2_driving_beginner_guide.md` 주행 task 표 / COBOT task 목록 / Milestone 8

### 3-D. 남은 작업

우선순위 순. 박서우는 "다음 기동(배터리 충전 후)" 항목이 최우선이다.

**박서우 — 다음 기동(배터리 충전 후) 최우선**
- [ ] 주문 E2E 실주행 완주: `MOVE_TO_PRODUCT` -> ... -> `DOCK_IN`/`CHARGING` 까지 picky1 단독으로 끝까지(§3-A 6단계 결과 실증). COBOT 차례는 수동 완료(`Order_Scenario_Test_Guide.md` §3.2).
- [ ] R1 재시작 복구 실동작 확인: RUNNING 중 Fleet 재시작 -> 현재 위치 점유 복원 + 텔레메트리 완료 재동기 + 타임아웃.

**박서우 — 그 외**
- [ ] PICKY2 멀티로봇 통합: Nav2 namespace화(현재 picky2 미해결) + zone 단일점유 기반 2대 충돌회피 실검증. (picky1 단독 E2E 통과 후 착수)
- [ ] pose TF fallback 도입 여부 — amcl_pose가 안 나오거나 갱신이 너무 드물면 robot별 namespaced TF buffer로. (실 테스트 후 결정, 현재 보류)
- [ ] `_at_dock` 부팅 가정 견고화(낮음): 부팅 시 물리 도크 여부 추정 개선.
- [ ] Q: `fleet_repository`(상태 전이/트랜잭션) 단위 테스트 추가. recovery/traffic 테스트는 `test/test_recovery.py`에 존재.

**이명제**
- [ ] `just_pick_it_interfaces` COBOT action(`ExecuteTask`) 메시지 최종 정의 반영.
- [ ] COBOT State Manager가 `/{cobot_ns}/execute_task` action server를 실제 패키지에서 제공하는지 통합 확인.
- [ ] Web LLM parser 실제 구현(`web/app/services/llm_client.py`).
- [ ] Q: `task_manager` 추가 흐름 테스트, `fleet_api_server` 엔드포인트 테스트.

**공동 / 회색지대**
- [ ] 전체 E2E(주문/진열/emergency) 2대 실로봇 검증. (박서우 picky1 E2E + 이명제 COBOT 연동 후)
- [ ] B3: emergency-stop이 HTTP 스레드에서 rclpy를 직접 호출(`trigger_emergency_stop` -> gateway). 통합계획 2.3/3.4대로 executor로 위임 검토.
- [ ] 관리자 엔드포인트 인증/인가.
- [ ] WebSocket push를 전체 스냅샷 -> 변경분(delta) 전환 검토.

---

## 4. 현재 로봇 연결 대기점

`Fleet_manager_interface.md` §4~6 계약 기준, 실로봇 전체 성공까지 필요한 서버:

```text
- PICKY MoveCommand / DockCommand action server
- 각 robot EmergencyControl service server
- COBOT ExecuteTask.action 정의 및 action server
```

없으면 주문/진열 생성·자동 task 생성까지는 확인되고, dispatch 단계에서 MOVE는 FAILED, COBOT은 ASSIGNED 유지(매 cycle 재시도)된다. 이는 Fleet Manager 오류가 아니라 로봇 실행 서버 부재를 의미한다.
