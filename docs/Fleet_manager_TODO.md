# Fleet Manager 작업 현황 (담당 · 결정 · TODO)

갱신: 2026-06-16. 설계·동작은 `docs/Fleet_manager.md`, 인터페이스 계약은 `docs/Fleet_manager_interface.md`.

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
8. **실로봇 주행 정밀도 튜닝 + 도착 정지 자세 정책 (2026-06-09~10, picky1 단독)** — ✅ 완료 (박서우)
   반복 주행 테스트로 발견·수정. 각 변경의 근거와 성과를 함께 남긴다.

   1. **맵 프레임 정합(origin)** — "목적지 도착 전 도착 오판 + 벽 충돌"의 근본 원인이 `map.yaml`
      `origin=[0,0,0]`. origin 은 SLAM 시작 오프셋을 반영하는 고정 기하값인데 [0,0,0]으로 둬 맵
      전체가 약 (0.06,0.12) 밀려 map 프레임이 arena 절대좌표와 어긋남(실측: (0.28,0.40)에 둔 로봇이
      amcl (0.352,0.505)). → **origin [-0.08,-0.12] 복원**. 성과: 조기 도착·벽 충돌 해소.
      (odom 은 1m=1m 로 정상임을 실측해 odom 스케일 가설을 배제 → 원인을 map 프레임으로 한정)
   2. **RPP 코너 cut** — 좁은 아레나(2x1m)에 lookahead 과대(저속 실효 0.3m)로 코너를 질러 벽 충돌.
      → **lookahead_dist 0.6→0.25, min 0.3→0.15, max 0.9→0.45**. 성과: 코너 cut 제거(경유지 추종 강화).
   3. **TRAFFIC 노드 수직 정합** — TRAFFIC_T/B x 가 같은 열 PRODUCT_ZONE 과 어긋나(0.70/1.05/1.40)
      복도→상품존 진입에 횡성분 발생 → 진입 heading 기욺. → **0.64/1.06/1.48 정렬**(이후 column-2 는
      매대 간섭 여유로 1.05). 성과: 순수 수직 진입 → 도착 자세 정확.
   4. **정밀접근 정지거리** — 매대 정중앙보다 5cm 짧게 정지. 원인은 `move_to_goal` 자체
      `xy_goal_tolerance`(0.05, Nav2 의 0.01 과 별개). → **0.02 로 축소 + 근거리(<3cm) 조향 가드**
      (`atan2(dy,dx)` 노이즈로 인한 목표 직전 wobble 방지). 성과: 5cm→2cm 접근, 떨림 없음.
   5. **Nav2 abort 완화 + CPU 절감** — `compute_path_to_pose` ack 타임아웃으로 navigation abort.
      원인은 보드 CPU 과부하(load avg 20+)로 planner_server 가 20ms 안에 goal 응답 못 함. →
      **default_server_timeout 20→100ms** + (origin 정합으로 불필요해진) **amcl update_min_d
      0.05→0.10**(amcl 갱신 빈도↓ = CPU 절반). 성과: abort 완화 + 보드 부하 경감.
   6. **도착 정지 자세 task별 정책** — 수평 진입(standby→product) 시 nearest-90 스냅이 매대를
      바라보고 정지하던 문제. → **task_type 별 고정**: MOVE_TO_PRODUCT/DISPLAY=법선(±y 중 회전 적은
      쪽), MOVE_TO_PICKUP=-x, MOVE_TO_STOCK=+x, RETURN_HOME(standby)=+y, 그 외 nearest-90. 정책은
      `state_manager`(STOP_MODE_BY_TASK), 회전 계산은 `move_to_goal`(_rotate_to_stop_pose)가 도착
      heading 으로 수행. 성과: 진입 방향과 무관하게 올바른 사이드 주차 자세.
   7. **AMR/COBOT 분리 테스트 지원** — `scripts/demo/fake_robot_servers.py` 에 `DEMO_MOCK_PICKY` /
      `DEMO_MOCK_COBOT` 플래그 추가. 실 AMR 주행 중 cobot 작업만 자동 처리(`DEMO_MOCK_PICKY=0`),
      반대로 실 cobot 테스트는 AMR 만 mock(`DEMO_MOCK_COBOT=0`). 성과: 한쪽 실로봇+한쪽 자동 혼합 테스트.

   같은 기간 반영: 콜드 스타트 시 양 충전 도크 점유 초기화, battery 텔레메트리 20s+ 미수신 시 0%로
   처리해 offline 로봇 배정 제외(RobotStateMonitor).
9. **Reverse Docking 실차 디버깅 + 견고화 (2026-06-10~15, picky1)** — ✅ **완성** (박서우). 두 마커 localization 으로 좁은 도크 안정 통과(x ~3~5mm, yaw <1°). 잔여는 보드 CPU 경량화·재캘리브(낮음).

   1. **카메라/캘리브레이션 직접화** — reverse_docking 이 ROS Image pub/sub 대신 Picamera2 를 직접
      열고(도킹 중에만) `camera_calibration.yaml` 을 직접 로드. udp_image_sender 는 UDP 전용 원복.
   2. **카메라 180° flip** — 카메라가 거꾸로 장착 + 캘리브레이션도 flip 기준이라, 검출 전 `cv2.flip(-1)`
      적용(`apriltag_detector_real` 와 동일). 안 하면 마커 횡/yaw·라인 좌우가 반전.
   3. **마커 dict/size 정정** — 도크 마커는 AprilTag `DICT_APRILTAG_36h11`, 한 변 `0.05m`(기존 ArUco
      4x4_50·0.10 오설정 → acquire timeout/깊이 오차였음).
   4. **3줄 주차선 대응** — 주차선이 3줄(왼\|중앙(공유)\|오)이라 dock1=왼+중앙, dock2=중앙+오. 컬럼
      히스토그램으로 라인 분리→x순→도크별 채널 2줄 선택. 2줄(가까움)/3줄(멀어짐) 전환 일관.
   5. **마커 상실 복구** — 정렬하느라 마커가 시야 벗어나는 건 정상 → 실패 처리 대신 라인으로 법선 복귀.
   6. **정렬 게인 하향** — 후진 진동 완화(marker 0.6, lane_lat 0.002/yaw 0.6, max_angular_vel 0.25).

   **(2026-06-11) 제어 아키텍처 전면 개정 — E2E 도킹 성공.** 위 1~6(검출 파이프라인) 후, 제어가
   발산/드리프트하던 근본원인이 **rvec yaw(psi)의 구조적 노이즈**(정지 상태 ±5~8°, pose-flip
   ambiguity)임을 확인 → "라인+실시간 마커 서보" 설계를 **"마커 1회 측정 + odom 실행"** 으로 뒤집음.
   - **횡:** 실시간 robot_x 추적 폐기 → **마커 1회 측정 Δx + 부드러운 후진 arc**(odom 으로 |Δx| 이동, 방향 자동반전 안전장치).
   - **측정 전 똑바로 세우기:** 각 패스에서 `psi→0` 후 측정 → `Δx≈−tx·scale`(저노이즈), phantom 제거·수렴.
   - **헤딩:** 후진 직전 odom yaw 를 θ_ref(법선) 앵커 → **후진 내내 odom 유지**(`yaw_drift` ±0.2°). psi 미사용.
   - **깊이/횡 스케일 분리:** depth_scale 1.36(원거리 dock) / lateral_scale 1.48(가까운 측정). 라인 횡조향은 기본 off(`use_lane_steering`).
   - **수치 정정:** marker_world_x 0.07→0.11, marker_world_y 0.655→0.635, cam_fwd 0.05→0.060, marker_size→0.05, dict→36h11, depth_scale 도입.
   - 상세 기록·표: `docs/Reverse_Docking_Design.md` §7.

   **(2026-06-15) 정밀도 완성 — fx 근본원인 + 두 마커.** §7 후에도 "같은 x 인데 깊이마다 x 가
   달라짐 + 후진 비뚤음 + 끝없는 부호 튜닝"이 남아, 정지 진단툴(`scripts/demo/marker_pose_check.py`)
   로 두 숨은 원인을 숫자로 격리해 완성:
   - **fx 오류:** 캘리브 fx(985)가 실제 도킹 영상모드보다 **~1.48배 작음**(센서 crop 불일치). `tx`(횡)는
     fx 무관(정확)이나 `tz`(깊이)는 fx 비례라 짧게 나옴 → `depth/lateral_scale` 은 그걸 덮던 fudge였음.
     `lateral_scale` 1.48→1.0(정렬 후 x 1.8cm 과보정 제거).
   - **두 마커 헤딩:** 단일 평면 마커 yaw 는 pose-flip 으로 구조적 병목(두 마커가 4° 불일치). 도크 마커
     2개의 **translation 만**(rvec 미사용) 강체정합해 로봇(x,y)·yaw 를 ambiguity 없이(σ±0.05°) 1회 측정.
   - **6단계 시퀀스:** ①주마커 정렬 ②법선 yaw ③10cm 후진 ④쌍마커 검출+pose 측정(가까우면 2cm씩
     추가후진 재시도) ⑤odom 정밀 곡선 이동(dock_x, approach_y=0.18, 법선; 횡델타 0.9배 미세보정)
     ⑥dock_y 까지 직진 후진. `_measure_two_marker_pose` + `_curve_to_pose_odom` 신설.
   - **결과(실측):** lateral ~3~5mm(0.11), yaw <1°, 깊이의존 편향 제거, 좁은 도크 안정 통과. 상세·비교표는
     `docs/Reverse_Docking_Design.md` §8, `docs/Reverse_Docking_Summary.md` ④.

   **남은 미세조정(낮음):** ① **보드 CPU 과부하**(도킹 비전이 Pi 포화 → 경량화/주기↓) ② 도킹 영상모드
   그대로 재캘리브하면 `fx_scale`/`*_scale` fudge 를 1.0 으로 제거 가능(perception 영역) ③ N회 반복 정량화.

10. **주행 견고성 (2026-06-10)** — ✅ 완료 (박서우)
    - **벽 충돌 안전망**: nav2 `use_collision_detection: true` + 짧은 시간지평(local costmap 기반,
      amcl 무관). odom 누적으로 벽 향할 때 임박 충돌만 정지(유연성 유지).
    - **예약 레이스 수정**: TrafficManager `notify_state` 가 갓 만든 경로 예약을 STANDBY 텔레메트리에
      지워(`예약 task=None`) goal 이 cancel/abort 되고 주문이 빈 메시지로 실패하던 것 → '이동/점유에서
      빠져나올 때(prev active)만' 해제하도록 수정. test 갱신+레이스 케이스 추가(62 pass).

11. **Reverse Docking 정밀도 완성 + picky1 코드 정리 (2026-06-15)** — ✅ 완료 (박서우)
    - 두-마커 translation localization + odom 정밀 곡선 이동(6단계)으로 좁은 도크 안정 통과
      (lateral ~3~5mm, yaw <1°). 근본원인=카메라 fx ~1.48배 작음(scale들은 fudge였음). 상세
      `docs/Reverse_Docking_Design.md` §8.
    - 6단계 리팩터로 죽은 코드 정리(reverse_docking.py 1419→1115줄, lane 서브시스템·옛 정렬경로
      제거), state_manager/move_to_goal 미사용 함수 제거. 동작 변경 없음.

12. **멀티로봇 통신 — Fast-DDS Discovery Server (2026-06-16)** — 🔶 진행 (박서우)
    2대 동시 bringup 시 WiFi 멀티캐스트 디스커버리 폭주로 nav 실패·battery 누락 → Discovery
    Server(유니캐스트)로 전환. 관제 PC가 로봇과 다른 서브넷이던 것(NAT로 보드→관제 불통)을
    같은 WiFi(192.168.1.73)로 정합. 전 호스트 `ROS_DOMAIN_ID=25`+`ROS_DISCOVERY_SERVER` 통일
    (picky2는 .bashrc/.profile/.envrc direnv 3곳에 숨은 도메인 덮어쓰기 27/66 발견·정정).
    `scripts/discovery_server.sh`, `scripts/dds_env.sh`, headless 스크립트 dds_env source.
    **검증**: picky1+picky2 양쪽 battery/amcl_pose/scan/odom 크로스호스트 OK. 운영 런북:
    `docs/Multi_Robot_Discovery_Server.md`.
    **남은 것**: ① ✅ 2대 동시 보드 CPU 포화 → **nav2 composition 으로 해소**(§3-A 13)
    ② discovery server systemd 자동기동+IP 고정 ③ fleet→로봇 커스텀 액션 호환은 **2대 E2E 로 확인**(§3-A 13).

13. **nav2 composition 경량화 + 2대 AMR E2E 성공 (2026-06-17)** — ✅ 완료 (박서우)
    2대 동시 가동 시 보드 CPU 포화로 nav goal 이 상위 로직에서 cancel 되던 과부하를 nav2 **composition**
    으로 해소. `picky1_nav.launch.py` 에 `use_composition:=True` 경로 추가 — nav2 노드 11개를 단일
    `component_container_isolated` 프로세스로 통합(**DDS participant 약 13→1**, participant 별 Fast-DDS
    전송 스레드 폴링 제거). `nav2_params.yaml` 은 무수정(amcl/costmap 주기 그대로 — 과거 costmap 주기↓가
    amcl 추정 깨던 회귀를 피하려 composition 만 단독 적용).
    - **핵심 버그 격리**: composed 컨테이너 Node 에 param 파일(`--params-file`)·`/tf` remap 을 안 주면
      controller 내부 costmap 서브노드가 params 를 못 받아 기본값(`global_frame=map`)으로 떠
      `base_link->map` TF timeout 으로 controller 활성 실패 abort. nav2_bringup 표준대로 컨테이너에
      `parameters=[configured_params]`+`/tf` remap 부여로 해결. (과거 "ARM 에서 composition 실패"의
      진짜 원인 — ARM 무관, **컨테이너 param 누락**이었음. 컴포넌트 11개 전부 정상 등록 확인됨.)
    - 결과: nav2 11프로세스 → 1프로세스, load 콜드스타트 5 → 정상 ~3(4코어). amcl 1회 오정위(zone4
      precision 램밍)는 **일회성 transient**(모드·파라미터 무변경 재실행서 재현 안 됨 — 초기화 운빨).
    - **2대 AMR E2E 성공**: picky1+picky2 동시, Discovery Server + composition 으로 nav 안정,
      주문 흐름 끝까지 완주.

> 박서우 미완(다음 기동): R1 재시작 복구 실동작, `_at_dock` 부팅 가정 견고화(낮음), PICKY2 Nav2 namespace화. **Reverse Docking 완성(§3-A 9, 11) · nav2 composition + 2대 E2E 완료(§3-A 13).** 상세는 §3-D.

### 3-B. 이명제 완료 작업

| 항목 | 내용 |
|---|---|
| S2 | COBOT 디스패치 영구 스킵 버그 수정(매 cycle 재시도, 경고만 rate-limit) |
| R2 | flow 종료 시 in-memory 임시 메모리 정리 |
| D5 | 진열 흐름 코드 반영(enum/API/TaskManager/Web/테스트) |
| — | Fleet 측 COBOT `send_cobot_task` 연결 준비 + STOWING_ARM 감지 -> `preplan_after_cobot_stowing` 호출 |
| — | (2026-06-17) 진열태스크 생성순서 수정: 주문 생성 시 자동진열 즉시 queue 조건 단순화(`_has_active_auto_display_context` → `has_appendable_display_item` 만) |

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
- [x] 주문 E2E 실주행 완주 (2026-06-17): picky1+picky2 **2대 동시 E2E 성공**(§3-A 13). COBOT 차례는 수동(`Order_Scenario_Test_Guide.md` §3.2).
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
- [~] 전체 E2E(주문/진열/emergency) 2대 실로봇 검증 — **AMR 2대 주행 E2E 성공(2026-06-17, §3-A 13)**. COBOT 연동분 잔여(이명제).
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
