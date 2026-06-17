# 멀티로봇 통신 — Fast-DDS Discovery Server (운영 런북)

> picky1 + picky2 + 관제 PC(Fleet)를 같은 `ROS_DOMAIN_ID=25`에서 안정적으로 묶기 위한 설정.
> 작성 2026-06-16 (박서우). 관련 코드: `scripts/discovery_server.sh`, `scripts/dds_env.sh`.

## 1. 왜 필요한가

WiFi에서 기본 멀티캐스트 디스커버리는 participant가 많을수록 폭주한다. 보드 2대 + 관제
PC = 40+ participant가 서로 멀티캐스트로 전수 통보 → **nav2 lifecycle 활성 실패 · battery
발행 지연 · 2대 동시 가동 시 nav 불안정**. Fast-DDS **Discovery Server**(유니캐스트 허브)로
디스커버리 경로를 서버 경유로 바꿔 폭주를 제거한다. (데이터 전송은 그대로 peer-to-peer.)

## 2. 전제 — 네트워크

- **관제 PC가 로봇과 같은 WiFi/서브넷(192.168.1.x)에 있어야 한다.** 다른 서브넷이면 로봇
  라우터 NAT로 보드→관제 통신이 막혀(ssh만 됨) ROS2가 안 된다. 관제 PC IP = `192.168.1.73`.
- IP는 DHCP라 바뀔 수 있음 → **고정(DHCP 예약) 권장**. 바뀌면 아래 모든 곳의 IP 갱신.

## 3. 호스트별 환경 (이미 적용됨)

모든 호스트가 `ROS_DOMAIN_ID=25` + `ROS_DISCOVERY_SERVER="192.168.1.73:11811"` 를 갖는다.

| 호스트 | 설정 위치 | direnv |
|---|---|---|
| 관제 PC | `~/.bashrc` | 없음 |
| picky1 | `~/.bashrc` (스크립트는 `scripts/dds_env.sh` source) | 없음 |
| picky2 | `~/.bashrc` + `~/.profile` + `~/just_pick_it/.envrc` | **있음** |

> ⚠️ **picky2 주의:** direnv(`.envrc`)와 `.profile`이 도메인을 덮어쓸 수 있어 `.bashrc`만으로는
> 부족(과거 .profile=66, .envrc=27이 숨어 실효값이 틀어졌음). **3곳 모두 25로 통일**해야 한다.
> 셸 컨텍스트(로그인/인터랙티브/repo 디렉토리)마다 출처가 달라지니 전부 확인할 것.

## 4. 실행 순서 (반드시 서버 먼저)

```
# 1) 관제 PC — 디스커버리 서버 (상시 켜둠. 없으면 디스커버리 자체가 안 됨)
bash ~/just_pick_it/scripts/discovery_server.sh

# 2) 관제 PC — Fleet (새 터미널, .bashrc env 상속)
cd ~/just_pick_it && ./run_all.sh

# 3) picky1 보드 (ssh)
bash ~/just_pick_it/scripts/navigation/run_picky1_all.sh

# 4) picky2 보드 (ssh) — 새 셸에서 env 확인 후
echo "$ROS_DOMAIN_ID $ROS_DISCOVERY_SERVER"   # 25  192.168.1.73:11811
bash ~/just_pick_it/scripts/navigation/run_picky2_all.sh
```

## 5. 검증 (관제 PC)

```
# CLI 전체 그래프는 super client 필요(+ 데몬 무관하게):
export ROS_SUPER_CLIENT=true
ros2 topic echo --once /picky1/battery/percent     # 데이터 수신되면 크로스호스트 OK
ros2 topic echo --once /picky2/battery/percent
ros2 topic echo --once /picky1/amcl_pose           # amcl 동작 확인
```

## 6. 트러블슈팅

- **`ros2 node list`가 빈다 / `lifecycle get` "Node not found"** → 서버 경유 super client의
  그래프 표시 quirk. **데이터(topic echo)·서비스는 정상**. 검증은 `topic echo`로, 또는 보드에서
  CLI 실행. 운영 무관.
- **노드가 서로 안 보임** → 그 노드가 `ROS_DISCOVERY_SERVER` 없이 떴을 가능성(비대화형 셸이
  `.bashrc` 미로드). 런치는 `dds_env`를 source하거나(picky1) 인터랙티브 셸에서 실행(picky2).
  `tr "\0" "\n" < /proc/<pid>/environ | grep ROS_DISCOVERY_SERVER` 로 확인.
- **picky2 tmux가 옛 env(도메인)로 뜸** → 옛 tmux 서버가 옛 env 보유. `tmux kill-server` 후
  새 셸에서 재기동.
- **서버 죽으면 전체 디스커버리 멈춤**(하드 의존성). 관제 PC 부팅 시 systemd 자동기동 권장(미적용).

## 7. 진행 / 남은 과제

- [x] **2대 동시 가동 시 보드 CPU 포화로 nav goal 이 상위 로직에서 cancel되는 현상** — **해소(2026-06-17)**.
  원인은 participant 별 Fast-DDS 전송 스레드 폴링(비composed nav2 ~13 participant)으로 인한 CPU 과부하.
  **nav2 composition**(`picky1_nav.launch.py use_composition:=True`, `component_container_isolated`)으로
  nav2 11노드를 단일 프로세스(participant 1개)로 합쳐 해소. `nav2_params.yaml` 무수정. 상세는
  `docs/Fleet_manager_TODO.md` §3-A 13. (scan/odom 은 보드 로컬이라 WiFi 무관.)
- [x] **2대 AMR E2E 성공(2026-06-17)** — picky1+picky2 동시 가동, 주문 흐름 끝까지 완주.
- [ ] discovery server systemd 유저서비스 자동기동 + 관제 PC IP 고정(DHCP 예약).
- [ ] fleet→로봇 커스텀 액션(MoveCommand/DockCommand) 의 feat/amr_mj↔feature/amr_sw COBOT 연동분
  E2E 확정(주행 E2E 는 통과).
