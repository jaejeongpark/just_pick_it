# 전체 시스템 실행 런북

이 문서는 전체 통합 테스트 때 **어느 장비에서 무엇을 어떤 순서로 켜야 하는지**만 정리한다.
세부 구조 설명은 각 설계/리뷰 문서를 보고, 실제 실행 순서는 이 파일을 기준으로 맞춘다.

명령어는 특별히 적지 않으면 각 장비의 `~/just_pick_it`에서 실행한다.

## 0. 공통 전제

- ROS 2 Jazzy 환경을 사용한다.
- Fleet Manager, PICKY1, PICKY2, fake COBOT 서버는 같은 ROS discovery 환경에 붙어야 한다.
- 현재 공용 DDS 설정은 `scripts/dds_env.sh`에 있다.

```bash
cat scripts/dds_env.sh
```

`ROS_DISCOVERY_SERVER`의 IP가 관제 PC IP와 다르면 먼저 이 값을 맞춘다.
Discovery Server를 쓰는 경우 서버가 모든 노드보다 먼저 떠 있어야 한다.

## 1. 빌드 / 초기화

### 관제 PC

최초 세팅 또는 전체 재빌드:

```bash
cd ~/just_pick_it
bash scripts/setup/reset_ws.sh
```

DB만 빠르게 초기화:

```bash
cd ~/just_pick_it
bash scripts/setup/reset_demo_data.sh
```

### PICKY1 보드

```bash
cd ~/just_pick_it
bash scripts/build_tools/build_picky1.sh --clean
```

일반 install 빌드가 필요하면:

```bash
cd ~/just_pick_it
bash scripts/build_tools/build_picky1.sh --no-symlink-install --clean
```

### PICKY2 보드

```bash
cd ~/just_pick_it
bash scripts/build_tools/build_picky2.sh --clean
```

일반 install 빌드가 필요하면:

```bash
cd ~/just_pick_it
bash scripts/build_tools/build_picky2.sh --no-symlink-install --clean
```

## 2. 실로봇 전체 통합 테스트

### 2.1 관제 PC: Discovery Server

첫 번째 터미널에서 켜고 계속 둔다.

```bash
cd ~/just_pick_it
bash scripts/discovery_server.sh
```

### 2.2 PICKY1 보드: bringup / Nav2 / state

```bash
cd ~/just_pick_it
tmux kill-session -t picky1 2>/dev/null || true
bash scripts/navigation/run_picky1_all.sh
```

tmux에서 빠져나오려면 `Ctrl+b d`를 누른다. 종료는:

```bash
tmux kill-session -t picky1
```

### 2.3 PICKY2 보드: bringup / Nav2 / state

```bash
cd ~/just_pick_it
tmux kill-session -t picky2 2>/dev/null || true
bash scripts/navigation/run_picky2_all.sh
```

tmux에서 빠져나오려면 `Ctrl+b d`를 누른다. 종료는:

```bash
tmux kill-session -t picky2
```

### 2.4 COBOT 호스트: driver / camera / state manager

COBOT 호스트에서는 실제 Jetcobot driver, camera UDP sender, `ExecuteTask` action server를 띄운다.
`camera_dest_ip`, `camera_dest_port`, `robot_name`, `cobot_robot_id`, `port`는 현장 장비 값에 맞춘다.

COBOT1 예시:

```bash
cd ~/just_pick_it
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source scripts/dds_env.sh
ros2 launch picky_cobot_1 picky_cobot.launch.py \
  robot_name:=jetcobot1 \
  cobot_robot_id:=COBOT1 \
  port:=/dev/ttyJETCOBOT \
  camera_dest_ip:=<AI_PC_IP> \
  camera_dest_port:=5003 \
  camera_dest_port_2:=5004 \
  dry_run:=false
```

COBOT2는 같은 launch를 쓰되 장비 이름, action 식별자, serial port, UDP port를 COBOT2 담당 값으로 맞춘다.

```bash
cd ~/just_pick_it
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source scripts/dds_env.sh
ros2 launch picky_cobot_1 picky_cobot.launch.py \
  robot_name:=jetcobot2 \
  cobot_robot_id:=COBOT2 \
  port:=/dev/ttyJETCOBOT \
  camera_dest_ip:=<AI_PC_IP> \
  camera_dest_port:=5013 \
  camera_dest_port_2:=5014 \
  dry_run:=false
```

### 2.5 AI PC: vision / pick / display agents

COBOT 카메라 UDP를 받는 AI PC에서 실행한다.
COBOT 호스트의 `camera_dest_port`와 여기 `udp_port`가 같아야 한다.

COBOT1 pick agent:

```bash
cd ~/just_pick_it
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source scripts/dds_env.sh
ros2 launch picky_cobot_1 ibvs_nn_pick_agent.launch.py \
  robot_name:=jetcobot1 \
  udp_port:=5003
```

COBOT1 display agent:

```bash
cd ~/just_pick_it
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source scripts/dds_env.sh
ros2 launch picky_cobot_1 display_agent.launch.py \
  robot_name:=jetcobot1 \
  udp_port:=5003 \
  with_detection:=false
```

COBOT2도 같은 방식으로 `robot_name`, `udp_port`를 COBOT2 값으로 맞춘다.
한 UDP 포트를 두 launch가 동시에 bind하면 충돌하므로, pick agent가 detection을 이미 띄운 상태에서는 display agent의 `with_detection:=false`를 사용한다.

### 2.6 관제 PC: Fleet Manager + Web Gateway

새 터미널에서 실행한다.

```bash
cd ~/just_pick_it
bash scripts/setup/reset_demo_data.sh
bash scripts/runtime/run_all.sh
```

브라우저:

```text
Admin UI    : http://localhost:8000/admin
Customer UI : http://localhost:8000/customer
Fleet API   : http://localhost:8100
```

## 3. 실로봇 PICKY + fake COBOT 통합 테스트

COBOT 실제 하드웨어 없이 PICKY1/PICKY2 주행과 Fleet 흐름만 볼 때 사용한다.
이 시나리오에서는 실제 COBOT launch를 켜지 않고 fake COBOT 서버만 켠다.

### 3.1 관제 PC: Discovery Server

```bash
cd ~/just_pick_it
bash scripts/discovery_server.sh
```

### 3.2 PICKY1 / PICKY2 보드

각 보드에서 실행한다.

```bash
cd ~/just_pick_it
bash scripts/navigation/run_picky1_all.sh
```

```bash
cd ~/just_pick_it
bash scripts/navigation/run_picky2_all.sh
```

### 3.3 관제 PC: fake COBOT 서버

실제 PICKY1/PICKY2를 쓰는 테스트에서는 fake PICKY action server를 꺼야 한다.
안 그러면 `/picky1/move_command`, `/picky2/move_command` action server가 겹칠 수 있다.

```bash
cd ~/just_pick_it
DEMO_MOCK_PICKY_IDS= DEMO_MOCK_COBOT_IDS=1,2 bash scripts/demo/run_fake_robot_servers.sh
```

### 3.4 관제 PC: Fleet Manager + Web Gateway

```bash
cd ~/just_pick_it
bash scripts/setup/reset_demo_data.sh
bash scripts/runtime/run_all.sh
```

## 4. fake PICKY + 실 COBOT 통합 테스트

PICKY 주행 하드웨어 없이 COBOT 실제 동작과 Fleet/Web 흐름을 볼 때 사용한다.
이 시나리오에서는 실제 PICKY launch를 켜지 않고 fake PICKY 서버만 켠다.

### 4.1 관제 PC: Discovery Server

```bash
cd ~/just_pick_it
bash scripts/discovery_server.sh
```

### 4.2 COBOT 호스트: driver / camera / state manager

COBOT1 예시:

```bash
cd ~/just_pick_it
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source scripts/dds_env.sh
ros2 launch picky_cobot_1 picky_cobot.launch.py \
  robot_name:=jetcobot1 \
  cobot_robot_id:=COBOT1 \
  port:=/dev/ttyJETCOBOT \
  camera_dest_ip:=<AI_PC_IP> \
  camera_dest_port:=5003 \
  camera_dest_port_2:=5004 \
  dry_run:=false
```

COBOT2는 장비 값에 맞춰 `robot_name`, `cobot_robot_id`, `port`, UDP port를 바꿔 실행한다.

### 4.3 AI PC: vision / pick / display agents

COBOT1 pick agent:

```bash
cd ~/just_pick_it
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source scripts/dds_env.sh
ros2 launch picky_cobot_1 ibvs_nn_pick_agent.launch.py \
  robot_name:=jetcobot1 \
  udp_port:=5003
```

COBOT1 display agent:

```bash
cd ~/just_pick_it
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source scripts/dds_env.sh
ros2 launch picky_cobot_1 display_agent.launch.py \
  robot_name:=jetcobot1 \
  udp_port:=5003 \
  with_detection:=false
```

COBOT2도 같은 방식으로 `robot_name`, `udp_port`를 COBOT2 값으로 맞춘다.

### 4.4 관제 PC: fake PICKY 서버

실제 COBOT을 쓰는 테스트에서는 fake COBOT action server를 꺼야 한다.
안 그러면 `/cobot1/execute_task`, `/cobot2/execute_task` action server가 겹칠 수 있다.

```bash
cd ~/just_pick_it
DEMO_MOCK_PICKY_IDS=1,2 DEMO_MOCK_COBOT_IDS= bash scripts/demo/run_fake_robot_servers.sh
```

### 4.5 관제 PC: Fleet Manager + Web Gateway

```bash
cd ~/just_pick_it
bash scripts/setup/reset_demo_data.sh
bash scripts/runtime/run_all.sh
```

## 5. 전체 fake 테스트

실로봇 없이 Fleet/Web 흐름만 볼 때 사용한다.

터미널 1:

```bash
cd ~/just_pick_it
USE_DDS=0 DEMO_MOCK_PICKY_IDS=1,2 DEMO_MOCK_COBOT_IDS=1,2 bash scripts/demo/run_fake_robot_servers.sh
```

터미널 2:

```bash
cd ~/just_pick_it
bash scripts/setup/reset_demo_data.sh
USE_DDS=0 bash scripts/runtime/run_all.sh
```

## 6. 실행 확인

관제 PC에서 확인한다.

```bash
cd ~/just_pick_it
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source scripts/dds_env.sh
export ROS_SUPER_CLIENT=true
ros2 daemon stop
```

PICKY action server 확인:

```bash
ros2 action info /picky1/move_command
ros2 action info /picky2/move_command
ros2 action info /picky1/dock_command
ros2 action info /picky2/dock_command
```

Nav2 action server 확인:

```bash
ros2 action info /picky1/navigate_to_pose
ros2 action info /picky2/navigate_through_poses
ros2 action info /picky2/navigate_to_pose
```

센서 주기 확인:

```bash
timeout -s INT 10s ros2 topic hz /picky1/odom
timeout -s INT 10s ros2 topic hz /picky2/odom
timeout -s INT 10s ros2 topic hz /picky1/scan
timeout -s INT 10s ros2 topic hz /picky2/scan
```

COBOT action server 확인:

```bash
ros2 action info /cobot1/execute_task
ros2 action info /cobot2/execute_task
```

실제 COBOT 상태 topic 확인:

```bash
timeout -s INT 10s ros2 topic echo /cobot1/cobot_state
timeout -s INT 10s ros2 topic echo /cobot2/cobot_state
```

## 7. 디버그 기록

실행 전에 관제 PC에서 켜고, 문제가 재현되면 `Ctrl+C`로 저장한다.

PICKY1/PICKY2 pair light 기록:

```bash
cd ~/just_pick_it
PICKY_PAIR_DEBUG_RECORD_CAMERA=0 bash scripts/navigation/record_picky_pair_debug.sh
```

PICKY2 도킹 이미지까지 기록:

```bash
cd ~/just_pick_it
PICKY2_DEBUG_RECORD_CAMERA=1 bash scripts/navigation/record_picky2_debug.sh
```

카메라/이미지 토픽은 네트워크와 저장 부하가 크다. 도킹 문제를 볼 때만 켠다.

## 8. 종료 순서

각 로봇 보드:

```bash
tmux kill-session -t picky1 2>/dev/null || true
tmux kill-session -t picky2 2>/dev/null || true
```

관제 PC:

```text
run_all.sh 터미널      : Ctrl+C
fake_robot_servers    : Ctrl+C
discovery_server      : Ctrl+C
debug recorder        : Ctrl+C
```

COBOT 호스트 / AI PC:

```text
picky_cobot.launch.py             : Ctrl+C
ibvs_nn_pick_agent.launch.py      : Ctrl+C
display_agent.launch.py           : Ctrl+C
```

## 9. 자주 틀리는 지점

- Discovery Server를 쓰는 실행에서는 서버가 먼저 떠 있어야 한다.
- `scripts/dds_env.sh`의 IP가 관제 PC IP와 다르면 로봇/Fleet/fake server가 서로 못 본다.
- 실제 PICKY와 fake server를 같이 쓸 때는 `DEMO_MOCK_PICKY_IDS=`처럼 PICKY fake 목록을 비운다.
- fake 없이 전체 실로봇 테스트를 할 때는 `scripts/demo/run_fake_robot_servers.sh`를 켜지 않는다.
- 실제 COBOT과 fake server를 같이 쓸 때는 `DEMO_MOCK_COBOT_IDS=`처럼 COBOT fake 목록을 비운다.
- COBOT host의 `camera_dest_port`와 AI PC agent의 `udp_port`가 다르면 vision/pick/display가 연결되지 않는다.
- pick agent와 display agent가 같은 UDP port에서 detection을 동시에 띄우면 bind 충돌이 날 수 있다. 같이 켤 때는 display 쪽 `with_detection:=false`를 쓴다.
- `ros2 node list`가 비어 보여도 Discovery Server 모드에서는 CLI graph 표시 문제일 수 있다. `ROS_SUPER_CLIENT=true`와 `ros2 daemon stop` 후 다시 확인한다.
- 도킹 debug image 기록은 필요할 때만 켠다.
