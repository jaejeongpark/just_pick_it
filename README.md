# just_pick_it

ROS2 기반 자율 시스템 워크스페이스. **Pinky Pro** 모바일 로봇과 **myCobot 280** 협동 로봇 암을 함께 활용하는 자율화 프로젝트를 위한 통합 개발 환경입니다.

> 협업 git 사용 방법은 **[인수인계서.md](인수인계서.md)** 를 참고하세요.

## 워크스페이스 구조

```
just_pick_it/
├── src/
│   ├── pinky_pro/          # Pinky Pro 로봇 패키지
│   ├── sllidar_ros2/       # SLAMTEC LiDAR ROS2 드라이버 (외부 BSD 라이선스)
│   ├── mycobot_ros2/       # myCobot 280 패키지
│   └── just_pick_it/       # 프로젝트 통합 패키지 (인식·매니퓰레이션·AMR·시뮬레이션)
├── scripts/
│   ├── mapping/            # 지도 생성 자동화 스크립트
│   └── navigation/         # 자율 주행 자동화 스크립트
├── web/                    # FastAPI Web Gateway 및 관제 UI (별도 README 참고)
├── db/                     # PostgreSQL schema 및 seed 데이터 (별도 README 참고)
├── docs/                   # 요구사항·시스템 아키텍처·시나리오 등 설계 문서 PDF
├── reset_ws.sh             # 워크스페이스 전체 재세팅
├── reset_demo_data.sh      # 데모 DB 데이터만 seed 기준으로 초기화
├── run_all.sh              # Fleet Manager + Web Gateway 통합 실행
├── build/                  # colcon 빌드 출력 (gitignore)
├── install/                # colcon 설치 공간 (gitignore)
└── log/                    # 실행 로그 (gitignore)
```

---

## 패키지 구성

### src/pinky_pro

Pinky Pro 모바일 로봇 전용 ROS2 패키지 모음입니다. 외부 제공 패키지이며, 본 프로젝트에서는 AMR 하드웨어 bringup, 로봇 모델, Gazebo/Nav2 참고 구현으로 사용합니다.

- 원본/출처: Pinky Pro ROS2 packages (`pinklab-art/pinky_pro` 기반)
- 라이선스: Apache License 2.0 (`src/pinky_pro/LICENSE` 유지)
- 본 프로젝트의 직접 구현 범위가 아니라 Pinky Pro 하드웨어/시뮬레이션 adapter 의존성입니다.
- 직접 구현 범위는 `src/just_pick_it/pinky_amr_2`의 AMR2 safety, custom navigation, task runner, planner/tracker 쪽으로 구분합니다.

| 패키지 | 설명 |
|--------|------|
| `pinky_bringup` | 실제 로봇 구동 (Dynamixel, 배터리 퍼블리셔) |
| `pinky_description` | URDF/xacro 로봇 모델 |
| `pinky_gz_sim` | Gazebo 시뮬레이션 환경 및 world |
| `pinky_navigation` | Nav2 기반 SLAM·자율 주행 launch |
| `pinky_interfaces` | 커스텀 ROS2 srv 인터페이스 |
| `pinky_emotion` | LCD 감정 표현 |
| `pinky_lamp_control` | 램프 제어 Gazebo 플러그인 |
| `pinky_led` | LED 제어 |
| `pinky_imu_bno055` | IMU 드라이버 |
| `pinky_sensor_adc` | ADC 센서 드라이버 |


### src/sllidar_ros2

SLAMTEC LiDAR를 ROS2 `sensor_msgs/msg/LaserScan` 토픽으로 publish하는 외부 드라이버 패키지입니다.
AMR2 실로봇 구동에서는 `pinky_bringup` launch가 이 패키지를 include하여 LiDAR `scan` 토픽을 생성하고, `pinky_amr_2`의 장애물 정지 및 custom navigation 입력으로 사용합니다.

- 원본: SLAMTEC `sllidar_ros2` / RPLIDAR ROS2 package
- 라이선스: BSD 계열 라이선스 (`src/sllidar_ros2/LICENSE` 유지)
- 본 프로젝트의 직접 구현 범위가 아니라 외부 LiDAR driver 의존성입니다.
- 소스 재배포 조건에 따라 기존 copyright, license 조건문, disclaimer를 유지합니다.


### src/mycobot_ros2

myCobot 280 협동 로봇 암 전용 패키지 모음. [automaticaddison/mycobot_ros2](https://github.com/automaticaddison/mycobot_ros2) 포크 기반.

| 패키지 | 설명 |
|--------|------|
| `mycobot_bringup` | 로봇 드라이버 구동 launch 파일 |
| `mycobot_description` | URDF/xacro 로봇 모델 |
| `mycobot_gazebo` | Gazebo 시뮬레이션 world 및 launch |
| `mycobot_interfaces` | MoveIt 계획 장면 생성용 커스텀 srv 인터페이스 |
| `mycobot_moveit_config` | MoveIt2 설정 및 SRDF |
| `mycobot_moveit_demos` | MoveIt2 기본 데모 (`hello_moveit.py` 포함) |
| `mycobot_mtc_demos` | MoveIt Task Constructor 데모 |
| `mycobot_mtc_pick_place_demo` | MTC 기반 pick & place + 포인트 클라우드 인식 |
| `mycobot_system_tests` | 통합·시스템 테스트 |

### src/just_pick_it

본 프로젝트(Just Pick It)에서 새로 작성하는 패키지 모음. 패키지별 담당자는 [인수인계서.md](인수인계서.md)를 참고하세요.

| 패키지 | 설명 |
|--------|------|
| `just_pick_it_interfaces` | 프로젝트 공용 msg/srv/action 정의 |
| `just_pick_it_bringup` | 전체 시스템 통합 launch |
| `just_pick_it_perception` | 비전·인식 모듈 |
| `just_pick_it_simulation` | Gazebo 시뮬레이션 launch·world·params |
| `jetcobot_inspection` | JetCobot 기반 검사 매니퓰레이션 |
| `jetcobot_sorting` | JetCobot 기반 분류 매니퓰레이션 |
| `pinky_amr_1` | AMR 인스턴스 1 (Pinky Pro) |
| `pinky_amr_2` | AMR 인스턴스 2 (Pinky Pro) |

---

## 환경 요구사항

- ROS2 Jazzy
- Gazebo Harmonic
- MoveIt2 (`ros-jazzy-moveit`)
- Terminator (스크립트 자동화)
- Python 3.12
- PostgreSQL

---

## 설치 및 빌드

### 최초 클론

```bash
git clone https://github.com/jaejeongpark/just_pick_it.git
cd just_pick_it
```

### 권장 자동 세팅

처음 세팅하거나 환경이 꼬였을 때는 루트 재세팅 스크립트를 사용합니다.

```bash
./reset_ws.sh
```

`reset_ws.sh`는 다음을 한 번에 수행합니다.

```text
1. Ubuntu 24.04 / ROS 2 Jazzy / Python 3.12 기준 확인
2. web/.venv 세팅
3. PostgreSQL role/database/schema/seed 세팅
4. rosdep 의존성 설치
5. build/install/log 삭제
6. colcon build --symlink-install 전체 빌드
```

팀원 환경을 맞출 때는 부분 초기화 옵션 없이 `./reset_ws.sh`를 그대로 실행합니다.

### 수동 의존성 설치 / 빌드

자동 스크립트 대신 수동으로 진행할 경우:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

특정 패키지만 빌드할 경우:

```bash
# Pinky Pro만
colcon build --symlink-install --packages-up-to pinky_gz_sim

# myCobot만
colcon build --symlink-install --packages-up-to mycobot_moveit_config
```

---

## 루트 실행 스크립트

| 스크립트 | 언제 사용 | 하는 일 |
|---|---|---|
| `./reset_ws.sh` | 최초 세팅, 환경 재설정, 의존성/빌드 상태를 깨끗하게 맞출 때 | `web/.venv`, PostgreSQL DB, rosdep, colcon 전체 symlink build를 순서대로 수행 |
| `./run_all.sh` | 평소 로컬 통합 실행 | Fleet Manager/Fleet API `:8100`을 띄우고 Web Gateway `:8000`을 실행 |
| `./reset_demo_data.sh` | 데모 중 주문/task/진열 데이터만 seed 기준으로 되돌릴 때 | DB schema는 유지하고 demo table을 비운 뒤 `db/seed.sql` 재적용 |

권장 순서:

```bash
# 최초 1회 또는 환경 재세팅
./reset_ws.sh

# 평소 통합 실행
./run_all.sh

# 데모 데이터만 초기화
./reset_demo_data.sh
```

`./run_all.sh`는 `./reset_ws.sh`가 끝난 워크스페이스를 기준으로 동작합니다.
Web Gateway만 단독 실행할 때는 `web/scripts/run.sh`를 사용하지만, 실제 데이터 조회는 Fleet API가 떠 있어야 정상 동작합니다.

---

## scripts 자동화

Terminator 레이아웃 기반 자동화 스크립트. 각 스크립트는 topic 감지를 통해 pane 간 순차 실행을 자동화합니다.

### 맵핑 (`scripts/mapping/`)

| 스크립트 | 설명 |
|---------|------|
| `sim_map_building.sh` | Gazebo 시뮬레이션 SLAM 맵핑 |
| `real_map_building.sh` | 실제 Pinky Pro 로봇 SLAM 맵핑 |

**시뮬레이션 맵핑 레이아웃:**
```
┌──────────────┬──────────────┐
│  1. Gazebo   │  3. RViz     │
├──────────────┼──────────────┤
│  2. SLAM     │  4. Teleop   │
├──────────────┴──────────────┤
│       5. Map Saver          │
└─────────────────────────────┘
```
자동 순서: Gazebo 실행 → `/clock` 감지 시 SLAM 실행 → `/map` 감지 시 RViz 실행
수동 조작: Teleop으로 주행 후 Map Saver에서 이름 입력하여 저장

**실제 로봇 맵핑 레이아웃:**
```
┌──────────────┬──────────────┐
│  1. Bringup  │  3. RViz     │
├──────────────┼──────────────┤
│  2. SLAM     │  4. Teleop   │
├──────────────┴──────────────┤
│       5. Map Saver          │
└─────────────────────────────┘
```
자동 순서: Bringup 실행 → `/scan` + `/odom` + `/imu` 모두 감지 시 SLAM 실행 → `/map` 감지 시 RViz 실행

### 내비게이션 (`scripts/navigation/`)

| 스크립트 | 설명 |
|---------|------|
| `sim_navigation.sh` | Gazebo 시뮬레이션 Nav2 자율 주행 |
| `real_navigation.sh` | 실제 Pinky Pro 로봇 Nav2 자율 주행 |

**시뮬레이션 내비게이션 레이아웃:**
```
┌──────────────┬──────────────┐
│  1. Gazebo   │  3. RViz     │
├──────────────┴──────────────┤
│     2. Nav2 (맵 선택 후 실행)│
└─────────────────────────────┘
```
자동 순서: Gazebo 실행 → `/clock` 감지 시 맵 선택 후 Nav2 실행 → `/amcl_pose` 감지 시 RViz 실행

**실제 로봇 내비게이션 레이아웃:**
```
┌──────────────┬──────────────┐
│  1. Bringup  │  3. RViz     │
├──────────────┴──────────────┤
│     2. Nav2 (맵 선택 후 실행)│
└─────────────────────────────┘
```
자동 순서: Bringup 실행 → `/scan` + `/odom` + `/imu` 감지 시 맵 선택 후 Nav2 실행 → `/amcl_pose` 감지 시 RViz 실행

### 사용법

```bash
# 시뮬레이션 맵핑
bash scripts/mapping/sim_map_building.sh

# 실제 로봇 맵핑
bash scripts/mapping/real_map_building.sh

# 시뮬레이션 내비게이션
bash scripts/navigation/sim_navigation.sh

# 실제 로봇 내비게이션
bash scripts/navigation/real_navigation.sh
```
