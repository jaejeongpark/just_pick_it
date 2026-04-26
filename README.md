# autonomous_sys_ws

ROS2 기반 자율 시스템 워크스페이스. **Pinky Pro** 모바일 로봇과 **myCobot 280** 협동 로봇 암을 함께 활용하는 자율화 프로젝트를 위한 통합 개발 환경입니다.

## 워크스페이스 구조

```
autonomous_sys_ws/
├── src/
│   ├── pinky_pro/          # Pinky Pro 로봇 패키지 (git submodule)
│   ├── mycobot280/         # myCobot 280 패키지 (git submodule, 예정)
│   └── <project_pkg>/      # 프로젝트 패키지 (pinky_pro + mycobot280 의존)
├── scripts/
│   ├── mapping/            # 지도 생성 자동화 스크립트
│   └── navigation/         # 자율 주행 자동화 스크립트
├── build/                  # colcon 빌드 출력 (gitignore)
├── install/                # colcon 설치 공간 (gitignore)
└── log/                    # 실행 로그 (gitignore)
```

### src/pinky_pro 패키지 구성

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

## 환경 요구사항

- ROS2 Jazzy
- Gazebo Harmonic
- Terminator (스크립트 자동화)
- Python 3.10+

## 설치 및 빌드

```bash
# 서브모듈 포함 클론
git clone --recurse-submodules https://github.com/jaejeongpark/autonomous_sys_ws.git
cd autonomous_sys_ws

# 의존성 설치
rosdep install --from-paths src --ignore-src -r -y

# 빌드
colcon build --symlink-install
source install/setup.bash
```

기존 클론 후 서브모듈이 비어 있는 경우:
```bash
git submodule update --init --recursive
```

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

## 협업 서브모듈 업데이트

```bash
# pinky_pro 최신 반영
git submodule update --remote src/pinky_pro

# 변경사항 커밋
git add src/pinky_pro
git commit -m "chore: update pinky_pro submodule"
```
