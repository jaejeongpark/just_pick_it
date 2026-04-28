# autonomous_sys_ws

ROS2 기반 자율 시스템 워크스페이스. **Pinky Pro** 모바일 로봇과 **myCobot 280** 협동 로봇 암을 함께 활용하는 자율화 프로젝트를 위한 통합 개발 환경입니다.

> 협업 git 사용 방법은 **[인수인계서.md](인수인계서.md)** 를 참고하세요.

## 워크스페이스 구조

```
autonomous_sys_ws/
├── src/
│   ├── pinky_pro/          # Pinky Pro 로봇 패키지 (git submodule, my-features 브랜치)
│   └── mycobot_ros2/       # myCobot 280 패키지 (git submodule, my_feature 브랜치)
├── scripts/
│   ├── mapping/            # 지도 생성 자동화 스크립트
│   └── navigation/         # 자율 주행 자동화 스크립트
├── build/                  # colcon 빌드 출력 (gitignore)
├── install/                # colcon 설치 공간 (gitignore)
└── log/                    # 실행 로그 (gitignore)
```

`src/` 하위의 두 패키지는 각각 독립된 GitHub 저장소를 git submodule로 참조합니다.

| 서브모듈 | 저장소 | 브랜치 |
|---------|--------|--------|
| `src/pinky_pro` | [jaejeongpark/pinky_pro](https://github.com/jaejeongpark/pinky_pro) | `my-features` |
| `src/mycobot_ros2` | [jaejeongpark/mycobot_ros2](https://github.com/jaejeongpark/mycobot_ros2) | `my_feature` |

---

## 패키지 구성

### src/pinky_pro

Pinky Pro 모바일 로봇 전용 패키지 모음.

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

---

## 환경 요구사항

- ROS2 Jazzy
- Gazebo Harmonic
- MoveIt2 (`ros-jazzy-moveit`)
- Terminator (스크립트 자동화)
- Python 3.10+

---

## 설치 및 빌드

### 최초 클론 (서브모듈 포함)

```bash
git clone --recurse-submodules https://github.com/jaejeongpark/autonomous_sys_ws.git
cd autonomous_sys_ws
```

이미 클론한 경우 서브모듈을 별도로 초기화합니다:

```bash
git submodule update --init --recursive
```

### 의존성 설치

```bash
rosdep install --from-paths src --ignore-src -r -y
```

### 빌드

```bash
colcon build --symlink-install
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

## 서브모듈 관리

### 서브모듈 최신화

```bash
# 전체
git submodule update --remote

# 개별
git submodule update --remote src/pinky_pro
git submodule update --remote src/mycobot_ros2
```

### 현재 상태 확인

```bash
git submodule status
```

### 서브모듈 직접 수정 후 반영

```bash
# 서브모듈 내에서 작업
cd src/mycobot_ros2
git checkout my_feature
# ... 수정 후 커밋 & 푸시 ...

# 워크스페이스에서 서브모듈 포인터 업데이트
cd ~/autonomous_sys_ws
git add src/mycobot_ros2
git commit -m "chore: update mycobot_ros2 submodule pointer"
git push origin main
```

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
