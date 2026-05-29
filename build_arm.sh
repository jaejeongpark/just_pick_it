#!/usr/bin/env bash
# 로봇팔(manipulation) 보드 전용 빌드 스크립트
# 주행(pinky / sllidar / pinky_amr) 계열 패키지는 빌드에서 제외한다.
# 공통 패키지(fleet_manager, just_pick_it_* 등)는 함께 빌드된다.
# 추가 colcon 인자는 그대로 전달된다. 예: ./build_arm.sh --symlink-install
set -euo pipefail

cd "$(dirname "$0")"

# AMR(주행) 보드에서만 쓰는 패키지 목록
AMR_ONLY_PACKAGES=(
  pinky_bringup
  pinky_description
  pinky_emotion
  pinky_gz_sim
  pinky_imu_bno055
  pinky_interfaces
  pinky_lamp_control
  pinky_led
  pinky_navigation
  pinky_sensor_adc
  sllidar_ros2
  pinky_amr_1
  pinky_amr_2
)

colcon build --packages-ignore "${AMR_ONLY_PACKAGES[@]}" "$@"
