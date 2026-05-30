#!/usr/bin/env bash
# AMR(주행) 보드 전용 빌드 스크립트
# 로봇팔(mycobot / jetcobot) 계열 패키지는 빌드에서 제외한다.
# 공통 패키지(fleet_manager, just_pick_it_* 등)는 함께 빌드된다.
# 추가 colcon 인자는 그대로 전달된다. 예: ./build_amr.sh --symlink-install
set -euo pipefail

cd "$(dirname "$0")"

# 로봇팔 보드에서만 쓰는 패키지 목록
ARM_ONLY_PACKAGES=(
  mycobot_bringup
  mycobot_description
  mycobot_gazebo
  mycobot_interfaces
  mycobot_moveit_config
  mycobot_moveit_demos
  mycobot_mtc_demos
  mycobot_mtc_pick_place_demo
  mycobot_ros2
  mycobot_system_tests
  jetcobot_bringup
  jetcobot_description
  jetcobot_moveit_config
  jetcobot_inspection
  jetcobot_sorting
  just_pick_it_simulation
)

colcon build --packages-ignore "${ARM_ONLY_PACKAGES[@]}" "$@"
