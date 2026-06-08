#!/usr/bin/env bash
# PICKY1 로봇 PC 전용 빌드 스크립트.
# 서버/Web/DB/Fleet/로봇팔 패키지는 제외하고, 실로봇 bringup/SLAM/Nav2/State Manager에
# 필요한 패키지만 빌드한다.
#
# 사용 예:
#   ./build_picky1.sh
#   ./build_picky1.sh --clean
#   ./build_picky1.sh --event-handlers console_direct+
set -euo pipefail

cd "$(dirname "$0")"

PACKAGES=(
  just_pick_it_interfaces
  just_pick_it_perception
  sllidar_ros2
  pinky_description
  pinky_bringup
  pinky_navigation
  pinky_amr_1
)

CLEAN=false
COLCON_ARGS=()
HAS_INSTALL_MODE=false

for arg in "$@"; do
  case "$arg" in
    --clean)
      CLEAN=true
      ;;
    --symlink-install|--merge-install)
      HAS_INSTALL_MODE=true
      COLCON_ARGS+=("$arg")
      ;;
    *)
      COLCON_ARGS+=("$arg")
      ;;
  esac
done

if [ "$HAS_INSTALL_MODE" = false ]; then
  COLCON_ARGS=(--symlink-install "${COLCON_ARGS[@]}")
fi

if [ "$CLEAN" = true ]; then
  echo "[build_picky1] clean selected package build/install outputs"
  for pkg in "${PACKAGES[@]}"; do
    rm -rf "build/$pkg" "install/$pkg"
  done
fi

echo "[build_picky1] packages: ${PACKAGES[*]}"
colcon build --packages-select "${PACKAGES[@]}" "${COLCON_ARGS[@]}"
