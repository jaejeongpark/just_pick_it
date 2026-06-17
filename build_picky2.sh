#!/usr/bin/env bash
# PICKY2 로봇 PC 전용 빌드 스크립트.
# 서버/Web/DB/Fleet/로봇팔 패키지는 제외하고, 실로봇 bringup/SLAM/Nav2/State Machine에
# 필요한 패키지만 빌드한다.
#
# 사용 예:
#   ./build_picky2.sh
#   ./build_picky2.sh --clean
#   ./build_picky2.sh --no-symlink-install --clean
#   ./build_picky2.sh --event-handlers console_direct+
set -euo pipefail

cd "$(dirname "$0")"

if [ -f /opt/ros/jazzy/setup.bash ] && [ -z "${ROS_DISTRO:-}" ]; then
  # shellcheck source=/opt/ros/jazzy/setup.bash
  source /opt/ros/jazzy/setup.bash
fi

REQUIRED_PACKAGES=(
  just_pick_it_interfaces
  just_pick_it_perception
  sllidar_ros2
  pinky_description
  pinky_bringup
  pinky_navigation
  pinky_amr_2
)

CLEAN=false
COLCON_ARGS=()
USE_SYMLINK_INSTALL=true

for arg in "$@"; do
  case "$arg" in
    --clean)
      CLEAN=true
      ;;
    --no-symlink-install)
      USE_SYMLINK_INSTALL=false
      ;;
    --symlink-install)
      USE_SYMLINK_INSTALL=true
      ;;
    *)
      COLCON_ARGS+=("$arg")
      ;;
  esac
done

if [ "$USE_SYMLINK_INSTALL" = true ]; then
  COLCON_ARGS=(--symlink-install "${COLCON_ARGS[@]}")
fi

mapfile -t DISCOVERED_PACKAGES < <(colcon list --base-paths src --names-only)
MISSING_PACKAGES=()

has_discovered_package() {
  local target="$1"
  local discovered

  for discovered in "${DISCOVERED_PACKAGES[@]}"; do
    if [ "$discovered" = "$target" ]; then
      return 0
    fi
  done

  return 1
}

for pkg in "${REQUIRED_PACKAGES[@]}"; do
  if ! has_discovered_package "$pkg"; then
    MISSING_PACKAGES+=("$pkg")
  fi
done

if [ ${#MISSING_PACKAGES[@]} -gt 0 ]; then
  echo "[build_picky2] missing packages in this repository: ${MISSING_PACKAGES[*]}" >&2
  echo "[build_picky2] check that the git repository was cloned with the full src tree." >&2
  exit 1
fi

if [ "$CLEAN" = true ]; then
  echo "[build_picky2] clean selected package build/install outputs"
  for pkg in "${REQUIRED_PACKAGES[@]}"; do
    rm -rf "build/$pkg" "install/$pkg"
  done
fi

echo "[build_picky2] packages: ${REQUIRED_PACKAGES[*]}"
echo "[build_picky2] colcon args: ${COLCON_ARGS[*]}"
colcon build --packages-select "${REQUIRED_PACKAGES[@]}" "${COLCON_ARGS[@]}"
