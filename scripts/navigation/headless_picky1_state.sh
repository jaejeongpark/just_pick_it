#!/bin/bash
# PICKY1 헤드리스 State Manager (모니터 없는 로봇 보드 / SSH 접속용)
#
# state_manager / move_to_goal / reverse_docking 을 /picky1 네임스페이스로 띄운다.
# Fleet Manager가 호출하는 /picky1/move_command, /picky1/dock_command 액션 서버를
# 제공하고, move_to_goal 은 /picky1/navigate_to_pose 로 Nav2에 목표를 전달한다.
#
# 사용법: bash scripts/navigation/headless_picky1_state.sh
# headless_picky1_bringup.sh, headless_picky1_nav.sh 가 먼저 떠 있어야 한다.
set -e

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source ~/venv/jazzy/bin/activate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
export ROS_DOMAIN_ID=25

echo "=== [PICKY1] State Manager (namespace /picky1, ROS_DOMAIN_ID=$ROS_DOMAIN_ID) ==="
exec ros2 launch pinky_amr_1 picky1_state_manager.launch.py
