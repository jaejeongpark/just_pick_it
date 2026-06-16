#!/bin/bash
# PICKY1 헤드리스 bringup (모니터 없는 로봇 보드 / SSH 접속용)
#
# 로봇 하드웨어(라이다, 모터, 배터리 등)를 /picky1 네임스페이스로 띄운다.
# GUI(terminator/RViz)가 필요 없으므로 SSH 세션에서 그대로 실행할 수 있다.
# real_navigation.sh(terminator 기반 GUI 스크립트)의 헤드리스 대체용이다.
#
# 사용법: bash scripts/navigation/headless_picky1_bringup.sh
# 별도 SSH 세션에서 headless_picky1_nav.sh, headless_picky1_state.sh 와 함께 띄운다.
set -e

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source ~/venv/jazzy/bin/activate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/scripts/dds_env.sh"   # 디스커버리 서버 env(공용)
export ROS_DOMAIN_ID=25

echo "=== [PICKY1] Bringup (namespace /picky1, ROS_DOMAIN_ID=$ROS_DOMAIN_ID) ==="
exec ros2 launch pinky_amr_1 picky1_bringup.launch.py
