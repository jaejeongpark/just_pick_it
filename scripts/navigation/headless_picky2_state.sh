#!/bin/bash
# PICKY2 헤드리스 State Machine
# run_picky2_all.sh에서 실행한다.
# ROS_DOMAIN_ID는 호출자가 정한다.

set -e

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"

source ~/venv/jazzy/bin/activate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
source "$WS_ROOT/scripts/dds_env.sh"   # 디스커버리 서버 env(공용)

echo "=== [PICKY2] State Machine (namespace /picky2, ROS_DOMAIN_ID=$ROS_DOMAIN_ID) ==="
exec ros2 launch pinky_amr_2 picky2_state_machine.launch.py
