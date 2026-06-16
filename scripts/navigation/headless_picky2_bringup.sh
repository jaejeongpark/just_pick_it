#!/bin/bash
# PICKY2 헤드리스 bringup (모니터 없는 로봇 보드 / SSH 접속용)
set -e

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source ~/venv/jazzy/bin/activate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
if [[ "${USE_DDS:-1}" != "0" && -f "$WS_ROOT/scripts/dds_env.sh" ]]; then
    source "$WS_ROOT/scripts/dds_env.sh"   # 디스커버리 서버 env(공용)
fi

echo "=== [PICKY2] Bringup (namespace /picky2, ROS_DOMAIN_ID=$ROS_DOMAIN_ID) ==="
exec ros2 launch pinky_amr_2 picky2_bringup.launch.py
