#!/bin/bash
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../" && pwd)"
source ~/venv/jazzy/bin/activate 2>/dev/null
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"

echo "=== [3] Teleop ==="
echo "i=전진  ,=후진  j=좌회전  l=우회전  k=정지  q/z=속도 증가/감소"
echo ""
ros2 run teleop_twist_keyboard teleop_twist_keyboard
exec bash
