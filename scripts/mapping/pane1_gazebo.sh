#!/bin/bash
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../" && pwd)"
source ~/venv/jazzy/bin/activate 2>/dev/null
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID=25

echo "=== [1] Gazebo Box Arena ==="
sleep 1   # terminator/X11 초기화 race 회피
ros2 launch pinky_gz_sim launch_box_arena.launch.xml
exec bash
