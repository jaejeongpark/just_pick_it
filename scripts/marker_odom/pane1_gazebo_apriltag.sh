#!/bin/bash
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../" && pwd)"
source ~/venv/jazzy/bin/activate 2>/dev/null
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"

echo "=== [1] Gazebo Box Arena + AprilTag ==="
sleep 1   # terminator/X11 초기화 race 회피
ros2 launch just_pick_it_simulation launch_box_arena_april_tag.launch.xml
exec bash
