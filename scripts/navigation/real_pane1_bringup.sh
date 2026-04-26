#!/bin/bash
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source ~/venv/jazzy/bin/activate 2>/dev/null
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID=13

echo "=== [1] Robot Bringup ==="
ros2 launch pinky_bringup bringup_robot.launch.xml
exec bash
