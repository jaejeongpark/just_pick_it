#!/bin/bash
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source ~/venv/jazzy/bin/activate 2>/dev/null
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID=13

echo "=== [3] RViz — SLAM /map 대기 중 ==="
until ros2 topic list 2>/dev/null | grep -q '/map'; do
    printf '\r[대기] SLAM 시작 중... /map 확인 중'
    sleep 2
done
echo -e "\n[완료] SLAM 감지 → RViz 시작"
ros2 launch pinky_navigation gz_map_view.launch.xml
exec bash
