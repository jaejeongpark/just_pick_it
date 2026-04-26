#!/bin/bash
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source ~/venv/jazzy/bin/activate 2>/dev/null
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID=13

echo "=== [2] SLAM — 로봇 준비 대기 중 (/scan + /odom + /imu) ==="
until ros2 topic list 2>/dev/null | grep -q '/scan' && \
      ros2 topic list 2>/dev/null | grep -q '/odom' && \
      ros2 topic list 2>/dev/null | grep -q '/imu'; do
    printf '\r[대기] /scan, /odom, /imu 모두 확인될 때까지 대기...'
    sleep 2
done
echo -e "\n[완료] 로봇 준비 완료 → SLAM 시작"
ros2 launch pinky_navigation map_building.launch.xml
exec bash
