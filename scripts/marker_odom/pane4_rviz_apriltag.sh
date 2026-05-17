#!/bin/bash
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../" && pwd)"
source ~/venv/jazzy/bin/activate 2>/dev/null
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"

RVIZ_CONFIG="${RVIZ_CONFIG:-$WS_ROOT/src/just_pick_it/just_pick_it_perception/config/apriltag_pose_estimator.rviz}"

echo "=== [4] RViz - /clock 대기 중 ==="
until ros2 topic list 2>/dev/null | grep -q '/clock'; do
    printf '\r[대기] Gazebo 시작 중... /clock 확인 중'
    sleep 2
done
echo -e "\n[완료] RViz 시작 (config: ${RVIZ_CONFIG})"
ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true
exec bash
