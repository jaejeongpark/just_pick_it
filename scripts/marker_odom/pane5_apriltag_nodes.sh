#!/bin/bash
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../" && pwd)"
source ~/venv/jazzy/bin/activate 2>/dev/null
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"

echo "=== [5] AprilTag Nodes - /clock 대기 중 ==="
until ros2 topic list 2>/dev/null | grep -q '/clock'; do
    printf '\r[대기] Gazebo 시작 중... /clock 확인 중'
    sleep 2
done
echo -e "\n[완료] static TF publisher 시작 (map -> odom)"

ros2 run tf2_ros static_transform_publisher \
    --frame-id map --child-frame-id odom \
    --x 0 --y 0 --z 0 --roll 0 --pitch 0 --yaw 0 &

echo "[완료] apriltag_map_tf_publisher 시작"
ros2 run just_pick_it_perception apriltag_map_tf_publisher &

echo "[대기] /camera/image_raw 토픽 대기 중"
until ros2 topic list 2>/dev/null | grep -q '^/camera/image_raw$'; do
    printf '\r[대기] /camera/image_raw 토픽 등록 확인 중'
    sleep 2
done
echo -e "\n[완료] apriltag_detector 시작"

ros2 run just_pick_it_perception apriltag_detector
exec bash
