#!/bin/bash
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../" && pwd)"
source ~/venv/jazzy/bin/activate 2>/dev/null
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"

OUTPUT_DIR="${OUTPUT_DIR:-$HOME/img_captures}"
INTERVAL="${INTERVAL:-1.0}"
TOPIC="${TOPIC:-/camera/image_raw}"

echo "=== [2] Image Capture - Gazebo /clock 대기 중 ==="
until ros2 topic list 2>/dev/null | grep -q '/clock'; do
    printf '\r[대기] Gazebo 시작 중... /clock 확인 중'
    sleep 2
done
echo -e "\n[완료] Gazebo 감지. ${TOPIC} 토픽 대기 중"
until ros2 topic list 2>/dev/null | grep -q "^${TOPIC}$"; do
    printf '\r[대기] %s 토픽 등록 확인 중' "$TOPIC"
    sleep 2
done
echo -e "\n[완료] 캡처 시작 (출력: ${OUTPUT_DIR}, 간격: ${INTERVAL}s)"
ros2 run just_pick_it_perception capture_image --ros-args \
    -p topic:="$TOPIC" \
    -p output_dir:="$OUTPUT_DIR" \
    -p interval:="$INTERVAL"
exec bash
