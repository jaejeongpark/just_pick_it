#!/bin/bash
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../" && pwd)"
source ~/venv/jazzy/bin/activate 2>/dev/null
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID}"

CALIB_FILE="${CALIB_FILE:-$WS_ROOT/src/just_pick_it/just_pick_it_perception/result/camera_calibration.yaml}"
USE_CAMERA_INFO="${USE_CAMERA_INFO:-false}"
PUBLISH_DEBUG_IMAGE="${PUBLISH_DEBUG_IMAGE:-true}"

echo "=== [2] AprilTag Pose Estimator - Gazebo /clock 대기 중 ==="
until ros2 topic list 2>/dev/null | grep -q '/clock'; do
    printf '\r[대기] Gazebo 시작 중... /clock 확인 중'
    sleep 2
done
echo -e "\n[완료] Gazebo 감지. /camera/image_raw 토픽 대기 중"
until ros2 topic list 2>/dev/null | grep -q '^/camera/image_raw$'; do
    printf '\r[대기] /camera/image_raw 토픽 등록 확인 중'
    sleep 2
done
echo -e "\n[완료] 추정기 시작 (calib: ${CALIB_FILE})"
ros2 launch just_pick_it_perception apriltag_pose_estimator.launch.xml \
    calibration_file:="$CALIB_FILE" \
    use_camera_info:="$USE_CAMERA_INFO" \
    publish_debug_image:="$PUBLISH_DEBUG_IMAGE"
exec bash
