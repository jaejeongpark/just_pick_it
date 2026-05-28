#!/usr/bin/env bash
#
# One-shot camera calibration workflow for PICKY robots.
#
# Usage:
#   bash calibrate_camera.sh picky_1
#   bash calibrate_camera.sh picky_2
#
# Phase 1: UDP image capture  (udp_image_receiver.py)
# Phase 2: Camera calibration (ros2 run just_pick_it_perception camera_calibrator)

set -euo pipefail

ROBOT="${1:-}"
if [[ -z "$ROBOT" ]]; then
    echo "Usage: $0 <picky_1|picky_2>" >&2
    exit 1
fi

case "$ROBOT" in
    picky_1) PORT=5001 ;;
    picky_2) PORT=5002 ;;
    *)
        echo "Unknown robot '${ROBOT}'. Valid options: picky_1, picky_2" >&2
        exit 1
        ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(realpath "$SCRIPT_DIR/../..")"
RECEIVER="$WS_ROOT/src/just_pick_it/just_pick_it_perception/just_pick_it_perception/udp_image_receiver.py"
SAVE_DIR="$HOME/img_capture/$ROBOT"
RESULT_DIR="$WS_ROOT/src/just_pick_it/just_pick_it_perception/result/$ROBOT"

echo "=== PICKY Camera Calibration ==="
echo "Robot    : $ROBOT"
echo "UDP port : $PORT"
echo "Save dir : $SAVE_DIR"
echo "Output   : $RESULT_DIR/camera_calibration.yaml"
echo ""

# Source ROS2 and the local workspace
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="$WS_ROOT/install/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
    echo "ROS2 setup not found at $ROS_SETUP" >&2
    exit 1
fi
# ROS2 setup scripts reference variables not yet set; disable -u around source calls
set +u
# shellcheck source=/dev/null
source "$ROS_SETUP"

if [[ -f "$WS_SETUP" ]]; then
    # shellcheck source=/dev/null
    source "$WS_SETUP"
else
    echo "Workspace not built. Run 'colcon build' from $WS_ROOT first." >&2
    exit 1
fi
set -u

mkdir -p "$SAVE_DIR" "$RESULT_DIR"

# ---- Phase 1: Image Capture ----
echo "--- Phase 1: Image Capture ---"
echo "Point the camera at the checkerboard."
echo "Press Space to capture, q to finish and start calibration."
echo ""

python3 "$RECEIVER" --port "$PORT" --save-dir "$SAVE_DIR"

IMG_COUNT=$(find "$SAVE_DIR" -maxdepth 1 -name '*.png' | wc -l)
echo ""
echo "Captured $IMG_COUNT images in $SAVE_DIR"

if (( IMG_COUNT < 10 )); then
    echo "Warning: at least 10 images are recommended (got $IMG_COUNT)."
    read -rp "Continue to calibration anyway? [y/N] " ans
    if [[ "${ans,,}" != "y" ]]; then
        echo "Aborted."
        exit 0
    fi
fi

# ---- Phase 2: Camera Calibration ----
echo ""
echo "--- Phase 2: Camera Calibration ---"

ros2 run just_pick_it_perception camera_calibrator \
    --ros-args \
    -p image_dir:="$SAVE_DIR" \
    -p robot_name:="$ROBOT" \
    -p output_file:="$RESULT_DIR/camera_calibration.yaml"

echo ""
echo "Done. Calibration file: $RESULT_DIR/camera_calibration.yaml"
