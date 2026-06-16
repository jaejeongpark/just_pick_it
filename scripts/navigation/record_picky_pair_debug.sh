#!/bin/bash
# PICKY1 + PICKY2 동시 주행 문제 재현용 PC-side 자동 기록 스크립트.
#
# 사용법:
#   bash scripts/navigation/record_picky_pair_debug.sh
#   bash scripts/navigation/record_picky_pair_debug.sh pair_nav_fail_001
#
# 옵션:
#   PICKY_PAIR_DEBUG_BASE=./bags
#   PICKY_PAIR_DEBUG_PROFILE=light
#   PICKY_PAIR_DEBUG_RECORD_CAMERA=0
#   PICKY_PAIR_DEBUG_MAX_BAG_DURATION=600
#   PICKY_PAIR_DEBUG_MCAP_PRESET=none
set -eo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source ~/venv/jazzy/bin/activate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
if [[ "${USE_DDS:-1}" != "0" && -f "$WS_ROOT/scripts/dds_env.sh" ]]; then
    source "$WS_ROOT/scripts/dds_env.sh"
fi

set -u

RUN_NAME="${1:-picky_pair_debug_$(date +%Y%m%d_%H%M%S)}"
BASE_DIR="${PICKY_PAIR_DEBUG_BASE:-$WS_ROOT/bags}"
OUT_DIR="$BASE_DIR/$RUN_NAME"
DEBUG_PROFILE="${PICKY_PAIR_DEBUG_PROFILE:-light}"
RECORD_CAMERA="${PICKY_PAIR_DEBUG_RECORD_CAMERA:-0}"
MAX_BAG_DURATION="${PICKY_PAIR_DEBUG_MAX_BAG_DURATION:-600}"
MCAP_PRESET="${PICKY_PAIR_DEBUG_MCAP_PRESET:-none}"
BAG_DIR="$OUT_DIR/rosbag"
START_STAMP="$OUT_DIR/start.stamp"

if [[ -e "$OUT_DIR" ]]; then
    OUT_DIR="${BASE_DIR}/${RUN_NAME}_$(date +%Y%m%d_%H%M%S)"
    BAG_DIR="$OUT_DIR/rosbag"
    START_STAMP="$OUT_DIR/start.stamp"
fi

mkdir -p "$OUT_DIR"
touch "$START_STAMP"
PIDS=()

log() {
    printf '[picky-pair-debug] %s\n' "$*"
}

start_bg() {
    local name="$1"
    shift
    log "starting $name"
    setsid "$@" >"$OUT_DIR/$name.log" 2>&1 &
    PIDS+=("$!")
}

kill_record_process() {
    local signal="$1"
    local pid="$2"
    if kill -0 "$pid" 2>/dev/null; then
        kill "-$signal" -- "-$pid" 2>/dev/null || kill "-$signal" "$pid" 2>/dev/null || true
    fi
}

snapshot_ros_graph() {
    local suffix="$1"
    {
        echo "=== date ==="
        date '+%F %T.%3N %z'
        echo
        echo "=== env ==="
        echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
        echo "ROS_DISCOVERY_SERVER=${ROS_DISCOVERY_SERVER:-unset}"
        echo
        echo "=== nodes ==="
        timeout 5s ros2 node list 2>&1 || true
        echo
        echo "=== topics ==="
        timeout 5s ros2 topic list -t 2>&1 || true
        echo
        echo "=== actions ==="
        timeout 5s ros2 action list -t 2>&1 || true
        echo
        echo "=== lifecycle ==="
        for ns in picky1 picky2; do
            for node in map_server amcl planner_server controller_server smoother_server behavior_server bt_navigator waypoint_follower velocity_smoother local_costmap/local_costmap global_costmap/global_costmap; do
                full="/${ns}/${node}"
                printf '%-42s ' "$full"
                timeout 2s ros2 lifecycle get "$full" 2>&1 || true
            done
        done
    } >"$OUT_DIR/ros_graph_${suffix}.log"
}

cleanup() {
    local exit_code=$?
    trap - INT TERM EXIT
    log "stopping recorders"
    for pid in "${PIDS[@]}"; do
        kill_record_process INT "$pid"
    done
    sleep 2
    for pid in "${PIDS[@]}"; do
        kill_record_process TERM "$pid"
    done
    for pid in "${PIDS[@]}"; do
        wait "$pid" 2>/dev/null || true
    done
    snapshot_ros_graph "final" || true
    {
        echo "run_name=$RUN_NAME"
        echo "out_dir=$OUT_DIR"
        echo "ros_domain_id=${ROS_DOMAIN_ID:-unset}"
        echo "ros_discovery_server=${ROS_DISCOVERY_SERVER:-unset}"
        echo "ended_at=$(date '+%F %T.%3N %z')"
        echo
        find "$OUT_DIR" -maxdepth 2 -type f | sort
    } >"$OUT_DIR/summary.txt"
    log "done: $OUT_DIR"
    exit "$exit_code"
}

trap cleanup INT TERM EXIT

{
    echo "run_name=$RUN_NAME"
    echo "out_dir=$OUT_DIR"
    echo "started_at=$(date '+%F %T.%3N %z')"
    echo "host=$(hostname)"
    echo "ros_domain_id=${ROS_DOMAIN_ID:-unset}"
    echo "ros_discovery_server=${ROS_DISCOVERY_SERVER:-unset}"
    echo "debug_profile=$DEBUG_PROFILE"
    echo "record_camera=$RECORD_CAMERA"
    echo "mcap_preset=$MCAP_PRESET"
    git -C "$WS_ROOT" rev-parse --short HEAD 2>/dev/null || true
    git -C "$WS_ROOT" status --short 2>/dev/null || true
} >"$OUT_DIR/manifest.txt"

CAMERA_TOPIC_PART=""
if [[ "$RECORD_CAMERA" == "1" ]]; then
    CAMERA_TOPIC_PART="camera/.*|docking/debug_image|.*image.*|"
fi

LIGHT_TOPIC_REGEX="^(/rosout|/diagnostics|/parameter_events|/(picky1|picky2)/(${CAMERA_TOPIC_PART}amcl_pose|battery/(percent|voltage)|behavior_tree_log|cmd_vel|cmd_vel_nav|cmd_vel_raw|goal_pose|initialpose|is_rotating_to_heading|lookahead_collision_arc|lookahead_point|map|odom|picky_state|plan|received_global_plan|robot_description|scan|tf|tf_static|.*transition_event|local_costmap/(published_footprint|local_costmap/transition_event)|global_costmap/(published_footprint|global_costmap/transition_event)|(move_command|dock_command|navigate_through_poses|follow_path|compute_path_through_poses|navigate_to_pose|compute_path_to_pose)/_action/(status|feedback)))$"
FULL_TOPIC_REGEX="^(/rosout|/diagnostics|/parameter_events|/(picky1|picky2)/(${CAMERA_TOPIC_PART}amcl_pose|battery/(percent|voltage)|behavior_tree_log|clicked_point|cmd_vel|cmd_vel_nav|cmd_vel_raw|cmd_vel_teleop|controller_selector|curvature_lookahead_point|goal_pose|initialpose|is_rotating_to_heading|joint_states|lookahead_collision_arc|lookahead_point|map|map_updates|odom|particle_cloud|picky_state|plan|plan_smoothed|planner_selector|preempt_teleop|received_global_plan|robot_description|scan|speed_limit|tf|tf_static|waypoints|.*transition_event|local_costmap/(costmap|costmap_updates|costmap_raw|costmap_raw_updates|footprint|published_footprint|obstacle_layer|obstacle_layer_updates|obstacle_layer_raw|obstacle_layer_raw_updates|local_costmap/transition_event)|global_costmap/(costmap|costmap_updates|costmap_raw|costmap_raw_updates|footprint|published_footprint|obstacle_layer|obstacle_layer_updates|obstacle_layer_raw|obstacle_layer_raw_updates|static_layer|static_layer_updates|static_layer_raw|static_layer_raw_updates|global_costmap/transition_event)|[^/]+/_action/(status|feedback)))$"

case "$DEBUG_PROFILE" in
    light) TOPIC_REGEX="$LIGHT_TOPIC_REGEX" ;;
    full) TOPIC_REGEX="$FULL_TOPIC_REGEX" ;;
    *) log "invalid PICKY_PAIR_DEBUG_PROFILE=$DEBUG_PROFILE (use light or full)"; exit 2 ;;
esac

snapshot_ros_graph "initial" || true
start_bg \
    "rosbag_record" \
    ros2 bag record \
        --include-hidden-topics \
        --regex "$TOPIC_REGEX" \
        --storage mcap \
        --storage-preset-profile "$MCAP_PRESET" \
        --max-bag-duration "$MAX_BAG_DURATION" \
        -o "$BAG_DIR"

log "recording to $OUT_DIR"
log "stop with Ctrl+C after the issue happens"

while true; do
    sleep 3600
done
