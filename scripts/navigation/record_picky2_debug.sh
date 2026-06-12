#!/bin/bash
# PICKY2 주행 문제 재현용 PC-side 자동 기록 스크립트.
#
# 기록 내용:
#   - Nav2/State/Fleet 분석에 필요한 /picky2 주요 ROS 토픽 rosbag
#   - /rosout 로그 토픽
#   - NavigateToPose / ComputePathToPose / FollowPath / MoveCommand action status/feedback
#   - scan, tf, odom, amcl, cmd_vel, plan, costmap, transition_event
#   - PC에서 실행 중 생성된 ~/.ros/log 파일 일부(Fleet/RViz/recorder 로그)
#
# 사용법:
#   bash scripts/navigation/record_picky2_debug.sh
#   bash scripts/navigation/record_picky2_debug.sh picky2_nav_fail_001
#
# 옵션:
#   PICKY2_DEBUG_BASE=./bags              저장 루트 변경
#   PICKY2_DEBUG_MAX_BAG_DURATION=600     bag 자동 split 시간(초)
#   PICKY2_DEBUG_RECORD_CAMERA=1          /picky2/camera 계열 토픽도 bag에 포함
#   PICKY2_DEBUG_WITH_PC_MONITOR=1        PC CPU/메모리 감시 로그 추가
#
# ROS setup.bash 는 내부에서 비어 있을 수 있는 환경 변수를 참조하므로, source 전에는
# nounset(-u)을 켜지 않는다.
set -eo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source ~/venv/jazzy/bin/activate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"

set -u

RUN_NAME="${1:-picky2_debug_$(date +%Y%m%d_%H%M%S)}"
BASE_DIR="${PICKY2_DEBUG_BASE:-$WS_ROOT/bags}"
OUT_DIR="$BASE_DIR/$RUN_NAME"
MAX_BAG_DURATION="${PICKY2_DEBUG_MAX_BAG_DURATION:-600}"
RECORD_CAMERA="${PICKY2_DEBUG_RECORD_CAMERA:-0}"
WITH_PC_MONITOR="${PICKY2_DEBUG_WITH_PC_MONITOR:-0}"

if [[ -e "$OUT_DIR" ]]; then
    OUT_DIR="${BASE_DIR}/${RUN_NAME}_$(date +%Y%m%d_%H%M%S)"
fi

BAG_DIR="$OUT_DIR/rosbag"
START_STAMP="$OUT_DIR/start.stamp"

mkdir -p "$OUT_DIR"
touch "$START_STAMP"

PIDS=()

log() {
    printf '[picky2-debug] %s\n' "$*"
}

start_bg() {
    local name="$1"
    shift
    log "starting $name"
    setsid "$@" >"$OUT_DIR/$name.log" 2>&1 &
    PIDS+=("$!")
}

start_shell_bg() {
    local name="$1"
    local script="$2"
    log "starting $name"
    setsid bash -lc "$script" >"$OUT_DIR/$name.log" 2>&1 &
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
        echo "=== ROS_DOMAIN_ID ==="
        echo "${ROS_DOMAIN_ID:-unset}"
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
        echo "skipped: lifecycle CLI can block on a loaded Raspberry Pi; rosbag and ros_logs contain transition events instead."
    } >"$OUT_DIR/ros_graph_${suffix}.log"
}

collect_ros_logs() {
    local src="$HOME/.ros/log"
    local dst="$OUT_DIR/ros_logs"
    mkdir -p "$dst"

    if [[ ! -d "$src" ]]; then
        echo "~/.ros/log not found" >"$dst/README.txt"
        return
    fi

    find "$src" -type f -newer "$START_STAMP" -size -20M -print0 2>/dev/null |
        while IFS= read -r -d '' file; do
            local rel="${file#$src/}"
            mkdir -p "$dst/$(dirname "$rel")"
            cp "$file" "$dst/$rel" 2>/dev/null || true
        done
}

cleanup() {
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
    collect_ros_logs || true

    {
        echo "run_name=$RUN_NAME"
        echo "out_dir=$OUT_DIR"
        echo "ros_domain_id=${ROS_DOMAIN_ID:-unset}"
        echo "ended_at=$(date '+%F %T.%3N %z')"
        echo
        echo "files:"
        find "$OUT_DIR" -maxdepth 2 -type f | sort
    } >"$OUT_DIR/summary.txt"

    log "done: $OUT_DIR"
}

trap cleanup INT TERM EXIT

{
    echo "run_name=$RUN_NAME"
    echo "out_dir=$OUT_DIR"
    echo "started_at=$(date '+%F %T.%3N %z')"
    echo "host=$(hostname)"
    echo "user=$(whoami)"
    echo "pwd=$WS_ROOT"
    echo "ros_domain_id=${ROS_DOMAIN_ID:-unset}"
    echo
    echo "git:"
    git -C "$WS_ROOT" rev-parse --short HEAD 2>/dev/null || true
    git -C "$WS_ROOT" status --short 2>/dev/null || true
} >"$OUT_DIR/manifest.txt"

TOPIC_REGEX='^(/rosout|/diagnostics|/parameter_events|/picky2/(amcl_pose|battery/(percent|voltage)|behavior_tree_log|clicked_point|cmd_vel|cmd_vel_nav|cmd_vel_teleop|controller_selector|curvature_lookahead_point|goal_pose|initialpose|is_rotating_to_heading|joint_states|lookahead_collision_arc|lookahead_point|map|map_updates|odom|particle_cloud|picky_state|plan|plan_smoothed|planner_selector|preempt_teleop|received_global_plan|robot_description|scan|speed_limit|tf|tf_static|waypoints|.*transition_event|local_costmap/(costmap|costmap_updates|costmap_raw|costmap_raw_updates|footprint|published_footprint|obstacle_layer|obstacle_layer_updates|obstacle_layer_raw|obstacle_layer_raw_updates|local_costmap/transition_event)|global_costmap/(costmap|costmap_updates|costmap_raw|costmap_raw_updates|footprint|published_footprint|obstacle_layer|obstacle_layer_updates|obstacle_layer_raw|obstacle_layer_raw_updates|static_layer|static_layer_updates|static_layer_raw|static_layer_raw_updates|global_costmap/transition_event)|[^/]+/_action/(status|feedback)))$'

if [[ "$RECORD_CAMERA" == "1" ]]; then
    TOPIC_REGEX='^(/rosout|/diagnostics|/parameter_events|/picky2/(camera/.*|.*image.*|amcl_pose|battery/(percent|voltage)|behavior_tree_log|clicked_point|cmd_vel|cmd_vel_nav|cmd_vel_teleop|controller_selector|curvature_lookahead_point|goal_pose|initialpose|is_rotating_to_heading|joint_states|lookahead_collision_arc|lookahead_point|map|map_updates|odom|particle_cloud|picky_state|plan|plan_smoothed|planner_selector|preempt_teleop|received_global_plan|robot_description|scan|speed_limit|tf|tf_static|waypoints|.*transition_event|local_costmap/(costmap|costmap_updates|costmap_raw|costmap_raw_updates|footprint|published_footprint|obstacle_layer|obstacle_layer_updates|obstacle_layer_raw|obstacle_layer_raw_updates|local_costmap/transition_event)|global_costmap/(costmap|costmap_updates|costmap_raw|costmap_raw_updates|footprint|published_footprint|obstacle_layer|obstacle_layer_updates|obstacle_layer_raw|obstacle_layer_raw_updates|static_layer|static_layer_updates|static_layer_raw|static_layer_raw_updates|global_costmap/transition_event)|[^/]+/_action/(status|feedback)))$'
fi

start_bg \
    "rosbag_record" \
    ros2 bag record \
        --include-hidden-topics \
        --regex "$TOPIC_REGEX" \
        --storage mcap \
        --storage-preset-profile fastwrite \
        --max-bag-duration "$MAX_BAG_DURATION" \
        -o "$BAG_DIR"

snapshot_ros_graph "initial" || true

if [[ "$WITH_PC_MONITOR" == "1" ]]; then
    start_shell_bg "pc_monitor" "
while true; do
    echo '=== '\"\$(date '+%F %T.%3N %z')\"' ==='
    uptime 2>&1 || true
    free -h 2>&1 || true
    ps -eo pid,comm,%cpu,%mem,rss,args --sort=-%cpu | head -30 2>&1 || true
    echo
    sleep 1
done
"
fi

log "recording to $OUT_DIR"
log "start this before placing an order; stop with Ctrl+C after the issue happens"

while true; do
    sleep 3600
done
