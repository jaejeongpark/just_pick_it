#!/bin/bash
# PICKY2 주행 문제 재현용 PC-side 자동 기록 스크립트.
#
# 기록 내용:
#   - 기본 light profile: Nav2/State/Fleet 분석에 필요한 /picky2 핵심 ROS 토픽 rosbag
#   - full profile: costmap raw/update까지 포함한 무거운 분석용 rosbag
#   - /rosout 로그 토픽
#   - NavigateThroughPoses / FollowPath / MoveCommand / DockCommand action status/feedback
#   - scan, tf, odom, amcl, cmd_vel, cmd_vel_nav, cmd_vel_raw, plan, map, transition_event
#   - PC에서 실행 중 생성된 ~/.ros/log 파일 일부(Fleet/RViz/recorder 로그)
#
# 사용법:
#   bash scripts/navigation/record_picky2_debug.sh
#   bash scripts/navigation/record_picky2_debug.sh picky2_nav_fail_001
#
# 옵션:
#   PICKY2_DEBUG_BASE=./bags              저장 루트 변경
#   PICKY2_DEBUG_PROFILE=light            light 또는 full
#   PICKY2_DEBUG_MAX_BAG_DURATION=600     bag 자동 split 시간(초)
#   PICKY2_DEBUG_RECORD_CAMERA=1          /picky2/camera 계열과 docking/debug_image도 bag에 포함
#   PICKY2_DEBUG_MCAP_PRESET=none         none은 MCAP 기본 인덱스를 남김, fastwrite는 분석 경고가 날 수 있음
#   PICKY2_DEBUG_WITH_PC_MONITOR=1        PC CPU/메모리 감시 로그 추가
#   PICKY2_DEBUG_LIFECYCLE_INTERVAL=0     lifecycle 상태 주기 기록(초, 0이면 비활성)
#   PICKY2_DEBUG_LIFECYCLE_TIMEOUT=2      lifecycle get 1회 timeout(초)
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
DEBUG_PROFILE="${PICKY2_DEBUG_PROFILE:-light}"
MAX_BAG_DURATION="${PICKY2_DEBUG_MAX_BAG_DURATION:-600}"
RECORD_CAMERA="${PICKY2_DEBUG_RECORD_CAMERA:-1}"
MCAP_PRESET="${PICKY2_DEBUG_MCAP_PRESET:-none}"
WITH_PC_MONITOR="${PICKY2_DEBUG_WITH_PC_MONITOR:-0}"
LIFECYCLE_INTERVAL="${PICKY2_DEBUG_LIFECYCLE_INTERVAL:-0}"
LIFECYCLE_TIMEOUT="${PICKY2_DEBUG_LIFECYCLE_TIMEOUT:-2}"

if [[ -e "$OUT_DIR" ]]; then
    OUT_DIR="${BASE_DIR}/${RUN_NAME}_$(date +%Y%m%d_%H%M%S)"
fi

BAG_DIR="$OUT_DIR/rosbag"
START_STAMP="$OUT_DIR/start.stamp"

mkdir -p "$OUT_DIR"
touch "$START_STAMP"

PIDS=()

LIFECYCLE_NODES=(
    "/picky2/map_server"
    "/picky2/amcl"
    "/picky2/planner_server"
    "/picky2/controller_server"
    "/picky2/smoother_server"
    "/picky2/behavior_server"
    "/picky2/bt_navigator"
    "/picky2/waypoint_follower"
    "/picky2/velocity_smoother"
    "/picky2/local_costmap/local_costmap"
    "/picky2/global_costmap/global_costmap"
)

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

lifecycle_snapshot() {
    echo "=== lifecycle snapshot: $(date '+%F %T.%3N %z') ==="
    echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"

    local node node_list status
    node_list="$(timeout 5s ros2 node list 2>&1 || true)"
    for node in "${LIFECYCLE_NODES[@]}"; do
        if ! grep -Fxq "$node" <<<"$node_list"; then
            status="node not found"
        else
            status="$(timeout "${LIFECYCLE_TIMEOUT}s" ros2 lifecycle get "$node" 2>&1 || true)"
        fi
        if [[ -z "$status" ]]; then
            status="no response"
        fi
        printf '%-42s %s\n' "$node" "$status"
    done
    echo
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
        lifecycle_snapshot
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
    exit "$exit_code"
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
    echo "debug_profile=$DEBUG_PROFILE"
    echo "record_camera=$RECORD_CAMERA"
    echo "mcap_preset=$MCAP_PRESET"
    echo "with_pc_monitor=$WITH_PC_MONITOR"
    echo "lifecycle_interval_sec=$LIFECYCLE_INTERVAL"
    echo "lifecycle_timeout_sec=$LIFECYCLE_TIMEOUT"
    echo
    echo "git:"
    git -C "$WS_ROOT" rev-parse --short HEAD 2>/dev/null || true
    git -C "$WS_ROOT" status --short 2>/dev/null || true
} >"$OUT_DIR/manifest.txt"

CAMERA_TOPIC_PART=""
if [[ "$RECORD_CAMERA" == "1" ]]; then
    CAMERA_TOPIC_PART="camera/.*|docking/debug_image|.*image.*|"
fi

if [[ "$MCAP_PRESET" == "fastwrite" ]]; then
    log "warning: MCAP fastwrite can omit message indexes; analysis may warn about no message index"
fi

LIGHT_TOPIC_REGEX="^(/rosout|/diagnostics|/parameter_events|/picky2/(${CAMERA_TOPIC_PART}amcl_pose|battery/(percent|voltage)|behavior_tree_log|cmd_vel|cmd_vel_nav|cmd_vel_raw|goal_pose|initialpose|is_rotating_to_heading|lookahead_collision_arc|lookahead_point|map|odom|picky_state|plan|received_global_plan|robot_description|scan|tf|tf_static|.*transition_event|local_costmap/(published_footprint|local_costmap/transition_event)|global_costmap/(published_footprint|global_costmap/transition_event)|(move_command|dock_command|navigate_through_poses|follow_path|compute_path_through_poses|navigate_to_pose|compute_path_to_pose)/_action/(status|feedback)))$"
FULL_TOPIC_REGEX="^(/rosout|/diagnostics|/parameter_events|/picky2/(${CAMERA_TOPIC_PART}amcl_pose|battery/(percent|voltage)|behavior_tree_log|clicked_point|cmd_vel|cmd_vel_nav|cmd_vel_teleop|controller_selector|curvature_lookahead_point|goal_pose|initialpose|is_rotating_to_heading|joint_states|lookahead_collision_arc|lookahead_point|map|map_updates|odom|particle_cloud|picky_state|plan|plan_smoothed|planner_selector|preempt_teleop|received_global_plan|robot_description|scan|speed_limit|tf|tf_static|waypoints|.*transition_event|local_costmap/(costmap|costmap_updates|costmap_raw|costmap_raw_updates|footprint|published_footprint|obstacle_layer|obstacle_layer_updates|obstacle_layer_raw|obstacle_layer_raw_updates|local_costmap/transition_event)|global_costmap/(costmap|costmap_updates|costmap_raw|costmap_raw_updates|footprint|published_footprint|obstacle_layer|obstacle_layer_updates|obstacle_layer_raw|obstacle_layer_raw_updates|static_layer|static_layer_updates|static_layer_raw|static_layer_raw_updates|global_costmap/transition_event)|[^/]+/_action/(status|feedback)))$"

case "$DEBUG_PROFILE" in
    light)
        TOPIC_REGEX="$LIGHT_TOPIC_REGEX"
        ;;
    full)
        TOPIC_REGEX="$FULL_TOPIC_REGEX"
        ;;
    *)
        log "invalid PICKY2_DEBUG_PROFILE=$DEBUG_PROFILE (use light or full)"
        exit 2
        ;;
esac

start_bg \
    "rosbag_record" \
    ros2 bag record \
        --include-hidden-topics \
        --regex "$TOPIC_REGEX" \
        --storage mcap \
        --storage-preset-profile "$MCAP_PRESET" \
        --max-bag-duration "$MAX_BAG_DURATION" \
        -o "$BAG_DIR"

if [[ "$LIFECYCLE_INTERVAL" != "0" ]]; then
    start_shell_bg "lifecycle_monitor" "
source ~/venv/jazzy/bin/activate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
source '$WS_ROOT/install/setup.bash'
LIFECYCLE_TIMEOUT='$LIFECYCLE_TIMEOUT'
LIFECYCLE_NODES=(
    '/picky2/map_server'
    '/picky2/amcl'
    '/picky2/planner_server'
    '/picky2/controller_server'
    '/picky2/smoother_server'
    '/picky2/behavior_server'
    '/picky2/bt_navigator'
    '/picky2/waypoint_follower'
    '/picky2/velocity_smoother'
    '/picky2/local_costmap/local_costmap'
    '/picky2/global_costmap/global_costmap'
)
while true; do
    echo '=== lifecycle snapshot: '\"\$(date '+%F %T.%3N %z')\"' ==='
    echo 'ROS_DOMAIN_ID='\"\${ROS_DOMAIN_ID:-unset}\"
    node_list=\"\$(timeout 5s ros2 node list 2>&1 || true)\"
    for node in \"\${LIFECYCLE_NODES[@]}\"; do
        if ! grep -Fxq \"\$node\" <<<\"\$node_list\"; then
            status='node not found'
        else
            status=\"\$(timeout \"\${LIFECYCLE_TIMEOUT}s\" ros2 lifecycle get \"\$node\" 2>&1 || true)\"
        fi
        if [[ -z \"\$status\" ]]; then
            status='no response'
        fi
        printf '%-42s %s\n' \"\$node\" \"\$status\"
    done
    echo
    sleep '$LIFECYCLE_INTERVAL'
done
"
fi

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
