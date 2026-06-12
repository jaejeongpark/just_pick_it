#!/bin/bash
# PICKY2 주행 문제 재현용 자동 기록 스크립트.
#
# 기록 내용:
#   - /picky2 아래 ROS 토픽 rosbag
#   - CPU/메모리/온도/throttled
#   - /picky2/odom, /picky2/scan, /picky2/joint_states 주기
#   - Nav2/action/graph 상태
#   - 커널 serial/voltage 로그(dmesg, sudo 권한이 있으면 실시간)
#   - 실행 중 생성된 ~/.ros/log 파일 일부
#
# 사용법:
#   bash scripts/navigation/record_picky2_debug.sh
#   bash scripts/navigation/record_picky2_debug.sh picky2_motor_fail_001
#
# 옵션:
#   PICKY2_DEBUG_BASE=./bags            저장 루트 변경
#   PICKY2_DEBUG_SYSTEM_INTERVAL=1      시스템 로그 주기(초)
#   PICKY2_DEBUG_RECORD_CAMERA=1        /picky2/camera 계열 토픽도 bag에 포함
#   PICKY2_DEBUG_EXTRA_CLI=1            topic hz/echo/graph watch도 추가 실행(부하 큼)
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
SYSTEM_INTERVAL="${PICKY2_DEBUG_SYSTEM_INTERVAL:-1}"
RECORD_CAMERA="${PICKY2_DEBUG_RECORD_CAMERA:-0}"
EXTRA_CLI="${PICKY2_DEBUG_EXTRA_CLI:-0}"

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

EXCLUDE_REGEX='/picky2/camera/.*|/picky2/.*image.*'
if [[ "$RECORD_CAMERA" == "1" ]]; then
    EXCLUDE_REGEX='^$'
fi

start_bg \
    "rosbag_record" \
    ros2 bag record \
        --regex '^/picky2/(odom|joint_states|scan|tf|tf_static|battery/percent|battery/voltage|picky_state|cmd_vel|cmd_vel_nav|amcl_pose|plan|received_global_plan|local_costmap/costmap|local_costmap/costmap_updates|global_costmap/costmap|global_costmap/costmap_updates|.*transition_event)$' \
        --exclude-regex "$EXCLUDE_REGEX" \
        --storage mcap \
        --storage-preset-profile fastwrite \
        --max-bag-duration 300 \
        -o "$BAG_DIR"

snapshot_ros_graph "initial" || true

start_shell_bg "system_monitor" "
while true; do
    echo '=== '\"\$(date '+%F %T.%3N %z')\"' ==='
    vcgencmd measure_temp 2>&1 || true
    vcgencmd get_throttled 2>&1 || true
    uptime 2>&1 || true
    free -h 2>&1 || true
    ps -eo pid,comm,%cpu,%mem,rss,args --sort=-%cpu | head -30 2>&1 || true
    echo
    sleep '$SYSTEM_INTERVAL'
done
"

if [[ "$EXTRA_CLI" == "1" ]]; then
    start_shell_bg "ros_graph_watch" "
while true; do
    echo '=== '\"\$(date '+%F %T.%3N %z')\"' ==='
    echo '-- nodes --'
    timeout 5s ros2 node list 2>&1 || true
    echo '-- actions --'
    timeout 5s ros2 action list -t 2>&1 || true
    echo
    sleep 10
done
"

    for topic in /picky2/odom /picky2/scan /picky2/joint_states /picky2/cmd_vel /picky2/cmd_vel_nav; do
        safe_name="${topic#/}"
        safe_name="${safe_name//\//_}"
        start_bg "hz_${safe_name}" ros2 topic hz "$topic"
    done

    for topic in /picky2/picky_state /picky2/battery/percent /picky2/amcl_pose; do
        safe_name="${topic#/}"
        safe_name="${safe_name//\//_}"
        start_bg "echo_${safe_name}" ros2 topic echo "$topic"
    done
fi

if sudo -n true 2>/dev/null; then
    start_shell_bg "dmesg_filtered" \
        "sudo dmesg -Tw | grep --line-buffered -iE 'ttyAMA5|ttyS0|serial|dynamixel|voltage|under|reset|disconnect|over-current|thrott'"
else
    {
        echo "sudo -n unavailable. Run this in another terminal if kernel log is needed:"
        echo "  sudo dmesg -Tw | grep -iE 'ttyAMA5|ttyS0|serial|dynamixel|voltage|under|reset|disconnect|over-current|thrott'"
        echo
        dmesg -Tw 2>&1 | grep -iE 'ttyAMA5|ttyS0|serial|dynamixel|voltage|under|reset|disconnect|over-current|thrott' || true
    } >"$OUT_DIR/dmesg_filtered.log"
fi

log "recording to $OUT_DIR"
log "stop with Ctrl+C after the issue happens"

while true; do
    sleep 3600
done
