#!/bin/bash
# PICKY2 헤드리스 Nav2 (모니터 없는 로봇 보드 / SSH 접속용)
set -e

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source ~/venv/jazzy/bin/activate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
export ROS_DOMAIN_ID=25

DEFAULT_MAP="$WS_ROOT/src/pinky_pro/pinky_navigation/map/sync_map.yaml"
MAP_PATH="${1:-$DEFAULT_MAP}"

if [[ ! -f "$MAP_PATH" ]]; then
    echo "맵 파일을 찾을 수 없습니다: $MAP_PATH" >&2
    exit 1
fi

wait_topic_msg() {
    local topic="$1"
    echo "=== [PICKY2] $topic 메시지 대기 ==="
    until timeout 5 ros2 topic echo "$topic" --once >/dev/null 2>&1; do
        printf '\r[대기] %s 실제 메시지 수신 확인 중...' "$topic"
        sleep 2
    done
    echo -e "\n[완료] $topic 메시지 수신됨"
}

wait_topic_msg /picky2/scan
wait_topic_msg /picky2/odom

echo "맵: $MAP_PATH"
echo "=== [PICKY2] Nav2 기동 (namespace /picky2, ROS_DOMAIN_ID=$ROS_DOMAIN_ID) ==="
exec ros2 launch pinky_amr_2 picky2_nav.launch.py \
    namespace:=picky2 \
    map:="$MAP_PATH" \
    use_composition:=False
