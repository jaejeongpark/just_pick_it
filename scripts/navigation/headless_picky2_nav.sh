#!/bin/bash
# PICKY2 헤드리스 Nav2 (모니터 없는 로봇 보드 / SSH 접속용)
set -e

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source ~/venv/jazzy/bin/activate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
if [[ "${USE_DDS:-1}" != "0" && -f "$WS_ROOT/scripts/dds_env.sh" ]]; then
    source "$WS_ROOT/scripts/dds_env.sh"   # 디스커버리 서버 env(공용)
fi

DEFAULT_MAP="$WS_ROOT/src/pinky_pro/pinky_navigation/map/sync_map.yaml"
MAP_PATH="${1:-$DEFAULT_MAP}"

if [[ ! -f "$MAP_PATH" ]]; then
    echo "맵 파일을 찾을 수 없습니다: $MAP_PATH" >&2
    exit 1
fi

echo "=== [PICKY2] Nav2 — 로봇 토픽 대기 (/picky2/scan, /picky2/odom) ==="
# Discovery Server 모드에서는 일반 ros2 topic list가 전체 그래프를 못 볼 수 있어
# super client + --no-daemon으로 현재 env 기준 그래프를 직접 확인한다.
until topics=$(ROS_SUPER_CLIENT=true ros2 topic list --no-daemon 2>/dev/null); \
      echo "$topics" | grep -q '/picky2/scan' && echo "$topics" | grep -q '/picky2/odom'; do
    printf '\r[대기] bringup의 /picky2/scan, /picky2/odom 확인 중...'
    sleep 2
done
echo -e "\n[완료] 로봇 토픽 확인됨"

echo "맵: $MAP_PATH"
echo "=== [PICKY2] Nav2 기동 (namespace /picky2, ROS_DOMAIN_ID=$ROS_DOMAIN_ID) ==="
exec ros2 launch pinky_amr_2 picky2_nav.launch.py \
    namespace:=picky2 \
    map:="$MAP_PATH" \
    use_composition:=False
