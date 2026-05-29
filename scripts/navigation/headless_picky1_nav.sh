#!/bin/bash
# PICKY1 헤드리스 Nav2 (모니터 없는 로봇 보드 / SSH 접속용)
#
# Nav2 스택을 /picky1 네임스페이스로 띄운다. 그래야 State Manager의 move_to_goal
# 이 보내는 /picky1/navigate_to_pose 액션 서버가 제공된다(주문 시나리오 연동에 필수).
# RViz는 띄우지 않는다. 위치추정(2D Pose Estimate)은 같은 ROS_DOMAIN_ID 로
# 접속한 다른 PC의 RViz에서 /picky1/initialpose 로 수행한다.
#
# 사용법:
#   bash scripts/navigation/headless_picky1_nav.sh [맵.yaml 경로]
# 맵 경로를 생략하면 sync_map.yaml(기본 운영 맵)을 사용한다.
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

echo "=== [PICKY1] Nav2 — 로봇 토픽 대기 (/picky1/scan, /picky1/odom) ==="
until ros2 topic list 2>/dev/null | grep -q '/picky1/scan' && \
      ros2 topic list 2>/dev/null | grep -q '/picky1/odom'; do
    printf '\r[대기] bringup의 /picky1/scan, /picky1/odom 확인 중...'
    sleep 2
done
echo -e "\n[완료] 로봇 토픽 확인됨"

echo "맵: $MAP_PATH"
echo "=== [PICKY1] Nav2 기동 (namespace /picky1, ROS_DOMAIN_ID=$ROS_DOMAIN_ID) ==="
exec ros2 launch pinky_navigation bringup_launch.xml \
    namespace:=picky1 \
    map:="$MAP_PATH"
