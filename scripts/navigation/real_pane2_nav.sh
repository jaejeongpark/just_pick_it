#!/bin/bash
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source ~/venv/jazzy/bin/activate 2>/dev/null
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID=13

MAP_DIR="$WS_ROOT/src/pinky_pro/pinky_navigation/map"

echo "=== [2] Nav2 — 로봇 준비 대기 중 (/scan + /odom + /imu) ==="
until ros2 topic list 2>/dev/null | grep -q '/scan' && \
      ros2 topic list 2>/dev/null | grep -q '/odom' && \
      ros2 topic list 2>/dev/null | grep -q '/imu'; do
    printf '\r[대기] /scan, /odom, /imu 모두 확인될 때까지 대기...'
    sleep 2
done
echo -e "\n[완료] 로봇 준비 완료"

echo ""
mapfile -t map_files < <(ls "$MAP_DIR"/*.yaml 2>/dev/null)
if [[ ${#map_files[@]} -eq 0 ]]; then
    echo "맵 없음 — $MAP_DIR 에 .yaml 파일이 없습니다."
    exec bash
fi

echo "사용 가능한 맵 ($MAP_DIR):"
for i in "${!map_files[@]}"; do
    echo "  $((i+1)). $(basename "${map_files[$i]}")"
done
echo ""

while true; do
    read -p "번호 선택 (1-${#map_files[@]}): " sel
    if [[ "$sel" =~ ^[0-9]+$ ]] && (( sel >= 1 && sel <= ${#map_files[@]} )); then
        MAP_PATH="${map_files[$((sel-1))]}"
        break
    fi
    echo "올바른 번호를 입력하세요."
done

echo "→ 맵 파일: $MAP_PATH"
echo ""
ros2 launch pinky_navigation bringup_launch.xml map:="$MAP_PATH"
exec bash
