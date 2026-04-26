#!/bin/bash
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../" && pwd)"
source ~/venv/jazzy/bin/activate 2>/dev/null
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/local_setup.bash"
export ROS_DOMAIN_ID=13

MAP_DIR="$WS_ROOT/src/pinky_pro/pinky_navigation/map"
mkdir -p "$MAP_DIR"

echo "=== [5] Map Saver ==="
echo "저장 위치: $MAP_DIR"
echo "맵 수집 완료 후 이름을 입력하고 Enter 누르세요."
echo ""

while true; do
    read -p "맵 이름 (빈칸 Enter = 종료): " map_name
    [[ -z "$map_name" ]] && echo "종료합니다." && break
    ros2 run nav2_map_server map_saver_cli -f "$MAP_DIR/$map_name"
    echo ""
    echo "============================================"
    echo " 저장 완료"
    echo "  YAML : $MAP_DIR/${map_name}.yaml"
    echo "  PGM  : $MAP_DIR/${map_name}.pgm"
    ls -lh "$MAP_DIR/${map_name}.yaml" "$MAP_DIR/${map_name}.pgm" 2>/dev/null
    echo "============================================"
    echo ""
done
exec bash
