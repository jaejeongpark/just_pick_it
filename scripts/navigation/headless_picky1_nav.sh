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
source "$WS_ROOT/scripts/dds_env.sh"   # 디스커버리 서버 env(공용)
export ROS_DOMAIN_ID=25

DEFAULT_MAP="$WS_ROOT/src/pinky_pro/pinky_navigation/map/sync_map.yaml"
MAP_PATH="${1:-$DEFAULT_MAP}"

if [[ ! -f "$MAP_PATH" ]]; then
    echo "맵 파일을 찾을 수 없습니다: $MAP_PATH" >&2
    exit 1
fi

echo "=== [PICKY1] Nav2 — 로봇 토픽 대기 (/picky1/scan, /picky1/odom) ==="
# 디스커버리 서버에선 그래프 조회(topic list)에 super client 가 필요하다(데이터 매칭은
# 일반 클라이언트로 되지만 "토픽 목록"은 full graph 라야 보임). --no-daemon 으로
# 데몬 상태와 무관하게 현재 env(서버)로 조회한다. nav 노드 자체는 일반 클라이언트.
until topics=$(ROS_SUPER_CLIENT=true ros2 topic list --no-daemon 2>/dev/null); \
      echo "$topics" | grep -q '/picky1/scan' && echo "$topics" | grep -q '/picky1/odom'; do
    printf '\r[대기] bringup의 /picky1/scan, /picky1/odom 확인 중...'
    sleep 2
done
echo -e "\n[완료] 로봇 토픽 확인됨"

echo "맵: $MAP_PATH"
echo "=== [PICKY1] Nav2 기동 (namespace /picky1, ROS_DOMAIN_ID=$ROS_DOMAIN_ID) ==="
# pinky_amr_1 picky1_nav.launch.py 를 쓴다. 이 launch 는 nav2_bringup 표준
# bringup_launch.py 를 use_namespace:=True 로 호출해 RewrittenYaml 로 params 에
# namespace 를 주입한다. pinky_navigation 의 XML bringup_launch 는 params 를 raw 로
# 로드해 /picky1 노드에 nav2_params 가 안 붙고 controller 가 기본 DWB(critics
# 없음)로 떠서 죽던 문제가 있었다.
# use_composition:=True : nav2 노드 11개를 단일 component_container_mt 에
# ComposableNode 로 올려 DDS participant 를 1개로 합친다. 2대 동시 가동 시
# participant 별 DDS 전송 스레드 폴링으로 보드 CPU 가 포화돼 nav goal 이
# 상위 로직에서 cancel 되던 과부하를 줄이기 위함. 컴포넌트 11개 전부 로드+활성
# 확인됨(과거 "ARM 로드 실패"는 nav2_bringup 표준의 route_server/collision_monitor
# 문제였고 이 커스텀 launch 와 무관).
exec ros2 launch pinky_amr_1 picky1_nav.launch.py \
    namespace:=picky1 \
    map:="$MAP_PATH" \
    use_composition:=True
