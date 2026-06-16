#!/bin/bash
# Fast-DDS Discovery Server — 시스템 전체(Fleet + 로봇들)의 디스커버리 인프라.
#
# 이 시스템의 모든 ROS2 통신(fleet_manager telemetry 구독, MoveCommand/DockCommand/
# ExecuteTask 액션, EmergencyControl 서비스, 로봇 bringup/nav/state/docking, web)이
# 같은 ROS_DOMAIN_ID(25)에서 도는데, WiFi 기본 멀티캐스트 디스커버리가 보드2+PC 의
# 40+ participant 전수 통보로 폭주 → nav2 lifecycle 활성 실패·battery 발행 지연.
#
# 해결: 관제 PC(항상 켜둠)에 이 디스커버리 서버 1대를 띄우고, 시스템의 모든 호스트
# (관제 PC + picky1 보드 + picky2 보드)가 .bashrc 에서
#     export ROS_DISCOVERY_SERVER="<관제 PC IP>:11811"
# 로 이 서버에만 유니캐스트로 붙는다. 그러면 멀티캐스트 폭주가 사라진다.
# (데이터 전송은 그대로 peer-to-peer, 디스커버리 경로만 서버 경유). ROS_DOMAIN_ID=25 유지.
#
# 주의:
#  - 이 서버는 "모든 노드 기동 전에" 떠 있어야 한다(항상 켜두는 게 안전). 죽으면 신규
#    디스커버리만 멈춘다(이미 매칭된 연결은 유지). 단일장애점이라 관제 PC 부팅 시 자동
#    기동(systemd/유저서비스) 권장.
#  - ros2 CLI(ros2 node/topic list)로 전체 그래프를 보려면 그 셸에
#    export ROS_SUPER_CLIENT=true 후 ros2 daemon stop (서버 클라이언트는 기본적으로
#    필요한 것만 디스커버리하므로).
#
# 사용:
#   bash scripts/discovery_server.sh          # 포트 11811(기본)
#   bash scripts/discovery_server.sh 11888    # 포트 지정
set -e
source /opt/ros/jazzy/setup.bash
PORT="${1:-11811}"
echo "=== Fast-DDS Discovery Server (server-id=0, port=${PORT}, listen 0.0.0.0) ==="
echo "모든 호스트(.bashrc):  export ROS_DISCOVERY_SERVER=\"<관제 PC IP>:${PORT}\""
echo "CLI 전체그래프:        export ROS_SUPER_CLIENT=true  (그 후 ros2 daemon stop)"
exec fastdds discovery -i 0 -p "${PORT}"
