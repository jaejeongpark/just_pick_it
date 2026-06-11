#!/bin/bash
# PICKY2 헤드리스 State Machine (모니터 없는 로봇 보드 / SSH 접속용)
set -e

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source ~/venv/jazzy/bin/activate 2>/dev/null || true
source /opt/ros/jazzy/setup.bash
source "$WS_ROOT/install/setup.bash"
export ROS_DOMAIN_ID=25

wait_lifecycle_active() {
    local node="$1"
    echo "=== [PICKY2] $node active 대기 ==="
    until timeout 5 ros2 lifecycle get "$node" 2>/dev/null | grep -q 'active \[3\]'; do
        printf '\r[대기] %s lifecycle active 확인 중...' "$node"
        sleep 2
    done
    echo -e "\n[완료] $node active"
}

wait_action_server() {
    local action="$1"
    echo "=== [PICKY2] $action action server 대기 ==="
    until timeout 5 ros2 action info "$action" 2>/dev/null | grep -q 'Action servers: 1'; do
        printf '\r[대기] %s action server 확인 중...' "$action"
        sleep 2
    done
    echo -e "\n[완료] $action action server ready"
}

wait_lifecycle_active /picky2/map_server
wait_lifecycle_active /picky2/amcl
wait_lifecycle_active /picky2/planner_server
wait_lifecycle_active /picky2/controller_server
wait_lifecycle_active /picky2/bt_navigator

wait_action_server /picky2/compute_path_to_pose
wait_action_server /picky2/follow_path
wait_action_server /picky2/navigate_to_pose

echo "=== [PICKY2] State Machine (namespace /picky2, ROS_DOMAIN_ID=$ROS_DOMAIN_ID) ==="
exec ros2 launch pinky_amr_2 picky2_state_machine.launch.py
