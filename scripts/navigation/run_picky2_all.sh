#!/bin/bash
# PICKY2 주행 스택 전체를 tmux 세션 하나에 띄운다 (bringup / nav / state).
#
# 사용법:
#   bash scripts/navigation/run_picky2_all.sh     # 세션 생성 후 자동 attach
#   tmux attach -t picky2                         # 나중에 다시 붙기
#   tmux kill-session -t picky2                   # 전체 종료
#
# tmux 단축키: 창 전환 Ctrl+b 0/1/2 | 떼기(유지) Ctrl+b d | 스크롤 Ctrl+b [
set -e

SESSION=picky2
WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
SETUP_CMD="cd '$WS_ROOT'; source ~/venv/jazzy/bin/activate 2>/dev/null || true; source /opt/ros/jazzy/setup.bash; source install/setup.bash; export ROS_DOMAIN_ID=25"
MAP_PATH="$WS_ROOT/src/pinky_pro/pinky_navigation/map/sync_map.yaml"

if ! command -v tmux >/dev/null 2>&1; then
    echo "tmux 가 설치돼 있지 않습니다. 설치: sudo apt install -y tmux" >&2
    exit 1
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "이미 '$SESSION' tmux 세션이 있습니다. 붙으려면: tmux attach -t $SESSION"
    echo "전체 종료 후 다시 띄우려면: tmux kill-session -t $SESSION"
    exit 0
fi

tmux new-session -d -s "$SESSION" -n bringup \
    "bash -lc \"$SETUP_CMD; echo '=== [PICKY2] Bringup (/picky2) ==='; ros2 launch pinky_amr_2 picky2_bringup.launch.py; bash\""

tmux new-window -t "$SESSION" -n nav \
    "bash -lc \"$SETUP_CMD; echo '=== [PICKY2] Nav2 — waiting /picky2/scan, /picky2/odom ==='; until ros2 topic list 2>/dev/null | grep -q '/picky2/scan' && ros2 topic list 2>/dev/null | grep -q '/picky2/odom'; do sleep 2; done; echo 'map: $MAP_PATH'; ros2 launch pinky_amr_2 picky2_nav.launch.py namespace:=picky2 map:='$MAP_PATH' use_composition:=False; bash\""

tmux new-window -t "$SESSION" -n state \
    "bash -lc \"$SETUP_CMD; echo '=== [PICKY2] State Machine (/picky2) ==='; ros2 launch pinky_amr_2 picky2_state_machine.launch.py; bash\""

tmux set-option -t "$SESSION" remain-on-exit on
tmux select-window -t "$SESSION:bringup"

echo "tmux 세션 '$SESSION' 시작 (창: bringup / nav / state)"
echo "  창 전환 Ctrl+b 0/1/2 | 떼기(노드 유지) Ctrl+b d | 전체 종료 tmux kill-session -t $SESSION"

tmux attach -t "$SESSION"
