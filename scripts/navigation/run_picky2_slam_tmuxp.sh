#!/usr/bin/env bash
# PICKY2 SLAM 맵 작성 스택을 tmuxp 세션 하나에 띄운다.
# 첫 pane: SSH로 로봇에 접속해 bringup 실행. 비밀번호는 사용자가 직접 입력한다.
# 나머지 pane: 현재 PC에서 TF check / SLAM / teleop / RViz / map saver memo 실행.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
CONFIG="$WS_DIR/scripts/tmuxp/picky2_slam.yaml"
SESSION="picky2-slam"

ROBOT_SSH="${1:-${PICKY2_ROBOT_SSH:-pinky@192.168.1.93}}"
export PICKY2_ROBOT_SSH="$ROBOT_SSH"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-25}"

if ! command -v tmux >/dev/null 2>&1; then
    echo "tmux 가 설치돼 있지 않습니다. 설치: sudo apt install -y tmux" >&2
    exit 1
fi

if ! command -v tmuxp >/dev/null 2>&1; then
    echo "tmuxp 가 설치돼 있지 않습니다. 설치 예: python3 -m pip install --user tmuxp" >&2
    exit 1
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "이미 '$SESSION' tmux 세션이 있습니다. 붙으려면: tmux attach -t $SESSION"
    echo "전체 종료 후 다시 띄우려면: tmux kill-session -t $SESSION"
    exit 0
fi

cd "$WS_DIR"
echo "PICKY2 로봇 SSH 대상: $PICKY2_ROBOT_SSH"
echo "첫 pane에서 비밀번호를 물어보면 로봇 비밀번호를 입력하세요."
exec tmuxp load "$CONFIG"
