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
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v tmux >/dev/null 2>&1; then
    echo "tmux 가 설치돼 있지 않습니다. 설치: sudo apt install -y tmux" >&2
    exit 1
fi

if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "이미 '$SESSION' tmux 세션이 있습니다. 붙으려면: tmux attach -t $SESSION"
    echo "전체 종료 후 다시 띄우려면: tmux kill-session -t $SESSION"
    exit 0
fi

tmux new-session -d -s "$SESSION" -n bringup "bash '$SCRIPT_DIR/headless_picky2_bringup.sh'"
tmux new-window  -t "$SESSION"   -n nav      "bash '$SCRIPT_DIR/headless_picky2_nav.sh'"
tmux new-window  -t "$SESSION"   -n state    "bash '$SCRIPT_DIR/headless_picky2_state.sh'"
tmux set-option -t "$SESSION" remain-on-exit on
tmux select-window -t "$SESSION:bringup"

echo "tmux 세션 '$SESSION' 시작 (창: bringup / nav / state)"
echo "  창 전환 Ctrl+b 0/1/2 | 떼기(노드 유지) Ctrl+b d | 전체 종료 tmux kill-session -t $SESSION"

tmux attach -t "$SESSION"
