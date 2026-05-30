#!/bin/bash
# PICKY1 주행 스택 전체를 tmux 세션 하나에 띄운다 (bringup / nav / state).
#
# 명령 하나로 3개 노드를 각각 별도 tmux 창에 띄운다. 로그는 창별로 분리되고,
# tmux 라 SSH 가 끊겨도 노드가 살아있다(Ctrl+b d 로 떼기). 한 창만 재시작할 수도 있다.
#
# 사용법:
#   bash scripts/navigation/run_picky1_all.sh     # 세션 생성 후 자동 attach
#   tmux attach -t picky1                          # 나중에 다시 붙기
#   tmux kill-session -t picky1                    # 전체 종료
#
# tmux 단축키: 창 전환 Ctrl+b 0/1/2 | 떼기(유지) Ctrl+b d | 스크롤 Ctrl+b [
set -e

SESSION=picky1
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

# 창1 bringup, 창2 nav, 창3 state. 각 스크립트가 source/도메인 설정을 알아서 한다.
tmux new-session -d -s "$SESSION" -n bringup "bash '$SCRIPT_DIR/headless_picky1_bringup.sh'"
tmux new-window  -t "$SESSION"   -n nav      "bash '$SCRIPT_DIR/headless_picky1_nav.sh'"
tmux new-window  -t "$SESSION"   -n state    "bash '$SCRIPT_DIR/headless_picky1_state.sh'"

# 노드가 죽어도 창을 닫지 않아 로그를 확인할 수 있다.
tmux set-option -t "$SESSION" remain-on-exit on
tmux select-window -t "$SESSION:bringup"

echo "tmux 세션 '$SESSION' 시작 (창: bringup / nav / state)"
echo "  창 전환 Ctrl+b 0/1/2 | 떼기(노드 유지) Ctrl+b d | 전체 종료 tmux kill-session -t $SESSION"

# 바로 화면에 붙는다. 백그라운드로 두고 싶으면 아래 줄을 지운다.
tmux attach -t "$SESSION"
