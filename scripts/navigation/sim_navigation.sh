#!/bin/bash
# Gazebo 시뮬레이션 Navigation 자동화 스크립트
# 사용법: bash scripts/navigation/sim_navigation.sh
#
# 레이아웃:
#   ┌──────────────┬──────────────┐
#   │  1. Gazebo   │  3. RViz     │
#   ├──────────────┴──────────────┤
#   │     2. Nav2 (맵 선택 후 실행)│
#   └─────────────────────────────┘
#
# 자동: 1(Gazebo) → /clock 감지 시 맵 선택 후 2(Nav2) → /amcl_pose 감지 시 3(RViz)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TMP_CONFIG=$(mktemp /tmp/terminator_sim_nav_XXXXXX.cfg)
trap "rm -f '$TMP_CONFIG'" EXIT

cat > "$TMP_CONFIG" << EOF
[global_config]
[keybindings]
[profiles]
  [[default]]
    scrollback_lines = 2000
    use_system_font = True
[layouts]
  [[sim_navigation]]
    [[[window0]]]
      type = Window
      parent = ""
      size = 1600, 960
    [[[vpaned0]]]
      type = VPaned
      parent = window0
      ratio = 0.6
    [[[hpaned0]]]
      type = HPaned
      parent = vpaned0
      ratio = 0.5
    [[[terminal_gazebo]]]
      type = Terminal
      parent = hpaned0
      command = $SCRIPT_DIR/sim_pane1_gazebo.sh
      title = 1. Gazebo
    [[[terminal_rviz]]]
      type = Terminal
      parent = hpaned0
      command = $SCRIPT_DIR/sim_pane3_rviz.sh
      title = 3. RViz
    [[[terminal_nav]]]
      type = Terminal
      parent = vpaned0
      command = $SCRIPT_DIR/sim_pane2_nav.sh
      title = 2. Nav2
[plugins]
EOF

terminator --no-dbus --config "$TMP_CONFIG" --layout sim_navigation
