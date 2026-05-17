#!/bin/bash
# Gazebo 시뮬레이션 마커 오도메트리용 카메라 캡처 + 텔레옵 자동화 스크립트
# 사용법: bash scripts/marker_odom/sim_marker_odom.sh
#
# 레이아웃:
#   ┌──────────────┬──────────────┐
#   │  1. Gazebo   │  4. RViz     │
#   ├──────────────┼──────────────┤
#   │  2. Capture  │  3. Teleop   │
#   └──────────────┴──────────────┘
#
# 자동: 1(Gazebo) 시작 → /clock 감지 시 2(Capture), 4(RViz) 시작
#       2(Capture)는 /camera/image_raw 추가 대기
# 수동: 3(Teleop)으로 pinky_pro 주행 (/cmd_vel 발행)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TMP_CONFIG=$(mktemp /tmp/terminator_sim_marker_odom_XXXXXX.cfg)
trap "rm -f '$TMP_CONFIG'" EXIT

cat > "$TMP_CONFIG" << EOF
[global_config]
[keybindings]
[profiles]
  [[default]]
    scrollback_lines = 2000
    use_system_font = True
    exit_action = hold
[layouts]
  [[sim_marker_odom]]
    [[[window0]]]
      type = Window
      parent = ""
      size = 1600, 960
    [[[hpaned0]]]
      type = HPaned
      parent = window0
      ratio = 0.5
    [[[vpaned_left]]]
      type = VPaned
      parent = hpaned0
      ratio = 0.5
    [[[terminal_gazebo]]]
      type = Terminal
      parent = vpaned_left
      command = $SCRIPT_DIR/pane1_gazebo.sh
      title = 1. Gazebo
    [[[terminal_capture]]]
      type = Terminal
      parent = vpaned_left
      command = $SCRIPT_DIR/pane2_capture.sh
      title = 2. Capture
    [[[vpaned_right]]]
      type = VPaned
      parent = hpaned0
      ratio = 0.5
    [[[terminal_rviz]]]
      type = Terminal
      parent = vpaned_right
      command = $SCRIPT_DIR/pane4_rviz.sh
      title = 4. RViz
    [[[terminal_teleop]]]
      type = Terminal
      parent = vpaned_right
      command = $SCRIPT_DIR/pane3_teleop.sh
      title = 3. Teleop
[plugins]
EOF

terminator --no-dbus --config "$TMP_CONFIG" --layout sim_marker_odom
