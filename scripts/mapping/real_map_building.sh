#!/bin/bash
# 실제 로봇 SLAM 맵핑 자동화 스크립트
# 사용법: bash scripts/mapping/real_map_building.sh
#
# 레이아웃:
#   ┌──────────────┬──────────────┐
#   │  1. Bringup  │  3. RViz     │
#   ├──────────────┼──────────────┤
#   │  2. SLAM     │  4. Teleop   │
#   ├──────────────┴──────────────┤
#   │       5. Map Saver          │
#   └─────────────────────────────┘
#
# 자동: 1(Bringup) → /scan+/odom+/imu 모두 감지 시 2(SLAM) → /map 감지 시 3(RViz)
# 수동: 4(Teleop)으로 주행 → 5(Map Saver)에서 이름 입력 후 저장

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TMP_CONFIG=$(mktemp /tmp/terminator_real_map_XXXXXX.cfg)
trap "rm -f '$TMP_CONFIG'" EXIT

cat > "$TMP_CONFIG" << EOF
[global_config]
[keybindings]
[profiles]
  [[default]]
    scrollback_lines = 2000
    use_system_font = True
[layouts]
  [[real_map_building]]
    [[[window0]]]
      type = Window
      parent = ""
      size = 1600, 960
    [[[vpaned0]]]
      type = VPaned
      parent = window0
      ratio = 0.85
    [[[hpaned0]]]
      type = HPaned
      parent = vpaned0
      ratio = 0.5
    [[[vpaned1]]]
      type = VPaned
      parent = hpaned0
      ratio = 0.5
    [[[terminal_bringup]]]
      type = Terminal
      parent = vpaned1
      command = $SCRIPT_DIR/real_pane1_bringup.sh
      title = 1. Bringup
    [[[terminal_slam]]]
      type = Terminal
      parent = vpaned1
      command = $SCRIPT_DIR/real_pane2_slam.sh
      title = 2. SLAM
    [[[vpaned2]]]
      type = VPaned
      parent = hpaned0
      ratio = 0.5
    [[[terminal_rviz]]]
      type = Terminal
      parent = vpaned2
      command = $SCRIPT_DIR/real_pane3_rviz.sh
      title = 3. RViz
    [[[terminal_teleop]]]
      type = Terminal
      parent = vpaned2
      command = $SCRIPT_DIR/pane4_teleop.sh
      title = 4. Teleop
    [[[terminal_map_saver]]]
      type = Terminal
      parent = vpaned0
      command = $SCRIPT_DIR/pane5_map_saver.sh
      title = 5. Map Saver
[plugins]
EOF

terminator --no-dbus --config "$TMP_CONFIG" --layout real_map_building
