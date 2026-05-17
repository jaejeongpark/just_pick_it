#!/bin/bash
# Gazebo 시뮬레이션에서 AprilTag 기반 pinky_pro 6DOF 자세 추정 자동화 스크립트
# 사용법: bash scripts/marker_odom/sim_apriltag_pose_estimation.sh
#
# 레이아웃:
#   ┌──────────────┬──────────────┐
#   │  1. Gazebo   │  4. RViz     │
#   ├──────────────┼──────────────┤
#   │  2. Estimator│  3. Teleop   │
#   └──────────────┴──────────────┘
#
# 자동: 1(Gazebo) 시작 시 /clock 발행
#       2(Estimator)는 /clock 과 /camera/image_raw 등장 시 시작
#       4(RViz)는 /clock 등장 시 시작 (map, april_odom TF 및 연결선 표시)
# 수동: 3(Teleop)으로 pinky_pro 주행 (cmd_vel 발행: i, ,, j, l, k 키)

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TMP_CONFIG=$(mktemp /tmp/terminator_sim_apriltag_pose_XXXXXX.cfg)
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
  [[sim_apriltag_pose]]
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
      command = $SCRIPT_DIR/pane1_gazebo_apriltag.sh
      title = 1. Gazebo
    [[[terminal_estimator]]]
      type = Terminal
      parent = vpaned_left
      command = $SCRIPT_DIR/pane2_apriltag_estimator.sh
      title = 2. Estimator
    [[[vpaned_right]]]
      type = VPaned
      parent = hpaned0
      ratio = 0.5
    [[[terminal_rviz]]]
      type = Terminal
      parent = vpaned_right
      command = $SCRIPT_DIR/pane4_rviz_apriltag.sh
      title = 4. RViz
    [[[terminal_teleop]]]
      type = Terminal
      parent = vpaned_right
      command = $SCRIPT_DIR/pane3_teleop.sh
      title = 3. Teleop
[plugins]
EOF

terminator --no-dbus --config "$TMP_CONFIG" --layout sim_apriltag_pose
