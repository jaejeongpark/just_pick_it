#!/bin/bash
set -e

TMP_CONFIG=$(mktemp /tmp/terminator_picky2_slam_XXXXXX.cfg)
trap "rm -f '$TMP_CONFIG'" EXIT

cat > "$TMP_CONFIG" << CFG
[global_config]
[keybindings]
[profiles]
  [[default]]
    scrollback_lines = 3000
    use_system_font = True
[layouts]
  [[picky2_slam]]
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
    [[[terminal_bringup]]]
      type = Terminal
      parent = vpaned_left
      title = 1. Robot Bringup
      command = bash -lc 'cd ~/just_pick_it; echo "SSH bringup: pinky@192.168.1.93"; ssh -t pinky@192.168.1.93 "cd ~/just_pick_it && source /opt/ros/jazzy/setup.bash && source install/setup.bash && ros2 launch pinky_amr_2 picky2_bringup.launch.py"; bash'
    [[[terminal_slam]]]
      type = Terminal
      parent = vpaned_left
      title = 2. SLAM
      command = bash -lc 'cd ~/just_pick_it; source /opt/ros/jazzy/setup.bash; source install/setup.bash; until ros2 topic list | grep -q "^/picky2/scan$"; do echo "waiting /picky2/scan..."; sleep 1; done; until timeout 5 bash -lc "ros2 topic echo /picky2/tf_static --once | grep -q rplidar_link"; do echo "waiting /picky2/tf_static rplidar_link..."; sleep 1; done; ros2 launch pinky_amr_2 picky2_slam.launch.py; bash'
    [[[vpaned_right]]]
      type = VPaned
      parent = hpaned0
      ratio = 0.5
    [[[terminal_teleop]]]
      type = Terminal
      parent = vpaned_right
      title = 3. Teleop
      command = bash -lc 'cd ~/just_pick_it; source /opt/ros/jazzy/setup.bash; source install/setup.bash; until ros2 topic list | grep -q "^/picky2/odom$"; do echo "waiting /picky2/odom..."; sleep 1; done; ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/picky2/cmd_vel; bash'
    [[[terminal_rviz]]]
      type = Terminal
      parent = vpaned_right
      title = 4. RViz
      command = bash -lc 'cd ~/just_pick_it; source /opt/ros/jazzy/setup.bash; source install/setup.bash; until ros2 topic list | grep -q "^/picky2/map$"; do echo "waiting /picky2/map..."; sleep 1; done; bash scripts/navigation/rviz_picky2.sh; bash'
[plugins]
CFG

exec terminator --no-dbus --config "$TMP_CONFIG" --layout picky2_slam
