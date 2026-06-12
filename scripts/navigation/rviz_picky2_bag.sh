#!/bin/bash
# PICKY2 rosbag 분석용 RViz (관제 PC에서 실행)
#
# rosbag 재생은 기록 당시 timestamp 를 다시 publish 하므로 RViz 도 /clock 기준
# sim time 을 써야 TF_OLD_DATA 없이 과거 scan/map/costmap/plan 을 볼 수 있다.
#
# 사용법:
#   bash scripts/navigation/rviz_picky2_bag.sh
#
# 다른 터미널에서 예:
#   ros2 bag play bags/<bag_dir> --clock --rate 0.5 \
#     --exclude-topics /picky2/cmd_vel /picky2/cmd_vel_nav
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/../../install/setup.bash"

RVIZ_CONFIG="$(ros2 pkg prefix pinky_navigation)/share/pinky_navigation/rviz/nav2_view.rviz"
NS=/picky2
RVIZ_RUN="${TMPDIR:-/tmp}/picky2_nav_view_bag.rviz"

sed "s#Value: /#Value: $NS/#" "$RVIZ_CONFIG" > "$RVIZ_RUN"

exec ros2 run rviz2 rviz2 -d "$RVIZ_RUN" --ros-args \
  -p use_sim_time:=true \
  -r __ns:=$NS \
  -r /tf:=$NS/tf \
  -r /tf_static:=$NS/tf_static
