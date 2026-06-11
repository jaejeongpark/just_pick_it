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
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-25}"

RVIZ_CONFIG="$SCRIPT_DIR/picky2.rviz"
NS=/picky2

exec ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" --ros-args \
  -p use_sim_time:=true \
  -r /tf:=$NS/tf \
  -r /tf_static:=$NS/tf_static \
  -r /scan:=$NS/scan \
  -r /map:=$NS/map \
  -r /map_updates:=$NS/map_updates \
  -r /initialpose:=$NS/initialpose \
  -r /goal_pose:=$NS/goal_pose \
  -r /clicked_point:=$NS/clicked_point \
  -r /particle_cloud:=$NS/particle_cloud \
  -r /plan:=$NS/plan \
  -r /local_plan:=$NS/local_plan \
  -r /waypoints:=$NS/waypoints \
  -r /marker:=$NS/marker \
  -r /global_costmap/costmap:=$NS/global_costmap/costmap \
  -r /global_costmap/costmap_updates:=$NS/global_costmap/costmap_updates \
  -r /global_costmap/published_footprint:=$NS/global_costmap/published_footprint \
  -r /global_costmap/voxel_marked_cloud:=$NS/global_costmap/voxel_marked_cloud \
  -r /local_costmap/costmap:=$NS/local_costmap/costmap \
  -r /local_costmap/costmap_updates:=$NS/local_costmap/costmap_updates \
  -r /local_costmap/published_footprint:=$NS/local_costmap/published_footprint \
  -r /local_costmap/voxel_marked_cloud:=$NS/local_costmap/voxel_marked_cloud \
  -r /downsampled_costmap:=$NS/downsampled_costmap \
  -r /downsampled_costmap_updates:=$NS/downsampled_costmap_updates
