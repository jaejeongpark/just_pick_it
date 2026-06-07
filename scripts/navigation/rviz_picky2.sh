#!/bin/bash
# PICKY2 관제용 RViz (관제 PC에서 실행)
#
# PICKY2 전용 RViz 설정을 쓰되, 보드가 /picky2 네임스페이스로 발행하는
# TF/스캔/맵/코스트맵/플랜/파티클을 보도록 글로벌 토픽을 /picky2/... 로 remap 한다.
# namespace 파라미터화 이후 robot_state_publisher 가 /picky2 네임스페이스로 들어가
# robot_description 을 /picky2/robot_description 으로 발행한다. RobotModel 디스플레이는
# .rviz 파일에 박힌 토픽 값으로 직접 구독해 --ros-args remap 이 안 먹으므로,
# scripts/navigation/picky2.rviz 안에도 /picky2/robot_description 을 직접 적어둔다.
#
# 사용법: bash scripts/navigation/rviz_picky2.sh
#
# RViz 안에서:
#   - Fixed Frame 은 map (위치추정 전이면 데이터가 안 맞을 수 있음)
#   - 2D Pose Estimate 로 위치추정 (자동으로 /picky2/initialpose 로 발행됨)
#   - 2D Goal Pose 는 /picky2/goal_pose 로 발행됨
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/../../install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-25}"

RVIZ_CONFIG="$SCRIPT_DIR/picky2.rviz"
NS=/picky2

# RViz 가 구독/발행하는 글로벌 토픽 전부를 /picky2 네임스페이스로 remap.
exec ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" --ros-args \
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
