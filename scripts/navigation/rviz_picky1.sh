#!/bin/bash
# PICKY1 관제용 RViz (관제 PC에서 실행)
#
# pinky_navigation 패키지의 nav2_view.rviz(pinkypro 현장 튜닝본)를 그대로 쓰되,
# 보드가 /picky1 네임스페이스로 발행하는 TF/스캔/맵/코스트맵/플랜/파티클을 보도록
# 글로벌 토픽을 /picky1/... 로 remap 한다.
# namespace 파라미터화 이후 robot_state_publisher 가 /picky1 네임스페이스로 들어가
# robot_description 을 /picky1/robot_description 으로 발행한다. RobotModel 디스플레이는
# .rviz 파일에 박힌 토픽 값으로 직접 구독해 --ros-args remap 이 안 먹으므로, 원본을
# 임시 복사본으로 떠서 그 토픽 값만 /picky1 로 바꿔 넘긴다(원본은 picky2 공용이라 보존).
#
# 사용법: bash scripts/navigation/rviz_picky1.sh
#
# RViz 안에서:
#   - Fixed Frame 은 nav2_view 기본값 map (위치추정 전이면 데이터가 안 맞을 수 있음)
#   - 2D Pose Estimate 로 위치추정 (자동으로 /picky1/initialpose 로 발행됨)
#   - 2D Goal Pose 는 /picky1/goal_pose 로 발행됨
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/../../install/setup.bash"
export ROS_DOMAIN_ID=25

# pinkypro 튜닝본 경로는 패키지 share 에서 찾는다(alias pinky 로 소싱하는 워크스페이스).
RVIZ_CONFIG="$(ros2 pkg prefix pinky_navigation)/share/pinky_navigation/rviz/nav2_view.rviz"
NS=/picky1

# RobotModel 의 Description Topic 값(/robot_description)만 /picky1 로 바꾼 임시 복사본 생성.
# 나머지 토픽은 아래 --ros-args remap 으로 처리되지만 RobotModel 은 remap 이 안 먹는다.
RVIZ_RUN="${TMPDIR:-/tmp}/picky1_nav_view.rviz"
sed "s#Value: /robot_description\$#Value: $NS/robot_description#" "$RVIZ_CONFIG" > "$RVIZ_RUN"

# nav2_view.rviz 가 구독/발행하는 글로벌 토픽 전부를 /picky1 네임스페이스로 remap.
exec ros2 run rviz2 rviz2 -d "$RVIZ_RUN" --ros-args \
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
