#!/bin/bash
# PICKY1 관제용 RViz (관제 PC에서 실행)
#
# 보드가 /picky1 네임스페이스로 TF/토픽을 발행하므로, RViz가 그것을 보도록
# tf/tf_static/initialpose/goal_pose 를 remap 한다.
# robot_description 은 robot_state_publisher 가 글로벌로 발행하므로 remap 하지
# 않는다(remap 하면 RobotModel 이 안 뜬다).
#
# 사용법: bash scripts/navigation/rviz_picky1.sh
#
# RViz 안에서:
#   - Fixed Frame: 위치추정 전이면 odom, 위치추정 후면 map
#   - Add > By topic: /picky1/map (Map), /picky1/scan (LaserScan)
#   - RobotModel 추가 시 Description Topic 은 /robot_description
#   - 2D Pose Estimate 로 위치추정 (자동으로 /picky1/initialpose 로 발행됨)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/../../install/setup.bash"
export ROS_DOMAIN_ID=25

# 저장된 RViz 설정(picky1.rviz)을 불러온다. Map Durability=Transient Local,
# Fixed Frame, Map/LaserScan/RobotModel Display 가 미리 세팅돼 있어 매번 수동
# 설정할 필요가 없다.
exec ros2 run rviz2 rviz2 -d "$SCRIPT_DIR/picky1.rviz" --ros-args \
  -r /tf:=/picky1/tf \
  -r /tf_static:=/picky1/tf_static \
  -r /initialpose:=/picky1/initialpose \
  -r /goal_pose:=/picky1/goal_pose
