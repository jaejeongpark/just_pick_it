#!/bin/bash
# PICKY2 관제용 RViz (관제 PC에서 실행)
#
# pinky_navigation 패키지의 nav2_view.rviz(pinkypro 현장 튜닝본)를 원본 그대로 두고,
# 실행 때 임시 복사본의 디스플레이 토픽만 /picky2/... 로 바꿔 사용한다.
# RViz 디스플레이 토픽은 --ros-args remap 이 안 먹는 항목이 있어 .rviz 값 자체를
# 치환한다. 원본 pinky_pro 파일은 수정하지 않는다.
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

RVIZ_CONFIG="$(ros2 pkg prefix pinky_navigation)/share/pinky_navigation/rviz/nav2_view.rviz"
NS=/picky2
RVIZ_RUN="${TMPDIR:-/tmp}/picky2_nav_view.rviz"

sed "s#Value: /#Value: $NS/#" "$RVIZ_CONFIG" > "$RVIZ_RUN"

exec ros2 run rviz2 rviz2 -d "$RVIZ_RUN" --ros-args \
  -r __ns:=$NS \
  -r /tf:=$NS/tf \
  -r /tf_static:=$NS/tf_static
