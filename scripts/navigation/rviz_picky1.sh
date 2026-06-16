#!/bin/bash
# PICKY1 관제용 RViz (관제 PC에서 실행)
#
# pinky_navigation 패키지의 nav2_view.rviz(pinkypro 현장 튜닝본)를 그대로 쓰되,
# 보드가 /picky1 네임스페이스로 발행하는 TF/스캔/맵/코스트맵/플랜/파티클을 보도록
# 디스플레이 토픽을 전부 /picky1/... 로 맞춘다.
# RViz2 디스플레이/RobotModel 구독은 --ros-args -r remap 이 안 먹어서(글로벌 /scan,
# /map 등을 그대로 구독) 토픽이 안 맞았다. 그래서 remap 에 의존하지 않고 .rviz 설정
# 파일의 토픽 값 자체를 /picky1 로 박은 임시 복사본을 떠서 넘긴다. TF 리스너만은
# 네임스페이스와 무관하게 절대 /tf 를 구독하므로(tf2 기본 동작) 그것만 remap 으로 잡는다.
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

# nav2_view.rviz 의 모든 디스플레이 토픽 값(Value: /... 줄)을 /picky1 로 바꾼 임시
# 복사본 생성. Value: / 로 시작하는 줄은 전부 토픽이라(프레임은 leading slash 없음)
# 일괄 치환해도 안전하고, 이미 /picky1 인 값이 없어 중복 접두어가 생기지 않는다.
# robot_description 포함 scan/map/costmap/plan/particle/initialpose/clicked_point 등이
# 모두 /picky1/... 로 박힌다. 원본은 picky2 공용이라 보존하고 복사본만 수정한다.
RVIZ_RUN="${TMPDIR:-/tmp}/picky1_nav_view.rviz"
sed "s#Value: /#Value: $NS/#" "$RVIZ_CONFIG" > "$RVIZ_RUN"

# RViz 노드를 /picky1 네임스페이스 안에 띄운다. nav2_rviz_plugins 의 Navigation 2
# 패널과 GoalTool 은 navigate_to_pose 액션 / lifecycle / amcl 을 노드 네임스페이스
# 기준 상대이름으로 찾으므로 __ns 가 있어야 /picky1/... 로 해석돼 패널·2D Goal 툴이
# 동작한다(__ns 없으면 글로벌 /navigate_to_pose 를 보고 /picky1 을 못 찾음).
# 디스플레이 토픽은 위 임시 .rviz 복사본에서 이미 /picky1 로 박아 remap 이 필요 없다.
# 2D Pose Estimate / Publish Point 툴 토픽(initialpose, clicked_point)도 복사본에서
# /picky1 로 박혔다. 남는 건 tf2 리스너가 강제로 구독하는 절대 /tf 뿐이라 그것만 remap.
exec ros2 run rviz2 rviz2 -d "$RVIZ_RUN" --ros-args \
  -r __ns:=$NS \
  -r /tf:=$NS/tf \
  -r /tf_static:=$NS/tf_static
