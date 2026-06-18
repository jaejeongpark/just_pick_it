# 공용 DDS 디스커버리 env — 모든 fleet/로봇 launch 스크립트가 상단에서 source 한다.
# (비대화형 셸은 ~/.bashrc 를 안 읽으므로, 디스커버리 서버 설정을 여기서 명시한다.)
# IP/포트가 바뀌면 이 파일만 수정하면 전 노드에 반영된다.
#
# 아래 둘 중 실제 discovery_server.sh 를 띄운 관제 PC IP 하나만 활성화한다.
# - 공용/기존 관제 PC: 192.168.1.73
# - 현재 개발 PC:     192.168.1.33
#
# 활성화한 관제 PC에서 `bash scripts/discovery_server.sh` 가 떠 있어야 한다.
# (ROS_DISCOVERY_SERVER 설정 시 멀티캐스트가 꺼져 서버 없으면 디스커버리 자체가 안 됨)
export ROS_DOMAIN_ID=25

export ROS_DISCOVERY_SERVER="192.168.1.73:11811"
# export ROS_DISCOVERY_SERVER="192.168.1.33:11811"
