#!/bin/bash
# 실제 로봇 Navigation 자동화 스크립트
# 사용법: bash scripts/navigation/real_navigation.sh
#
# 레이아웃:
#   ┌──────────────┬──────────────┐
#   │  1. Bringup  │  3. RViz     │
#   ├──────────────┴──────────────┤
#   │     2. Nav2 (맵 선택 후 실행)│
#   └─────────────────────────────┘
#
# 자동: 1(Bringup) → /scan+/odom+/imu 감지 시 맵 선택 후 2(Nav2) → /amcl_pose 감지 시 3(RViz)

terminator --layout real_navigation
