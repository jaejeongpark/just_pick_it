#!/bin/bash
# Gazebo 시뮬레이션 Navigation 자동화 스크립트
# 사용법: bash scripts/navigation/sim_navigation.sh
#
# 레이아웃:
#   ┌──────────────┬──────────────┐
#   │  1. Gazebo   │  3. RViz     │
#   ├──────────────┴──────────────┤
#   │     2. Nav2 (맵 선택 후 실행)│
#   └─────────────────────────────┘
#
# 자동: 1(Gazebo) → /clock 감지 시 맵 선택 후 2(Nav2) → /amcl_pose 감지 시 3(RViz)

terminator --layout sim_navigation
