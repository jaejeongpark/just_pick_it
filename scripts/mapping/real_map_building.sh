#!/bin/bash
# 실제 로봇 SLAM 맵핑 자동화 스크립트
# 사용법: bash scripts/real_map_building.sh
#
# 레이아웃:
#   ┌──────────────┬──────────────┐
#   │  1. Bringup  │  3. RViz     │
#   ├──────────────┼──────────────┤
#   │  2. SLAM     │  4. Teleop   │
#   ├──────────────┴──────────────┤
#   │       5. Map Saver          │
#   └─────────────────────────────┘
#
# 자동: 1(Bringup) → /scan+/odom+/imu 모두 감지 시 2(SLAM) → /map 감지 시 3(RViz)
# 수동: 4(Teleop)으로 주행 → 5(Map Saver)에서 이름 입력 후 저장

terminator --layout real_map_building
