#!/usr/bin/env python3
"""A4 출력용 AprilTag board (tag36h11, 4x3, IDs 0-11) 생성 스크립트.

출력 방법:
  1. 이 스크립트를 실행하면 apriltag_board_a4.png 가 생성된다.
  2. 프린터 설정에서 "실제 크기(100%)" 또는 "맞춤 없음" 으로 출력한다.
     (비율 자동 조정 켜면 태그 크기가 달라져 캘리브레이션 결과가 틀린다.)
  3. 출력 후 태그 한 변 길이가 정확히 40mm 인지 자로 확인한다.
"""

import argparse
import os

import cv2
import numpy as np

# A4 치수
A4_W_MM, A4_H_MM = 210.0, 297.0

# 기본 보드 스펙
COLS, ROWS = 4, 3
DEFAULT_TAG_MM = 40.0
DEFAULT_SPACING_MM = 10.0
DEFAULT_DPI = 300


def mm2px(mm: float, dpi: int) -> int:
    return round(mm * dpi / 25.4)


def main():
    parser = argparse.ArgumentParser(description='A4 출력용 AprilTag board 생성')
    parser.add_argument('--output', default='apriltag_board_a4.png', help='출력 파일명')
    parser.add_argument('--tag-size-mm', type=float, default=DEFAULT_TAG_MM, help='태그 한 변 (mm)')
    parser.add_argument('--spacing-mm', type=float, default=DEFAULT_SPACING_MM, help='태그 간격 (mm)')
    parser.add_argument('--dpi', type=int, default=DEFAULT_DPI, help='출력 해상도')
    args = parser.parse_args()

    dpi = args.dpi
    tag_mm = args.tag_size_mm
    spacing_mm = args.spacing_mm

    # 보드 콘텐츠 영역 (mm 및 px)
    board_w_mm = COLS * tag_mm + (COLS - 1) * spacing_mm
    board_h_mm = ROWS * tag_mm + (ROWS - 1) * spacing_mm
    board_w_px = mm2px(board_w_mm, dpi)
    board_h_px = mm2px(board_h_mm, dpi)

    # A4 캔버스
    a4_w_px = mm2px(A4_W_MM, dpi)
    a4_h_px = mm2px(A4_H_MM, dpi)

    # 보드를 A4 중앙에 배치하기 위한 오프셋
    x_offset = (a4_w_px - board_w_px) // 2
    y_offset = (a4_h_px - board_h_px) // 2

    print(f'--- AprilTag Board 생성 ---')
    print(f'패밀리    : tag36h11')
    print(f'구성      : {COLS}x{ROWS} (IDs 0-{COLS * ROWS - 1})')
    print(f'태그 크기 : {tag_mm:.1f}mm')
    print(f'간격      : {spacing_mm:.1f}mm')
    print(f'보드 크기 : {board_w_mm:.1f} x {board_h_mm:.1f} mm')
    print(f'A4 여백   : 좌우 {x_offset * 25.4 / dpi:.1f}mm, 상하 {y_offset * 25.4 / dpi:.1f}mm')
    print(f'해상도    : {dpi} DPI  ({a4_w_px}x{a4_h_px}px)')
    print()

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

    tag_px = mm2px(tag_mm, dpi)
    spacing_px = mm2px(spacing_mm, dpi)

    # 개별 마커를 직접 배치 (GridBoard.draw 버전 호환성 문제 우회)
    # drawMarker: OpenCV 4.x 전 버전에서 동작
    canvas = np.full((a4_h_px, a4_w_px), 255, dtype=np.uint8)
    for row in range(ROWS):
        for col in range(COLS):
            tag_id = row * COLS + col
            marker_img = cv2.aruco.drawMarker(dictionary, tag_id, tag_px, borderBits=1)
            x = x_offset + col * (tag_px + spacing_px)
            y = y_offset + row * (tag_px + spacing_px)
            canvas[y:y + tag_px, x:x + tag_px] = marker_img

    out_path = os.path.abspath(args.output)
    cv2.imwrite(out_path, canvas)
    print(f'저장 완료: {out_path}')
    print(f'출력 시 반드시 "실제 크기(100%)" 로 인쇄하세요.')


if __name__ == '__main__':
    main()
