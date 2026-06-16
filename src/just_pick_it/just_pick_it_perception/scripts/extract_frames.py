#!/usr/bin/env python3
"""AVI 비디오에서 학습용 프레임을 추출한다.

Usage:
    python3 extract_frames.py \
        --video-dir result/jetcobot_1 \
        --out-dir result/jetcobot_1/frames \
        --interval 5 \
        --max-per-video 200

    # 비디오 하나만 지정할 수도 있다
    python3 extract_frames.py \
        --video result/jetcobot_1/video_1.avi \
        --out-dir result/jetcobot_1/frames/video_1 \
        --interval 5 --max-per-video 200
"""

import argparse
import sys
from pathlib import Path

import cv2


def extract(video_path: Path, out_dir: Path, interval: int, max_frames: int) -> int:
    out_dir.mkdir(parents=True, exist_ok=True)

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        print(f'ERROR: 열 수 없는 비디오: {video_path}', file=sys.stderr)
        return 0

    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print(f'{video_path.name}: {total_frames}프레임, {fps:.1f}fps, {w}x{h}')

    saved = 0
    frame_indices = range(0, total_frames, interval)

    for idx in frame_indices:
        if saved >= max_frames:
            break

        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ret, frame = cap.read()
        if not ret:
            continue

        out_path = out_dir / f'frame_{idx:06d}.jpg'
        cv2.imwrite(str(out_path), frame, [cv2.IMWRITE_JPEG_QUALITY, 95])
        saved += 1

    cap.release()
    print(f'  저장: {saved}장 → {out_dir}')
    return saved


def main():
    parser = argparse.ArgumentParser(description='비디오에서 학습용 프레임 추출')
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--video-dir', help='AVI 파일이 있는 디렉터리 (모든 .avi 처리)')
    group.add_argument('--video', help='단일 AVI 파일 경로')

    parser.add_argument('--out-dir', required=True, help='프레임 저장 경로')
    parser.add_argument('--interval', type=int, default=5,
                        help='N번째 프레임마다 저장 (기본: 5)')
    parser.add_argument('--max-per-video', type=int, default=200,
                        help='비디오당 최대 저장 프레임 수 (기본: 200)')
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    total = 0

    if args.video:
        video_path = Path(args.video)
        total = extract(video_path, out_dir, args.interval, args.max_per_video)
    else:
        video_dir = Path(args.video_dir)
        videos = sorted(video_dir.glob('*.avi'))
        if not videos:
            print(f'ERROR: {video_dir} 에 .avi 파일이 없습니다.', file=sys.stderr)
            sys.exit(1)
        for video in videos:
            sub_dir = out_dir / video.stem
            total += extract(video, sub_dir, args.interval, args.max_per_video)

    print(f'\n총 {total}장 추출 완료')


if __name__ == '__main__':
    main()
