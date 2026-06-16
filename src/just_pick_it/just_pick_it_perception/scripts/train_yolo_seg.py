#!/usr/bin/env python3
"""YOLOv8n-seg 커스텀 모델 학습 스크립트.

Usage:
    # Roboflow에서 YOLOv8 포맷으로 다운로드한 데이터셋 사용
    python3 train_yolo_seg.py \
        --data datasets/watermelon-fanta-seg-1/data.yaml \
        --epochs 100 \
        --output result/models

    # 학습 후 best.pt 경로 출력
    # → result/models/watermelon_fanta_v1/weights/best.pt
    # 이 파일을 result/models/watermelon_fanta_v1.pt 로 복사해 사용한다

Requirements:
    pip install ultralytics
"""

import argparse
import shutil
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description='YOLOv8n-seg 커스텀 학습')
    parser.add_argument('--data', required=True,
                        help='data.yaml 경로 (Roboflow YOLOv8 export)')
    parser.add_argument('--epochs', type=int, default=100)
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--batch', type=int, default=16,
                        help='배치 사이즈 (-1 이면 자동)')
    parser.add_argument('--output', type=str, default='result/models',
                        help='학습 결과 저장 디렉터리')
    parser.add_argument('--name', type=str, default='watermelon_fanta_v1',
                        help='실험 이름')
    parser.add_argument('--base-model', type=str, default='yolov8n-seg.pt',
                        help='베이스 모델 (yolov8n-seg.pt, yolo11n-seg.pt 등)')
    parser.add_argument('--device', type=str, default='',
                        help='학습 장치 (cuda:0, cpu 등, 기본: 자동)')
    args = parser.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit('ultralytics 패키지가 없습니다: pip install ultralytics')

    model = YOLO(args.base_model)

    train_kwargs = dict(
        data=args.data,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        project=args.output,
        name=args.name,
        optimizer='AdamW',
        lr0=0.001,
        lrf=0.01,
        weight_decay=0.0005,
        warmup_epochs=3,
        patience=30,
        mosaic=1.0,
        mixup=0.1,
        degrees=10.0,
        translate=0.1,
        scale=0.4,
        flipud=0.0,
        fliplr=0.5,
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
        save_period=10,
        plots=True,
        val=True,
    )
    if args.device:
        train_kwargs['device'] = args.device

    model.train(**train_kwargs)

    best_pt = Path(args.output) / args.name / 'weights' / 'best.pt'
    deploy_pt = Path(args.output) / f'{args.name}.pt'

    if best_pt.exists():
        shutil.copy2(best_pt, deploy_pt)
        print(f'\n학습 완료!')
        print(f'  best.pt    : {best_pt}')
        print(f'  배포용 모델 : {deploy_pt}')
        print(f'\ndetection_params.yaml 의 model_path 를 다음으로 변경하세요:')
        print(f'  model_path: "{deploy_pt}"')
    else:
        print(f'\n학습 완료 (best.pt를 찾을 수 없음: {best_pt})')


if __name__ == '__main__':
    main()
