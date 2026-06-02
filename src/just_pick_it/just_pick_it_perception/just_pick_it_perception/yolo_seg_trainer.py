"""
Roboflow API로 데이터셋을 내려받아 YOLOv8-seg 모델을 학습한다.

사용법:
    ros2 run just_pick_it_perception yolo_seg_trainer \\
        --api-key YOUR_KEY \\
        --workspace YOUR_WORKSPACE \\
        --project YOUR_PROJECT \\
        --version 1

    또는 직접 실행:
    python yolo_seg_trainer.py --api-key KEY --workspace WS --project PROJ

주요 옵션:
    --base-model   베이스 가중치 (기본: yolov8n-seg.pt)
    --epochs       학습 횟수 (기본: 100)
    --batch        배치 크기 (기본: 16, VRAM 부족 시 줄임)
    --imgsz        입력 이미지 크기 (기본: 640)
    --device       학습 장치 (기본: 0=GPU, cpu 지정 가능)
    --output-dir   결과 저장 경로 (기본: runs/segment)
    --run-name     실험 이름 (기본: custom_seg)
    --dataset-dir  데이터셋 저장 경로 (기본: ~/datasets)
"""

import argparse
import os
import sys


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Roboflow 데이터셋으로 YOLOv8-seg 학습',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )

    roboflow = parser.add_argument_group('Roboflow')
    roboflow.add_argument('--api-key', required=True, help='Roboflow API 키')
    roboflow.add_argument('--workspace', required=True, help='Roboflow 워크스페이스 이름')
    roboflow.add_argument('--project', required=True, help='Roboflow 프로젝트 이름')
    roboflow.add_argument('--version', type=int, default=1, help='데이터셋 버전 번호')
    roboflow.add_argument(
        '--dataset-dir',
        default=os.path.expanduser('~/datasets'),
        help='데이터셋 저장 경로',
    )

    train = parser.add_argument_group('학습')
    train.add_argument('--base-model', default='yolov8n-seg.pt', help='베이스 가중치 파일')
    train.add_argument('--epochs', type=int, default=100, help='학습 epoch 수')
    train.add_argument('--batch', type=int, default=16, help='배치 크기')
    train.add_argument('--imgsz', type=int, default=640, help='입력 이미지 크기')
    train.add_argument('--device', default='0', help='학습 장치 (0: GPU, cpu: CPU)')
    train.add_argument('--output-dir', default='runs/segment', help='결과 저장 경로')
    train.add_argument('--run-name', default='custom_seg', help='실험 이름')

    return parser.parse_args()


def download_dataset(args: argparse.Namespace) -> str:
    try:
        from roboflow import Roboflow
    except ImportError:
        print('[ERROR] roboflow 패키지가 없습니다: pip install roboflow')
        sys.exit(1)

    print(f'[INFO] Roboflow 데이터셋 다운로드: {args.workspace}/{args.project} v{args.version}')
    rf = Roboflow(api_key=args.api_key)
    project = rf.workspace(args.workspace).project(args.project)
    dataset = project.version(args.version).download(
        'yolov8',
        location=os.path.join(args.dataset_dir, f'{args.project}_v{args.version}'),
    )
    data_yaml = os.path.join(dataset.location, 'data.yaml')
    print(f'[INFO] 데이터셋 저장 위치: {dataset.location}')
    return data_yaml


def train(args: argparse.Namespace, data_yaml: str):
    try:
        from ultralytics import YOLO
    except ImportError:
        print('[ERROR] ultralytics 패키지가 없습니다: pip install ultralytics')
        sys.exit(1)

    print(f'[INFO] 베이스 모델 로드: {args.base_model}')
    model = YOLO(args.base_model)

    print('[INFO] 학습 시작')
    results = model.train(
        data=data_yaml,
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        rect=True,
        device=args.device,
        project=args.output_dir,
        name=args.run_name,
    )

    best_pt = os.path.join(args.output_dir, args.run_name, 'weights', 'best.pt')
    print()
    print('=' * 60)
    print('[완료] 학습이 끝났습니다.')
    print(f'  best.pt 위치: {best_pt}')
    print()
    print('detection_tracker 노드에 적용하는 방법:')
    print(f'  ros2 run just_pick_it_perception detection_tracker \\')
    print(f'    --ros-args -p model_path:={best_pt}')
    print('=' * 60)

    return results


def main():
    args = parse_args()
    data_yaml = download_dataset(args)
    train(args, data_yaml)


if __name__ == '__main__':
    main()
