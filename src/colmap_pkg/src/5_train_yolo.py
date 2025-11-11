#!/usr/bin/env python3
"""
5_train_yolo.py
YOLO segmentation 모델 학습
- YOLOv11 (ultralytics) 사용
- 4_mask_to_yolo.py로 만든 데이터셋으로 학습
"""

import argparse
from pathlib import Path
import torch

def main():
    parser = argparse.ArgumentParser(description="YOLO segmentation 모델 학습")
    parser.add_argument("--data", type=Path, required=True,
                        help="data.yaml 파일 경로")
    parser.add_argument("--model", type=str, default="yolo11n-seg.pt",
                        choices=["yolo11n-seg.pt", "yolo11s-seg.pt", 
                                "yolo11m-seg.pt", "yolo11l-seg.pt"],
                        help="사전학습 모델 (n=nano, s=small, m=medium, l=large)")
    parser.add_argument("--epochs", type=int, default=100,
                        help="학습 에포크 수")
    parser.add_argument("--batch", type=int, default=16,
                        help="배치 크기")
    parser.add_argument("--imgsz", type=int, default=640,
                        help="입력 이미지 크기")
    parser.add_argument("--project", type=str, default="runs/segment",
                        help="프로젝트 저장 디렉토리")
    parser.add_argument("--name", type=str, default="train",
                        help="실험 이름")
    parser.add_argument("--device", type=str, default=None,
                        help="디바이스 (0, 1, cpu 등, None=자동)")
    args = parser.parse_args()
    
    if not args.data.exists():
        raise FileNotFoundError(f"data.yaml 파일이 없습니다: {args.data}")
    
    # ultralytics 임포트
    try:
        from ultralytics import YOLO
    except ImportError:
        print("❌ ultralytics가 설치되어 있지 않습니다.")
        print("설치 명령어: pip3 install ultralytics --user")
        return
    
    # GPU 확인
    if args.device is None:
        device = '0' if torch.cuda.is_available() else 'cpu'
    else:
        device = args.device
    
    print(f"🎯 YOLO Segmentation 학습 시작")
    print(f"  모델: {args.model}")
    print(f"  데이터: {args.data}")
    print(f"  에포크: {args.epochs}")
    print(f"  배치: {args.batch}")
    print(f"  이미지 크기: {args.imgsz}")
    print(f"  디바이스: {device}")
    
    # 모델 로드
    model = YOLO(args.model)
    
    # 학습
    results = model.train(
        data=str(args.data),
        epochs=args.epochs,
        batch=args.batch,
        imgsz=args.imgsz,
        project=args.project,
        name=args.name,
        device=device,
        patience=20,  # Early stopping patience
        save=True,
        save_period=10,  # 10 에포크마다 체크포인트 저장
        plots=True,  # 학습 플롯 생성
        # 데이터 증강 옵션
        hsv_h=0.015,  # 색조 증강
        hsv_s=0.7,    # 채도 증강
        hsv_v=0.4,    # 명도 증강
        degrees=10,   # 회전 증강
        translate=0.1,  # 이동 증강
        scale=0.5,    # 스케일 증강
        flipud=0.0,   # 상하 반전 (물체의 경우 비활성화)
        fliplr=0.5,   # 좌우 반전
        mosaic=1.0,   # 모자이크 증강
    )
    
    # 검증
    print("\n✅ 학습 완료! 검증 실행 중...")
    val_results = model.val()
    
    # 결과 출력
    save_dir = Path(args.project) / args.name
    print(f"\n✅ 모델 저장 위치: {save_dir}")
    print(f"  Best 모델: {save_dir / 'weights' / 'best.pt'}")
    print(f"  Last 모델: {save_dir / 'weights' / 'last.pt'}")
    
    # mAP 출력
    if hasattr(val_results, 'box'):
        print(f"\n📊 검증 결과:")
        print(f"  mAP50: {val_results.box.map50:.4f}")
        print(f"  mAP50-95: {val_results.box.map:.4f}")
    
    if hasattr(val_results, 'seg'):
        print(f"  Mask mAP50: {val_results.seg.map50:.4f}")
        print(f"  Mask mAP50-95: {val_results.seg.map:.4f}")
    
    print(f"\n다음 단계:")
    print(f"  1. 학습 결과 확인: {save_dir}")
    print(f"  2. 추론 실행: yolo predict model={save_dir}/weights/best.pt source=<이미지경로>")
    print(f"  3. 모델 export: yolo export model={save_dir}/weights/best.pt format=onnx")

if __name__ == "__main__":
    main()

