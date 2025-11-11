#!/usr/bin/env python3
"""
6_yolo_inference.py
학습된 YOLO 모델로 추론 (inference)
- 단일 이미지, 비디오, 디렉토리 지원
- 결과를 이미지/비디오로 저장
"""

import argparse
from pathlib import Path

def main():
    parser = argparse.ArgumentParser(description="YOLO 모델 추론")
    parser.add_argument("--model", type=Path, required=True,
                        help="학습된 모델 경로 (best.pt)")
    parser.add_argument("--source", type=str, required=True,
                        help="추론 소스 (이미지, 비디오, 디렉토리)")
    parser.add_argument("--conf", type=float, default=0.25,
                        help="Confidence threshold")
    parser.add_argument("--iou", type=float, default=0.7,
                        help="IoU threshold for NMS")
    parser.add_argument("--imgsz", type=int, default=640,
                        help="입력 이미지 크기")
    parser.add_argument("--save", action="store_true", default=True,
                        help="결과 저장")
    parser.add_argument("--show", action="store_true",
                        help="결과 화면에 표시")
    parser.add_argument("--project", type=str, default="runs/segment",
                        help="결과 저장 디렉토리")
    parser.add_argument("--name", type=str, default="predict",
                        help="실험 이름")
    parser.add_argument("--device", type=str, default=None,
                        help="디바이스 (0, 1, cpu 등)")
    args = parser.parse_args()
    
    if not args.model.exists():
        raise FileNotFoundError(f"모델 파일이 없습니다: {args.model}")
    
    # ultralytics 임포트
    try:
        from ultralytics import YOLO
    except ImportError:
        print("❌ ultralytics가 설치되어 있지 않습니다.")
        print("설치 명령어: pip3 install ultralytics --user")
        return
    
    print(f"🎯 YOLO 추론 시작")
    print(f"  모델: {args.model}")
    print(f"  소스: {args.source}")
    print(f"  Confidence: {args.conf}")
    print(f"  IoU: {args.iou}")
    
    # 모델 로드
    model = YOLO(str(args.model))
    
    # 추론
    results = model.predict(
        source=args.source,
        conf=args.conf,
        iou=args.iou,
        imgsz=args.imgsz,
        save=args.save,
        show=args.show,
        project=args.project,
        name=args.name,
        device=args.device,
        stream=False,  # 결과를 리스트로 반환
    )
    
    # 결과 출력
    save_dir = Path(args.project) / args.name
    print(f"\n✅ 추론 완료!")
    print(f"  총 {len(results)}개 이미지/프레임 처리")
    
    if args.save:
        print(f"  결과 저장: {save_dir}")
    
    # 탐지 통계
    total_detections = 0
    for r in results:
        if r.boxes is not None:
            total_detections += len(r.boxes)
    
    print(f"  총 탐지 수: {total_detections}")
    
    # 세그멘테이션 결과 확인
    has_masks = any(r.masks is not None for r in results)
    if has_masks:
        print(f"  ✅ Segmentation masks 생성됨")

if __name__ == "__main__":
    main()

