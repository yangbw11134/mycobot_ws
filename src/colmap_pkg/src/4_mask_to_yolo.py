#!/usr/bin/env python3
"""
4_mask_to_yolo.py
SAM 마스크를 YOLO segmentation 포맷으로 변환
- masks/ 폴더의 이진 마스크를 읽어서
- YOLO polygon 형식으로 변환
- 데이터셋 구조 생성 (train/val split)
"""

import argparse
import cv2
import numpy as np
from pathlib import Path
import shutil
import yaml

def mask_to_yolo_polygon(mask: np.ndarray, img_w: int, img_h: int) -> list:
    """
    이진 마스크를 YOLO polygon 좌표로 변환
    Args:
        mask: 0/255 이진 마스크
        img_w, img_h: 이미지 크기
    Returns:
        normalized polygon coordinates [x1, y1, x2, y2, ...]
    """
    # 윤곽선 찾기
    if mask.max() > 1:
        mask = (mask > 127).astype(np.uint8)
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) == 0:
        return []
    
    # 가장 큰 윤곽선 선택
    contour = max(contours, key=cv2.contourArea)
    
    # 너무 작은 객체는 제외
    if cv2.contourArea(contour) < 100:
        return []
    
    # 폴리곤 단순화 (점의 개수 줄이기)
    epsilon = 0.005 * cv2.arcLength(contour, True)
    contour = cv2.approxPolyDP(contour, epsilon, True)
    
    # YOLO 형식으로 변환: normalized coordinates
    polygon = []
    for point in contour.squeeze():
        if len(point.shape) == 0:  # 단일 점인 경우
            continue
        x, y = point
        x_norm = float(x) / img_w
        y_norm = float(y) / img_h
        # 범위 제한
        x_norm = max(0.0, min(1.0, x_norm))
        y_norm = max(0.0, min(1.0, y_norm))
        polygon.extend([x_norm, y_norm])
    
    return polygon

def create_yolo_label(mask_path: Path, img_w: int, img_h: int, class_id: int = 0) -> str:
    """
    마스크 파일로부터 YOLO 라벨 문자열 생성
    """
    mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
    if mask is None:
        return ""
    
    polygon = mask_to_yolo_polygon(mask, img_w, img_h)
    
    if len(polygon) < 6:  # 최소 3개 점 필요
        return ""
    
    # YOLO segmentation format: class_id x1 y1 x2 y2 x3 y3 ...
    label_str = f"{class_id}"
    for coord in polygon:
        label_str += f" {coord:.6f}"
    
    return label_str

def split_dataset(image_files: list, train_ratio: float = 0.8):
    """데이터셋을 train/val로 분할"""
    np.random.seed(42)
    np.random.shuffle(image_files)
    
    n_train = int(len(image_files) * train_ratio)
    train_files = image_files[:n_train]
    val_files = image_files[n_train:]
    
    return train_files, val_files

def main():
    parser = argparse.ArgumentParser(description="SAM 마스크를 YOLO 데이터셋으로 변환")
    parser.add_argument("--sam-dir", type=Path, required=True, 
                        help="SAM 출력 디렉토리 (masks, masked_images 포함)")
    parser.add_argument("--out-dir", type=Path, default=Path("yolo_dataset"),
                        help="YOLO 데이터셋 출력 디렉토리")
    parser.add_argument("--class-name", type=str, default="object",
                        help="객체 클래스 이름")
    parser.add_argument("--train-ratio", type=float, default=0.8,
                        help="학습 데이터 비율 (0.0~1.0)")
    args = parser.parse_args()
    
    masks_dir = args.sam_dir / "masks"
    images_dir = args.sam_dir / "masked_images"
    
    if not masks_dir.exists():
        raise FileNotFoundError(f"마스크 디렉토리가 없습니다: {masks_dir}")
    if not images_dir.exists():
        raise FileNotFoundError(f"이미지 디렉토리가 없습니다: {images_dir}")
    
    # 출력 디렉토리 구조 생성
    out_dir = args.out_dir
    train_img_dir = out_dir / "images" / "train"
    train_lbl_dir = out_dir / "labels" / "train"
    val_img_dir = out_dir / "images" / "val"
    val_lbl_dir = out_dir / "labels" / "val"
    
    for d in [train_img_dir, train_lbl_dir, val_img_dir, val_lbl_dir]:
        d.mkdir(parents=True, exist_ok=True)
    
    # 마스크 파일 찾기
    mask_files = sorted(masks_dir.glob("*_mask.png"))
    
    if len(mask_files) == 0:
        raise FileNotFoundError(f"마스크 파일을 찾을 수 없습니다: {masks_dir}")
    
    print(f"✅ 총 {len(mask_files)}개 마스크 파일 발견")
    
    # 이미지 파일 매칭
    image_files = []
    for mask_file in mask_files:
        # frame_050_mask.png -> frame_050.png
        stem = mask_file.stem.replace("_mask", "")
        img_file = images_dir / f"{stem}.png"
        if not img_file.exists():
            img_file = images_dir / f"{stem}.jpg"
        
        if img_file.exists():
            image_files.append((mask_file, img_file))
    
    print(f"✅ {len(image_files)}개 이미지-마스크 쌍 매칭됨")
    
    # Train/Val 분할
    train_pairs, val_pairs = split_dataset(image_files, args.train_ratio)
    print(f"✅ Train: {len(train_pairs)} / Val: {len(val_pairs)}")
    
    # 이미지 크기 (첫 번째 이미지에서 가져오기)
    sample_img = cv2.imread(str(image_files[0][1]))
    img_h, img_w = sample_img.shape[:2]
    print(f"✅ 이미지 크기: {img_w}x{img_h}")
    
    # Train 데이터 처리
    print("\n[Train 데이터 변환 중...]")
    train_count = 0
    for mask_path, img_path in train_pairs:
        # 라벨 생성
        label_str = create_yolo_label(mask_path, img_w, img_h, class_id=0)
        
        if not label_str:
            print(f"⚠️ 스킵: {mask_path.name} (유효한 마스크 없음)")
            continue
        
        # 이미지 복사
        dst_img = train_img_dir / img_path.name
        shutil.copy(img_path, dst_img)
        
        # 라벨 저장
        dst_lbl = train_lbl_dir / f"{img_path.stem}.txt"
        dst_lbl.write_text(label_str + "\n")
        
        train_count += 1
        if train_count % 20 == 0:
            print(f"  {train_count}/{len(train_pairs)}")
    
    # Val 데이터 처리
    print("\n[Val 데이터 변환 중...]")
    val_count = 0
    for mask_path, img_path in val_pairs:
        # 라벨 생성
        label_str = create_yolo_label(mask_path, img_w, img_h, class_id=0)
        
        if not label_str:
            print(f"⚠️ 스킵: {mask_path.name} (유효한 마스크 없음)")
            continue
        
        # 이미지 복사
        dst_img = val_img_dir / img_path.name
        shutil.copy(img_path, dst_img)
        
        # 라벨 저장
        dst_lbl = val_lbl_dir / f"{img_path.stem}.txt"
        dst_lbl.write_text(label_str + "\n")
        
        val_count += 1
        if val_count % 20 == 0:
            print(f"  {val_count}/{len(val_pairs)}")
    
    # data.yaml 생성
    data_yaml = {
        "path": str(out_dir.resolve()),
        "train": "images/train",
        "val": "images/val",
        "nc": 1,  # number of classes
        "names": [args.class_name]
    }
    
    yaml_path = out_dir / "data.yaml"
    with open(yaml_path, 'w') as f:
        yaml.dump(data_yaml, f, default_flow_style=False)
    
    print(f"\n✅ YOLO 데이터셋 생성 완료!")
    print(f"출력 디렉토리: {out_dir.resolve()}")
    print(f"Train: {train_count}개")
    print(f"Val: {val_count}개")
    print(f"data.yaml: {yaml_path}")
    print(f"\n다음 단계: 5_train_yolo.py로 학습을 시작하세요!")

if __name__ == "__main__":
    main()

