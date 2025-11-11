#!/usr/bin/env python3
"""
2_sam_video_v2.py - 개선된 SAM2 비디오 segmentation
주요 개선사항:
1. 프레임 인덱스 미스매치 해결 - 추출된 프레임 직접 사용
2. 마스크-이미지 일치 보장
3. 품질 검증 기능 추가

사용법:
python3 2_sam_video_v2.py \
    --video rotation_scan.mp4 \
    --sam2-checkpoint /path/to/sam2_hiera_large.pt \
    --sam2-config sam2_hiera_l \
    --out-dir sam2_out_fixed \
    --prompt-type center \
    --max-frames 100 \
    --bg white \
    --device cuda


python3 /home/yangbi/mycobot_ws/src/colmap_pkg/src/2_sam_video.py \
  --video /home/yangbi/mycobot_ws/src/colmap_pkg/video_scans/KakaoTalk_20251111_130937055.mp4 \
  --sam2-checkpoint /home/yangbi/mycobot_ws/src/colmap_pkg/segment-anything-2/sam2_hiera_large.pt \
  --sam2-config /home/yangbi/mycobot_ws/src/colmap_pkg/segment-anything-2/sam2/configs/sam2/sam2_hiera_l.yaml \
  --out-dir /home/yangbi/mycobot_ws/src/colmap_pkg/sam2_out \
  --prompt-type box \
  --max-frames 200 \
  --bg white \
  --device cuda
"""

import argparse
import cv2
import numpy as np
import torch
from pathlib import Path
import json
import shutil


def extract_frames_with_mapping(video_path: str, temp_dir: Path, max_frames: int = 100):
    """
    프레임 추출 + 매핑 정보 저장
    Returns: (frame_paths, mapping) where mapping[saved_idx] = original_frame_idx
    """
    temp_dir.mkdir(parents=True, exist_ok=True)
    
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        raise ValueError(f"Cannot open video: {video_path}")
    
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    if total_frames <= 0:
        print("⚠️ 전체 프레임 수를 알 수 없습니다.")
        total_frames = max_frames * 10
    
    # 샘플링 인덱스 계산
    if total_frames <= max_frames:
        sample_indices = list(range(total_frames))
    else:
        sample_indices = np.linspace(0, total_frames - 1, max_frames, dtype=int)
    
    print(f"프레임 추출: 전체 {total_frames}개 중 {len(sample_indices)}개")
    
    frame_paths = []
    frame_mapping = {}  # saved_idx -> original_frame_idx
    saved_count = 0
    
    for idx in sample_indices:
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ret, frame = cap.read()
        
        if not ret:
            continue
        
        # 저장
        frame_path = temp_dir / f"{saved_count:05d}.jpg"
        cv2.imwrite(str(frame_path), frame)
        frame_paths.append(frame_path)
        frame_mapping[saved_count] = int(idx)
        saved_count += 1
        
        if saved_count % 10 == 0:
            print(f"... {saved_count}/{len(sample_indices)}")
    
    cap.release()
    
    # 매핑 정보 저장
    mapping_file = temp_dir / "frame_mapping.json"
    with open(mapping_file, 'w') as f:
        json.dump({
            "total_original_frames": total_frames,
            "extracted_frames": saved_count,
            "fps": fps,
            "resolution": [width, height],
            "mapping": frame_mapping
        }, f, indent=2)
    
    print(f"✅ 추출 완료: {saved_count}개 프레임")
    print(f"✅ 매핑 정보 저장: {mapping_file}")
    
    return frame_paths, frame_mapping, (width, height)


def get_user_prompt(frame_path: Path, prompt_type: str = "box"):
    """사용자 프롬프트 받기"""
    img = cv2.imread(str(frame_path))
    if img is None:
        raise ValueError(f"Cannot load frame: {frame_path}")
    
    h, w = img.shape[:2]
    
    if prompt_type == "center":
        # 자동 중앙 박스
        cx, cy = w // 2, h // 2
        box_w, box_h = int(w * 0.4), int(h * 0.4)
        
        x1 = max(0, cx - box_w // 2)
        y1 = max(0, cy - box_h // 2)
        x2 = min(w, cx + box_w // 2)
        y2 = min(h, cy + box_h // 2)
        
        print(f"\n📦 자동 중앙 박스: ({x1}, {y1}) - ({x2}, {y2})")
        
        return {
            "type": "box",
            "box": np.array([x1, y1, x2, y2], dtype=np.float32)
        }
    
    elif prompt_type == "box":
        print("\n📦 물체를 드래그하여 선택하세요 (q=완료, r=다시)")
        
        drawing = False
        start_point = None
        end_point = None
        current_img = img.copy()
        
        def mouse_callback(event, x, y, flags, param):
            nonlocal drawing, start_point, end_point, current_img
            
            if event == cv2.EVENT_LBUTTONDOWN:
                drawing = True
                start_point = (x, y)
            elif event == cv2.EVENT_MOUSEMOVE and drawing:
                end_point = (x, y)
                current_img = img.copy()
                cv2.rectangle(current_img, start_point, end_point, (0, 255, 0), 2)
                cv2.imshow("Select Box", current_img)
            elif event == cv2.EVENT_LBUTTONUP:
                drawing = False
                end_point = (x, y)
                cv2.rectangle(current_img, start_point, end_point, (0, 255, 0), 2)
                cv2.imshow("Select Box", current_img)
        
        cv2.namedWindow("Select Box")
        cv2.setMouseCallback("Select Box", mouse_callback)
        cv2.imshow("Select Box", current_img)
        
        while True:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break
            elif key == ord('r'):
                current_img = img.copy()
                start_point = None
                cv2.imshow("Select Box", current_img)
        
        cv2.destroyAllWindows()
        
        if start_point is None or end_point is None:
            raise ValueError("No box selected!")
        
        x1 = min(start_point[0], end_point[0])
        y1 = min(start_point[1], end_point[1])
        x2 = max(start_point[0], end_point[0])
        y2 = max(start_point[1], end_point[1])
        
        return {
            "type": "box",
            "box": np.array([x1, y1, x2, y2], dtype=np.float32)
        }
    
    elif prompt_type == "point":
        print("\n📍 물체 클릭 (좌클릭=전경, 우클릭=배경, q=완료)")
        
        points = []
        labels = []
        
        def mouse_callback(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                points.append([x, y])
                labels.append(1)
                cv2.circle(img, (x, y), 5, (0, 255, 0), -1)
                cv2.imshow("Select Points", img)
            elif event == cv2.EVENT_RBUTTONDOWN:
                points.append([x, y])
                labels.append(0)
                cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
                cv2.imshow("Select Points", img)
        
        cv2.namedWindow("Select Points")
        cv2.setMouseCallback("Select Points", mouse_callback)
        cv2.imshow("Select Points", img)
        
        while cv2.waitKey(1) & 0xFF not in [ord('q'), 27]:
            pass
        
        cv2.destroyAllWindows()
        
        if not points:
            raise ValueError("No points selected!")
        
        return {
            "type": "point",
            "points": np.array(points, dtype=np.float32),
            "labels": np.array(labels, dtype=np.int32)
        }


def sam2_video_segment(video_dir: Path, checkpoint: str, config: str, 
                       prompt_data: dict, device: str = "cuda"):
    """SAM2 비디오 segmentation"""
    try:
        from sam2.build_sam import build_sam2_video_predictor
    except ImportError:
        raise ImportError("SAM2가 설치되지 않았습니다.")
    
    print(f"\n🎯 SAM2 segmentation 시작")
    
    config_name = Path(config).stem
    predictor = build_sam2_video_predictor(config_name, checkpoint, device=device)
    
    inference_state = predictor.init_state(video_path=str(video_dir))
    
    # 첫 프레임에 프롬프트 추가
    frame_idx = 0
    obj_id = 1
    
    if prompt_data["type"] == "point":
        predictor.add_new_points_or_box(
            inference_state=inference_state,
            frame_idx=frame_idx,
            obj_id=obj_id,
            points=prompt_data["points"],
            labels=prompt_data["labels"],
        )
    else:  # box
        predictor.add_new_points_or_box(
            inference_state=inference_state,
            frame_idx=frame_idx,
            obj_id=obj_id,
            box=prompt_data["box"],
        )
    
    print(f"✅ 프롬프트 추가 완료")
    print("🔄 비디오 segmentation 진행 중...")
    
    # Propagation
    video_segments = {}
    for out_frame_idx, out_obj_ids, out_mask_logits in predictor.propagate_in_video(inference_state):
        video_segments[out_frame_idx] = {
            out_obj_id: (out_mask_logits[i] > 0.0).cpu().numpy()
            for i, out_obj_id in enumerate(out_obj_ids)
        }
    
    print(f"✅ Segmentation 완료: {len(video_segments)} 프레임")
    
    # obj_id=1 마스크 추출
    masks = {idx: seg[1][0] for idx, seg in video_segments.items() if 1 in seg}
    
    return masks


def save_results_fixed(temp_dir: Path, masks: dict, frame_mapping: dict,
                       out_dir: Path, bg_color: str = "white"):
    """
    개선된 결과 저장 - 추출된 프레임 직접 사용
    """
    masked_dir = out_dir / "masked_images"
    masks_dir = out_dir / "masks"
    quality_dir = out_dir / "quality_check"
    
    for d in [masked_dir, masks_dir, quality_dir]:
        d.mkdir(parents=True, exist_ok=True)
    
    bgc = (255, 255, 255) if bg_color == "white" else (0, 0, 0)
    
    saved_count = 0
    print(f"\n💾 결과 저장 중...")
    
    # 추출된 프레임 직접 사용
    frame_files = sorted(temp_dir.glob("*.jpg"))
    
    for frame_idx, frame_path in enumerate(frame_files):
        if frame_idx not in masks:
            print(f"⚠️ Frame {frame_idx}: 마스크 없음")
            continue
        
        # 원본 이미지 로드
        frame = cv2.imread(str(frame_path))
        if frame is None:
            continue
        
        mask = masks[frame_idx]
        
        # Masked image 생성
        masked_img = np.full_like(frame, bgc, dtype=np.uint8)
        masked_img[mask] = frame[mask]
        
        # 저장
        img_path = masked_dir / f"frame_{saved_count:03d}.png"
        cv2.imwrite(str(img_path), masked_img)
        
        # Mask 저장
        mask_u8 = (mask.astype(np.uint8) * 255)
        mask_path = masks_dir / f"frame_{saved_count:03d}_mask.png"
        cv2.imwrite(str(mask_path), mask_u8)
        
        # Quality check - 오버레이 이미지
        overlay = frame.copy()
        overlay[mask] = cv2.addWeighted(frame[mask], 0.7, 
                                       np.full_like(frame[mask], [0, 255, 0]), 0.3, 0)
        quality_path = quality_dir / f"frame_{saved_count:03d}_overlay.png"
        cv2.imwrite(str(quality_path), overlay)
        
        saved_count += 1
        
        if saved_count % 20 == 0:
            print(f"  저장: {saved_count} 프레임")
    
    print(f"✅ 총 {saved_count}개 프레임 저장")
    print(f"  Masked images: {masked_dir.resolve()}")
    print(f"  Masks: {masks_dir.resolve()}")
    print(f"  Quality check: {quality_dir.resolve()}")
    
    return saved_count


def main():
    parser = argparse.ArgumentParser(description="개선된 SAM2 비디오 segmentation")
    parser.add_argument("--video", type=str, required=True)
    parser.add_argument("--sam2-checkpoint", type=str, required=True)
    parser.add_argument("--sam2-config", type=str, required=True)
    parser.add_argument("--out-dir", type=Path, default=Path("sam2_out_fixed"))
    parser.add_argument("--prompt-type", choices=["box", "point", "center"],
                        default="center")
    parser.add_argument("--max-frames", type=int, default=200,
                        help="추출할 프레임 수 (권장: 200-300)")
    parser.add_argument("--bg", choices=["white", "black"], default="white")
    parser.add_argument("--device", type=str, default="cuda")
    parser.add_argument("--keep-temp", action="store_true",
                        help="임시 프레임 유지 (디버깅용)")
    
    args = parser.parse_args()
    
    if not Path(args.video).exists():
        raise FileNotFoundError(f"비디오 파일 없음: {args.video}")
    
    print("=" * 60)
    print("SAM2 비디오 객체 분할 (개선 버전)")
    print("=" * 60)
    
    temp_dir = args.out_dir / "_temp_frames"
    
    # 1. 프레임 추출 + 매핑
    print("\n[1/4] 프레임 추출 및 매핑...")
    frame_paths, frame_mapping, (width, height) = extract_frames_with_mapping(
        args.video, temp_dir, args.max_frames
    )
    
    # 2. 사용자 프롬프트
    print("\n[2/4] 객체 선택...")
    first_frame = temp_dir / "00000.jpg"
    prompt_data = get_user_prompt(first_frame, args.prompt_type)
    
    # 프롬프트 저장
    prompt_path = args.out_dir / "prompt.json"
    prompt_path.parent.mkdir(parents=True, exist_ok=True)
    with open(prompt_path, 'w') as f:
        json.dump({
            "type": prompt_data["type"],
            "box": prompt_data.get("box", []).tolist() if "box" in prompt_data else [],
            "points": prompt_data.get("points", []).tolist() if "points" in prompt_data else [],
            "labels": prompt_data.get("labels", []).tolist() if "labels" in prompt_data else []
        }, f, indent=2)
    
    # 3. SAM2 segmentation
    print("\n[3/4] SAM2 segmentation...")
    masks = sam2_video_segment(
        video_dir=temp_dir,
        checkpoint=args.sam2_checkpoint,
        config=args.sam2_config,
        prompt_data=prompt_data,
        device=args.device
    )
    
    # 4. 결과 저장 (고정된 버전)
    print("\n[4/4] 결과 저장...")
    saved_count = save_results_fixed(
        temp_dir=temp_dir,
        masks=masks,
        frame_mapping=frame_mapping,
        out_dir=args.out_dir,
        bg_color=args.bg
    )
    
    # 임시 파일 정리
    if not args.keep_temp and temp_dir.exists():
        shutil.rmtree(temp_dir)
        print("✅ 임시 파일 삭제")
    elif args.keep_temp:
        print(f"✅ 임시 파일 유지: {temp_dir.resolve()}")
    
    print("\n" + "=" * 60)
    print("✅ 완료!")
    print(f"출력: {args.out_dir.resolve()}")
    print(f"저장: {saved_count}개 프레임")
    print(f"\nQuality check: {args.out_dir}/quality_check 폴더를 확인하세요!")
    print("=" * 60)


if __name__ == "__main__":
    main()