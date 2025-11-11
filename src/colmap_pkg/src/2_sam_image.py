#!/usr/bin/env python3
"""
2_sam_image_v4.py - 배경색 자동 감지 추가
개선사항: 회전판 색상과 무관하게 작동

cd /home/yangbi/mycobot_ws/src/colmap_pkg && python3 src/2_sam_image.py --frames-dir frames_100 --out-dir sam_out --sam /home/yangbi/models/sam/sam_vit_h_4b8939.pth

"""


import argparse, cv2, numpy as np, torch
from pathlib import Path
from segment_anything import SamPredictor, sam_model_registry


def load_sam(ckpt: str, device: str = "cuda"):
    model_type = "vit_h"
    sam = sam_model_registry[model_type](checkpoint=ckpt)
    sam.to(device=device)
    return SamPredictor(sam)


def auto_detect_background_color(rgb: np.ndarray) -> dict:
    """
    모서리 영역 분석으로 배경색 자동 감지
    Returns: {'type': 'white'|'gray'|'black'|'colored', 'hsv_range': (h,s,v)}
    """
    h, w = rgb.shape[:2]
    border = 50
    
    # 모서리 4개 영역 샘플링
    corners = [
        rgb[:border, :border],           # 좌상
        rgb[:border, -border:],          # 우상
        rgb[-border:, :border],          # 좌하
        rgb[-border:, -border:]          # 우하
    ]
    
    # HSV 변환 및 통계
    hsv_samples = [cv2.cvtColor(corner, cv2.COLOR_RGB2HSV) for corner in corners]
    
    # 평균 HSV 계산
    all_hsv = np.vstack([sample.reshape(-1, 3) for sample in hsv_samples])
    mean_h, mean_s, mean_v = np.mean(all_hsv, axis=0)
    std_s, std_v = np.std(all_hsv[:, 1]), np.std(all_hsv[:, 2])
    
    print(f"📊 배경 분석: H={mean_h:.1f}, S={mean_s:.1f}±{std_s:.1f}, V={mean_v:.1f}±{std_v:.1f}")
    
    # 배경 타입 분류
    if mean_s < 30 and mean_v > 200:
        bg_type = 'white'
        print(f"✅ 감지: 흰색 배경")
    elif mean_s < 30 and mean_v < 50:
        bg_type = 'black'
        print(f"✅ 감지: 검은색 배경")
    elif mean_s < 50 and 50 <= mean_v <= 200:
        bg_type = 'gray'
        print(f"✅ 감지: 회색 배경 (V={mean_v:.0f})")
    else:
        bg_type = 'colored'
        print(f"✅ 감지: 유색 배경 (H={mean_h:.0f}, S={mean_s:.0f})")
    
    return {
        'type': bg_type,
        'hsv_mean': (mean_h, mean_s, mean_v),
        'hsv_std': (0, std_s, std_v)
    }


def adaptive_background_removal(rgb: np.ndarray, bg_info: dict = None) -> np.ndarray:
    """
    적응형 배경 제거 - 자동 감지된 배경색 사용
    """
    if bg_info is None:
        bg_info = auto_detect_background_color(rgb)
    
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    h, s, v = cv2.split(hsv)
    
    bg_type = bg_info['type']
    mean_h, mean_s, mean_v = bg_info['hsv_mean']
    _, std_s, std_v = bg_info['hsv_std']
    
    # 적응형 임계값 (평균 ± 2*표준편차)
    if bg_type == 'white':
        s_thresh = min(50, mean_s + 2 * std_s)
        v_thresh = max(150, mean_v - 2 * std_v)
        bg_mask = (s < s_thresh) & (v > v_thresh)
        
    elif bg_type == 'black':
        s_thresh = min(50, mean_s + 2 * std_s)
        v_thresh = max(80, mean_v + 2 * std_v)
        bg_mask = (s < s_thresh) & (v < v_thresh)
        
    elif bg_type == 'gray':
        s_thresh = min(60, mean_s + 2 * std_s)
        v_min = max(0, mean_v - 2 * std_v)
        v_max = min(255, mean_v + 2 * std_v)
        bg_mask = (s < s_thresh) & (v > v_min) & (v < v_max)
        
    else:  # colored
        # 색상(H) + 채도(S) + 명도(V) 모두 고려
        h_thresh = 15  # 색상 범위
        s_thresh = min(255, mean_s + 2 * std_s)
        v_min = max(0, mean_v - 2 * std_v)
        v_max = min(255, mean_v + 2 * std_v)
        
        h_dist = np.minimum(np.abs(h - mean_h), 180 - np.abs(h - mean_h))
        bg_mask = (h_dist < h_thresh) & (s < s_thresh) & (v > v_min) & (v < v_max)
    
    # 공간 정보 결합 (배경은 주로 모서리)
    spatial_weight = np.ones_like(bg_mask, dtype=float)
    border = 50
    spatial_weight[border:-border, border:-border] *= 0.5  # 중앙 영역 가중치 낮춤
    
    bg_mask_weighted = bg_mask.astype(float) * spatial_weight
    bg_mask = (bg_mask_weighted > 0.3).astype(np.uint8)
    
    # 모폴로지 연산
    kernel = np.ones((7, 7), np.uint8)
    bg_mask = cv2.morphologyEx(bg_mask, cv2.MORPH_CLOSE, kernel)
    
    return 1 - bg_mask


def get_smart_points(rgb: np.ndarray, mask_hint: np.ndarray = None, n_points: int = 9) -> tuple:
    """스마트 포인트 생성"""
    h, w = rgb.shape[:2]
    
    if mask_hint is not None:
        fg_region = mask_hint > 0
    else:
        cx, cy = w // 2, h // 2
        region_w, region_h = int(w * 0.3), int(h * 0.3)
        fg_region = np.zeros((h, w), dtype=bool)
        fg_region[cy-region_h//2:cy+region_h//2, cx-region_w//2:cx+region_w//2] = True
    
    fg_points = []
    y_coords, x_coords = np.where(fg_region)
    
    if len(y_coords) > 0:
        try:
            from sklearn.cluster import KMeans
            coords = np.column_stack([x_coords, y_coords])
            n_fg = min(n_points, len(coords))
            
            if n_fg >= 3:
                kmeans = KMeans(n_clusters=n_fg, random_state=42, n_init=10)
                kmeans.fit(coords)
                fg_points = kmeans.cluster_centers_.astype(int)
            else:
                indices = np.random.choice(len(coords), size=n_fg, replace=False)
                fg_points = coords[indices]
        except ImportError:
            # sklearn 없으면 그리드 샘플링
            grid_size = int(np.sqrt(n_points))
            step_y = max(1, len(y_coords) // grid_size)
            step_x = max(1, len(x_coords) // grid_size)
            for i in range(0, len(y_coords), step_y):
                for j in range(0, len(x_coords), step_x):
                    if len(fg_points) < n_points:
                        fg_points.append([x_coords[j], y_coords[i]])
            fg_points = np.array(fg_points)
    
    # 배경 포인트
    border = 20
    bg_regions = [
        (border, border), (w//2, border), (w-border, border),
        (border, h-border), (w//2, h-border), (w-border, h-border),
        (border, h//2), (w-border, h//2)
    ]
    bg_points = np.array([p for p in bg_regions if 0 <= p[0] < w and 0 <= p[1] < h])
    
    return np.array(fg_points), bg_points


def grabcut_refine(rgb: np.ndarray, initial_mask: np.ndarray, iterations: int = 5) -> np.ndarray:
    """GrabCut 정제"""
    mask_gc = np.zeros(rgb.shape[:2], dtype=np.uint8)
    mask_gc[initial_mask > 0] = cv2.GC_PR_FGD
    mask_gc[initial_mask == 0] = cv2.GC_PR_BGD
    
    border = 10
    h, w = rgb.shape[:2]
    mask_gc[:border, :] = cv2.GC_BGD
    mask_gc[-border:, :] = cv2.GC_BGD
    mask_gc[:, :border] = cv2.GC_BGD
    mask_gc[:, -border:] = cv2.GC_BGD
    
    center_region = initial_mask.copy()
    kernel = np.ones((11, 11), np.uint8)
    center_region = cv2.erode(center_region, kernel, iterations=2)
    mask_gc[center_region > 0] = cv2.GC_FGD
    
    bgd_model = np.zeros((1, 65), np.float64)
    fgd_model = np.zeros((1, 65), np.float64)
    
    try:
        cv2.grabCut(rgb, mask_gc, None, bgd_model, fgd_model, 
                    iterations, cv2.GC_INIT_WITH_MASK)
        refined_mask = np.where((mask_gc == cv2.GC_FGD) | (mask_gc == cv2.GC_PR_FGD), 1, 0).astype(np.uint8)
        return refined_mask
    except:
        print("⚠️ GrabCut 실패 - 원본 마스크 사용")
        return initial_mask


def adaptive_sam_mask(
    predictor: SamPredictor,
    rgb: np.ndarray,
    method: str = "adaptive",
    roi_center_x: float = 0.5,
    roi_center_y: float = 0.5,
    roi_width: float = 0.5,
    roi_height: float = 0.5,
    use_grabcut: bool = True,
    bg_info: dict = None,
) -> np.ndarray:
    """적응형 SAM 마스킹"""
    predictor.set_image(rgb)
    h, w = rgb.shape[:2]
    
    # 1. 배경 힌트 생성 (자동 감지)
    bg_hint = adaptive_background_removal(rgb, bg_info)
    
    # 2. 스마트 포인트 생성
    fg_points, bg_points = get_smart_points(rgb, bg_hint, n_points=9)
    
    # 3. SAM 예측
    if method == "adaptive":
        cx, cy = w * roi_center_x, h * roi_center_y
        half_w, half_h = (w * roi_width) / 2.0, (h * roi_height) / 2.0
        box = np.array([
            max(0, cx - half_w), max(0, cy - half_h),
            min(w, cx + half_w), min(h, cy + half_h)
        ])
        
        all_points = np.vstack([fg_points, bg_points]) if len(bg_points) > 0 else fg_points
        all_labels = np.array([1] * len(fg_points) + [0] * len(bg_points))
        
        masks, scores, _ = predictor.predict(
            point_coords=all_points,
            point_labels=all_labels,
            box=box,
            multimask_output=True
        )
        
    elif method == "multipoint":
        all_points = np.vstack([fg_points, bg_points]) if len(bg_points) > 0 else fg_points
        all_labels = np.array([1] * len(fg_points) + [0] * len(bg_points))
        
        masks, scores, _ = predictor.predict(
            point_coords=all_points,
            point_labels=all_labels,
            multimask_output=True
        )
    
    else:  # box
        cx, cy = w * roi_center_x, h * roi_center_y
        half_w, half_h = (w * roi_width) / 2.0, (h * roi_height) / 2.0
        box = np.array([
            max(0, cx - half_w), max(0, cy - half_h),
            min(w, cx + half_w), min(h, cy + half_h)
        ])
        
        masks, scores, _ = predictor.predict(
            box=box,
            multimask_output=True
        )
    
    best_idx = np.argmax(scores)
    mask01 = masks[best_idx].astype(np.uint8)
    
    # 4. 배경 힌트 적용
    mask01 = mask01 * bg_hint
    
    # 5. GrabCut 정제
    if use_grabcut and mask01.sum() > 100:
        mask01 = grabcut_refine(rgb, mask01, iterations=3)
    
    return mask01


def refine_mask_advanced(mask: np.ndarray, min_area: int = 1000) -> np.ndarray:
    """고급 마스크 후처리"""
    num_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    
    refined = np.zeros_like(mask)
    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        if area >= min_area:
            refined[labels == i] = 1
    
    kernel_close = np.ones((7, 7), np.uint8)
    refined = cv2.morphologyEx(refined, cv2.MORPH_CLOSE, kernel_close)
    
    kernel_open = np.ones((3, 3), np.uint8)
    refined = cv2.morphologyEx(refined, cv2.MORPH_OPEN, kernel_open)
    
    return refined


def apply_mask(rgb: np.ndarray, mask01: np.ndarray, bg="white") -> np.ndarray:
    bgc = (255, 255, 255) if bg == "white" else (0, 0, 0)
    out = np.full_like(rgb, bgc, dtype=np.uint8)
    out[mask01.astype(bool)] = rgb[mask01.astype(bool)]
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames-dir", type=Path, required=True)
    ap.add_argument("--out-dir", type=Path, default=Path("sam_out_v4"))
    ap.add_argument("--sam", type=Path, required=True)
    ap.add_argument("--bg", choices=["white","black"], default="white")
    
    ap.add_argument("--method", choices=["adaptive", "multipoint", "box"],
                    default="adaptive")
    ap.add_argument("--use-grabcut", action="store_true", default=True)
    ap.add_argument("--roi-width", type=float, default=0.5)
    ap.add_argument("--roi-height", type=float, default=0.5)
    ap.add_argument("--roi-center-x", type=float, default=0.5)
    ap.add_argument("--roi-center-y", type=float, default=0.5)
    ap.add_argument("--min-area", type=int, default=1000)
    
    args = ap.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    predictor = load_sam(str(args.sam), device=device)

    masks_dir = args.out_dir / "masks"
    masked_dir = args.out_dir / "masked_images"
    debug_dir = args.out_dir / "debug"
    
    for d in [masks_dir, masked_dir, debug_dir]:
        d.mkdir(parents=True, exist_ok=True)

    imgs = sorted([p for p in args.frames_dir.glob("*.png")] + 
                  [p for p in args.frames_dir.glob("*.jpg")])
    
    if not imgs:
        raise SystemExit(f"No images in {args.frames_dir}")

    print(f"\n🎯 Segmentation 설정")
    print(f"   방식: {args.method}")
    print(f"   GrabCut: {'사용' if args.use_grabcut else '미사용'}")
    print(f"   ROI: center=({args.roi_center_x:.2f}, {args.roi_center_y:.2f}), "
          f"size=({args.roi_width:.2f}, {args.roi_height:.2f})")
    
    # 첫 이미지로 배경색 감지
    first_bgr = cv2.imread(str(imgs[0]))
    first_rgb = cv2.cvtColor(first_bgr, cv2.COLOR_BGR2RGB)
    bg_info = auto_detect_background_color(first_rgb)
    print(f"\n{'='*60}")
    
    for i, imgp in enumerate(imgs):
        bgr = cv2.imread(str(imgp), cv2.IMREAD_COLOR)
        if bgr is None:
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        # SAM 마스킹 (배경 정보 전달)
        mask01 = adaptive_sam_mask(
            predictor, rgb,
            method=args.method,
            roi_center_x=args.roi_center_x,
            roi_center_y=args.roi_center_y,
            roi_width=args.roi_width,
            roi_height=args.roi_height,
            use_grabcut=args.use_grabcut,
            bg_info=bg_info,
        )

        mask01 = refine_mask_advanced(mask01, args.min_area)

        # 저장
        mask_u8 = (mask01 * 255).astype(np.uint8)
        cv2.imwrite(str(masks_dir / f"{imgp.stem}_mask.png"), mask_u8)

        masked = apply_mask(bgr, mask01, bg=args.bg)
        cv2.imwrite(str(masked_dir / f"{imgp.stem}.png"), masked)

        # 디버그
        debug = bgr.copy()
        overlay = np.zeros_like(bgr)
        overlay[mask01.astype(bool)] = [0, 255, 0]
        debug = cv2.addWeighted(bgr, 0.7, overlay, 0.3, 0)
        
        bg_hint = adaptive_background_removal(rgb, bg_info)
        fg_points, bg_points = get_smart_points(rgb, bg_hint, n_points=9)
        for pt in fg_points:
            cv2.circle(debug, tuple(pt), 5, (0, 255, 0), -1)
        for pt in bg_points:
            cv2.circle(debug, tuple(pt), 5, (0, 0, 255), -1)
        
        cv2.imwrite(str(debug_dir / f"{imgp.stem}_debug.png"), debug)

        if (i+1) % 10 == 0:
            print(f"   진행: {i+1}/{len(imgs)}")

    print(f"\n✅ 완료!")
    print(f"   Masks:  {masks_dir.resolve()}")
    print(f"   Images: {masked_dir.resolve()}")
    print(f"   Debug:  {debug_dir.resolve()}")


if __name__ == "__main__":
    main()