import os, cv2, numpy as np, torch
from segment_anything import sam_model_registry, SamAutomaticMaskGenerator

# 0) 안전 옵션
os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "max_split_size_mb:64,expandable_segments:True,garbage_collection_threshold:0.8"

# 1) 경로/모델
D1_IMG  = "/home/yuni/mycobot_ws/capture_D1.jpg"
D2_IMG  = "/home/yuni/mycobot_ws/capture_D2.jpg"
CKPT = "/home/yuni/models/sam_vit_b_01ec64.pth"
MODEL = "vit_b"
OUTROOT = "/home/yuni/mycobot_ws/sam_output"
os.makedirs(OUTROOT, exist_ok=True)

# 2) 모델 초기화
device = "cuda" if torch.cuda.is_available() else "cpu"
sam = sam_model_registry[MODEL](checkpoint=CKPT).to(device).eval()

# 3) AutomaticMaskGenerator 생성
gen = SamAutomaticMaskGenerator(
    model=sam,
    points_per_side=24,
    pred_iou_thresh=0.88,
    stability_score_thresh=0.95,
    crop_n_layers=0,
    min_mask_region_area=0
)

# 4) 이미지 처리 함수
def process_one(IMG_PATH: str, tag: str):
    outdir = os.path.join(OUTROOT, tag)
    os.makedirs(outdir, exist_ok=True)

    img_bgr = cv2.imread(IMG_PATH)
    assert img_bgr is not None, f"이미지 없음: {IMG_PATH}"
    H, W = img_bgr.shape[:2]
    MAX_EDGE = 960
    if max(H, W) > MAX_EDGE:
        scale = MAX_EDGE / max(H, W)
        img_bgr = cv2.resize(img_bgr, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    img_rgb = np.ascontiguousarray(img_rgb)

    with torch.inference_mode():
        masks = gen.generate(img_rgb)

        H, W = img_rgb.shape[:2]

        def mask_props(seg):
            ys, xs = np.nonzero(seg)
            if len(xs) == 0:
                return None
            x0, x1 = xs.min(), xs.max()
            y0, y1 = ys.min(), ys.max()
            cx, cy = xs.mean(), ys.mean()
            area = seg.sum()
            return (x0, y0, x1, y1, cx, cy, int(area))

        EDGE_TOL = 1  # 가장자리 접촉 허용 여유(px)

        # 중앙 ROI (예: 가로 35~65%, 세로 30~80%)
        CX_LO, CX_HI = 0.35, 0.65
        CY_LO, CY_HI = 0.30, 0.80

        good = []

        for m in masks:
            seg = m["segmentation"].astype(np.uint8)
            props = mask_props(seg)
            if props is None:
                continue
            x0, y0, x1, y1, cx, cy, area = props

            # (b) 가장자리와 닿은 마스크 제외
            touches_edge = (x0 <= EDGE_TOL or y0 <= EDGE_TOL or
                            x1 >= W-1-EDGE_TOL or y1 >= H-1-EDGE_TOL)
            if touches_edge:
                continue

            # (c) 중심이 중앙 ROI 안에 있어야 함
            if not (CX_LO*W <= cx <= CX_HI*W and CY_LO*H <= cy <= CY_HI*H):
                continue

            good.append((area, m))

        if len(good) > 0:
            _, best = max(good, key=lambda t: t[0])
            selected = (best["segmentation"].astype(np.uint8) * 255)
            cv2.imwrite(os.path.join(outdir, "capture_mask_main.png"), selected)
            print("✅ (b,c) 필터 기반 자동 선택 완료(capture_mask_main.png)")
        else:
            print("⚠️ 필터 일치 없음 → 가장 중앙에 가까운 마스크 사용")
            cx_t, cy_t = W/2, H/2
            def center_dist(m):
                seg = m["segmentation"].astype(np.uint8)
                ys, xs = np.nonzero(seg)
                if len(xs) == 0: return 1e9
                cx, cy = xs.mean(), ys.mean()
                return ((cx - cx_t)**2 + (cy - cy_t)**2)**0.5

            if len(masks) > 0:
                best = min(masks, key=center_dist)
                selected = (best["segmentation"].astype(np.uint8) * 255)
                cv2.imwrite(os.path.join(outdir, "capture_mask_main.png"), selected)
            else:
                print("❌ SAM 마스크가 생성되지 않았습니다.")



    # --- (1) 전체 마스크 저장 ---
    for i, ms in enumerate(masks):
        binmask = (ms["segmentation"].astype(np.uint8) * 255)
        cv2.imwrite(os.path.join(outdir, f"capture_mask_{i:03d}.png"), binmask)

    if torch.cuda.is_available():
        torch.cuda.empty_cache()

# ===== D1, D2 순차 처리 =====
process_one(D1_IMG, "D1")
process_one(D2_IMG, "D2")

if torch.cuda.is_available():
    torch.cuda.empty_cache()
