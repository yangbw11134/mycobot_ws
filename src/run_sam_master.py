#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_sam_safe_master.py
- 단일 이미지("/home/yuni/mycobot_ws/capture_master.jpg")만 SAM 자동 마스킹
- 가장자리 접촉 배제 + 중앙 ROI 포함 마스크 중 최대 면적을 선택
- 선택 결과를 "/home/yuni/mycobot_ws/sam_output/master_mask.png" 로 저장

필수: segment-anything, torch, opencv-python, numpy
"""

import os, cv2, numpy as np, torch
from segment_anything import sam_model_registry, SamAutomaticMaskGenerator

# 0) 안전 옵션
os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "max_split_size_mb:64,expandable_segments:True,garbage_collection_threshold:0.8"

# 1) 경로/모델 (요청한 경로로 고정)
IMG_PATH  = "/home/yuni/mycobot_ws/capture_master.jpg"
OUTROOT   = "/home/yuni/mycobot_ws/sam_output"
FINAL_OUT = os.path.join(OUTROOT, "master_mask.png")
os.makedirs(OUTROOT, exist_ok=True)

CKPT  = "/home/yuni/models/sam_vit_b_01ec64.pth"
MODEL = "vit_b"

# 2) 모델 초기화
device = "cuda" if torch.cuda.is_available() else "cpu"
sam = sam_model_registry[MODEL](checkpoint=CKPT).to(device).eval()

# 3) AutomaticMaskGenerator 생성 (기존 설정 유지)
gen = SamAutomaticMaskGenerator(
    model=sam,
    points_per_side=24,
    pred_iou_thresh=0.88,
    stability_score_thresh=0.95,
    crop_n_layers=0,
    min_mask_region_area=0
)

# 4) 단일 이미지 처리
def process_one(IMG_PATH: str):
    img_bgr = cv2.imread(IMG_PATH)
    assert img_bgr is not None, f"이미지 없음: {IMG_PATH}"

    # 너무 큰 이미지 축소 (성능/메모리 세이브)
    H0, W0 = img_bgr.shape[:2]
    MAX_EDGE = 960
    if max(H0, W0) > MAX_EDGE:
        scale = MAX_EDGE / max(H0, W0)
        img_bgr = cv2.resize(img_bgr, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    img_rgb = np.ascontiguousarray(img_rgb)
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

    # 필터 파라미터 (요청 로직 유지)
    EDGE_TOL = 1
    CX_LO, CX_HI = 0.35, 0.65
    CY_LO, CY_HI = 0.30, 0.80

    with torch.inference_mode():
        masks = gen.generate(img_rgb)

        good = []
        for m in masks:
            seg = m["segmentation"].astype(np.uint8)
            props = mask_props(seg)
            if props is None:
                continue
            x0, y0, x1, y1, cx, cy, area = props

            # (b) 프레임 가장자리 접촉 제외
            touches_edge = (x0 <= EDGE_TOL or y0 <= EDGE_TOL or
                            x1 >= W - 1 - EDGE_TOL or y1 >= H - 1 - EDGE_TOL)
            if touches_edge:
                continue

            # (c) 중심이 중앙 ROI 안
            if not (CX_LO * W <= cx <= CX_HI * W and CY_LO * H <= cy <= CY_HI * H):
                continue

            good.append((area, m))

        if len(good) > 0:
            _, best = max(good, key=lambda t: t[0])
            selected = (best["segmentation"].astype(np.uint8) * 255)
            cv2.imwrite(FINAL_OUT, selected)
            print(f"✅ (b,c) 필터 기반 자동 선택 완료 → {FINAL_OUT}")
        else:
            print("⚠️ 필터 일치 없음 → 가장 중앙에 가까운 마스크 사용")
            cx_t, cy_t = W / 2, H / 2

            def center_dist(m):
                seg = m["segmentation"].astype(np.uint8)
                ys, xs = np.nonzero(seg)
                if len(xs) == 0:
                    return 1e9
                cx, cy = xs.mean(), ys.mean()
                return ((cx - cx_t) ** 2 + (cy - cy_t) ** 2) ** 0.5

            if len(masks) > 0:
                best = min(masks, key=center_dist)
                selected = (best["segmentation"].astype(np.uint8) * 255)
                cv2.imwrite(FINAL_OUT, selected)
                print(f"✅ 중앙 근접 마스크 선택 저장 → {FINAL_OUT}")
            else:
                print("❌ SAM 마스크를 생성하지 못했습니다.")

    if torch.cuda.is_available():
        torch.cuda.empty_cache()

# 실행
if __name__ == "__main__":
    process_one(IMG_PATH)
