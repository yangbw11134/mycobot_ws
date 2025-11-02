# 2_sam_image.py
import argparse, cv2, numpy as np, torch
from pathlib import Path
from segment_anything import SamPredictor, sam_model_registry

def load_sam(ckpt: str, device: str = "cuda"):
    model_type = "vit_h"
    sam = sam_model_registry[model_type](checkpoint=ckpt)
    sam.to(device=device)
    return SamPredictor(sam)

def center_prompt_mask(predictor: SamPredictor, rgb: np.ndarray) -> np.ndarray:
    predictor.set_image(rgb)
    h, w, _ = rgb.shape
    pt = np.array([[w/2.0, h/2.0]])
    lbl = np.array([1], dtype=np.int32)  # foreground
    masks, scores, _ = predictor.predict(
        point_coords=pt,
        point_labels=lbl,
        multimask_output=False
    )
    return masks[0].astype(np.uint8)  # 0/1

def apply_mask(rgb: np.ndarray, mask01: np.ndarray, bg="white") -> np.ndarray:
    if bg == "white":
        bgc = (255, 255, 255)
    else:
        bgc = (0, 0, 0)
    out = np.full_like(rgb, bgc, dtype=np.uint8)
    out[mask01.astype(bool)] = rgb[mask01.astype(bool)]
    return out

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--frames-dir", type=Path, required=True)
    ap.add_argument("--out-dir", type=Path, default=Path("sam_out"))
    ap.add_argument("--sam", type=Path, required=True, help="sam_vit_h_4b8939.pth 경로")
    ap.add_argument("--bg", choices=["white","black"], default="white")
    args = ap.parse_args()

    device = "cuda" if torch.cuda.is_available() else "cpu"
    predictor = load_sam(str(args.sam), device=device)

    masks_dir = args.out_dir / "masks"
    masked_dir = args.out_dir / "masked_images"
    masks_dir.mkdir(parents=True, exist_ok=True)
    masked_dir.mkdir(parents=True, exist_ok=True)

    imgs = sorted([p for p in args.frames_dir.glob("*.png")] + [p for p in args.frames_dir.glob("*.jpg")])
    if not imgs:
        raise SystemExit(f"No images in {args.frames_dir}")

    for i, imgp in enumerate(imgs):
        bgr = cv2.imread(str(imgp), cv2.IMREAD_COLOR)
        if bgr is None: 
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)

        mask01 = center_prompt_mask(predictor, rgb)  # 0/1
        # 저장: 이진 마스크
        mask_u8 = (mask01 * 255).astype(np.uint8)
        cv2.imwrite(str(masks_dir / f"{imgp.stem}_mask.png"), mask_u8)

        # 저장: 배경 제거 컬러
        masked = apply_mask(bgr, mask01, bg=args.bg)
        cv2.imwrite(str(masked_dir / f"{imgp.stem}.png"), masked)

        if (i+1) % 10 == 0:
            print(f"... {i+1}/{len(imgs)}")

    print(f"✅ masks  → {masks_dir.resolve()}")
    print(f"✅ images → {masked_dir.resolve()}")

if __name__ == "__main__":
    main()
