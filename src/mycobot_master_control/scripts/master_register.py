#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
master_register.py (기본 경로 내장 버전)
- 정상품 이미지/마스크를 월드(mm) 평면으로 워프하고 PCA 정규화
- 결과를 master_ref 디렉토리에 자동 저장
- 기본 경로는 /home/yuni/mycobot_ws 기준
"""

import os, json, argparse, math
import numpy as np
import cv2

# ===== 기본 경로 설정 =====
DEFAULT_IMAGE  = "/home/yangbi/mycobot_ws/src/master_mask.png"
DEFAULT_MASK   = "/home/yangbi/mycobot_ws/src/master_mask.png"
DEFAULT_HOMO   = "/home/yangbi/mycobot_ws/src/homography.yaml"
DEFAULT_OUTDIR = "/home/yangbi/mycobot_ws/src/master_ref"
DEFAULT_WORLD_RES = 0.5  # mm per pixel
# ========================

def load_yaml_like(path):
    try:
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)
    except Exception:
        with open(path, "r", encoding="utf-8") as f:
            txt = f.read()
        try:
            return json.loads(txt)
        except Exception as e:
            raise RuntimeError(f"[ERR] YAML/JSON 로드 실패: {e}")

def load_image(path):
    img = cv2.imread(path, cv2.IMREAD_COLOR)
    if img is None:
        raise FileNotFoundError(f"[ERR] 이미지를 열 수 없음: {path}")
    return img

def load_mask(mask_path, shape=None, invert=False):
    if mask_path.endswith(".npy"):
        m = np.load(mask_path)
        if m.ndim == 3:
            m = m[..., 0]
        m = (m > 0).astype(np.uint8)
    else:
        m = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
        if m is None:
            raise FileNotFoundError(f"[ERR] 마스크를 열 수 없음: {mask_path}")
        m = (m > 127).astype(np.uint8)
    if invert:
        m = 1 - m
    if shape is not None and (m.shape[0] != shape[0] or m.shape[1] != shape[1]):
        m = cv2.resize(m, (shape[1], shape[0]), interpolation=cv2.INTER_NEAREST)
    return m

def image_corners(w, h):
    return np.array([[0,0,1],[w-1,0,1],[w-1,h-1,1],[0,h-1,1]], dtype=np.float64).T

def apply_H_to_points(H, pts3xN):
    q = H @ pts3xN
    q /= np.maximum(q[2:3, :], 1e-12)
    return q

def make_world_warp(H, W, Himg, world_res_mm):
    P = image_corners(W, Himg)
    Q = apply_H_to_points(H, P)
    Xs, Ys = Q[0, :], Q[1, :]
    xmin, xmax = float(np.min(Xs)), float(np.max(Xs))
    ymin, ymax = float(np.min(Ys)), float(np.max(Ys))
    sx = 1.0 / world_res_mm
    sy = 1.0 / world_res_mm
    W_out = max(1, int(math.ceil((xmax - xmin) * sx)))
    H_out = max(1, int(math.ceil((ymax - ymin) * sy)))
    S = np.array([
        [ sx,  0.0, -xmin * sx],
        [ 0.0, -sy,  ymax * sy],
        [ 0.0,  0.0, 1.0]
    ], dtype=np.float64)
    H_final = S @ H
    return H_final, (xmin, xmax, ymin, ymax), (W_out, H_out)

def pca_on_mask(mask):
    ys, xs = np.where(mask > 0)
    if xs.size == 0:
        raise RuntimeError("[ERR] 유효한 마스크 픽셀이 없습니다.")
    pts = np.stack([xs, ys], axis=1).astype(np.float32)
    mean = np.mean(pts, axis=0)
    X = pts - mean
    cov = (X.T @ X) / max(len(pts) - 1, 1)
    eigvals, eigvecs = np.linalg.eigh(cov)
    idx = np.argsort(eigvals)[::-1]
    eigvals, eigvecs = eigvals[idx], eigvecs[:, idx]
    major = eigvecs[:, 0]
    angle_deg = math.degrees(math.atan2(major[1], major[0]))
    lam1, lam2 = float(eigvals[0]), float(eigvals[1])
    aniso = lam1 / max(lam2, 1e-9)
    return (float(mean[0]), float(mean[1])), angle_deg, (lam1, lam2), aniso

def center_rotate_normalize(mask, angle_deg, center_xy=None):
    Hc, Wc = mask.shape[:2]
    if center_xy is None:
        ys, xs = np.where(mask > 0)
        cx = float(np.mean(xs))
        cy = float(np.mean(ys))
    else:
        cx, cy = center_xy
    Mrot = cv2.getRotationMatrix2D((cx, cy), -angle_deg, 1.0)
    rot = cv2.warpAffine(mask * 255, Mrot, (Wc, Hc), flags=cv2.INTER_NEAREST, borderValue=0)
    rot = (rot > 127).astype(np.uint8)
    ys, xs = np.where(rot > 0)
    if xs.size == 0:
        return rot
    cx2 = float(np.mean(xs))
    cy2 = float(np.mean(ys))
    tx = (Wc - 1) / 2.0 - cx2
    ty = (Hc - 1) / 2.0 - cy2
    Mshift = np.array([[1, 0, tx], [0, 1, ty]], dtype=np.float32)
    out = cv2.warpAffine(rot * 255, Mshift, (Wc, Hc), flags=cv2.INTER_NEAREST, borderValue=0)
    return (out > 127).astype(np.uint8)

def refine_mask(mask, morph_open=1, morph_close=1, largest_contour=True):
    m = mask.copy().astype(np.uint8)
    if morph_open > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        for _ in range(morph_open):
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    if morph_close > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        for _ in range(morph_close):
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
    if largest_contour:
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if cnts:
            idx = int(np.argmax([cv2.contourArea(c) for c in cnts]))
            m = np.zeros_like(m)
            cv2.drawContours(m, [cnts[idx]], -1, 1, -1)
    return m

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", default=DEFAULT_IMAGE)
    ap.add_argument("--mask", default=DEFAULT_MASK)
    ap.add_argument("--homography_yaml", default=DEFAULT_HOMO)
    ap.add_argument("--world_res_mm", type=float, default=DEFAULT_WORLD_RES)
    ap.add_argument("--out_dir", default=DEFAULT_OUTDIR)
    ap.add_argument("--invert_mask", action="store_true")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    img = load_image(args.image)
    Himg, Wimg = img.shape[:2]
    mask = load_mask(args.mask, shape=(Himg, Wimg), invert=args.invert_mask)
    mask = refine_mask(mask, 1, 1, True)

    dataH = load_yaml_like(args.homography_yaml)
    Hmat = np.array(dataH["H"], dtype=np.float64)

    Hfinal, world_roi, out_size = make_world_warp(Hmat, Wimg, Himg, args.world_res_mm)
    Wc, Hc = out_size
    world_mask = cv2.warpPerspective(mask * 255, Hfinal, (int(Wc), int(Hc)), flags=cv2.INTER_NEAREST, borderValue=0)
    world_mask = (world_mask > 127).astype(np.uint8)

    (cx, cy), ang, (lam1, lam2), aniso = pca_on_mask(world_mask)
    norm_mask = center_rotate_normalize(world_mask, ang, center_xy=(cx, cy))

    area_px = int(norm_mask.sum())
    area_mm2 = area_px * (args.world_res_mm ** 2)
    cnts, _ = cv2.findContours(norm_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    peri_mm = None
    if cnts:
        cm = max(cnts, key=cv2.contourArea)
        peri_mm = float(cv2.arcLength(cm, True) * args.world_res_mm)

    master_png = os.path.join(args.out_dir, "master_mask_norm.png")
    master_feats = os.path.join(args.out_dir, "master_features.json")
    cv2.imwrite(master_png, norm_mask * 255)

    payload = {
        "meta": {
            "image": os.path.abspath(args.image),
            "mask": os.path.abspath(args.mask),
            "homography_yaml": os.path.abspath(args.homography_yaml),
            "world_res_mm": args.world_res_mm,
            "world_roi": {"xmin": world_roi[0], "xmax": world_roi[1],
                          "ymin": world_roi[2], "ymax": world_roi[3]},
            "canvas_size": {"width": Wc, "height": Hc}
        },
        "pca": {
            "angle_deg": ang,
            "eigvals": {"lambda1": lam1, "lambda2": lam2},
            "anisotropy_ratio": aniso
        },
        "features": {
            "area_px": area_px,
            "area_mm2": area_mm2,
            "perimeter_mm": peri_mm
        },
        "files": {
            "master_mask_norm_png": master_png
        }
    }

    with open(master_feats, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)

    print("=== ✅ Master Registered ===")
    print(f"Canvas: {Wc}x{Hc}px @ {args.world_res_mm:.3f} mm/px")
    print(f"PCA angle: {ang:.2f} deg | λ1/λ2={aniso:.2f}")
    print(f"Area: {area_mm2:.2f} mm²  (px={area_px})")
    print(f"Saved PNG : {master_png}")
    print(f"Saved JSON: {master_feats}")

if __name__ == "__main__":
    main()