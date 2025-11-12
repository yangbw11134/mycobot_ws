#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sample_inspect.py  (PCA+matchShapes 중심 판정 버전)

- 샘플 이미지/마스크를 마스터와 동일한 월드 평면으로 워프
- 동일 규칙(PCA 정규화: 중심 정렬 + 주성분 회전 보정) 적용
- 마스터 정규화 마스크와 형상 비교 → PASS/FAIL 판정(JSON/PNG 출력)
  * 기본 판정 기준: PCA 주축 각도 차이, 이방성(λ1/λ2) 차이, matchShapes
  * IoU/면적/Hausdorff는 리포트만 남기고 기본 판정에는 미사용(옵션 --use_iou로 포함 가능)

출력:
- new_features.json
- comparison_result.json
- overlay.png (정규화된 마스터/샘플 시각 비교)
- overlay_scaled.png (없음; 스케일 보정 미사용 버전)

기본 경로는 /home/yuni/mycobot_ws 하위로 지정되어 있어 인자 없이 실행 가능.
"""

import os, json, argparse, math
import numpy as np
import cv2

# -------------------- IO 유틸 --------------------
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

def load_json(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)

def load_image(path):
    img = cv2.imread(path, cv2.IMREAD_COLOR)
    if img is None:
        raise FileNotFoundError(f"[ERR] 이미지를 열 수 없음: {path}")
    return img

def load_mask(mask_path, shape=None, invert=False):
    if mask_path.endswith(".npy"):
        m = np.load(mask_path)
        if m.ndim == 3: m = m[...,0]
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

# -------------------- 마스크 처리 --------------------
def refine_mask(mask, morph_open=0, morph_close=0, largest_contour=True):
    m = mask.copy().astype(np.uint8)
    if morph_open>0:
        k=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
        for _ in range(morph_open): m=cv2.morphologyEx(m, cv2.MORPH_OPEN, k)
    if morph_close>0:
        k=cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
        for _ in range(morph_close): m=cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)
    if largest_contour:
        cnts,_=cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if cnts:
            idx=int(np.argmax([cv2.contourArea(c) for c in cnts]))
            m=np.zeros_like(m); cv2.drawContours(m,[cnts[idx]],-1,1,-1)
    return m

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
    major = eigvecs[:,0]
    angle_deg = math.degrees(math.atan2(major[1], major[0]))  # 0°=u+, 90°=v+
    lam1, lam2 = float(eigvals[0]), float(eigvals[1])
    aniso = lam1 / max(lam2, 1e-9)
    return (float(mean[0]), float(mean[1])), angle_deg, (lam1,lam2), aniso

def center_rotate_normalize(mask, angle_deg, center_xy=None):
    """주성분 각도만큼 회전(-angle)하고 무게중심을 캔버스 중앙으로 정렬."""
    Hc, Wc = mask.shape[:2]
    if center_xy is None:
        ys, xs = np.where(mask>0)
        cx = float(np.mean(xs)); cy = float(np.mean(ys))
    else:
        cx, cy = center_xy
    Mrot = cv2.getRotationMatrix2D((cx,cy), -angle_deg, 1.0)
    rot = cv2.warpAffine(mask*255, Mrot, (Wc,Hc), flags=cv2.INTER_NEAREST, borderValue=0)
    rot = (rot>127).astype(np.uint8)
    ys, xs = np.where(rot>0)
    if xs.size == 0: return rot
    cx2 = float(np.mean(xs)); cy2 = float(np.mean(ys))
    tx = (Wc-1)/2.0 - cx2; ty = (Hc-1)/2.0 - cy2
    Mshift = np.array([[1,0,tx],[0,1,ty]], dtype=np.float32)
    out = cv2.warpAffine(rot*255, Mshift, (Wc,Hc), flags=cv2.INTER_NEAREST, borderValue=0)
    return (out>127).astype(np.uint8)

# -------------------- 지표/시각화 --------------------
def sample_metrics(master_norm, sample_norm, world_res_mm):
    # IoU / Dice / area diff (리포트용)
    A_m = int(master_norm.sum()); A_s = int(sample_norm.sum())
    inter = int((master_norm & sample_norm).sum())
    union = int((master_norm | sample_norm).sum())
    iou  = inter / max(union, 1)
    dice = (2*inter) / max(A_m + A_s, 1)
    area_diff_pct = abs(A_s - A_m) / max(A_m,1) * 100.0

    # Contour 기반 matchShapes / Hausdorff(mm)
    cnts_m,_ = cv2.findContours(master_norm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    cnts_s,_ = cv2.findContours(sample_norm, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    matchShapes = None
    hausdorff_mm = None
    if cnts_m and cnts_s:
        cm = max(cnts_m, key=cv2.contourArea)
        cs = max(cnts_s, key=cv2.contourArea)
        matchShapes = float(cv2.matchShapes(cm, cs, cv2.CONTOURS_MATCH_I1, 0.0))
        a = cm.reshape(-1,2).astype(np.float32)
        b = cs.reshape(-1,2).astype(np.float32)
        da = np.sqrt(((a[:,None,:]-b[None,:,:])**2).sum(axis=2)).min(axis=1)
        db = np.sqrt(((b[:,None,:]-a[None,:,:])**2).sum(axis=2)).min(axis=1)
        Hpix = float(max(da.max(), db.max()))
        hausdorff_mm = Hpix * world_res_mm

    return {
        "iou": float(iou),
        "dice": float(dice),
        "area_diff_pct": float(area_diff_pct),
        "matchShapes": matchShapes,
        "hausdorff_mm": hausdorff_mm
    }

def overlay_pair(master_norm, sample_norm, out_path):
    """마스터=Green, 샘플=Red, 교집합 가시화."""
    Hc,Wc = master_norm.shape[:2]
    rgb = np.zeros((Hc,Wc,3), dtype=np.uint8)
    rgb[...,1] = (master_norm*255).astype(np.uint8)   # G
    rgb[...,2] = (sample_norm*255).astype(np.uint8)   # R
    both = ((master_norm & sample_norm)*255).astype(np.uint8)
    rgb[...,0] = both // 2                            # 약간의 B → 노란기
    cv2.imwrite(out_path, rgb)

def angle_diff_deg(a, b):
    """두 각도의 최소 차이(도). 180° 기준 wrap."""
    d = abs(a - b) % 360.0
    if d > 180.0: d = 360.0 - d
    return d

# -------------------- 메인 --------------------
def main():
    ap=argparse.ArgumentParser()
    # 입력(기본 경로 내장)
    ap.add_argument("--image", default="/home/yangbi/mycobot_ws/src/sample_mask.png")
    ap.add_argument("--mask",  default="//home/yangbi/mycobot_ws/src/sample_mask.png")
    ap.add_argument("--homography_yaml", default="/home/yangbi/mycobot_ws/src/homography.yaml")
    ap.add_argument("--compare_to", default="/home/yangbi/mycobot_ws/src/master_ref/master_features.json",
                    help="master_features.json 경로")
    ap.add_argument("--out_dir", default="//home/yangbi/mycobot_ws/src/sample_result")
    ap.add_argument("--world_res_mm", type=float, default=None, help="마스터와 동일 사용(권장), 강제로 재지정 가능")
    # 마스크 정제
    ap.add_argument("--morph_open", type=int, default=1)
    ap.add_argument("--morph_close", type=int, default=1)
    ap.add_argument("--invert_mask", action="store_true")
    # 판정 임계값 (PCA/이방성/매치셰이프 중심)
    ap.add_argument("--angle_max_deg", type=float, default=10.0,
                    help="마스터 대비 PCA 주축 각도 허용 오차(도)")
    ap.add_argument("--aniso_max_diff", type=float, default=0.6,
                    help="마스터 대비 이방성(λ1/λ2) 허용 차이")
    ap.add_argument("--matchShapes_max", type=float, default=0.15)
    # (옵션) IoU/면적/Hausdorff도 판정에 포함하고 싶을 때만 사용
    ap.add_argument("--use_iou", action="store_true")
    ap.add_argument("--iou_min", type=float, default=0.92)
    ap.add_argument("--hausdorff_max_mm", type=float, default=2.0)
    ap.add_argument("--area_diff_max_pct", type=float, default=5.0)

    args=ap.parse_args()
    os.makedirs(args.out_dir, exist_ok=True)

    # 마스터 메타 로드
    master = load_json(args.compare_to)
    world_res = args.world_res_mm if args.world_res_mm is not None else float(master["meta"]["world_res_mm"])
    Wc = int(master["meta"]["canvas_size"]["width"])
    Hc = int(master["meta"]["canvas_size"]["height"])

    # 이미지→월드→캔버스 변환행렬(Hfinal) 구성(마스터와 동일 ROI)
    dataH = load_yaml_like(args.homography_yaml)
    Hmat = np.array(dataH["H"], dtype=np.float64)
    xr = master["meta"]["world_roi"]
    xmin,xmax,ymin,ymax = float(xr["xmin"]), float(xr["xmax"]), float(xr["ymin"]), float(xr["ymax"])
    sx = 1.0/world_res; sy = 1.0/world_res
    S = np.array([[ sx, 0.0, -xmin*sx],
                  [ 0.0,-sy,  ymax*sy],
                  [ 0.0, 0.0, 1.0]], dtype=np.float64)
    Hfinal = S @ Hmat

    # 입력 로드/정제 → 월드 워프 → PCA 정규화
    img = load_image(args.image); Himg,Wimg = img.shape[:2]
    mask = load_mask(args.mask, shape=(Himg,Wimg), invert=args.invert_mask)
    mask = refine_mask(mask, args.morph_open, args.morph_close, largest_contour=True)

    sample_world = cv2.warpPerspective(mask*255, Hfinal, (Wc,Hc), flags=cv2.INTER_NEAREST, borderValue=0)
    sample_world = (sample_world>127).astype(np.uint8)

    (cx,cy), ang, (l1,l2), aniso = pca_on_mask(sample_world)
    sample_norm = center_rotate_normalize(sample_world, ang, center_xy=(cx,cy))

    # 마스터 정규화 마스크 & PCA 기준
    master_norm_path = master["files"]["master_mask_norm_png"]
    master_norm = load_mask(master_norm_path, shape=(Hc,Wc), invert=False)
    master_angle = float(master["pca"]["angle_deg"])
    master_aniso = float(master["pca"]["anisotropy_ratio"])

    # PCA 차이
    d_angle = angle_diff_deg(ang, master_angle)
    d_aniso = abs(aniso - master_aniso)

    # 형상 지표(리포트용)
    feats = sample_metrics(master_norm, sample_norm, world_res)
    feats["pca_angle_deg"] = float(ang)
    feats["pca_angle_diff_deg"] = float(d_angle)
    feats["anisotropy_ratio"] = float(aniso)
    feats["anisotropy_diff"] = float(d_aniso)

    # 판정: PCA 각도/이방성 + matchShapes (기본)
    failed = []
    if d_angle > args.angle_max_deg:
        failed.append(f"angle_diff>{args.angle_max_deg}deg")
    if d_aniso > args.aniso_max_diff:
        failed.append(f"anisotropy_diff>{args.aniso_max_diff}")
    if feats["matchShapes"] is not None and feats["matchShapes"] > args.matchShapes_max:
        failed.append(f"matchShapes>{args.matchShapes_max}")

    # (옵션) IoU/면적/Hausdorff도 판정에 포함하고 싶을 때
    if args.use_iou:
        if feats["iou"] < args.iou_min:
            failed.append(f"iou<{args.iou_min}")
        if feats["area_diff_pct"] > args.area_diff_max_pct:
            failed.append(f"area_diff_pct>{args.area_diff_max_pct}")
        if feats["hausdorff_mm"] is not None and feats["hausdorff_mm"] > args.hausdorff_max_mm:
            failed.append(f"hausdorff_mm>{args.hausdorff_max_mm}")

    decision = "PASS" if len(failed)==0 else "FAIL"

    # 저장물
    overlay_path = os.path.join(args.out_dir, "overlay.png")
    overlay_pair(master_norm, sample_norm, overlay_path)

    new_feats = {
        "meta": {
            "image": os.path.abspath(args.image),
            "mask": os.path.abspath(args.mask),
            "homography_yaml": os.path.abspath(args.homography_yaml),
            "world_res_mm": world_res,
            "canvas_size": {"width": Wc, "height": Hc}
        },
        "pca": {
            "angle_deg": ang,
            "eigvals": {"lambda1": l1, "lambda2": l2},
            "anisotropy_ratio": aniso
        },
        "features": feats,
        "files": {
            "overlay_png": overlay_path
        }
    }
    with open(os.path.join(args.out_dir, "new_features.json"), "w", encoding="utf-8") as f:
        json.dump(new_feats, f, ensure_ascii=False, indent=2)

    comp = {
        "metrics": feats,
        "thresholds": {
            "angle_max_deg": args.angle_max_deg,
            "aniso_max_diff": args.aniso_max_diff,
            "matchShapes_max": args.matchShapes_max,
            "use_iou": args.use_iou,
            "iou_min": args.iou_min,
            "hausdorff_max_mm": args.hausdorff_max_mm,
            "area_diff_max_pct": args.area_diff_max_pct
        },
        "decision": decision,
        "failed_rules": failed
    }
    with open(os.path.join(args.out_dir, "comparison_result.json"), "w", encoding="utf-8") as f:
        json.dump(comp, f, ensure_ascii=False, indent=2)

    # 콘솔 로그
    print("=== Sample inspected (PCA + matchShapes decision) ===")
    # 3) 콘솔 출력: PASS / FAIL 명확히 표시
    if decision == "PASS":
        print("------PASS!------")
    else:
        print("------FAILED------")

    print(f"PCA angle={feats['pca_angle_deg']:.2f}° (d={feats['pca_angle_diff_deg']:.2f}°), "
          f"aniso={feats['anisotropy_ratio']:.3f} (d={feats['anisotropy_diff']:.3f}), "
          f"matchShapes={feats['matchShapes']}")
    print(f"IoU={feats['iou']:.3f}, Dice={feats['dice']:.3f}, "
          f"Hausdorff={feats['hausdorff_mm'] if feats['hausdorff_mm'] is not None else 'NA'} mm, "
          f"AreaDiff={feats['area_diff_pct']:.2f}%")
    print(f"[Saved] overlay : {overlay_path}")
    print(f"[Saved] new_features.json / comparison_result.json → {args.out_dir}")

if __name__=="__main__":
    main()