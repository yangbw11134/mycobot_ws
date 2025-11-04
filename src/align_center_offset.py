#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
align_center_offset.py
- 외부 마스크(흑백 PNG 또는 .npy)만 사용해 물체 중심/PCA와
  프레임 중심 간 2D 오프셋을 계산하고 시각화
- PCA 고유값/이방성(λ1/λ2) 출력 → 각도 신뢰도 판단
- Homography YAML를 넣으면 픽셀→실좌표(mm)로 변환하여 ΔX, ΔY(mm) 자동 산출
  * homography_click_calibrate.py로 H(3x3) 저장 (키: "H")

사용방식
python3 align_center_offset.py \
  --image /home/yuni/mycobot_ws/capture_D1.jpg \
  --mask  /home/yuni/mycobot_ws/sam_output/D1/capture_mask_main.png \
  --largest_contour --morph_open 1 --morph_close 1 \
  --homography_yaml /home/yuni/mycobot_ws/homography.yaml \
  --out_dir /home/yuni/mycobot_ws/PCA_json

"""

import os, json, argparse, math
import numpy as np
import cv2

def load_yaml_like(path):
    """YAML(권장) 또는 JSON 텍스트를 로드."""
    try:
        import yaml  # pip install pyyaml
        with open(path, "r", encoding="utf-8") as f:
            return yaml.safe_load(f)
    except Exception:
        import json as _json
        with open(path, "r", encoding="utf-8") as f:
            txt = f.read()
        try:
            return _json.loads(txt)
        except Exception as e:
            raise RuntimeError(f"[ERR] YAML/JSON 로드 실패: {e}")

def apply_H(H, u, v):
    """픽셀 (u,v) → 실좌표(mm) (X,Y), Homography 3x3 적용."""
    p = np.array([u, v, 1.0], dtype=np.float64)
    q = H @ p
    q /= (q[2] if abs(q[2]) > 1e-12 else 1.0)
    return float(q[0]), float(q[1])

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

def refine_mask(mask, morph_open=0, morph_close=0, largest_contour=False):
    """간단한 정제: morphology + 최대 컨투어 선택(옵션)"""
    m = mask.copy().astype(np.uint8)

    if morph_open > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        for _ in range(morph_open):
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, k)

    if morph_close > 0:
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3,3))
        for _ in range(morph_close):
            m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, k)

    if largest_contour:
        cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        if len(cnts) > 0:
            # 가장 큰 외곽만 남김
            areas = [cv2.contourArea(c) for c in cnts]
            idx = int(np.argmax(areas))
            m = np.zeros_like(m)
            cv2.drawContours(m, [cnts[idx]], -1, color=1, thickness=-1)

    return m

def pca_on_mask(mask):
    ys, xs = np.where(mask > 0)
    if xs.size == 0:
        raise RuntimeError("[ERR] 유효한 마스크 픽셀이 없습니다.")
    pts = np.stack([xs, ys], axis=1).astype(np.float32)  # (N,2) [u,v]
    mean = np.mean(pts, axis=0)
    X = pts - mean
    cov = (X.T @ X) / max(len(pts) - 1, 1)
    eigvals, eigvecs = np.linalg.eigh(cov)  # 오름차순
    idx = np.argsort(eigvals)[::-1]         # 내림차순 정렬 인덱스
    eigvals, eigvecs = eigvals[idx], eigvecs[:, idx]
    major = eigvecs[:, 0]
    angle_deg = math.degrees(math.atan2(major[1], major[0]))  # 0°=우측, 90°=아래
    lam1, lam2 = float(eigvals[0]), float(eigvals[1])
    anisotropy = lam1 / max(lam2, 1e-9)  # λ1/λ2
    return (float(mean[0]), float(mean[1])), angle_deg, (lam1, lam2), anisotropy

def draw_overlay(img, mask, cam_cx, cam_cy, obj_cx, obj_cy, angle_deg, du, dv,
                 anisotropy, out_path):
    ov = img.copy()
    mask_rgb = np.dstack([mask*0, mask*255, mask*0]).astype(np.uint8)
    ov = cv2.addWeighted(ov, 1.0, mask_rgb, 0.25, 0)

    # 중심 마커
    cv2.drawMarker(ov, (int(round(cam_cx)), int(round(cam_cy))), (255,255,255),
                   markerType=cv2.MARKER_CROSS, markerSize=18, thickness=2)
    cv2.circle(ov, (int(round(obj_cx)), int(round(obj_cy))), 7, (0,0,255), -1)

    # PCA 주성분 화살표
    L = 80
    theta = math.radians(angle_deg)
    x2 = int(round(obj_cx + L*math.cos(theta)))
    y2 = int(round(obj_cy + L*math.sin(theta)))
    cv2.arrowedLine(ov, (int(round(obj_cx)), int(round(obj_cy))), (x2, y2),
                    (0,0,255), 2, tipLength=0.2)

    # 텍스트
    txt = f"Δu={du:.1f}px, Δv={dv:.1f}px, angle={angle_deg:.1f}°, λ1/λ2={anisotropy:.2f}"
    # 가독성 박스
    w = 10 + 8*len(txt)
    cv2.rectangle(ov, (10,10), (10+int(w), 44), (0,0,0), -1)
    cv2.putText(ov, txt, (16,36), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2, cv2.LINE_AA)

    cv2.imwrite(out_path, ov)

def main():
    ap = argparse.ArgumentParser(description="프레임 중심과 물체 중심의 2D 오프셋 계산(외부 마스크 전용)")
    ap.add_argument("--image", required=True)
    ap.add_argument("--mask",  required=True, help="흑백 PNG(0/255) 또는 .npy(bool/0-1)")
    ap.add_argument("--invert_mask", action="store_true", help="마스크가 반전되어 있을 때 사용")
    ap.add_argument("--morph_open", type=int, default=0, help="열기 반복 횟수(노이즈 제거)")
    ap.add_argument("--morph_close", type=int, default=0, help="닫기 반복 횟수(구멍 메우기)")
    ap.add_argument("--largest_contour", action="store_true", help="가장 큰 컨투어만 남김")
    ap.add_argument("--mm_per_px", type=float, default=None, help="픽셀→mm 스케일(예: 0.35)")
    ap.add_argument("--homography_yaml", default=None, help="픽셀→mm 변환용 Homography YAML 경로(H 키 포함)")
    ap.add_argument("--out_dir", default="./out")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)

    img = load_image(args.image)
    H, W = img.shape[:2]
    mask = load_mask(args.mask, shape=(H, W), invert=args.invert_mask)
    mask = refine_mask(mask,
                       morph_open=args.morph_open,
                       morph_close=args.morph_close,
                       largest_contour=args.largest_contour)

    (obj_cx, obj_cy), angle_deg, (lam1, lam2), anisotropy = pca_on_mask(mask)
    cam_cx = (W - 1) / 2.0
    cam_cy = (H - 1) / 2.0
    du = obj_cx - cam_cx
    dv = obj_cy - cam_cy
    u_norm = du / (W * 0.5)
    v_norm = dv / (H * 0.5)

    # mm 변환 (우선순위: Homography → mm_per_px)
    dx_mm = dy_mm = None
    X_mm = Y_mm = Xc_mm = Yc_mm = None  # (선택) 실제 좌표 기록

    if args.homography_yaml:
        dataH = load_yaml_like(args.homography_yaml)
        Hmat = np.array(dataH["H"], dtype=np.float64)  # 키 이름: "H"
        # 물체 중심 / 카메라 중심을 mm 좌표로 변환
        X_mm, Y_mm   = apply_H(Hmat, obj_cx, obj_cy)
        Xc_mm, Yc_mm = apply_H(Hmat, cam_cx, cam_cy)
        dx_mm = X_mm - Xc_mm
        dy_mm = Y_mm - Yc_mm
    elif args.mm_per_px is not None:
        dx_mm = du * args.mm_per_px
        dy_mm = dv * args.mm_per_px


    overlay_path = os.path.join(args.out_dir, "overlay.png")
    draw_overlay(img, mask, cam_cx, cam_cy, obj_cx, obj_cy, angle_deg, du, dv,
                 anisotropy, overlay_path)

    metrics = {
        "image_size": {"width": W, "height": H},
        "camera_center_px": {"u": cam_cx, "v": cam_cy},
        "object_center_px": {"u": obj_cx, "v": obj_cy},
        "offset_px": {"du": du, "dv": dv},
        "offset_norm": {"u_norm": u_norm, "v_norm": v_norm},
        "principal_angle_deg": angle_deg,
        "eigvals": {"lambda1": lam1, "lambda2": lam2},
        "anisotropy_ratio": anisotropy,  # ← 각도 신뢰도 판단 지표
        "has_mm": args.mm_per_px is not None,
        "mm_per_px": args.mm_per_px,
        "has_mm": (dx_mm is not None and dy_mm is not None),   # ← H 또는 mm_per_px로 계산됐는지 여부
        "offset_mm": {"dx_mm": dx_mm, "dy_mm": dy_mm} if (dx_mm is not None and dy_mm is not None) else None,
        "homography_yaml": args.homography_yaml,
        "object_world_mm": {"X": X_mm, "Y": Y_mm} if (X_mm is not None and Y_mm is not None) else None,
        "center_world_mm": {"Xc": Xc_mm, "Yc": Yc_mm} if (Xc_mm is not None and Yc_mm is not None) else None,
        "offset_mm": {"dx_mm": dx_mm, "dy_mm": dy_mm} if args.mm_per_px is not None else None
    }
    with open(os.path.join(args.out_dir, "metrics.json"), "w", encoding="utf-8") as f:
        json.dump(metrics, f, ensure_ascii=False, indent=2)

    print("==== 2D Centering Metrics (Depth-free; mask-only) ====")
    print(f"Camera center (px):  u0={cam_cx:.2f}, v0={cam_cy:.2f}")
    print(f"Object  center (px):  u ={obj_cx:.2f}, v ={obj_cy:.2f}")
    print(f"PCA major angle:     {angle_deg:.2f} deg")
    print(f"Anisotropy λ1/λ2:    {anisotropy:.3f}")
    if anisotropy < 1.2:
        print("⚠️  정사각형/원형 또는 노이즈 영향 → 각도 신뢰도 낮음(사용 비추천).")
    if dx_mm is not None and dy_mm is not None:
        src = "H" if args.homography_yaml else "mm_per_px"
        print(f"Offset  (mm):        dx={dx_mm:.2f} mm, dy={dy_mm:.2f} mm   [{src}]")
    if X_mm is not None and Y_mm is not None:
        print(f"Object world (mm):   X={X_mm:.2f}, Y={Y_mm:.2f}")
    if Xc_mm is not None and Yc_mm is not None:
        print(f"Center world (mm):   Xc={Xc_mm:.2f}, Yc={Yc_mm:.2f}")

    print(f"[Saved] overlay: {overlay_path}")
    print(f"[Saved] metrics: {os.path.join(args.out_dir, 'metrics.json')}")
    print("======================================================")

if __name__ == "__main__":
    main()
