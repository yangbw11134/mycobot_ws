#!/usr/bin/env python3
import argparse, os, glob
import cv2
import numpy as np

# -------------------------------
# (NEW) 후처리 + 면적 안정화 유틸
# -------------------------------
def _postprocess(binmask: np.ndarray, k: int = 1, use_hull: bool = False) -> np.ndarray:
    """
    동일한 형태학 보정으로 경계 민감도를 낮춰 면적 안정화.
    k: 0이면 보정 끔, 1~2 권장
    use_hull: True면 컨벡스 헐로 작은 구멍/톱니를 메움
    """
    if k > 0:
        kernel = np.ones((3, 3), np.uint8)
        binmask = cv2.morphologyEx(binmask, cv2.MORPH_CLOSE, kernel, iterations=k)
        binmask = cv2.dilate(binmask, kernel, iterations=k)
    if use_hull:
        cnts, _ = cv2.findContours(binmask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        hullmask = np.zeros_like(binmask)
        for c in cnts:
            hull = cv2.convexHull(c)
            cv2.drawContours(hullmask, [hull], -1, 255, thickness=-1)
        binmask = hullmask
    return binmask

def _largest_component_area(binmask: np.ndarray) -> int:
    """
    가장 큰 연결 성분의 면적만 사용(작은 점/파편 영향 제거).
    """
    num, labels, stats, _ = cv2.connectedComponentsWithStats(binmask, connectivity=8)
    if num <= 1:  # 배경만
        return 0
    # stats: [label, x, y, w, h, area], 0은 배경
    largest = 1 + np.argmax(stats[1:, cv2.CC_STAT_AREA])
    return int(stats[largest, cv2.CC_STAT_AREA])

# -------------------------------
# (UPDATED) 폴더/파일 입력 처리
# -------------------------------
def load_area_from_path(path: str, morph_k: int = 1, use_hull: bool = False) -> int:
    """
    path가 폴더면 *.png를 union 후 동일 후처리 → 가장 큰 성분 면적.
    path가 파일이면 그 파일 1장에 동일 후처리 → 가장 큰 성분 면적.
    """
    if os.path.isdir(path):
        mask_paths = sorted(glob.glob(os.path.join(path, "*.png")))
        if not mask_paths:
            raise FileNotFoundError(f"No mask PNGs in dir: {path}")
        union = None
        for p in mask_paths:
            m = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
            if m is None:
                continue
            m_bin = (m > 0).astype(np.uint8) * 255
            union = m_bin if union is None else np.maximum(union, m_bin)
        if union is None:
            raise RuntimeError(f"Failed to read masks in {path}")
        union = _postprocess(union, k=morph_k, use_hull=use_hull)
        return _largest_component_area(union)
    else:
        m = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
        if m is None:
            raise FileNotFoundError(f"Mask not found: {path}")
        m_bin = (m > 0).astype(np.uint8) * 255
        m_bin = _postprocess(m_bin, k=morph_k, use_hull=use_hull)
        return _largest_component_area(m_bin)

def main():
    ap = argparse.ArgumentParser(
        description="Compute absolute distances D1,D2 from SAM mask areas and known deltaD."
    )
    ap.add_argument("--S1", required=True,
                    help="D1에서 얻은 마스크(폴더 또는 단일 mask.png)")
    ap.add_argument("--S2", required=True,
                    help="D2에서 얻은 마스크(폴더 또는 단일 mask.png)")
    ap.add_argument("--deltaD", type=float, required=True,
                    help="카메라 이동거리 (단위 cm). 기본 가정: D2 = D1 - deltaD (가까워짐)")
    ap.add_argument("--direction", choices=["closer","farther"], default="closer",
                    help="'closer'면 D2=D1-deltaD, 'farther'면 D2=D1+deltaD")
    # (NEW) 안정화 옵션
    ap.add_argument("--morph", type=int, default=1,
                    help="형태학 보정 강도 k (0=off, 1~2 권장). default=1")
    ap.add_argument("--hull", action="store_true",
                    help="컨벡스 헐 사용(틈새/구멍 메움)")
    # (NEW) 방향 자동 판별
    ap.add_argument("--auto_direction", action="store_true",
                    help="A2>A1이면 closer, A2<A1이면 farther로 자동 판단")
    args = ap.parse_args()

    A1 = load_area_from_path(args.S1, morph_k=args.morph, use_hull=args.hull)
    A2 = load_area_from_path(args.S2, morph_k=args.morph, use_hull=args.hull)

    if A1 <= 0 or A2 <= 0:
        raise ValueError("Mask areas must be positive.")

    # 거리비 r = sqrt(A1/A2) (주의: A2>A1이면 가까워진 것)
    r = np.sqrt(A1 / A2)

    # (NEW) 자동 방향 결정을 우선 적용
    direction = args.direction
    if args.auto_direction:
        direction = "closer" if A2 > A1 else "farther"
        print(f"[auto] direction = {direction} (A1={A1}, A2={A2})")

    # 수치적으로 아주 근접할 때 1e-9 여유
    eps = 1e-9
    if direction == "closer":
        # D2 = D1 - ΔD  ->  D1 = ΔD / (1 - r)
        denom = (1.0 - r)
        if denom <= eps:
            raise ValueError(f"Invalid ratio: r={r:.6f}. Expect r<1 when getting closer (A2>A1).")
        D1 = args.deltaD / denom
        D2 = D1 - args.deltaD
    else:
        # D2 = D1 + ΔD (멀어짐)  ->  D1 = ΔD / (r - 1)
        denom = (r - 1.0)
        if denom <= eps:
            raise ValueError(f"Invalid ratio: r={r:.6f}. Expect r>1 when getting farther (A2<A1).")
        D1 = args.deltaD / denom
        D2 = D1 + args.deltaD

    print("==== Results ====")
    print(f"A1 (px): {A1}")
    print(f"A2 (px): {A2}")
    print(f"ratio r = sqrt(A1/A2): {r:.6f}")
    print(f"ΔD (cm): {args.deltaD:.4f}  (direction: {direction})")
    print(f"--> D1 (cm): {D1:.4f}")
    print(f"--> D2 (cm): {D2:.4f}")

if __name__ == "__main__":
    main()


""" 사용방식 아래 코드 터미널에서 실행
python3 distance_from_masks.py\
--S1 /(마스크 이미지 경로)\
--S2 /(마스크 이미지 경로)\
--deltaD (cm로 수치 넣기)\
--auto_dirction \
--morph 1
"""
