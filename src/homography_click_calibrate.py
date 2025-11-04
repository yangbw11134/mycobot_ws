#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
homography_click_calibrate.py
이미지에서 네 점(TL, TR, BR, BL 순서) 클릭 → 실제(mm) 좌표와 매칭 → Homography H 계산 및 YAML 저장.

사용 예:
python3 homography_click_calibrate.py \
  --image /home/yuni/mycobot_ws/capture_D1.jpg \
  --square_w_mm 100 --square_h_mm 100 \
  --out /home/yuni/mycobot_ws/homography.yaml
"""

import cv2, os, argparse, numpy as np
from datetime import datetime

pts_img = []

def on_mouse(event, x, y, flags, param):
    global pts_img
    if event == cv2.EVENT_LBUTTONDOWN:
        if len(pts_img) < 4:
            pts_img.append([x, y])
            print(f"[CLICK {len(pts_img)}] (u,v)=({x},{y})")
        else:
            print("[INFO] 이미 4점을 선택했습니다. 'r'로 리셋하세요.")

def maybe_save_yaml(path, data):
    # yaml이 있으면 yaml로, 없으면 간단 문자열로 저장
    try:
        import yaml  # type: ignore
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    except Exception:
        # 최소한의 YAML 호환 문자열로 저장
        def arr(a): return "[" + ", ".join(str(x) for x in a) + "]"
        def arr2(a2d): return "[" + ", ".join("[" + ", ".join(f"{v:.8f}" if isinstance(v, float) else str(v) for v in row) + "]" for row in a2d) + "]"
        with open(path, "w", encoding="utf-8") as f:
            f.write(f"created: {data['created']}\n")
            f.write(f"image_path: {data['image_path']}\n")
            f.write(f"image_size: {{width: {data['image_size']['width']}, height: {data['image_size']['height']}}}\n")
            f.write(f"units: {data['units']}\n")
            f.write(f"order: {data['order']}\n")
            f.write(f"img_pts: {arr2(data['img_pts'])}\n")
            f.write(f"world_pts: {arr2(data['world_pts'])}\n")
            f.write(f"H: {arr2(data['H'])}\n")
            f.write(f"H_inv: {arr2(data['H_inv'])}\n")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True, help="기준 패턴이 보이는 이미지")
    ap.add_argument("--square_w_mm", type=float, required=True, help="기준 사각형 가로(mm)")
    ap.add_argument("--square_h_mm", type=float, required=True, help="기준 사각형 세로(mm)")
    ap.add_argument("--out", default="./homography.yaml", help="저장 경로")
    args = ap.parse_args()

    img = cv2.imread(args.image, cv2.IMREAD_COLOR)
    if img is None:
        raise FileNotFoundError(f"이미지를 열 수 없음: {args.image}")
    H, W = img.shape[:2]

    print("===================================================")
    print("화면에서 네 점을 순서대로 클릭하세요: TL → TR → BR → BL")
    print("리셋: r,  종료/저장: s  (s는 4점을 모두 찍은 후)")
    print("===================================================")

    cv2.namedWindow("calib", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("calib", on_mouse)

    while True:
        disp = img.copy()
        # 그려주기
        for i, (x, y) in enumerate(pts_img):
            cv2.circle(disp, (x, y), 6, (0, 0, 255), -1)
            cv2.putText(disp, f"{i+1}", (x+6, y-6), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,255), 2)
        cv2.imshow("calib", disp)
        key = cv2.waitKey(20) & 0xFF
        if key == ord('r'):
            pts_img.clear()
            print("[RESET] 포인트 초기화")
        elif key == ord('s'):
            if len(pts_img) != 4:
                print("[WARN] 4점을 모두 찍어야 저장됩니다.")
                continue
            break
        elif key == 27:  # ESC
            print("[EXIT] 취소됨")
            cv2.destroyAllWindows()
            return

    cv2.destroyAllWindows()

    img_pts = np.array(pts_img, dtype=np.float32)

    # 세계 좌표 (mm) - TL(0,0), TR(w,0), BR(w,h), BL(0,h)
    world_pts = np.array([
        [0.0,              0.0],
        [args.square_w_mm, 0.0],
        [args.square_w_mm, args.square_h_mm],
        [0.0,              args.square_h_mm]
    ], dtype=np.float32)

    Hmat, mask = cv2.findHomography(img_pts, world_pts)  # 픽셀→mm
    if Hmat is None:
        raise RuntimeError("findHomography 실패: 점들이 일직선이거나 잘못 선택됨")
    Hinv = np.linalg.inv(Hmat)

    save_data = {
        "created": datetime.now().isoformat(timespec="seconds"),
        "image_path": os.path.abspath(args.image),
        "image_size": {"width": W, "height": H},
        "units": "mm",
        "order": "TL,TR,BR,BL",
        "img_pts": img_pts.tolist(),
        "world_pts": world_pts.tolist(),
        "H": Hmat.tolist(),
        "H_inv": Hinv.tolist()
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    maybe_save_yaml(args.out, save_data)
    print(f"[SAVED] Homography → {args.out}")
    print("H =\n", Hmat)

if __name__ == "__main__":
    main()
