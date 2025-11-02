#!/usr/bin/env python3
# calibrate_intrinsics.py
# Usage:
#  1) 보정 이미지 수집:
#     python3 calibrate_intrinsics.py capture --cam 0 --out ./calib_imgs --width 1280 --height 720
#     (스페이스로 저장, q로 종료)
#  2) 내부파라미터 보정 + ROS YAML 저장:
#     python3 calibrate_intrinsics.py calibrate --imgs ./calib_imgs --cols 9 --rows 6 --square 0.024 \
#         --yaml ocam_1280x720.yaml

import cv2
import numpy as np
import argparse
import os
import glob
from datetime import datetime

def save_ros_yaml(yaml_path, K, dist, w, h, camera_name="ocam5cro", model="plumb_bob"):
    fx, fy, cx, cy = K[0,0], K[1,1], K[0,2], K[1,2]
    # plumb_bob 기준 5개(k1,k2,p1,p2,k3)만 저장
    d = np.zeros(5, dtype=float)
    d[:min(5, dist.size)] = dist.ravel()[:min(5, dist.size)]
    content = f"""image_width: {w}
image_height: {h}
camera_name: {camera_name}
camera_matrix:
  rows: 3
  cols: 3
  data: [{fx:.8f}, 0.0, {cx:.8f}, 0.0, {fy:.8f}, {cy:.8f}, 0.0, 0.0, 1.0]
distortion_model: {model}
distortion_coefficients:
  rows: 1
  cols: 5
  data: [{d[0]:.12e}, {d[1]:.12e}, {d[2]:.12e}, {d[3]:.12e}, {d[4]:.12e}]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0,  0.0, 1.0, 0.0,  0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [{fx:.8f}, 0.0, {cx:.8f}, 0.0,  0.0, {fy:.8f}, {cy:.8f}, 0.0,  0.0, 0.0, 1.0, 0.0]
"""
    with open(yaml_path, "w") as f:
        f.write(content)

def draw_text(img, lines, org=(10,30)):
    y = org[1]
    for s in lines:
        cv2.putText(img, s, (org[0], y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2, cv2.LINE_AA)
        y += 24

def cmd_capture(args):
    os.makedirs(args.out, exist_ok=True)
    cap = cv2.VideoCapture(args.cam, cv2.CAP_V4L2)
    if args.width and args.height:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if not cap.isOpened():
        raise RuntimeError("카메라 열기 실패")

    print("[SPACE]=저장, [q]=종료. 체커보드를 다양한 각도/거리/화면 구석까지 고르게 찍으세요.")
    count = 0
    while True:
        ok, frame = cap.read()
        if not ok: break
        disp = frame.copy()
        draw_text(disp, [f"capture -> {args.out}", "SPACE: save, q: quit"])
        cv2.imshow("capture", disp)
        k = cv2.waitKey(1) & 0xFF
        if k == ord(' '):
            ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            path = os.path.join(args.out, f"img_{ts}.png")
            cv2.imwrite(path, frame)
            count += 1
            print(f"saved: {path}")
        elif k == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
    print(f"총 {count}장 저장")

def reprojection_error(objpoints, imgpoints, rvecs, tvecs, K, dist):
    tot_err = 0.0; tot_pts = 0
    for i in range(len(objpoints)):
        proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], K, dist)
        proj = proj.reshape(-1,2)
        err = cv2.norm(imgpoints[i].reshape(-1,2), proj, cv2.NORM_L2)
        tot_err += err*err
        tot_pts += len(objpoints[i])
    return np.sqrt(tot_err / max(tot_pts,1))

def cmd_calibrate(args):
    # 내부 코너 개수(가로 cols, 세로 rows) 주의!
    pattern_size = (args.cols, args.rows)
    square = args.square  # 미터 단위

    # 3D 체커보드 좌표(한 장당 동일)
    objp = np.zeros((pattern_size[0]*pattern_size[1], 3), np.float32)
    objp[:,:2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1,2) * square

    images = sorted(glob.glob(os.path.join(args.imgs, "*.png")) +
                    glob.glob(os.path.join(args.imgs, "*.jpg")) +
                    glob.glob(os.path.join(args.imgs, "*.jpeg")))
    if not images:
        raise RuntimeError("보정용 이미지가 없습니다.")

    objpoints, imgpoints = [], []
    imsize = None
    print(f"{len(images)}장 처리 중…")
    for fn in images:
        img = cv2.imread(fn)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        if imsize is None:
            imsize = (gray.shape[1], gray.shape[0])
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        ok, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
        if not ok: 
            continue
        corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1),
                                   (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3))
        objpoints.append(objp.copy())
        imgpoints.append(corners)
    if len(objpoints) < 10:
        raise RuntimeError(f"체커보드가 검출된 이미지가 부족합니다({len(objpoints)}). 최소 10장 이상 권장.")

    # 보정
    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, imsize, None, None,
        flags=cv2.CALIB_RATIONAL_MODEL)  # 필요 시 RATIONAL_MODEL

    rms = reprojection_error(objpoints, imgpoints, rvecs, tvecs, K, dist)

    print("\n=== Calibration Result ===")
    print(f"Image size     : {imsize[0]}x{imsize[1]}")
    print(f"RMS reproj err : {rms:.4f} px")
    print("Camera Matrix K:\n", K)
    print("Dist Coeffs    :", dist.ravel())

    # 저장
    npz_path = os.path.splitext(args.yaml)[0] + ".npz"
    np.savez(npz_path, K=K, dist=dist, rvecs=rvecs, tvecs=tvecs, image_size=imsize)
    save_ros_yaml(args.yaml, K, dist, imsize[0], imsize[1], camera_name=args.cam_name)
    print(f"\nSaved: {npz_path}")
    print(f"Saved: {args.yaml}")

    # 보정 확인용 샘플 undistort 저장
    sample = cv2.imread(images[0])
    newK, _ = cv2.getOptimalNewCameraMatrix(K, dist, imsize, 0)
    und = cv2.undistort(sample, K, dist, None, newK)
    cv2.imwrite("undistort_check.png", und)
    print("Saved: undistort_check.png  (보정 확인용)")

def main():
    ap = argparse.ArgumentParser(description="Camera intrinsic calibration (images capture + calibrate + ROS YAML)")
    sub = ap.add_subparsers(dest="cmd", required=True)

    ap_cap = sub.add_parser("capture", help="카메라에서 보정용 이미지 수집")
    ap_cap.add_argument("--cam", type=int, default=0, help="VideoCapture index (default 0)")
    ap_cap.add_argument("--out", type=str, required=True, help="출력 폴더")
    ap_cap.add_argument("--width", type=int, default=0, help="캡처 해상도 가로")
    ap_cap.add_argument("--height", type=int, default=0, help="캡처 해상도 세로")
    ap_cap.set_defaults(func=cmd_capture)

    ap_cal = sub.add_parser("calibrate", help="저장된 이미지로 내부 파라미터 보정")
    ap_cal.add_argument("--imgs", type=str, required=True, help="보정 이미지 폴더")
    ap_cal.add_argument("--cols", type=int, required=True, help="체커보드 내부 코너 가로 개수")
    ap_cal.add_argument("--rows", type=int, required=True, help="체커보드 내부 코너 세로 개수")
    ap_cal.add_argument("--square", type=float, required=True, help="한 칸 변 길이(미터)")
    ap_cal.add_argument("--yaml", type=str, required=True, help="저장할 ROS YAML 파일명")
    ap_cal.add_argument("--cam_name", type=str, default="ocam5cro", help="camera_name")
    ap_cal.set_defaults(func=cmd_calibrate)

    args = ap.parse_args()
    args.func(args)

if __name__ == "__main__":
    main()
