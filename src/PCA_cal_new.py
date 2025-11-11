import cv2
import json
import numpy as np
from math import atan2, degrees, radians, cos, sin

def _wrap_deg(a):
    # [-180, 180) 로 정규화
    a = (a + 180.0) % 360.0 - 180.0
    return a

def pca_from_mask(
    mask_path,
    out_path="pca_result.jpg",
    json_path=None,
    # --- (옵션) 카메라 내참 & 깊이 (있으면 픽셀→미터 변환 수행)
    fx=None, fy=None, cx_i=None, cy_i=None, depth_m=None,
    # --- (옵션) 오버레이 길이 배수
    axis_scale=120
):
    """
    반환/저장 항목(핵심):
      - major_axis_deg, minor_axis_deg, gripper_yaw_deg (= minor)
      - centroid_px (cx, cy)
      - width_px (minor 축 방향 폭), (옵션) width_m
      - (옵션) cam_offset_m: 이미지 중심 대비 실제 x/y 오프셋 (카메라 프레임 기준)
    """
    # 1) 마스크
    mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
    if mask is None:
        raise FileNotFoundError(f"Cannot load {mask_path}")

    H, W = mask.shape[:2]
    ys, xs = np.where(mask > 0)
    pts = np.column_stack([xs, ys]).astype(np.float32)
    if pts.shape[0] < 10:
        raise ValueError("Not enough foreground pixels in mask")

    # 2) PCA
    mean = pts.mean(axis=0)           # (cx, cy) in pixels
    X = pts - mean
    cov = np.cov(X, rowvar=False)
    eigvals, eigvecs = np.linalg.eigh(cov)  # 열벡터: 고유벡터
    order = np.argsort(eigvals)[::-1]
    eigvals, eigvecs = eigvals[order], eigvecs[:, order]

    cx, cy = mean
    major_vec = eigvecs[:, 0] / np.linalg.norm(eigvecs[:, 0])  # x-방향 유사 (열: [vx, vy])
    minor_vec = eigvecs[:, 1] / np.linalg.norm(eigvecs[:, 1])

    # 각도(이미지 좌표: x→오른쪽, y→아래)
    major_deg = _wrap_deg(degrees(atan2(major_vec[1], major_vec[0])))
    minor_deg = _wrap_deg(major_deg + 90.0)

    # 그리퍼 yaw는 "닫힘 축 == 물체의 부축(minor)"에 정렬
    gripper_yaw_deg = minor_deg

    # 3) 부축(minor) 방향 폭 추정 (마스크 점들을 minor 축으로 투영하여 extent 계산)
    #    s = (x - cx, y - cy)·v_minor
    s = X @ minor_vec
    width_px = float(s.max() - s.min())

    # 4) (옵션) 픽셀 → 미터 변환
    width_m = None
    cam_offset_m = None
    if fx and fy and depth_m:
        # 이미지 중심(카메라 주점) 기본값: 정중앙
        if cx_i is None: cx_i = (W - 1) * 0.5
        if cy_i is None: cy_i = (H - 1) * 0.5

        du = (cx - cx_i)
        dv = (cy - cy_i)
        # 핀홀 모델: X = (u-cx)/fx * Z, Y = (v-cy)/fy * Z
        X_m = float(du * depth_m / fx)
        Y_m = float(dv * depth_m / fy)
        cam_offset_m = {"x_m": X_m, "y_m": Y_m, "z_m": float(depth_m)}

        # 폭은 방향이 기울어져 있으니 근사적으로 fx,fy 평균 사용 (또는 주성분 방향에 맞춰 보정 가능)
        f_eff = 0.5 * (fx + fy)
        width_m = float(width_px * depth_m / f_eff)

    # 5) 시각화 오버레이
    vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    cv2.circle(vis, (int(round(cx)), int(round(cy))), 6, (0, 0, 255), -1)

    def _draw_axis(vec, color, text):
        L = axis_scale
        p1 = (int(round(cx - vec[0] * L)), int(round(cy - vec[1] * L)))
        p2 = (int(round(cx + vec[0] * L)), int(round(cy + vec[1] * L)))
        cv2.line(vis, p1, p2, color, 2)
        cv2.putText(vis, text, (p2[0]+5, p2[1]+5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    _draw_axis(major_vec, (0, 220, 0),  f"major {major_deg:.1f}°")
    _draw_axis(minor_vec, (220, 0, 0),  f"minor {minor_deg:.1f}°")
    # 그리퍼 닫힘 축(=minor) 강조 표시 (굵게/길게)
    Lg = int(axis_scale * 1.2)
    p1g = (int(round(cx - minor_vec[0] * Lg)), int(round(cy - minor_vec[1] * Lg)))
    p2g = (int(round(cx + minor_vec[0] * Lg)), int(round(cy + minor_vec[1] * Lg)))
    cv2.line(vis, p1g, p2g, (0, 180, 255), 3)
    cv2.putText(vis, f"gripper_yaw={gripper_yaw_deg:.1f} deg",
                (int(cx)+8, int(cy)-8), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 180, 255), 2, cv2.LINE_AA)

    # 폭 가시화 (minor 축으로 투영한 극값 위치 점)
    # t_max, t_min 를 minor 축으로 되돌려 그려줌
    t_min, t_max = float(s.min()), float(s.max())
    p_min = (int(round(cx + minor_vec[0]*t_min)), int(round(cy + minor_vec[1]*t_min)))
    p_max = (int(round(cx + minor_vec[0]*t_max)), int(round(cy + minor_vec[1]*t_max)))
    cv2.circle(vis, p_min, 4, (255, 255, 0), -1)
    cv2.circle(vis, p_max, 4, (255, 255, 0), -1)
    cv2.line(vis, p_min, p_max, (255, 255, 0), 2)
    cv2.putText(vis, f"width_px={width_px:.1f}", (p_max[0]+6, p_max[1]+6),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 1, cv2.LINE_AA)

    cv2.imwrite(out_path, vis)

    # 6) 결과 정리
    result = {
        "centroid_px": {"cx": float(cx), "cy": float(cy)},
        "major_axis_deg": float(major_deg),
        "minor_axis_deg": float(minor_deg),
        "gripper_yaw_deg": float(gripper_yaw_deg),  # == minor_axis_deg
        "eigvals": {"major": float(eigvals[0]), "minor": float(eigvals[1])},
        "width_px_minor": width_px,
        "width_m_minor": width_m,
        "cam_offset_m": cam_offset_m,               # (옵션) 이미지 중심 대비 카메라 프레임 X,Y 오프셋
        "overlay_image": out_path,
        "image_size": {"H": int(H), "W": int(W)}
    }

    if json_path:
        with open(json_path, "w", encoding="utf-8") as f:
            json.dump(result, f, ensure_ascii=False, indent=2)

    return result

if __name__ == "__main__":
    # 예시: JSON 저장 + 내참/깊이 제공 시 미터 값까지 계산
    result = pca_from_mask(
        r"C:\Users\kamig\Downloads\totoro_forPCA_mask.png",
        out_path=r"C:\Users\kamig\Downloads\totoro_pca_result.jpg",
        json_path=r"C:\Users\kamig\Downloads\totoro_pca_result.json",
        fx=None, fy=None, cx_i=None, cy_i=None, depth_m=None  # 내참/깊이 모르면 그대로 두기
    )
    print(json.dumps(result, indent=2))
