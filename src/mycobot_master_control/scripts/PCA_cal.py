import cv2
import numpy as np
from math import atan2, degrees

def pca_from_mask(mask_path, out_path="pca_result.jpg"):
    # 1. 마스크 로드 (0/255 값)
    mask = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
    if mask is None:
        raise FileNotFoundError(f"Cannot load {mask_path}")

    # 2. 흰색 픽셀 좌표 추출
    ys, xs = np.where(mask > 0)
    pts = np.column_stack([xs, ys]).astype(np.float32)

    if pts.shape[0] < 10:
        raise ValueError("Not enough foreground pixels in mask")

    # 3. PCA 계산
    mean = pts.mean(axis=0)
    X = pts - mean
    cov = np.cov(X, rowvar=False)
    eigvals, eigvecs = np.linalg.eigh(cov)

    # 고유값 큰 순서로 정렬
    order = np.argsort(eigvals)[::-1]
    eigvals, eigvecs = eigvals[order], eigvecs[:, order]

    # 중심/각도
    cx, cy = mean
    angle = degrees(atan2(eigvecs[1, 0], eigvecs[0, 0]))

    # 4. 시각화
    vis = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    cv2.circle(vis, (int(cx), int(cy)), 6, (0, 0, 255), -1)  # 중심 빨간 점
    for k in range(2):
        vec = eigvecs[:, k]
        length = 100 * np.sqrt(eigvals[k]) / np.sqrt(max(eigvals[0], 1e-6))
        p1 = (int(cx - vec[0] * length), int(cy - vec[1] * length))
        p2 = (int(cx + vec[0] * length), int(cy + vec[1] * length))
        color = (0, 255, 0) if k == 0 else (255, 0, 0)  # 주축: 초록, 부축: 파랑
        cv2.line(vis, p1, p2, color, 2)

    cv2.imwrite(out_path, vis)

    # 5. 결과 리턴
    return {
        "centroid": (float(cx), float(cy)),
        "angle_deg": angle,
        "eigvals": (float(eigvals[0]), float(eigvals[1])),
        "output_image": out_path
    }


if __name__ == "__main__":
    # 1. 입력 파일 경로 (사용자님이 원하시는 경로)
    input_file = "/home/yangbi/mycobot_ws/src/master_mask.png"
    
    # 2. 결과 이미지 저장 경로 (이름은 원하시는 대로 변경 가능)
    output_file = "/home/yangbi/mycobot_ws/src/master_pca_result.jpg"

    # 스크립트 실행
    result = pca_from_mask(input_file, output_file)
    print(" - Centroid:", result["centroid"])
    print(" - Angle (deg):", result["angle_deg"])
    print(" - Eigenvalues:", result["eigvals"])
    print("Result image saved to:", result["output_image"])