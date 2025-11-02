# clean_ply.py
import os
import open3d as o3d
import numpy as np
from pathlib import Path

# CPU 스레드 수를 설정하여 과부하를 방지할 수 있습니다.
os.environ["OMP_NUM_THREADS"] = os.environ.get("OMP_NUM_THREADS", "8")
os.environ["MKL_NUM_THREADS"] = os.environ.get("MKL_NUM_THREADS", "8")


# ===== 사용자 경로 설정 =====
# 이전 스크립트에서 생성된 PLY 파일 경로
PLY_IN  = r"C:\Users\Kangmg\Downloads\popscan\sparse\0\points3D.ply"
# 결과물을 저장할 폴더 경로
OUT_DIR = r"C:\Users\Kangmg\Downloads\popscan\clean_safe"
# ===========================

# 후처리 파라미터 (모델의 크기나 상태에 따라 조절)
VOXEL_SIZE   = 0.003   # 다운샘플 크기 (클수록 점 개수가 줄어듦)
NB_NEIGHBORS = 16      # 통계적 노이즈 제거 시 주변점으로 참고할 개수
STD_RATIO    = 2.0     # 표준편차 기준. 이 값보다 멀리 떨어진 점을 노이즈로 판단

def save(pcd, name):
    """포인트 클라우드를 지정된 이름으로 저장하는 함수"""
    path = str(Path(OUT_DIR) / name)
    o3d.io.write_point_cloud(path, pcd)
    print(f"저장 완료: {path} | 포인트 개수: {len(pcd.points)}")

def main():
    """메인 실행 함수"""
    Path(OUT_DIR).mkdir(parents=True, exist_ok=True)

    print("[1] PLY 파일 불러오기")
    pcd = o3d.io.read_point_cloud(PLY_IN)
    if not pcd.has_points():
        raise FileNotFoundError(f"PLY 파일을 불러오지 못했거나 파일에 포인트가 없습니다: {PLY_IN}")
    print(f"불러온 포인트 개수: {len(pcd.points)}")
    save(pcd, "step0_raw.ply")

    # 다운샘플링 (메모리 사용량과 처리 속도 개선의 핵심)
    print("\n[2] Voxel 다운샘플링 진행")
    pcd_down = pcd.voxel_down_sample(voxel_size=VOXEL_SIZE)
    save(pcd_down, "step1_voxel.ply")

    # 통계적 노이즈(이상치) 제거
    print("\n[3] 통계적 노이즈 제거 진행")
    pcd_clean, ind = pcd_down.remove_statistical_outlier(
        nb_neighbors=NB_NEIGHBORS, std_ratio=STD_RATIO
    )
    # pcd_clean = pcd_down.select_by_index(ind) # 주석 처리된 원본 코드와 달리, 반환된 pcd_clean을 바로 사용
    save(pcd_clean, "step2_stat_clean.ply")

    # 최종 결과 저장
    save(pcd_clean, "final_clean_safe.ply")
    print("\n✅ PLY 후처리 완료")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n[오류] {e}")