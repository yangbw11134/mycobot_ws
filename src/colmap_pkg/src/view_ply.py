# view_ply.py
import open3d as o3d
import numpy as np

# ===== 사용자 경로 설정 =====
# 확인하고 싶은 최종 PLY 파일 경로
ply_path = r"C:\Users\Kangmg\Downloads\popscan\clean_safe\final_clean_safe.ply"
# ===========================

# 포인트 클라우드 파일 불러오기
try:
    pcd = o3d.io.read_point_cloud(ply_path)
    if not pcd.has_points():
        raise FileNotFoundError("파일에 포인트가 없습니다.")
except Exception as e:
    print(f"[오류] 파일을 불러오는 데 실패했습니다: {ply_path}\n{e}")
    exit()

# 기본 정보 출력
print(pcd)
print(f"포인트 개수: {len(pcd.points)}")

# 좌표와 색상 정보가 있다면 배열로 가져와서 일부 출력
if pcd.has_points():
    points = np.asarray(pcd.points)
    print("첫 5개 좌표:\n", points[:5])

if pcd.has_colors():
    colors = np.asarray(pcd.colors)
    print("첫 5개 색상 (RGB 0~1):\n", colors[:5])

# Open3D 시각화 창으로 결과물 확인
print("\n3D 뷰어 창을 실행합니다...")
o3d.visualization.draw_geometries([pcd])