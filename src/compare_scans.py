import open3d as o3d
import numpy as np
import copy
import os
import glob

# --- align_scan_to_master, analyze_and_visualize_deviation 함수는 이전과 동일 ---
def align_scan_to_master(scan_pcd, master_mesh, threshold=0.02, trans_init=np.identity(4)):
    """스캔 포인트 클라우드를 마스터 메시에 정밀하게 정렬(ICP)합니다."""
    print("스캔 모델을 마스터 모델에 ICP로 정렬합니다...")
    master_pcd_for_icp = master_mesh.sample_points_uniformly(number_of_points=len(scan_pcd.points) * 2)
    master_pcd_for_icp.estimate_normals()
    scan_pcd.estimate_normals()
    
    reg_p2l = o3d.pipelines.registration.registration_icp(
        scan_pcd, master_pcd_for_icp, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    
    scan_pcd_aligned = copy.deepcopy(scan_pcd)
    scan_pcd_aligned.transform(reg_p2l.transformation)
    
    print("정렬 완료.")
    return scan_pcd_aligned

def analyze_and_visualize_deviation(aligned_scan_pcd, master_mesh, tolerance=0.005):
    """정렬된 스캔과 마스터 메시 간의 거리를 계산하고 오차 지도를 생성합니다."""
    print("스캔과 마스터 간의 오차를 계산합니다...")
    
    master_mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(master_mesh)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(master_mesh_t)
    
    query_points = np.asarray(aligned_scan_pcd.points)
    signed_distances = scene.compute_signed_distance(query_points.astype(np.float32)).numpy()

    print(f"오차 지도(Deviation Map)를 생성합니다... (허용 오차: {tolerance*1000:.2f}mm)")
    colors = np.full((len(query_points), 3), [0.0, 0.8, 0.0])
    colors[signed_distances > tolerance] = [1, 0, 0]
    colors[signed_distances < -tolerance] = [0, 0, 1]
    
    pcd_colored = copy.deepcopy(aligned_scan_pcd)
    pcd_colored.colors = o3d.utility.Vector3dVector(colors)
    
    o3d.io.write_point_cloud("deviation_report.ply", pcd_colored)
    print("성공! 오차 분석 결과가 'deviation_report.ply' 파일로 저장되었습니다.")
    
    return pcd_colored, signed_distances

# --- 메인 실행 부분을 자동으로 경로를 찾도록 수정 ---

def main():
    """메인 실행 함수"""
    # 1. 'scans' 폴더에서 가장 최신 스캔 2개를 자동으로 찾습니다.
    try:
        scan_dir = "scans"
        # scans 폴더 안의 모든 하위 폴더 목록을 가져옵니다.
        all_subdirs = [os.path.join(scan_dir, d) for d in os.listdir(scan_dir) if os.path.isdir(os.path.join(scan_dir, d))]
        if len(all_subdirs) < 2:
            raise FileNotFoundError("비교할 스캔 폴더가 2개 미만입니다.")
        
        # 폴더를 이름순(시간순)으로 정렬하고 가장 최신 2개를 선택합니다.
        all_subdirs.sort()
        latest_two_dirs = all_subdirs[-2:]
        
        # 각 폴더 안의 .ply 파일을 찾습니다.
        scan_file_path = glob.glob(os.path.join(latest_two_dirs[0], "*.ply"))[0]
        master_file_path = glob.glob(os.path.join(latest_two_dirs[1], "*.ply"))[0]

    except (FileNotFoundError, IndexError) as e:
        print(f"❌ 스캔 파일을 찾는 데 실패했습니다: {e}")
        print("'scans' 폴더 구조를 확인해주세요.")
        return

    print(f"'{master_file_path}'을 기준으로 '{scan_file_path}'을 비교합니다.")
    
    # 2. 데이터 로드
    master_pcd = o3d.io.read_point_cloud(master_file_path)
    scan_pcd = o3d.io.read_point_cloud(scan_file_path)

    # (이하 분석 코드는 이전과 동일)

    print("기준 모델(마스터)을 분석용 메시로 변환합니다...")
    master_pcd.estimate_normals()
    master_mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(master_pcd, depth=9)
    densities = np.asarray(densities)
    vertices_to_remove = densities < np.quantile(densities, 0.01)
    master_mesh.remove_vertices_by_mask(vertices_to_remove)
    master_mesh.compute_vertex_normals()

    aligned_pcd = align_scan_to_master(scan_pcd, master_mesh)
    deviation_pcd, deviation_values = analyze_and_visualize_deviation(aligned_pcd, master_mesh, tolerance=0.001)
    
    tolerance = 0.01
    positive_deviation = deviation_values[deviation_values > tolerance]
    negative_deviation = deviation_values[deviation_values < -tolerance]
    
    print("\n--- 📈 분석 결과 ---")
    print(f"총 {len(deviation_values)}개 포인트 분석 완료")
    print(f"평균 오차: {np.mean(deviation_values)*1000:.4f} mm")
    print(f"최대 돌출 오차: {np.max(deviation_values)*1000:.4f} mm")
    print(f"최대 함몰 오차: {np.min(deviation_values)*1000:.4f} mm")
    print(f"오차 발생 지점 (돌출): {len(positive_deviation)}개")
    print(f"오차 발생 지점 (함몰): {len(negative_deviation)}개")

    print("\n최종 오차 지도를 표시합니다.")
    print("녹색: 정상 (오차 1mm 이내) / 빨간색: 돌출 / 파란색: 함몰")
    o3d.visualization.draw_geometries([deviation_pcd], window_name="Deviation Map", width=800, height=600)

if __name__ == "__main__":
    main()