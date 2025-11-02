import open3d as o3d
import numpy as np
import copy
import matplotlib.pyplot as plt

# --- 이전 코드의 align_scan_to_master 함수는 그대로 사용합니다 ---
def align_scan_to_master(scan_pcd, master_mesh, threshold=0.02, trans_init=np.identity(4)):
    """
    스캔 포인트 클라우드를 마스터 메시에 정밀하게 정렬(ICP)합니다.
    """
    print("스캔 모델을 마스터 모델에 ICP로 정렬합니다...")
    master_pcd_for_icp = master_mesh.sample_points_uniformly(number_of_points=len(scan_pcd.points) * 2)
    master_pcd_for_icp.estimate_normals() # Point-to-Plane ICP를 위해 법선 계산
    scan_pcd.estimate_normals()
    
    reg_p2l = o3d.pipelines.registration.registration_icp(
        scan_pcd, master_pcd_for_icp, threshold, trans_init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    
    scan_pcd_aligned = copy.deepcopy(scan_pcd)
    scan_pcd_aligned.transform(reg_p2l.transformation)
    
    print("정렬 완료.")
    return scan_pcd_aligned

# --- 여기가 핵심: 거리 계산, 오차 분석, 시각화 함수 ---
def analyze_and_visualize_deviation(aligned_scan_pcd, master_mesh, tolerance=0.005):
    """
    정렬된 스캔과 마스터 메시 간의 부호 있는 거리를 계산하고,
    허용 오차에 따라 색상을 입혀 '오차 지도'를 생성 및 저장합니다.
    
    Args:
        aligned_scan_pcd (o3d.geometry.PointCloud): 마스터에 정렬된 스캔 포인트 클라우드
        master_mesh (o3d.geometry.TriangleMesh): 마스터 메시 모델
        tolerance (float): 허용 오차 (미터 단위). 이 값을 벗어나면 불량으로 간주.
        
    Returns:
        o3d.geometry.PointCloud: 오차 정보가 색상으로 입혀진 포인트 클라우드
    """
    print("스캔과 마스터 간의 부호 있는 거리(오차)를 계산합니다...")
    
    # 1. 마스터 메시로부터 RaycastingScene 생성
    master_mesh_t = o3d.t.geometry.TriangleMesh.from_legacy(master_mesh)
    scene = o3d.t.geometry.RaycastingScene()
    scene.add_triangles(master_mesh_t)

    # 2. 정렬된 스캔 포인트 클라우드의 점들을 쿼리로 준비
    query_points = np.asarray(aligned_scan_pcd.points)
    
    # 3. 부호 있는 거리 계산 (Cloud-to-Mesh)
    # compute_signed_distance는 점이 메시 내부에 있으면 음수, 외부에 있으면 양수를 반환합니다.
    signed_distances = scene.compute_signed_distance(query_points.astype(np.float32)).numpy()

    # 4. 오차에 따라 포인트 클라우드에 색상 입히기 (오차 지도 생성)
    print(f"오차 지도(Deviation Map)를 생성합니다... (허용 오차: {tolerance*1000:.2f}mm)")
    
    # 색상을 저장할 배열을 생성하고, 기본색을 녹색(정상)으로 초기화합니다.
    colors = np.full((len(query_points), 3), [0.0, 0.8, 0.0]) # [R, G, B]
    
    # NumPy의 불리언 인덱싱을 사용하여 조건에 맞는 점들의 색상을 한 번에 변경합니다.
    # 허용 오차보다 많이 돌출된 부분 (양의 오차) -> 빨간색
    colors[signed_distances > tolerance] = [1, 0, 0]
    # 허용 오차보다 많이 함몰된 부분 (음의 오차) -> 파란색
    colors[signed_distances < -tolerance] = [0, 0, 1]
    
    # 원본 포인트 클라우드의 복사본을 만들어 색상을 입힙니다.
    pcd_colored = copy.deepcopy(aligned_scan_pcd)
    pcd_colored.colors = o3d.utility.Vector3dVector(colors)

    # 5. 편차 정보를 담은 포인트 클라우드를 파일로 저장
    o3d.io.write_point_cloud("deviation_report.pcd", pcd_colored)
    print("성공! 오차 분석 결과가 'deviation_report.pcd' 파일로 저장되었습니다.")
    
    return pcd_colored, signed_distances


def main():
    """메인 실행 함수"""
    # 1. 데이터 로드
    try:
        armadillo_mesh_data = o3d.data.ArmadilloMesh()
        master_model = o3d.io.read_triangle_mesh(armadillo_mesh_data.path)
        master_model.compute_vertex_normals()

        # 비교 대상을 만들기 위해 마스터 모델을 약간 변형하고 샘플링
        compare_pcd = copy.deepcopy(master_model).translate((0.01, 0.02, -0.01)).scale(1.02, center=master_model.get_center())
        compare_pcd = compare_pcd.sample_points_uniformly(number_of_points=20000)
    except Exception as e:
        print(f"데이터 로드 실패: {e}")
        return

    # 2. 최종 정렬
    trans_init = np.identity(4)
    aligned_pcd = align_scan_to_master(compare_pcd, master_model, trans_init=trans_init)

    # 3. 거리 계산 및 시각화용 오차 지도 생성
    # 허용 오차를 2mm로 설정
    deviation_pcd, deviation_values = analyze_and_visualize_deviation(aligned_pcd, master_model, tolerance=0.002)
    
    # 통계 출력
    positive_deviation = deviation_values[deviation_values > 0.002]
    negative_deviation = deviation_values[deviation_values < -0.002]
    
    print("\n--- 분석 결과 ---")
    print(f"총 {len(deviation_values)}개 포인트 분석 완료")
    print(f"평균 오차: {np.mean(deviation_values)*1000:.4f} mm")
    print(f"최대 돌출 오차: {np.max(deviation_values)*1000:.4f} mm")
    print(f"최대 함몰 오차: {np.min(deviation_values)*1000:.4f} mm")
    print(f"오차 발생 지점 (돌출): {len(positive_deviation)}개")
    print(f"오차 발생 지점 (함몰): {len(negative_deviation)}개")


    # 4. 최종 결과 시각화
    print("\n최종 오차 지도를 표시합니다.")
    print("녹색: 정상 / 빨간색: 돌출 / 파란색: 함몰")
    o3d.visualization.draw_geometries([deviation_pcd], window_name="Deviation Map")

if __name__ == "__main__":
    main()