import open3d as o3d
import numpy as np
import glob
import os
import argparse

def preprocess_point_cloud(pcd, voxel_size):
    """포인트 클라우드 전처리: 다운샘플링, 법선 추정"""
    pcd_down = pcd.voxel_down_sample(voxel_size)
    pcd_down.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 2, max_nn=30))
    return pcd_down

def execute_global_registration(source_down, target_down, voxel_size):
    """
    FPFH + RANSAC을 이용한 전역 정합을 수행하여 대략적인 초기 변환을 찾습니다.
    """
    distance_threshold = voxel_size * 1.5
    source_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        source_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100))
    target_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        target_down,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size * 5, max_nn=100))
    
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result.transformation

def pairwise_registration(source, target, max_correspondence_distance_fine, init_transformation):
    """
    두 포인트 클라우드 간의 정밀한 쌍별 정합(ICP)을 수행합니다.
    """
    icp_fine = o3d.pipelines.registration.registration_icp(
        source, target, max_correspondence_distance_fine,
        init_transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())
    transformation_icp = icp_fine.transformation
    information_icp = o3d.pipelines.registration.get_information_matrix_from_point_clouds(
        source, target, max_correspondence_distance_fine, icp_fine.transformation)
    return transformation_icp, information_icp

def build_pose_graph_robust(pcds, voxel_size):
    """
    전역 정합을 포함하여 여러 포인트 클라우드로부터 안정적인 포즈 그래프를 생성합니다.
    """
    pose_graph = o3d.pipelines.registration.PoseGraph()
    odometry = np.identity(4)
    pose_graph.nodes.append(o3d.pipelines.registration.PoseGraphNode(odometry))
    n_pcds = len(pcds)

    for source_id in range(n_pcds):
        for target_id in range(source_id + 1, n_pcds):
            if target_id == source_id + 1:  # Odometry edge (이웃 스캔)
                print(f"Computing odometry edge between {source_id} and {target_id}")
                # 이웃 스캔은 위치가 가까우므로 ICP를 바로 사용해도 안정적입니다.
                # 더 빠른 계산을 위해 Coarse-to-Fine ICP를 사용합니다.
                max_correspondence_distance_coarse = voxel_size * 15
                max_correspondence_distance_fine = voxel_size * 1.5
                
                icp_coarse = o3d.pipelines.registration.registration_icp(
                    pcds[source_id], pcds[target_id], max_correspondence_distance_coarse, np.identity(4),
                    o3d.pipelines.registration.TransformationEstimationPointToPlane())
                
                transformation_icp, information_icp = pairwise_registration(
                    pcds[source_id], pcds[target_id], max_correspondence_distance_fine, icp_coarse.transformation)
                
                odometry = np.dot(transformation_icp, odometry)
                pose_graph.nodes.append(
                    o3d.pipelines.registration.PoseGraphNode(np.linalg.inv(odometry)))
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id, target_id, transformation_icp,
                                                            information_icp, uncertain=False))
            else:  # Loop closure edge (이웃하지 않은 스캔)
                print(f"Computing loop closure edge between {source_id} and {target_id}")
                # 이웃하지 않은 스캔은 위치가 멀 수 있으므로, 전역 정합을 먼저 수행합니다.
                # 1. 전역 정합으로 대략적인 초기 위치를 찾습니다.
                initial_guess = execute_global_registration(pcds[source_id], pcds[target_id], voxel_size)
                
                # 2. 전역 정합 결과를 초기값으로 하여 ICP로 정밀하게 다듬습니다.
                max_correspondence_distance_fine = voxel_size * 1.5
                transformation_icp, information_icp = pairwise_registration(
                    pcds[source_id], pcds[target_id], max_correspondence_distance_fine, initial_guess)
                
                pose_graph.edges.append(
                    o3d.pipelines.registration.PoseGraphEdge(source_id, target_id, transformation_icp,
                                                            information_icp, uncertain=True))
    return pose_graph

def optimize_and_combine(pose_graph, pcds, voxel_size):
    """포즈 그래프를 최적화하고, 모든 포인트 클라우드를 하나로 병합합니다."""
    print("포즈 그래프를 최적화합니다...")
    option = o3d.pipelines.registration.GlobalOptimizationOption(
        max_correspondence_distance=voxel_size * 1.5,
        edge_prune_threshold=0.25,
        reference_node=0)
    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        o3d.pipelines.registration.global_optimization(
            pose_graph,
            o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt(),
            o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(),
            option)
    
    print("최적화된 포즈로 전체 포인트 클라우드를 합칩니다...")
    pcd_combined = o3d.geometry.PointCloud()
    for point_id in range(len(pcds)):
        # 원본 포인트 클라우드에 최적화된 변환을 적용합니다.
        pcds[point_id].transform(pose_graph.nodes[point_id].pose)
        pcd_combined += pcds[point_id]

    pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size=voxel_size)
    o3d.io.write_point_cloud("multiway_registration_final.pcd", pcd_combined_down)
    print("성공! 'multiway_registration_final.pcd' 파일이 저장되었습니다.")
    
    return pcd_combined_down

def main():
    """메인 실행 함수"""
    base_dir = "scans"
    
    # 'scans' 폴더 안의 모든 하위 폴더 리스트를 가져옵니다.
    all_subdirs = [os.path.join(base_dir, d) for d in os.listdir(base_dir) if os.path.isdir(os.path.join(base_dir, d))]
    
    if not all_subdirs:
        print(f"'{base_dir}' 안에 스캔 폴더가 없습니다.")
        return
        
    # 폴더 이름(타임스탬프)을 기준으로 가장 최근 폴더를 찾습니다.
    latest_dir = max(all_subdirs)
    print(f"가장 최근 스캔 폴더인 '{latest_dir}'를 자동으로 선택합니다.")
    
    scan_dir = latest_dir # 가장 최근 폴더를 경로로 사용
    
    pcd_files = glob.glob(os.path.join(scan_dir, "scan_*.pcd"))
        
    pcd_files.sort(key=lambda f: int(''.join(filter(str.isdigit, f))))
    print(f"총 {len(pcd_files)}개의 파일을 찾았습니다: {pcd_files}")
    
    voxel_size = 0.02  # 다운샘플링 복셀 크기, 모든 계산의 기준이 됨

    # 포인트 클라우드를 불러와 전처리합니다.
    pcds = []
    for file in pcd_files:
        pcd = o3d.io.read_point_cloud(file)
        pcd_down = preprocess_point_cloud(pcd, voxel_size)
        pcds.append(pcd_down)

    # 1. 안정성이 강화된 포즈 그래프 생성
    pose_graph = build_pose_graph_robust(pcds, voxel_size)

    # 2. 최적화 및 병합하여 최종 포인트 클라우드 생성
    pcd_final = optimize_and_combine(pose_graph, pcds, voxel_size)

    # 3. 최종 결과 시각화 (메시 대신 포인트 클라우드)
    print("최종 정합된 포인트 클라우드를 표시합니다.")
    o3d.visualization.draw_geometries([pcd_final],
                                      window_name="Final Registered Point Cloud")

if __name__ == "__main__":
    main()