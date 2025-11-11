# 파일 생성
#!/usr/bin/env python3
"""
8_mesh_processing.py - COLMAP PLY를 메쉬로 변환

사용법:
source ~/venv-open3d/bin/activate

python /home/yangbi/mycobot_ws/src/colmap_pkg/src/7_mesh_processing.py \
  --input /home/yangbi/mycobot_ws/src/colmap_pkg/colmap_project_sam2_out/sparse/0/points3D.ply \
  --output /home/yangbi/mycobot_ws/src/colmap_pkg/mesh_outputs/sam2_out_mesh.ply \
  --quality high \
  --no-vis
"""

import open3d as o3d
import numpy as np
import argparse
from pathlib import Path
import sys


def load_point_cloud(ply_path: Path):
    """포인트 클라우드 로드"""
    print(f"\n📂 포인트 클라우드 로드: {ply_path}")
    
    if not ply_path.exists():
        raise FileNotFoundError(f"PLY 파일 없음: {ply_path}")
    
    pcd = o3d.io.read_point_cloud(str(ply_path))
    
    print(f"  포인트 수: {len(pcd.points):,}")
    print(f"  색상: {'있음' if pcd.has_colors() else '없음'}")
    print(f"  노말: {'있음' if pcd.has_normals() else '없음'}")
    
    return pcd


def preprocess_point_cloud(pcd, voxel_size=0.002):
    """포인트 클라우드 전처리"""
    print("\n🔧 전처리...")
    
    # Voxel downsampling
    print(f"  Downsampling (voxel: {voxel_size})...")
    pcd_down = pcd.voxel_down_sample(voxel_size=voxel_size)
    print(f"    {len(pcd.points):,} → {len(pcd_down.points):,}")
    
    # Outlier 제거
    print("  Outlier 제거...")
    pcd_clean, ind = pcd_down.remove_statistical_outlier(
        nb_neighbors=20, std_ratio=2.0
    )
    print(f"    {len(pcd_down.points):,} → {len(pcd_clean.points):,}")
    
    # Normal 추정
    print("  Normal 추정...")
    pcd_clean.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_size * 10, max_nn=30
        )
    )
    pcd_clean.orient_normals_consistent_tangent_plane(k=15)
    
    return pcd_clean


def create_mesh_poisson(pcd, depth=9):
    """Poisson 메쉬 재구성"""
    print(f"\n🎨 Poisson 메쉬 재구성 (깊이: {depth})...")
    
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=depth, width=0, scale=1.1, linear_fit=False
    )
    
    print(f"  정점: {len(mesh.vertices):,}")
    print(f"  삼각형: {len(mesh.triangles):,}")
    
    # 저밀도 제거
    print("  저밀도 영역 제거...")
    densities = np.asarray(densities)
    vertices_to_remove = densities < np.quantile(densities, 0.01)
    mesh.remove_vertices_by_mask(vertices_to_remove)
    
    print(f"  정제 후 정점: {len(mesh.vertices):,}")
    print(f"  정제 후 삼각형: {len(mesh.triangles):,}")
    
    return mesh


def postprocess_mesh(mesh, smooth_iterations=5):
    """메쉬 후처리"""
    print("\n🔨 후처리...")
    
    # 중복 제거
    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    
    # 비연결 컴포넌트 제거
    print("  비연결 컴포넌트 제거...")
    triangle_clusters, cluster_n_triangles, _ = mesh.cluster_connected_triangles()
    triangle_clusters = np.asarray(triangle_clusters)
    cluster_n_triangles = np.asarray(cluster_n_triangles)
    
    largest_cluster_idx = cluster_n_triangles.argmax()
    triangles_to_remove = triangle_clusters != largest_cluster_idx
    mesh.remove_triangles_by_mask(triangles_to_remove)
    mesh.remove_unreferenced_vertices()
    
    # Smoothing
    if smooth_iterations > 0:
        print(f"  Smoothing ({smooth_iterations}회)...")
        mesh = mesh.filter_smooth_laplacian(number_of_iterations=smooth_iterations)
    
    mesh.compute_vertex_normals()
    
    return mesh


def save_mesh(mesh, output_path: Path):
    """메쉬 저장"""
    print(f"\n💾 저장...")
    
    output_path = Path(output_path)
    output_dir = output_path.parent
    output_stem = output_path.stem
    
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # PLY
    ply_path = output_dir / f"{output_stem}.ply"
    o3d.io.write_triangle_mesh(str(ply_path), mesh, write_ascii=False, compressed=True)
    print(f"  PLY: {ply_path}")
    
    # OBJ
    obj_path = output_dir / f"{output_stem}.obj"
    o3d.io.write_triangle_mesh(str(obj_path), mesh)
    print(f"  OBJ: {obj_path}")
    
    # STL
    stl_path = output_dir / f"{output_stem}.stl"
    o3d.io.write_triangle_mesh(str(stl_path), mesh)
    print(f"  STL: {stl_path}")
    
    return [ply_path, obj_path, stl_path]


def main():
    parser = argparse.ArgumentParser(description="COLMAP PLY → 메쉬")
    
    parser.add_argument("--input", type=Path, help="입력 PLY")
    parser.add_argument("--project-dir", type=Path, help="COLMAP 프로젝트 디렉토리")
    parser.add_argument("--output", type=Path, default=Path("meshes/final_mesh.ply"))
    parser.add_argument("--quality", choices=["low", "medium", "high", "extreme"],
                        default="high")
    parser.add_argument("--no-vis", action="store_true", help="시각화 스킵")
    
    args = parser.parse_args()
    
    # 입력 결정
    if args.input:
        input_path = args.input
    elif args.project_dir:
        candidates = [
            args.project_dir / "sparse" / "0" / "points3D.ply",
            args.project_dir / "sparse" / "points3D.ply",
        ]
        input_path = None
        for c in candidates:
            if c.exists():
                input_path = c
                break
        if not input_path:
            raise FileNotFoundError(f"PLY 찾을 수 없음: {args.project_dir}")
    else:
        raise ValueError("--input 또는 --project-dir 필요")
    
    print("=" * 60)
    print("COLMAP PLY → 메쉬 변환")
    print("=" * 60)
    print(f"입력: {input_path}")
    print(f"출력: {args.output}")
    print(f"품질: {args.quality}")
    print("=" * 60)
    
    # 품질 파라미터
    params = {
        "low": {"voxel": 0.005, "depth": 8, "smooth": 3},
        "medium": {"voxel": 0.003, "depth": 9, "smooth": 5},
        "high": {"voxel": 0.002, "depth": 10, "smooth": 7},
        "extreme": {"voxel": 0.001, "depth": 11, "smooth": 10},
    }[args.quality]
    
    # 처리
    pcd = load_point_cloud(input_path)
    pcd_clean = preprocess_point_cloud(pcd, voxel_size=params["voxel"])
    mesh = create_mesh_poisson(pcd_clean, depth=params["depth"])
    mesh = postprocess_mesh(mesh, smooth_iterations=params["smooth"])
    
    # 저장
    files = save_mesh(mesh, args.output)
    
    # 시각화
    if not args.no_vis:
        print("\n👁️ 시각화...")
        print("  마우스 드래그: 회전")
        print("  Shift+드래그: 이동")
        print("  휠: 확대/축소")
        print("  Q/ESC: 닫기")
        o3d.visualization.draw_geometries([mesh], window_name="Mesh", 
                                         width=1280, height=720)
    
    print("\n" + "=" * 60)
    print("✅ 완료!")
    print("=" * 60)
    print(f"정점: {len(mesh.vertices):,}")
    print(f"삼각형: {len(mesh.triangles):,}")
    print(f"\n저장:")
    for f in files:
        print(f"  - {f}")
    print("=" * 60)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n❌ 오류: {e}", file=sys.stderr)
        sys.exit(1)