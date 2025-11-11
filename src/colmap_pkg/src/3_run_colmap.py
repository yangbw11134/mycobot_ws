'''
cd /home/yangbi/mycobot_ws/src/colmap_pkg

python3 src/3_run_colmap.py \
  --images-dir /home/yangbi/mycobot_ws/src/colmap_pkg/sam2_out/masked_images \
  --project-dir /home/yangbi/mycobot_ws/src/colmap_pkg/colmap_project_sam2_out \
  --matcher sequential \
  --quality high

'''


import subprocess
import sys
from pathlib import Path

COLMAP_EXE = "colmap"

def run(cmd):
    """명령어 실행"""
    print(">>>", " ".join(map(str, cmd)))
    r = subprocess.run(cmd, shell=False)
    if r.returncode != 0:
        raise RuntimeError(f"명령 실패(returncode={r.returncode})")

def main():
    import argparse
    
    parser = argparse.ArgumentParser(description="COLMAP 3D 복원 (구버전 호환)")
    parser.add_argument("--images-dir", type=Path, required=True)
    parser.add_argument("--project-dir", type=Path, required=True)
    parser.add_argument("--matcher", choices=["sequential", "exhaustive"], 
                        default="sequential")
    parser.add_argument("--quality", choices=["low", "medium", "high", "extreme"],
                        default="high")
    
    args = parser.parse_args()
    
    images_dir = args.images_dir
    project_dir = args.project_dir
    
    db_path = project_dir / "project.db"
    sparse_dir = project_dir / "sparse"
    
    project_dir.mkdir(parents=True, exist_ok=True)
    sparse_dir.mkdir(parents=True, exist_ok=True)
    
    if not images_dir.exists():
        raise FileNotFoundError(f"이미지 폴더 없음: {images_dir}")
    
    if not any(images_dir.glob("*.*")):
        raise FileNotFoundError(f"이미지 폴더 비어있음: {images_dir}")
    
    print("=" * 60)
    print("COLMAP 3D 복원 (구버전 호환 - 최소 옵션)")
    print("=" * 60)
    print(f"이미지: {images_dir}")
    print(f"출력: {project_dir}")
    print(f"품질: {args.quality}")
    print(f"매칭: {args.matcher}")
    print("=" * 60)
    
    # 품질 설정 (구버전 지원 옵션만)
    quality_params = {
        "low": {"max_image_size": 1600, "max_num_features": 4096},
        "medium": {"max_image_size": 2400, "max_num_features": 8192},
        "high": {"max_image_size": 3200, "max_num_features": 16384},
        "extreme": {"max_image_size": 4800, "max_num_features": 32768},
    }
    
    quality = quality_params[args.quality]
    
    # 1) 특징점 추출
    print("\n[1/4] 특징점 추출...")
    feat_cmd = [
        COLMAP_EXE, "feature_extractor",
        "--database_path", str(db_path),
        "--image_path", str(images_dir),
        "--ImageReader.camera_model", "SIMPLE_RADIAL",
        "--ImageReader.single_camera", "1",
        # 구버전 지원 옵션만 사용
        "--SiftExtraction.max_image_size", str(quality["max_image_size"]),
        "--SiftExtraction.max_num_features", str(quality["max_num_features"]),
    ]
    run(feat_cmd)
    
    # 2) 특징점 매칭 (구버전 기본 옵션만)
    print(f"\n[2/4] 특징점 매칭 ({args.matcher})...")
    
    if args.matcher == "sequential":
        match_cmd = [
            COLMAP_EXE, "sequential_matcher",
            "--database_path", str(db_path),
            "--SequentialMatching.overlap", "10",  # 더 많은 이웃
        ]
    else:
        match_cmd = [
            COLMAP_EXE, "exhaustive_matcher",
            "--database_path", str(db_path),
        ]
    
    run(match_cmd)
    
    # 3) Sparse 재구성 (구버전 지원 옵션만)
    print("\n[3/4] Sparse 재구성...")
    run([
        COLMAP_EXE, "mapper",
        "--database_path", str(db_path),
        "--image_path", str(images_dir),
        "--output_path", str(sparse_dir),
        # 카메라 파라미터 고정 (기본 옵션)
        "--Mapper.ba_refine_focal_length", "0",
        "--Mapper.ba_refine_principal_point", "0",
        "--Mapper.ba_refine_extra_params", "0",
    ])
    
    # 모델 폴더 찾기
    model_dir = sparse_dir / "0"
    if not model_dir.exists():
        candidates = sorted([p for p in sparse_dir.iterdir() if p.is_dir()])
        if not candidates:
            raise FileNotFoundError("Sparse 재구성 실패")
        model_dir = candidates[0]
    
    # 4) PLY 변환
    print("\n[4/4] PLY 변환...")
    ply_sparse = model_dir / "points3D.ply"
    run([
        COLMAP_EXE, "model_converter",
        "--input_path", str(model_dir),
        "--output_path", str(ply_sparse),
        "--output_type", "PLY",
    ])
    
    print("\n" + "=" * 60)
    print("✅ COLMAP 완료!")
    print("=" * 60)
    print(f"데이터베이스: {db_path}")
    print(f"모델 폴더: {model_dir}")
    print(f"PLY 파일: {ply_sparse}")
    print("\n💡 결과 확인:")
    print(f"  meshlab {ply_sparse}")
    print(f"  cloudcompare {ply_sparse}")
    print("=" * 60)


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n❌ 오류: {e}", file=sys.stderr)
        sys.exit(1)