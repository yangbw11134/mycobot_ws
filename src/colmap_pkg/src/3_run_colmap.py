# run_colmap.py
import subprocess
import sys
from pathlib import Path

# ============ 사용자 설정 ============
# Linux에서는 시스템에 설치된 colmap을 사용합니다
COLMAP_EXE = "colmap"

# 프로젝트 폴더 경로를 지정해주세요 (명령줄 인수로도 받을 수 있습니다)
PROJECT_DIR = Path("colmap_project")

# 3D로 복원할 이미지가 담긴 폴더 경로를 지정해주세요 (명령줄 인수로도 받을 수 있습니다)
IMAGES_DIR  = Path("sam_out/masked_images")

# 매칭 방식: 이미지 순서가 연속적이면 'sequential', 아니면 'exhaustive'
MATCHER = "sequential"
# =====================================

def run(cmd):
    """주어진 명령어를 실행하고 결과를 확인하는 함수"""
    # 실행할 명령어를 보기 쉽게 출력합니다.
    print(">>>", " ".join(map(str, cmd)))
    # subprocess를 사용하여 명령어를 실행합니다.
    r = subprocess.run(cmd, shell=False)
    # 명령이 실패하면 오류를 발생시킵니다.
    if r.returncode != 0:
        raise RuntimeError(f"명령 실패(returncode={r.returncode}): {' '.join(cmd)}")

def main():
    """메인 실행 함수"""
    import argparse
    
    # 명령줄 인수 파싱
    parser = argparse.ArgumentParser(description="COLMAP으로 3D 복원 실행")
    parser.add_argument("--images-dir", type=Path, default=IMAGES_DIR, 
                        help="입력 이미지 폴더 경로")
    parser.add_argument("--project-dir", type=Path, default=PROJECT_DIR,
                        help="COLMAP 프로젝트 출력 폴더 경로")
    parser.add_argument("--matcher", choices=["sequential", "exhaustive"], 
                        default=MATCHER, help="매칭 방식")
    args = parser.parse_args()
    
    images_dir = args.images_dir
    project_dir = args.project_dir
    matcher = args.matcher
    
    db_path     = project_dir / "project.db"
    sparse_dir  = project_dir / "sparse"
    
    # 필요한 폴더들이 없으면 생성합니다.
    project_dir.mkdir(parents=True, exist_ok=True)
    sparse_dir.mkdir(parents=True, exist_ok=True)

    # 이미지 폴더가 존재하는지 확인합니다.
    if not images_dir.exists():
        raise FileNotFoundError(f"이미지 폴더가 존재하지 않습니다: {images_dir}")
    
    # 이미지 폴더가 비어있는지 확인합니다.
    if not any(images_dir.glob("*.*")):
        raise FileNotFoundError(f"이미지 폴더가 비었습니다: {images_dir}")

    # 1) 특징점 추출 (Feature Extraction)
    print("\n[1/4] 특징점 추출을 시작합니다...")
    run([
        COLMAP_EXE, "feature_extractor",
        "--database_path", str(db_path),
        "--image_path",    str(images_dir),
        "--ImageReader.camera_model", "SIMPLE_RADIAL",
        "--ImageReader.single_camera", "1",
    ])

    # 2) 특징점 매칭 (Matching)
    print("\n[2/4] 특징점 매칭을 시작합니다...")
    if matcher.lower() == "sequential":
        run([
            COLMAP_EXE, "sequential_matcher",
            "--database_path", str(db_path),
            "--SequentialMatching.overlap", "5",
        ])
    else:
        run([
            COLMAP_EXE, "exhaustive_matcher",
            "--database_path", str(db_path),
        ])

    # 3) 희소 복원 (Sparse Reconstruction, SfM)
    print("\n[3/4] 희소 복원을 시작합니다...")
    run([
        COLMAP_EXE, "mapper",
        "--database_path", str(db_path),
        "--image_path",    str(images_dir),
        "--output_path",   str(sparse_dir),
        "--Mapper.ba_refine_focal_length", "0",
        "--Mapper.ba_refine_principal_point", "0",
        "--Mapper.ba_refine_extra_params", "0",
    ])

    # 생성된 모델 폴더를 찾습니다. (보통 sparse/0)
    model_dir = sparse_dir / "0"
    if not model_dir.exists():
        candidates = sorted([p for p in sparse_dir.iterdir() if p.is_dir()])
        if not candidates:
            raise FileNotFoundError("희소 복원(Sparse) 결과가 생성되지 않았습니다.")
        model_dir = candidates[0]

    # 4) PLY 파일로 변환
    print("\n[4/4] 모델을 PLY 파일로 변환합니다...")
    ply_path = model_dir / "points3D.ply"
    run([
        COLMAP_EXE, "model_converter",
        "--input_path",  str(model_dir),
        "--output_path", str(ply_path),
        "--output_type", "PLY",
    ])

    print("\n✅ COLMAP 작업 완료")
    print(f"- 데이터베이스: {db_path}")
    print(f"- 모델 폴더:    {model_dir}")
    print(f"- 결과 PLY 파일: {ply_path}")


if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"\n[오류] {e}")
        sys.exit(1)