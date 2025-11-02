# 1_capture_frame_image.py
import argparse, cv2, numpy as np
from pathlib import Path

def sample_indices(n_frames: int, n_samples: int) -> np.ndarray:
    n_samples = min(n_samples, n_frames)
    return np.unique(np.linspace(0, n_frames - 1, n_samples).astype(int))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True, help="입력 비디오 경로")
    ap.add_argument("--out", type=Path, default=Path("frames_100"), help="프레임 저장 폴더")
    ap.add_argument("--num", type=int, default=100, help="추출 프레임 수")
    args = ap.parse_args()

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise SystemExit(f"Cannot open video: {args.video}")

    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if total <= 0:
        # 프레임수를 못 읽으면 한 바퀴 스캔: 그냥 순차 추출
        total = 300

    idxs = sample_indices(total, args.num)

    args.out.mkdir(parents=True, exist_ok=True)

    saved = []
    for k, idx in enumerate(idxs):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(idx))
        ok, frame = cap.read()
        if not ok: 
            continue
        fn = args.out / f"frame_{k:03d}.png"
        cv2.imwrite(str(fn), frame)
        saved.append(fn)

    cap.release()
    print(f"✅ saved {len(saved)} frames → {args.out.resolve()}")

    # COLMAP용 리스트(선택): 나중에 정렬 확인용
    (args.out / "imagelist.txt").write_text("\n".join(str(p.name) for p in saved))

if __name__ == "__main__":
    main()
