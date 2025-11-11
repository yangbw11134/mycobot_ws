#!/usr/bin/env python3
"""
0_run_video_with_realtime_masking.py
아두이노 회전 + 비디오 촬영 + 실시간 SAM2 마스킹

개선사항:
1. 촬영과 동시에 SAM2 마스킹
2. 첫 프레임에서 물체 선택
3. 완료되면 바로 마스킹된 프레임들 저장

사용법:
python3 0_run_video_with_realtime_masking.py \
    --sam2-checkpoint /path/to/sam2_hiera_large.pt \
    --sam2-config sam2_hiera_l \
    --output-dir scan_results \
    --rotation-timeout 120
"""

import cv2
import numpy as np
import os, time, serial, glob, argparse
from datetime import datetime
from pathlib import Path
import threading
import queue


class RealtimeMaskingCamera:
    """실시간 마스킹 카메라"""
    
    def __init__(self, camera_id=2, output_dir="scan_results", 
                 rotation_timeout=120, sam2_checkpoint=None, sam2_config=None):
        self.camera_id = camera_id
        self.output_dir = output_dir
        self.rotation_timeout = rotation_timeout
        
        os.makedirs(output_dir, exist_ok=True)
        
        # 아두이노 연결
        self.arduino = self.connect_arduino()
        
        # 카메라 초기화
        self.cap = self.init_camera()
        
        # SAM2 설정
        self.sam2_checkpoint = sam2_checkpoint
        self.sam2_config = sam2_config
        self.sam2_predictor = None
        self.sam2_ready = False
        
        # 프레임 큐 (촬영 → SAM2)
        self.frame_queue = queue.Queue(maxsize=10)
        self.masked_frames = []  # (frame_idx, masked_frame, mask)
        
        # 프롬프트 데이터
        self.prompt_data = None
        
    def connect_arduino(self):
        """아두이노 연결"""
        try:
            ports = glob.glob('/dev/ttyUSB*') + glob.glob('/dev/ttyACM*')
            if not ports:
                print("❌ 아두이노 포트를 찾을 수 없습니다.")
                return None
            
            port = ports[0]
            ser = serial.Serial(port, 9600, timeout=1)
            time.sleep(2)
            print(f"✅ 아두이노 연결됨: {port}")
            return ser
        except Exception as e:
            print(f"❌ 아두이노 연결 실패: {e}")
            return None
    
    def init_camera(self):
        """카메라 초기화"""
        camera_ids = [0, 2, 1, 3]
        cap = None
        
        print("🔍 외장 카메라 검색 중...")
        
        for cam_id in camera_ids:
            print(f"  카메라 {cam_id} 확인 중...", end=" ")
            test_cap = cv2.VideoCapture(cam_id)
            
            if not test_cap.isOpened():
                print("❌")
                test_cap.release()
                continue
            
            test_cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            test_cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            
            width = int(test_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            height = int(test_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            if width >= 1280 and height >= 720:
                print(f"✅ ({width}x{height})")
                cap = test_cap
                self.camera_id = cam_id
                break
            else:
                print(f"⚠️ 해상도 부족")
                test_cap.release()
        
        if cap is None or not cap.isOpened():
            print(f"❌ 외장 카메라를 찾을 수 없습니다.")
            return None
        
        cap.set(cv2.CAP_PROP_FPS, 30)
        
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        
        print(f"✅ 카메라 초기화: {self.camera_id}")
        print(f"   해상도: {width}x{height}, FPS: {fps}")
        
        return cap
    
    def init_sam2(self):
        """SAM2 초기화"""
        if self.sam2_checkpoint is None or self.sam2_config is None:
            print("⚠️ SAM2 비활성화 (체크포인트 미지정)")
            return False
        
        try:
            from sam2.build_sam import build_sam2_video_predictor
            print(f"\n🎯 SAM2 로딩 중...")
            
            config_name = Path(self.sam2_config).stem
            self.sam2_predictor = build_sam2_video_predictor(
                config_name, self.sam2_checkpoint, device="cuda"
            )
            
            print(f"✅ SAM2 로딩 완료")
            self.sam2_ready = True
            return True
            
        except Exception as e:
            print(f"❌ SAM2 로딩 실패: {e}")
            return False
    
    def get_first_frame_prompt(self, first_frame):
        """첫 프레임에서 물체 선택"""
        print("\n📦 첫 프레임에서 물체를 선택하세요")
        print("   마우스 드래그로 박스 그리기")
        print("   q 또는 ESC: 완료")
        
        img = first_frame.copy()
        h, w = img.shape[:2]
        
        drawing = False
        start_point = None
        end_point = None
        current_img = img.copy()
        
        def mouse_callback(event, x, y, flags, param):
            nonlocal drawing, start_point, end_point, current_img
            
            if event == cv2.EVENT_LBUTTONDOWN:
                drawing = True
                start_point = (x, y)
            elif event == cv2.EVENT_MOUSEMOVE and drawing:
                end_point = (x, y)
                current_img = img.copy()
                cv2.rectangle(current_img, start_point, end_point, (0, 255, 0), 2)
                cv2.imshow("Select Object", current_img)
            elif event == cv2.EVENT_LBUTTONUP:
                drawing = False
                end_point = (x, y)
                cv2.rectangle(current_img, start_point, end_point, (0, 255, 0), 2)
                cv2.imshow("Select Object", current_img)
        
        cv2.namedWindow("Select Object")
        cv2.setMouseCallback("Select Object", mouse_callback)
        cv2.imshow("Select Object", current_img)
        
        while True:
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q') or key == 27:
                break
        
        cv2.destroyAllWindows()
        
        if start_point is None or end_point is None:
            print("❌ 박스가 선택되지 않았습니다. 중앙 박스 사용")
            cx, cy = w // 2, h // 2
            box_w, box_h = int(w * 0.4), int(h * 0.4)
            start_point = (cx - box_w//2, cy - box_h//2)
            end_point = (cx + box_w//2, cy + box_h//2)
        
        x1 = min(start_point[0], end_point[0])
        y1 = min(start_point[1], end_point[1])
        x2 = max(start_point[0], end_point[0])
        y2 = max(start_point[1], end_point[1])
        
        self.prompt_data = {
            "type": "box",
            "box": np.array([x1, y1, x2, y2], dtype=np.float32)
        }
        
        print(f"✅ 선택 완료: ({x1}, {y1}) - ({x2}, {y2})")
        return self.prompt_data
    
    def start_continuous_rotation(self):
        """아두이노 회전 시작"""
        if not self.arduino:
            return False
        
        try:
            self.arduino.reset_input_buffer()
            self.arduino.reset_output_buffer()
            time.sleep(0.2)
            
            gear_ratio = 6.0
            steps_per_rev = 2048.0
            total_steps = int(360.0 * gear_ratio * steps_per_rev / 360.0)
            
            if total_steps >= 0:
                command = f"0{total_steps:05d}"
            else:
                command = f"1{abs(total_steps):05d}"
            
            full_command = command + "\n"
            print(f"✅ 회전 명령 전송: {command}")
            self.arduino.write(full_command.encode())
            self.arduino.flush()
            
            return True
        except Exception as e:
            print(f"❌ 회전 명령 실패: {e}")
            return False
    
    def wait_for_rotation_complete(self):
        """회전 완료 대기"""
        if not self.arduino:
            return False
        
        start_time = time.time()
        print("🔄 360도 회전 완료 대기 중...")
        
        while True:
            if time.time() - start_time > self.rotation_timeout:
                print(f"⏱️ 타임아웃 ({self.rotation_timeout}초)")
                return False
            
            if self.arduino.in_waiting > 0:
                try:
                    line = self.arduino.readline().decode('utf-8').strip()
                    if line and any(kw in line.upper() for kw in ["DONE", "COMPLETE", "FINISH", "END"]):
                        print("✅ 회전 완료!")
                        return True
                except:
                    pass
            
            time.sleep(0.1)
    
    def capture_with_realtime_masking(self):
        """촬영 + 실시간 마스킹"""
        if not self.cap:
            print("❌ 카메라가 초기화되지 않았습니다.")
            return False
        
        print(f"\n{'='*60}")
        print("🎥 실시간 마스킹 촬영 시작")
        print(f"{'='*60}")
        
        # 1. 첫 프레임 캡처
        ret, first_frame = self.cap.read()
        if not ret:
            print("❌ 첫 프레임을 읽을 수 없습니다.")
            return False
        
        # 2. SAM2 초기화
        sam2_enabled = self.init_sam2()
        
        # 3. 물체 선택 (SAM2 있을 때만)
        if sam2_enabled:
            self.get_first_frame_prompt(first_frame)
            print("\n3초 후 촬영 시작...")
            for i in range(3, 0, -1):
                print(f"   {i}...")
                time.sleep(1)
        else:
            print("\n⚠️ SAM2 없이 일반 촬영만 진행")
            print("3초 후 시작...")
            time.sleep(3)
        
        # 4. 회전 시작
        rotation_started = False
        if self.arduino:
            rotation_started = self.start_continuous_rotation()
            if rotation_started:
                time.sleep(0.3)
        
        # 5. 촬영 시작
        print("\n🎬 촬영 및 마스킹 시작!")
        
        start_time = time.time()
        frame_count = 0
        raw_frames = []  # 원본 프레임 저장
        
        # 회전 완료 체크 스레드
        rotation_complete = threading.Event()
        
        def check_rotation():
            if self.arduino and rotation_started:
                if self.wait_for_rotation_complete():
                    rotation_complete.set()
        
        rotation_thread = threading.Thread(target=check_rotation, daemon=True)
        rotation_thread.start()
        
        # 프레임 캡처 루프
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break
            
            raw_frames.append(frame.copy())
            frame_count += 1
            
            # 진행률 표시
            if frame_count % 30 == 0:  # 1초마다
                elapsed = time.time() - start_time
                print(f"⏱️ {elapsed:.1f}초 - 프레임: {frame_count}")
            
            # 회전 완료 확인
            if rotation_started and rotation_complete.is_set():
                print(f"\n✅ 촬영 완료 - 총 {frame_count}프레임")
                break
            
            # 타임아웃
            if time.time() - start_time > self.rotation_timeout:
                print(f"\n⏱️ 타임아웃 - 총 {frame_count}프레임")
                break
        
        self.cap.release()
        
        # 6. 후처리: SAM2 마스킹 (촬영 완료 후)
        if sam2_enabled and len(raw_frames) > 0:
            print(f"\n🎯 SAM2 마스킹 시작 ({len(raw_frames)}개 프레임)...")
            masked_frames = self.process_frames_with_sam2(raw_frames)
            
            # 7. 결과 저장
            self.save_results(masked_frames, raw_frames)
        else:
            # SAM2 없으면 원본만 저장
            print(f"\n💾 원본 프레임 저장 중...")
            self.save_raw_frames(raw_frames)
        
        if self.arduino:
            self.arduino.close()
        
        return True
    
    def process_frames_with_sam2(self, frames):
        """프레임들을 SAM2로 마스킹"""
        # 임시 디렉토리에 프레임 저장
        temp_dir = Path(self.output_dir) / "_temp"
        temp_dir.mkdir(parents=True, exist_ok=True)
        
        for i, frame in enumerate(frames):
            cv2.imwrite(str(temp_dir / f"{i:05d}.jpg"), frame)
        
        # SAM2 inference
        inference_state = self.sam2_predictor.init_state(video_path=str(temp_dir))
        
        self.sam2_predictor.add_new_points_or_box(
            inference_state=inference_state,
            frame_idx=0,
            obj_id=1,
            box=self.prompt_data["box"],
        )
        
        print("   SAM2 propagation 진행 중...")
        video_segments = {}
        for out_frame_idx, out_obj_ids, out_mask_logits in self.sam2_predictor.propagate_in_video(inference_state):
            video_segments[out_frame_idx] = {
                out_obj_id: (out_mask_logits[i] > 0.0).cpu().numpy()
                for i, out_obj_id in enumerate(out_obj_ids)
            }
        
        masks = {idx: seg[1][0] for idx, seg in video_segments.items() if 1 in seg}
        
        # 마스크 적용
        masked_frames = []
        for i, frame in enumerate(frames):
            if i in masks:
                mask = masks[i]
                masked = np.full_like(frame, (255, 255, 255), dtype=np.uint8)
                masked[mask] = frame[mask]
                masked_frames.append((i, masked, mask))
            else:
                masked_frames.append((i, frame, None))
        
        # 임시 파일 삭제
        import shutil
        shutil.rmtree(temp_dir)
        
        print(f"✅ 마스킹 완료: {len(masked_frames)}개")
        
        return masked_frames
    
    def save_results(self, masked_frames, raw_frames):
        """결과 저장"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        raw_dir = Path(self.output_dir) / f"raw_{timestamp}"
        masked_dir = Path(self.output_dir) / f"masked_{timestamp}"
        masks_dir = Path(self.output_dir) / f"masks_{timestamp}"
        
        for d in [raw_dir, masked_dir, masks_dir]:
            d.mkdir(parents=True, exist_ok=True)
        
        print(f"\n💾 결과 저장 중...")
        
        for i, (idx, masked, mask) in enumerate(masked_frames):
            # 원본
            cv2.imwrite(str(raw_dir / f"frame_{i:03d}.png"), raw_frames[idx])
            
            # 마스킹
            cv2.imwrite(str(masked_dir / f"frame_{i:03d}.png"), masked)
            
            # 마스크
            if mask is not None:
                mask_u8 = (mask.astype(np.uint8) * 255)
                cv2.imwrite(str(masks_dir / f"frame_{i:03d}_mask.png"), mask_u8)
        
        print(f"✅ 저장 완료!")
        print(f"   원본: {raw_dir.resolve()}")
        print(f"   마스킹: {masked_dir.resolve()}")
        print(f"   마스크: {masks_dir.resolve()}")
    
    def save_raw_frames(self, frames):
        """원본 프레임만 저장"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        raw_dir = Path(self.output_dir) / f"raw_{timestamp}"
        raw_dir.mkdir(parents=True, exist_ok=True)
        
        for i, frame in enumerate(frames):
            cv2.imwrite(str(raw_dir / f"frame_{i:03d}.png"), frame)
        
        print(f"✅ 원본 프레임 저장: {raw_dir.resolve()}")


def main():
    parser = argparse.ArgumentParser(description="실시간 마스킹 비디오 촬영")
    parser.add_argument("--sam2-checkpoint", type=str, default=None,
                        help="SAM2 체크포인트 (.pt)")
    parser.add_argument("--sam2-config", type=str, default=None,
                        help="SAM2 설정 파일")
    parser.add_argument("--output-dir", type=str, default="scan_results",
                        help="출력 디렉토리")
    parser.add_argument("--rotation-timeout", type=int, default=120,
                        help="회전 최대 대기 시간 (초)")
    parser.add_argument("--camera-id", type=int, default=2,
                        help="카메라 ID")
    
    args = parser.parse_args()
    
    print("=" * 60)
    print("실시간 마스킹 비디오 촬영")
    print("=" * 60)
    
    if args.sam2_checkpoint:
        print(f"✅ SAM2 활성화")
        print(f"   체크포인트: {args.sam2_checkpoint}")
        print(f"   설정: {args.sam2_config}")
    else:
        print(f"⚠️ SAM2 비활성화 (일반 촬영)")
    
    camera = RealtimeMaskingCamera(
        camera_id=args.camera_id,
        output_dir=args.output_dir,
        rotation_timeout=args.rotation_timeout,
        sam2_checkpoint=args.sam2_checkpoint,
        sam2_config=args.sam2_config
    )
    
    success = camera.capture_with_realtime_masking()
    
    if success:
        print("\n🎉 촬영 및 마스킹 완료!")
    else:
        print("\n❌ 촬영 실패")


if __name__ == "__main__":
    main()