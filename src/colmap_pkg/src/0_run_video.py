#!/usr/bin/env python3
"""
아두이노로 360도 회전하면서 비디오 촬영하는 스크립트
- 로지텍 C922 카메라로 비디오 촬영
- 아두이노로 물체를 360도 회전
- 비디오 파일로 저장
"""

import cv2
import numpy as np
import os
import time
import serial
import glob
from datetime import datetime

class RotateCamera:
    """회전하면서 비디오 촬영하는 클래스"""
    
    def __init__(self, camera_id=4, output_dir="video_scans", rotation_timeout=120):
        self.camera_id = camera_id
        self.output_dir = output_dir
        self.rotation_timeout = rotation_timeout  # 회전 완료 최대 대기 시간 (초)
        
        # 출력 디렉토리 생성
        os.makedirs(output_dir, exist_ok=True)
        
        # 아두이노 연결
        self.arduino = self.connect_arduino()
        
        # 카메라 초기화
        self.cap = self.init_camera()
        
        # 비디오 라이터 설정
        self.video_writer = None
        
    def connect_arduino(self):
        """아두이노 연결"""
        try:
            # 포트 자동 감지
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
        """로지텍 C922 카메라 초기화"""
        cap = cv2.VideoCapture(self.camera_id)
        if not cap.isOpened():
            print(f"❌ 카메라 {self.camera_id}를 열 수 없습니다.")
            return None
        
        # 로지텍 C922 최적 설정
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        
        # 카메라 정보 확인
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = cap.get(cv2.CAP_PROP_FPS)
        
        print(f"✅ 카메라 초기화됨: {self.camera_id}")
        print(f"   해상도: {width}x{height}")
        print(f"   FPS: {fps}")
        
        return cap
    
    def setup_video_writer(self, width, height, fps):
        """비디오 라이터 설정"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"rotation_scan_{timestamp}.mp4"
        filepath = os.path.join(self.output_dir, filename)
        
        # MP4 코덱으로 설정
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(filepath, fourcc, fps, (width, height))
        
        if not video_writer.isOpened():
            print("❌ 비디오 라이터 초기화 실패")
            return None
        
        print(f"✅ 비디오 파일 생성: {filepath}")
        return video_writer, filepath
    
    def start_continuous_rotation(self):
        """연속 회전 시작"""
        if not self.arduino:
            print("아두이노가 연결되지 않았습니다.")
            return False
        
        try:
            # 아두이노 버퍼 비우기
            self.arduino.reset_input_buffer()
            
            # 연속 회전 명령 (360도)
            gear_ratio = 6.0
            steps_per_rev = 2048.0
            total_steps = int(360.0 * gear_ratio * steps_per_rev / 360.0)
            
            # 6자리 숫자: [부호][스텝수5자리]
            if total_steps >= 0:
                command = f"0{total_steps:05d}"
            else:
                command = f"1{abs(total_steps):05d}"
            
            self.arduino.write(command.encode())
            self.arduino.write(b'\n')
            print(f"✅ 연속 회전 명령 전송: {total_steps} 스텝 (360도)")
            
            return True
        except Exception as e:
            print(f"❌ 회전 명령 실패: {e}")
            return False
    
    def wait_for_rotation_complete(self, timeout=None):
        """아두이노로부터 회전 완료 신호 대기"""
        if not self.arduino:
            return False
        
        if timeout is None:
            timeout = self.rotation_timeout
        
        start_time = time.time()
        print("🔄 360도 회전 완료를 기다리는 중...")
        
        while True:
            if time.time() - start_time > timeout:
                print(f"⏱️ 타임아웃 ({timeout}초) - 회전이 완료되지 않았습니다.")
                return False
            
            # 아두이노로부터 메시지 확인
            if self.arduino.in_waiting > 0:
                try:
                    line = self.arduino.readline().decode('utf-8').strip()
                    if line:
                        print(f"📨 아두이노 메시지: {line}")
                        # "DONE", "COMPLETE", "FINISH" 등의 키워드 확인
                        if any(keyword in line.upper() for keyword in ["DONE", "COMPLETE", "FINISH", "END"]):
                            print("✅ 회전 완료 신호를 받았습니다!")
                            return True
                except Exception as e:
                    # 디코딩 오류는 무시하고 계속 진행
                    pass
            
            # 짧은 대기 (CPU 부하 감소)
            time.sleep(0.1)
    
    def capture_rotation_video(self):
        """회전하면서 비디오 촬영 (360도 회전 완료까지)"""
        if not self.cap:
            print("카메라가 초기화되지 않았습니다.")
            return False
        
        print(f"\n🎥 아두이노 360도 회전 비디오 촬영을 시작합니다.")
        print("3초 후 자동으로 촬영을 시작합니다...")
        
        # 카메라 정보 가져오기
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self.cap.get(cv2.CAP_PROP_FPS)
        
        # 비디오 라이터 설정
        video_writer, video_path = self.setup_video_writer(width, height, fps)
        if not video_writer:
            return False
        
        # 3초 카운트다운
        for i in range(3, 0, -1):
            print(f"촬영 시작까지 {i}초...")
            time.sleep(1)
        
        print("🎬 촬영 시작!")
        
        # 아두이노로 연속 회전 시작
        rotation_started = False
        if self.arduino:
            rotation_started = self.start_continuous_rotation()
        
        start_time = time.time()
        frame_count = 0
        rotation_complete = False
        
        # 별도 스레드로 회전 완료 감지
        import threading
        rotation_event = threading.Event()
        
        def check_rotation_complete():
            nonlocal rotation_complete
            if self.arduino and rotation_started:
                if self.wait_for_rotation_complete():
                    rotation_complete = True
                    rotation_event.set()
            else:
                # 아두이노가 없으면 타임아웃 후 종료하지 않음 (무한 촬영)
                pass
        
        # 회전 완료 체크 시작
        rotation_thread = threading.Thread(target=check_rotation_complete, daemon=True)
        rotation_thread.start()
        
        while True:
            ret, frame = self.cap.read()
            if not ret:
                print("프레임을 읽을 수 없습니다.")
                break
            
            # 비디오에 프레임 저장
            video_writer.write(frame)
            frame_count += 1
            
            current_time = time.time()
            elapsed_time = current_time - start_time
            
            # 진행률 표시 (매 5초마다)
            if frame_count % 150 == 0:  # 5초마다 (30fps * 5초 = 150프레임)
                print(f"⏱️ 경과 시간: {elapsed_time:.1f}초 - 프레임: {frame_count}")
            
            # 회전 완료 확인 (아두이노가 있는 경우)
            if rotation_started and rotation_complete:
                print(f"\n✅ 360도 회전 완료! 촬영 종료 - 총 {frame_count}프레임 저장됨")
                break
            
            # 아두이노가 없거나 타임아웃된 경우에도 계속 촬영
            # 사용자가 Ctrl+C로 중단할 수 있음
            if not rotation_started and elapsed_time > self.rotation_timeout:
                print(f"\n⚠️ 타임아웃 ({self.rotation_timeout}초) - 촬영을 계속합니다...")
        
        # 정리
        self.cap.release()
        video_writer.release()
        
        # 아두이노 연결 해제
        if self.arduino:
            self.arduino.close()
        
        if frame_count > 0:
            actual_duration = time.time() - start_time
            print(f"\n✅ 비디오 촬영 완료!")
            print(f"비디오 파일: {video_path}")
            print(f"총 프레임 수: {frame_count}")
            print(f"총 촬영 시간: {actual_duration:.2f}초")
            print(f"실제 FPS: {frame_count / actual_duration:.2f}")
            return video_path
        else:
            print("❌ 비디오 촬영에 실패했습니다.")
            return None

def main():
    """메인 함수"""
    import sys
    
    rotation_timeout = 120  # 기본값: 120초 (최대 대기 시간)
    if len(sys.argv) > 1:
        rotation_timeout = int(sys.argv[1])
    
    print(f"아두이노 360도 회전 비디오 촬영을 시작합니다.")
    print(f"회전 완료 대기 시간: 최대 {rotation_timeout}초")
    print("💡 아두이노가 물체를 자동으로 360도 회전시킵니다!")
    print("💡 회전이 완료되면 자동으로 촬영이 종료됩니다.")
    
    # 회전 카메라 생성 및 실행
    camera = RotateCamera(rotation_timeout=rotation_timeout)
    video_path = camera.capture_rotation_video()
    
    if video_path:
        print(f"\n🎉 비디오 촬영 완료!")
        print(f"다음 단계: 1_captrue_frame_image.py로 프레임 추출을 실행하세요.")
        print(f"비디오 파일: {video_path}")
    else:
        print("❌ 비디오 촬영에 실패했습니다.")

if __name__ == "__main__":
    main()