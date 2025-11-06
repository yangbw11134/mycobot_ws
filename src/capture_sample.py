import os, cv2

# GStreamer 비활성화 (이게 핵심)
os.environ["OPENCV_VIDEOIO_PRIORITY_GSTREAMER"] = "0"

# V4L2 백엔드 강제
cam = cv2.VideoCapture(0, cv2.CAP_V4L2)
if not cam.isOpened():
    print("❌ 카메라 열기 실패")
    exit()

# --- 자세 1 촬영 ---
ret, frame = cam.read()
if not ret:
    print("⚠️ 프레임 읽기 실패 (자세 1)")
    cam.release()
    exit()

cv2.imshow("Camera Preview", frame)
cv2.imwrite("capture_sample.jpg", frame)
print("✅ capture_sample.jpg 저장 완료")

cv2.waitKey(1000)  # 1초 동안 창 띄우기
cam.release()
cv2.destroyAllWindows()
