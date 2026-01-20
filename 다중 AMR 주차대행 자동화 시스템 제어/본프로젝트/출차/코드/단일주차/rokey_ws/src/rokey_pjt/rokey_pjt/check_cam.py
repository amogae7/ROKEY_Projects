import cv2

for idx in range(5):  # 0~4번까지만 테스트, 필요하면 범위 늘리기
    cap = cv2.VideoCapture(idx)
    if cap.isOpened():
        print(f'✅ Camera {idx} 열림!')
        ret, frame = cap.read()
        if ret:
            print(f'   해상도: {frame.shape[1]}x{frame.shape[0]}')
        cap.release()
    else:
        print(f'❌ Camera {idx} 안 열림')
