import cv2
from ultralytics import YOLO

# 1) 학습한 가중치 경로 (필요에 맞게 수정)
# 예: /content/drive/... 에서 로컬로 복사했다면 아래처럼
model_path = '/home/rokey/rokey_ws/src/rokey_pjt/rokey_pjt/(yolov8n_640,100)best.pt'  # 네 best.pt 경로로 수정
model = YOLO(model_path)

# 2) 사용할 카메라 번호 선택 (0 또는 2 중 하나)
cam_index = 0
cap = cv2.VideoCapture(cam_index)

if not cap.isOpened():
    print(f'카메라 {cam_index} 열기 실패')
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # 3) YOLO 추론 (conf 임계값 설정)
    results = model.predict(
        source=frame,
        conf=0.5,      # 최소 confidence threshold (0.0 ~ 1.0 사이로 조절)
        verbose=False
    )

    # 4) 박스가 그려진 프레임 얻기
    annotated = results[0].plot()

    # 5) 박스별 conf 터미널 출력
    boxes = results[0].boxes
    if boxes is not None:
        for box in boxes:
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            print(f'class={cls_id}, conf={conf:.3f}')

    # 6) 화면에 보여주기
    # cv2.imshow(f'YOLO cam {cam_index}', annotated)
    cv2.imshow("Camera Raw View", frame)
    # q 키 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
