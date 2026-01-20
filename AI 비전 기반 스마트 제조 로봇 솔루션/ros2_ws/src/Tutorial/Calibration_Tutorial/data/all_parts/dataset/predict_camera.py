from ultralytics import YOLO
import cv2
import os
import random

def predict_camera():
    # ==========================================
    # [설정] 모델 경로
    # ==========================================
    current_dir = os.getcwd()
    # 경로가 맞는지 꼭 다시 확인하세요! (train_all_parts 폴더가 맞나요?)
    model_path = r"/home/rokey/ros2_ws/src/Tutorial/Calibration_Tutorial/data/all_parts/dataset/runs/detect/train_all_parts/weights/best.pt"
    
    print(f"모델 불러오는 중: {model_path}")

    try:
        model = YOLO(model_path)
        # [확인용] 모델이 아는 클래스 출력
        print(f"▶ 모델 클래스 정보: {model.names}") 
    except Exception as e:
        print(f"[오류] 모델 로드 실패: {e}")
        return

    # ==========================================
    # [설정] 색상표 만들기 (클래스별 고유 색상)
    # ==========================================
    # 랜덤하게 클래스 개수만큼 색상을 미리 만들어둡니다.
    random.seed(42)
    colors = []
    for _ in range(len(model.names)):
        b = random.randint(0, 255)
        g = random.randint(0, 255)
        r = random.randint(0, 255)
        colors.append((b, g, r))
    # ==========================================

    camera_index = 6
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print(f"카메라({camera_index}번)를 열 수 없습니다.")
        return

    print("=== 실시간 감지 시작 (종료: 'q') ===")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # conf를 0.4 정도로 살짝 낮춰보세요 (다양한 물체 감지 위해)
        results = model.predict(source=frame, conf=0.4, verbose=False)
        
        for box in results[0].boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            class_name = model.names[cls_id]

            # 중심점 계산
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            # [핵심] 클래스 ID에 맞는 색상 가져오기
            # 만약 클래스 ID가 색상표보다 많을 경우를 대비해 % 연산 사용
            box_color = colors[cls_id % len(colors)]

            # 1. 박스 그리기 (고유 색상)
            cv2.rectangle(frame, (x1, y1), (x2, y2), box_color, 2)
            
            # 2. 중심점 (빨간 점)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            # 3. 텍스트 표시
            label_text = f"{class_name} {conf:.2f}"
            coord_text = f"({cx}, {cy})"
            
            cv2.putText(frame, label_text, (x1, y1 - 20), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, box_color, 2)
            cv2.putText(frame, coord_text, (x1, y1 - 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

            print(f"감지: {class_name} (ID:{cls_id}) | 좌표: {cx}, {cy}")

        cv2.imshow("YOLOv8 Multi-Class Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    predict_camera()