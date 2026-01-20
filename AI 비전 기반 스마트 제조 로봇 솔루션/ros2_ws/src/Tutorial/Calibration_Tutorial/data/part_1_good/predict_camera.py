from ultralytics import YOLO
import cv2
import os

def predict_camera():
    # ==========================================
    # [설정] 모델 경로 및 카메라 설정
    # ==========================================
    # 1. 학습된 모델 경로 (자동으로 찾기)
    current_dir = os.getcwd()
    model_path = os.path.join(current_dir, "runs/detect/train_part_1_good/weights/best.pt")
    
    # 혹시 경로가 꼬일 경우를 대비해 절대 경로가 맞는지 확인
    print(f"모델 불러오는 중: {model_path}")

    # 2. 모델 로드
    try:
        model = YOLO(model_path)
    except Exception as e:
        print(f"[오류] 모델을 찾을 수 없습니다! 경로를 확인하세요.\n{e}")
        return

    # 3. 카메라 설정
    # 노트북 내장 캠: 0 / USB 캠: 2, 4, 6 등 (아까 6번 쓰셨죠?)
    camera_index = 6  
    cap = cv2.VideoCapture(camera_index)

    # 카메라 해상도 설정 (속도 향상을 위해 640x480 권장)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # ==========================================

    if not cap.isOpened():
        print(f"카메라({camera_index}번)를 열 수 없습니다. 인덱스를 확인해주세요.")
        return

    print("=== 실시간 감지 시작 (종료: 'q' 키) ===")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break

        # ------------------------------------------------------
        # 1. YOLO 예측 실행
        # conf=0.5: 확신이 50% 이상일 때만 박스 그리기 (조절 가능)
        # ------------------------------------------------------
        results = model.predict(source=frame, conf=0.99, verbose=False)
        
        # 결과 처리
        result = results[0]
        boxes = result.boxes

        for box in boxes:
            # 좌표 가져오기 (정수형 변환)
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            
            # 클래스 이름 및 신뢰도
            cls_id = int(box.cls[0])
            conf = float(box.conf[0])
            class_name = model.names[cls_id]

            # --------------------------------------------------
            # [핵심] Pick & Place를 위한 중심점(Center) 계산
            # --------------------------------------------------
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            # --------------------------------------------------
            # [시각화] 화면에 그리기
            # --------------------------------------------------
            # 1. 박스 그리기 (초록색)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # 2. 중심점 찍기 (빨간 점)
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)

            # 3. 텍스트 정보 표시 (이름 + 확률 + 중심좌표)
            label_text = f"{class_name} ({conf:.2f})"
            coord_text = f"Center: ({cx}, {cy})"
            
            # 박스 위에 이름
            cv2.putText(frame, label_text, (x1, y1 - 25), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            # 박스 위에 좌표 (로봇에게 보낼 값)
            cv2.putText(frame, coord_text, (x1, y1 - 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

            # 터미널에도 좌표 출력 (디버깅용)
            print(f"감지됨: {class_name} | 좌표: ({cx}, {cy})")

        # 화면 출력
        cv2.imshow("YOLOv8 Real-time Detection", frame)

        # 'q' 키 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 종료 처리
    cap.release()
    cv2.destroyAllWindows()
    print("프로그램 종료")

if __name__ == '__main__':
    predict_camera()