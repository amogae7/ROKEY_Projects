from ultralytics import YOLO
import cv2
import os
import math

def predict_camera_obb():
    # ==========================================
    # [설정 1] 모델 경로 수정 (아까 학습시킨 경로로!)
    # ==========================================
    # 아까 train.py에서 project='rokey_obb_project', name='run_1' 이라고 했습니다.
    # 따라서 경로는 아래와 같아야 합니다. (폴더명 확인 필요)
    model_path = os.path.join(os.getcwd(), "rokey_obb_project/run_1/weights/best.pt")
    
    print(f"▶ 모델 불러오는 중: {model_path}")

    try:
        model = YOLO(model_path)
        print(f"▶ 모델 로드 성공! 클래스: {model.names}") 
    except Exception as e:
        print(f"🚨 [오류] 모델을 찾을 수 없습니다: {e}")
        print("경로를 다시 확인해주세요.")
        return

    # ==========================================
    # [설정 2] 카메라 설정
    # ==========================================
    camera_index = 4  # ⚠️ 노트북 웹캠은 보통 0번입니다. (안되면 6번이나 2번 등으로 변경)
    cap = cv2.VideoCapture(camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print(f"🚨 카메라({camera_index}번)를 열 수 없습니다.")
        return

    print("=== 🎥 OBB 실시간 감지 시작 (종료: 'q') ===")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # ==========================================
        # 🔍 OBB 예측 실행
        # ==========================================
        results = model.predict(source=frame, conf=0.8, verbose=False)
        
        # 결과에서 OBB 정보 가져오기
        result = results[0]
        
        # 1. 화면에 그리기 (Ultralytics 내장 함수가 제일 깔끔하게 그려줍니다)
        # 회전된 박스와 라벨을 자동으로 그려줍니다.
        annotated_frame = result.plot() 

        # 2. 로봇 좌표용 데이터 추출 (콘솔 출력용)
        # OBB 결과가 있을 때만 실행
        if result.obb is not None:
            for obb in result.obb:
                # OBB 데이터 추출: 중심x, 중심y, 너비, 높이, 각도(rad)
                # xywhr 형식을 사용합니다.
                c_x, c_y, w, h, rot_rad = obb.xywhr[0]
                
                cls_id = int(obb.cls[0])
                class_name = model.names[cls_id]
                
                # 라디안 -> 도(Degree) 변환 (로봇이 이해하기 쉽게)
                rot_deg = math.degrees(rot_rad)

                # 콘솔에 출력 (Pick & Place 할 때 이 좌표와 각도를 로봇에게 보내면 됩니다)
                print(f"📦 물체: {class_name} | 좌표: ({int(c_x)}, {int(c_y)}) | 각도: {rot_deg:.2f}°")

        # 화면 출력
        cv2.imshow("YOLOv11m-OBB Real-time", annotated_frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    predict_camera_obb()
