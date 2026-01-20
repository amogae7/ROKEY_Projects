from ultralytics import YOLO
import os

def train_yolo():
    # ==========================================
    # [설정] 모델 및 데이터 경로
    # ==========================================
    # 1. 모델 선택
    # - yolov8s.pt: 성능과 속도 균형이 좋아 추천 (n보다 정확함)
    model_name = 'yolov8s.pt' 

    # 2. data.yaml 경로 (현재 폴더 기준)
    # [중요] 이 yaml 파일 안에 'nc: 6' 이라고 적혀 있어야 6개를 학습합니다!
    yaml_path = r"/home/rokey/ros2_ws/src/Tutorial/Calibration_Tutorial/data/all_parts/dataset/data.yaml"
    # ==========================================

    print(f"모델 로드 중: {model_name}")
    model = YOLO(model_name)

    print(f"학습 시작! 데이터 경로: {yaml_path}")
    
    # 3. 학습 실행 (Train)
    results = model.train(
        data=yaml_path,      # 데이터 설정 파일
        epochs=100,          # 100번 반복
        imgsz=640,           # 이미지 크기
        batch=16,            # 배치 사이즈
        
        # [수정된 부분] 결과 저장 폴더 이름을 명확하게 변경!
        # 기존: 'train_part_1_good' -> 변경: 'train_all_parts'
        project='runs/detect',     
        name='train_all_parts',    
        exist_ok=True,             # 덮어쓰기 허용
        
        # [옵션] 조기 종료 (성능 정체 시 30 epoch 후 중단)
        patience=30,  
    )

    print("\n" + "="*30)
    print("학습 완료! 저장된 모델을 확인하세요.")
    print("="*30)

    # 4. 검증 (Validation)
    # 6개 클래스에 대한 각각의 정확도가 출력될 것입니다.
    metrics = model.val(split='test') 
    
    print(f"\n최종 mAP50 (전체 정확도): {metrics.box.map50:.3f}")
    
    # 저장 경로도 'train_all_parts'로 변경됨
    save_path = os.path.join(os.getcwd(), 'runs/detect/train_all_parts/weights/best.pt')
    print(f"★ 최종 모델 파일 위치: {save_path}")

if __name__ == '__main__':
    train_yolo()