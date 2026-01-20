from ultralytics import YOLO
import os

def train_yolo():
    # ==========================================
    # [설정] 모델 및 데이터 경로
    # ==========================================
    # 1. 모델 선택
    # - yolov8n.pt: 가장 가볍고 빠름 (로봇/노트북 추천)
    # - yolov8s.pt: n보다 조금 무겁지만 성능은 더 좋음
    model_name = 'yolov8s.pt' 

    # 2. data.yaml 경로 (현재 폴더 기준)
    # 절대 경로로 변환해서 넣어주면 에러 확률이 0%가 됩니다.
    base_dir = os.getcwd()
    yaml_path = r"/home/rokey/ros2_ws/src/Tutorial/Calibration_Tutorial/data/part_1_good/data.yaml"
    # ==========================================

    print(f"모델: {model_name} 로드 중...")
    model = YOLO(model_name)  # 처음 실행 시 자동으로 다운로드됩니다.

    print(f"학습 시작! 데이터 경로: {yaml_path}")
    
    # 3. 학습 실행 (Train)
    results = model.train(
        data=yaml_path,      # 데이터 설정 파일
        epochs=100,          # 100번 반복 학습 (충분함)
        imgsz=640,           # 이미지 크기 (표준)
        batch=16,            # 한 번에 학습할 양 (메모리 부족 시 8로 줄이세요)
        
        # 결과 저장 설정
        project='runs/detect',     # 결과가 저장될 상위 폴더
        name='train_part_1_good',  # 결과 폴더 이름
        exist_ok=True,             # 폴더가 있어도 덮어쓰기 허용 (에러 방지)
        
        # [옵션] 조기 종료 (Patience)
        # 30번 동안 성능이 안 오르면 100번 다 안 채우고 멈춤 (시간 절약)
        patience=30,  
        
        # [옵션] 데이터 증강 (필요하면 주석 해제)
        # augment=True,
        # degrees=15.0,      # 회전 15도
        # mosaic=1.0,        # 모자이크 켜기
    )

    print("\n" + "="*30)
    print("학습 완료! 이제 테스트 데이터로 검증합니다.")
    print("="*30)

    # 4. 테스트 데이터셋으로 최종 평가 (Test)
    # data.yaml에 'test: ...' 줄이 있어야 작동합니다.
    metrics = model.val(split='test') 
    
    print(f"\n최종 mAP50 (정확도): {metrics.box.map50:.3f}")
    print(f"최종 mAP50-95: {metrics.box.map:.3f}")
    print("="*30)
    print(f"모델 저장 위치: {os.path.join(base_dir, 'runs/detect/train_part_1_good/weights/best.pt')}")

if __name__ == '__main__':
    train_yolo()