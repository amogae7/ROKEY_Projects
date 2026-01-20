from ultralytics import YOLO
import os

def main():
    # 1. 모델 불러오기
    # OBB(Oriented Bounding Box) 전용 모델을 사용해야 합니다.
    # n(nano)은 가장 가볍고 빠르며, s(small), m(medium)으로 갈수록 성능은 좋지만 느려집니다.
    model = YOLO('yolo11m-obb.pt') 

    # 2. 데이터셋 경로 설정 (절대 경로 추천)
    # 현재 데이터셋 폴더 경로를 정확하게 적어줍니다.
    data_path = '/home/rokey/ros2_ws/src/Tutorial/Calibration_Tutorial/data/assemble/data.yaml'

    # 3. 학습 시작 (Training)
    results = model.train(
        data=data_path,
        epochs=100,        # 100으로 상향 조정
        imgsz=640,
        batch=-1,          # -1로 설정하면 GPU 메모리에 맞춰 자동으로 최적 배치 설정
        device=0,
        project='rokey_obb_project',
        name='run_1',
        patience=20        # 20 epoch 동안 개선 없으면 조기 종료 (옵션)
    )

    print("학습 완료! 결과는 rokey_obb_project/run_1 폴더를 확인하세요.")

if __name__ == '__main__':
    main()
