from ultralytics import YOLO
import os

def main():
    # 1. 모델 불러오기
    # OBB(Oriented Bounding Box) 전용 모델을 사용해야 합니다.
    # n(nano)은 가장 가볍고 빠르며, s(small), m(medium)으로 갈수록 성능은 좋지만 느려집니다.
    model = YOLO('yolo11s-obb.pt') 

    # 2. 데이터셋 경로 설정 (절대 경로 추천)
    # 현재 데이터셋 폴더 경로를 정확하게 적어줍니다.
    data_path = '/home/rokey/ros2_ws/src/Tutorial/Calibration_Tutorial/data/please/data.yaml'

    # 3. 학습 시작 (Training)
    results = model.train(
        data=data_path,   # data.yaml 파일 경로
        epochs=50,        # 학습 반복 횟수 (보통 50~100 추천)
        imgsz=640,        # 이미지 크기
        batch=16,         # 배치 사이즈 (메모리 부족시 줄이세요: 8 or 4)
        device=0,         # GPU 사용 (0번), CPU만 쓴다면 'cpu'로 변경
        project='rokey_obb_project', # 결과가 저장될 폴더 이름
        name='run_1'      # 이번 학습의 이름
    )

    print("학습 완료! 결과는 rokey_obb_project/run_1 폴더를 확인하세요.")

if __name__ == '__main__':
    main()