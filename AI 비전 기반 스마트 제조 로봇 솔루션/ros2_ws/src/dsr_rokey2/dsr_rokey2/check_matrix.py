import numpy as np

# 파일 경로
file_path = r'/home/rokey/ros2_ws/src/dsr_rokey2/resource/T_gripper2camera.npy' 

try:
    # 1. 행렬 로드
    gripper2cam = np.load(file_path)
    print("=== [1] 캘리브레이션 행렬 로드 성공 ===")
    print(gripper2cam)
    
    # 2. 단위 보정 체크 (mm -> m)
    # 이동 거리(Translation)가 1.0(1m)보다 크면 mm 단위일 확률이 높음 -> m로 변환 필요
    if abs(gripper2cam[0, 3]) > 1.0 or abs(gripper2cam[1, 3]) > 1.0 or abs(gripper2cam[2, 3]) > 1.0:
        print("\n⚠️ 경고: 이동 거리 값이 큽니다. mm 단위로 추정되어 m 단위로 변환합니다.")
        gripper2cam[:3, 3] /= 1000.0
        print("-> 변환 후 행렬:\n", gripper2cam)

    # 3. 가상 테스트
    # "만약 카메라 정면 30cm(0.3m) 지점에 물체가 있다면?"
    fake_camera_point = np.array([0.0, 0.0, 0.3, 1.0]) # x, y, z, 1
    
    # 변환 계산 (Robot Base가 아니라 Gripper 기준 좌표까지만 확인 가능)
    # 실제로는 Base -> Gripper -> Camera 순으로 곱해야 하지만, 
    # 여기서는 "행렬이 정상 작동하는지"만 봅니다.
    
    # 단순히 행렬만 곱해봄 (검증용)
    result = np.dot(gripper2cam, fake_camera_point)
    
    print("\n=== [2] 가상 변환 테스트 ===")
    print(f"가상 물체 위치 (카메라 기준): [0, 0, 0.3m]")
    print(f"변환된 위치 (그리퍼 기준): x={result[0]:.3f}, y={result[1]:.3f}, z={result[2]:.3f}")
    
    print("\n-> 이 값이 너무 터무니없다면(예: 수십 미터) 캘리브레이션을 다시 해야 합니다.")

except Exception as e:
    print(f"오류 발생: {e}")
