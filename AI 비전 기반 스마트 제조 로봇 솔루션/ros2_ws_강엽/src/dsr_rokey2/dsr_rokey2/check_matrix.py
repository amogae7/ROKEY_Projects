import numpy as np
import os

# 파일이 있는 경로로 수정하거나 같은 폴더에서 실행하세요
file_path = r'/home/rokey/ros2_ws/src/dsr_rokey2/resource/T_gripper2camera.npy' 

try:
    data = np.load(file_path)
    print("=== T_gripper2camera.npy 내용 ===")
    print(data)
    print("================================")
    
    # 간단한 검증
    print("\n[분석]")
    print(f"1. 행렬 크기: {data.shape} -> (4, 4)여야 함")
    print(f"2. 이동 거리(Translation): x={data[0,3]:.4f}, y={data[1,3]:.4f}, z={data[2,3]:.4f}")
    
except Exception as e:
    print(f"파일을 읽는데 실패했습니다: {e}")