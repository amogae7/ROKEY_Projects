import numpy as np
from scipy.spatial.transform import Rotation as R

# 1. 파일 로드
try:
    T = np.load('T_gripper2camera.npy')
except:
    print("T_gripper2camera.npy 파일을 찾을 수 없습니다.")
    exit()

# 2. 정보 추출 (mm -> m 변환 포함)
x = T[0, 3] / 1000.0
y = T[1, 3] / 1000.0
z = T[2, 3] / 1000.0

# 3. 회전 행렬 -> Euler (Yaw, Pitch, Roll) 변환
# ROS2 static_transform_publisher는 'yaw pitch roll' 순서를 사용합니다.
r = R.from_matrix(T[:3, :3])
yaw, pitch, roll = r.as_euler('zyx', degrees=False)

print("="*60)
print("👇 아래 명령어를 복사해서 터미널 3번에 붙여넣으세요 (각도 방식)")
print("-" * 60)
print(f"ros2 run tf2_ros static_transform_publisher {x:.5f} {y:.5f} {z:.5f} {yaw:.5f} {pitch:.5f} {roll:.5f} link6 camera_link")
print("="*60)