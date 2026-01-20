#!/usr/bin/env python3
import sys
import os
import time
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
from scipy.spatial.transform import Rotation as R

# [중요] 두산 로봇 라이브러리
import DR_init
from dsr_msgs2.srv import Robotiq2FOpen, Robotiq2FClose

# =================================================================
# 1. 전역 설정 및 DSR 라이브러리 초기화
# =================================================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

rclpy.init()
dsr_node = rclpy.create_node("dsr_control_node", namespace=ROBOT_ID)

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, DR_BASE
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit(1)


# =================================================================
# 2. 로직 처리용 노드 클래스
# =================================================================
class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        
        # 홈 위치
        self.POS_HOME = [0.094, -13.665, 59.737, -0.854, 116.752, -270.851]
        self.calib_path = r'/home/wook/ros2_ws/src/dsr_rokey2/resource/T_gripper2camera.npy'
        self.gripper2cam = None 
        self.target_camera_coord = None 

        self.load_calibration_matrix()

        # 그리퍼 클라이언트
        self.cli_open = self.create_client(Robotiq2FOpen, '/dsr01/gripper/robotiq_2f_open')
        self.cli_close = self.create_client(Robotiq2FClose, '/dsr01/gripper/robotiq_2f_close')

        # Subscribers
        self.create_subscription(String, '/search_for_part_n_bad', self.move_to_home_callback, 10)
        self.create_subscription(String, '/part_n_bad_dispose', self.dispose_bad_part_callback, 10)
        self.create_subscription(String, '/part_n_bad_show', self.show_bad_part_callback, 10)
        self.create_subscription(Float32MultiArray, '/yolo_object_pos', self.coord_callback, 10)

        self.get_logger().info("🤖 Real Robot Control Ready (Down Orientation Fixed)")
        self.move_to_home()

    def load_calibration_matrix(self):
        try:
            if not os.path.exists(self.calib_path):
                self.get_logger().error(f"❌ 파일 없음: {self.calib_path}")
                return
            
            self.gripper2cam = np.load(self.calib_path)
            self.get_logger().info(f"✅ Calibration Matrix Loaded:\n{self.gripper2cam}")
            
            if abs(self.gripper2cam[0, 3]) < 1.0 and abs(self.gripper2cam[2, 3]) < 1.0:
                self.gripper2cam[:3, 3] *= 1000.0
                self.get_logger().info("⚠️ Converted Calibration Matrix to MM.")
            
        except Exception as e:
            self.get_logger().error(f"❌ Matrix Load Failed: {e}")

    def coord_callback(self, msg):
        self.target_camera_coord = msg.data

    def get_robot_pose_safe(self):
        max_retries = 3
        for i in range(max_retries):
            try:
                pose = get_current_posx(ref=DR_BASE)[0]
                if pose and len(pose) == 6:
                    return pose
                else:
                    self.get_logger().warn(f"⚠️ Empty pose received. Retrying {i+1}/{max_retries}...")
            except Exception as e:
                self.get_logger().warn(f"⚠️ Pose Error: {e}. Retrying...")
            time.sleep(0.1)
        return None

    def show_bad_part_callback(self, msg):
        print("\n=== [DEBUG] 불량 부품 처리 시작 ===")
        
        if self.target_camera_coord is None:
            self.get_logger().error("❌ 물체 좌표(Camera Coords) 없음!")
            return
        
        if self.gripper2cam is None:
            self.get_logger().error("❌ Calibration Matrix가 로드되지 않음!")
            return

        # 1. YOLO 입력 확인
        cam_x, cam_y, cam_z = self.target_camera_coord
        print(f"📸 1. YOLO Camera Input (mm): X={cam_x:.2f}, Y={cam_y:.2f}, Z={cam_z:.2f}")

        # 2. 현재 로봇 위치 확인
        current_pose = self.get_robot_pose_safe()
        if current_pose is None: return

        print(f"🤖 2. Current Robot Pose (mm): {current_pose}")

        # 3. 좌표 변환 계산
        final_xyz = self.transform_to_base(current_pose, self.target_camera_coord)
        if final_xyz is None: return

        print(f"🎯 3. Calculated Base Target: X={final_xyz[0]:.2f}, Y={final_xyz[1]:.2f}, Z={final_xyz[2]:.2f}")

        # 4. 안전 검사
        if final_xyz[2] > 800.0:
            self.get_logger().warn(f"⚠️ 목표 높이({final_xyz[2]:.2f})가 너무 높습니다.")
        elif final_xyz[2] < -50.0:
            self.get_logger().error(f"⛔ [SAFETY STOP] 바닥 충돌 위험 ({final_xyz[2]:.2f}). 정지합니다.")
            return

        # 5. 이동 명령 (방향 수정됨)
        # [수정] ZYZ Euler에서 Ry=180도가 되어야 뒤집혀서 바닥을 봅니다.
        DOWN_ORIENTATION = [0.0, 180.0, 0.0] 

        target_pos = [final_xyz[0], final_xyz[1], final_xyz[2]] + DOWN_ORIENTATION
        approach_pos = list(target_pos)
        approach_pos[2] += 150.0 

        self.get_logger().info(f"🚀 Moving to Approach: {approach_pos}")
        movel(approach_pos, vel=VELOCITY, acc=ACC)

    def transform_to_base(self, robot_pose, camera_coords):
        try:
            base2gripper = self.pose_to_matrix(robot_pose)
            cam_point = np.array([camera_coords[0], camera_coords[1], camera_coords[2], 1.0])
            base2cam = np.dot(base2gripper, self.gripper2cam)
            target_base = np.dot(base2cam, cam_point)
            return target_base[:3]
        except Exception as e:
            self.get_logger().error(f"좌표 변환 실패: {e}")
            return None

    def pose_to_matrix(self, pose):
        x, y, z = pose[:3]
        rx, ry, rz = pose[3:]
        # ZYZ Euler 변환
        rotation = R.from_euler('ZYZ', [rx, ry, rz], degrees=True)
        rot_matrix = rotation.as_matrix()
        T = np.eye(4)
        T[:3, :3] = rot_matrix
        T[:3, 3] = [x, y, z]
        return T

    def move_to_home_callback(self, msg):
        self.move_to_home()

    def move_to_home(self):
        movej(self.POS_HOME, vel=VELOCITY, acc=ACC)

    def dispose_bad_part_callback(self, msg):
        pass 

    def control_gripper(self, action):
        if action == "open": self.cli_open.call_async(Robotiq2FOpen.Request())
        elif action == "close": self.cli_close.call_async(Robotiq2FClose.Request())

def main(args=None):
    node = RobotControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()