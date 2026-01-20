#!/usr/bin/env python3
import os
import time
import sys
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

# 메시지 및 서비스
from std_msgs.msg import String, Float32MultiArray
from std_srvs.srv import Trigger
from od_msg.srv import SrvDepthPosition
from ament_index_python.packages import get_package_share_directory

# 두산 로봇 라이브러리
import DR_init
from robot_control.onrobot import RG

# =================================================================
# 1. 전역 설정 (DSR)
# =================================================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

# 그리퍼 설정
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

# 오프셋 설정
DEPTH_OFFSET = -5.0
MIN_DEPTH = 2.0

# DSR 초기화
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
dsr_node = rclpy.create_node("dsr_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, DR_BASE
except ImportError as e:
    print(f"Error importing DSR_ROBOT2: {e}")
    sys.exit(1)

# 그리퍼 초기화
gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)


class RobotController(Node):
    def __init__(self):
        super().__init__("robot_control_v2")
        
        try:
            self.package_path = get_package_share_directory("pick_and_place_voice")
            self.calib_path = os.path.join(self.package_path, "resource", "T_gripper2camera.npy")
        except Exception:
            print('cannot find package -> hard coding path used')
            self.calib_path = r'/home/wook/ros2_ws/src/dsr_rokey2/resource/T_gripper2camera.npy'

        self.latest_camera_coords = None 
        
        # --- [좌표 설정] ---
        self.POS_HOME = [0.094, -13.665, 59.737, -0.854, 116.752, 90.0]
        
        # [NEW] 전달주신 폐기 장소 좌표 (Joint 각도)
        self.POS_DISPOSE = [-88.692, 17.798, 86.462, 0.326, 72.262, -358.097]

        # ---------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------
        self.create_subscription(String, '/part_n_bad_show', self.show_bad_part_callback, 10)
        self.create_subscription(String, '/part_n_bad_dispose', self.dispose_bad_part_callback, 10)
        
        self.create_subscription(Float32MultiArray, '/yolo_object_pos', self.coord_callback, 10)

        # ---------------------------------------------------------
        # Service Clients
        # ---------------------------------------------------------
        self.get_position_client = self.create_client(SrvDepthPosition, "/get_3d_position")
        self.get_keyword_client = self.create_client(Trigger, "/get_keyword")

        self.get_logger().info("🤖 Robot Control V3 Ready (Pick & Lift & Place Mode)")
        self.init_robot()

    # =============================================================
    # Callback Functions
    # =============================================================
    def coord_callback(self, msg):
        self.latest_camera_coords = msg.data

    def get_robot_pose_safe(self):
        max_retries = 5
        for i in range(max_retries):
            try:
                pose_list = get_current_posx(ref=DR_BASE)
                if pose_list and len(pose_list) > 0:
                    pose = pose_list[0]
                    if pose and len(pose) == 6:
                        return pose
                self.get_logger().warn(f"⚠️ Empty pose received. Retrying {i+1}/{max_retries}...")
            except Exception as e:
                self.get_logger().warn(f"⚠️ Pose Error: {e}. Retrying...")
            time.sleep(0.1)
        self.get_logger().error("❌ Failed to get robot pose after retries.")
        return None

    def show_bad_part_callback(self, msg):
        """단순히 물체 위치로 가서 보여주기만 하는 함수 (Hovering)"""
        self.get_logger().info(f"📢 Show Command: {msg.data}")
        self.execute_move(msg, action="show")

    def dispose_bad_part_callback(self, msg):
        """물체를 집어서 들어올리는 함수 (Pick & Lift & Place)"""
        self.get_logger().info(f"📢 Dispose Command: {msg.data}")
        self.execute_move(msg, action="dispose")

    def execute_move(self, msg, action="show"):
        """공통 이동 로직"""
        if self.latest_camera_coords is None:
            self.get_logger().error("❌ YOLO coordinates missing.")
            return

        # 1. 계산용 로봇 위치 확보
        capture_robot_pos = self.get_robot_pose_safe()
        if capture_robot_pos is None:
            return

        # 2. 좌표 변환
        try:
            td_coord = self.transform_to_base(self.latest_camera_coords, self.calib_path, capture_robot_pos)
        except Exception as e:
            self.get_logger().error(f"❌ Transform Error: {e}")
            return

        # 3. Z축 보정
        if td_coord[2] and sum(td_coord) != 0:
            td_coord[2] += DEPTH_OFFSET
            td_coord[2] = max(td_coord[2], MIN_DEPTH)
        
        self.get_logger().info(f"🎯 Calculated XYZ: {td_coord}")

        # 5. 타겟 설정 (수직 하강)
        VERTICAL_ORIENTATION = [0.0, 180.0, 0.0]
        final_target_pos = list(td_coord[:3]) + VERTICAL_ORIENTATION
        
        # === 동작 분기 ===
        if action == "show":
            # Show 모드: 70mm 위에서 멈춤
            target_pos = list(final_target_pos)
            target_pos[2] += 70.0  
            
            self.get_logger().info(f"🚀 [Show] Hovering at: {target_pos}")
            gripper.close_gripper() 
            movel(target_pos, vel=VELOCITY, acc=ACC)
            
        elif action == "dispose":
            # 1) 그리퍼 열고 접근
            self.get_logger().info("👐 Opening Gripper...")
            gripper.open_gripper()
            time.sleep(1.0)
            
            # 2) 물체 위치로 하강
            target_pos = list(final_target_pos)
            target_pos[2] -= 50.0 
            self.get_logger().info(f"🚀 [Pick] Moving to Object: {target_pos}")
            movel(target_pos, vel=VELOCITY, acc=ACC)
            mwait()

            # 3) 잡기
            self.get_logger().info("✊ Closing Gripper...")
            gripper.close_gripper()
            time.sleep(1.5)

            # 4) 들어올리기 (Lift)
            lift_pos = list(final_target_pos)
            lift_pos[2] += 70.0
            self.get_logger().info(f"⬆️ [Lift] Moving up to: {lift_pos}")
            movel(lift_pos, vel=VELOCITY, acc=ACC)
            mwait()

            # --- [NEW] 5) 폐기 장소로 이동 (Place) ---
            self.get_logger().info(f"🗑️ [Place] 폐기 위치로 이동 중... {self.POS_DISPOSE}")
            # 먼 거리는 movej가 안전합니다.
            movej(self.POS_DISPOSE, vel=VELOCITY, acc=ACC)
            mwait()
            
            # 6) 놓기
            self.get_logger().info("👐 Dropping Object...")
            gripper.open_gripper()
            time.sleep(1.0)
            
            # 7) 홈으로 복귀
            self.get_logger().info("🏠 Going Home...")
            movej(self.POS_HOME, vel=VELOCITY, acc=ACC)
            mwait()

    # =============================================================
    # Helper Functions
    # =============================================================
    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R_mat = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R_mat
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        if not os.path.exists(gripper2cam_path):
            self.get_logger().error(f"Calibration file not found: {gripper2cam_path}")
            return np.array([0,0,0])

        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)

        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)
        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)

        return td_coord[:3]

    def init_robot(self):
        movej(self.POS_HOME, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

def main(args=None):
    node = RobotController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()