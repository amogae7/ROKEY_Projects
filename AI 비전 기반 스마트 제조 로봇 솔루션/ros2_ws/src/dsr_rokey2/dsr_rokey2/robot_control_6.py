#!/usr/bin/env python3
import os
import time
import sys
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup 

# 메시지 및 서비스
from std_msgs.msg import String, Float32MultiArray
from std_srvs.srv import Trigger
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
        super().__init__("robot_control_v4") # 노드 이름 유지
        
        # [핵심] 콜백 그룹: 이게 있어야 통신이 안 막히고 잘 됩니다.
        self.callback_group = ReentrantCallbackGroup()

        try:
            self.package_path = get_package_share_directory("pick_and_place_voice")
            self.calib_path = os.path.join(self.package_path, "resource", "T_gripper2camera.npy")
        except Exception:
            self.calib_path = r'/home/wook/ros2_ws/src/dsr_rokey2/resource/T_gripper2camera.npy'

        self.latest_camera_coords = None 
        self.stop_flag = False 
        
        # 좌표 설정 (홈/버리기)
        self.POS_HOME = [0.094, -13.665, 59.737, -0.854, 116.752, 90.0]
        self.POS_DISPOSE = [-88.692, 17.798, 86.462, 0.326, 72.262, 90.0]

        # ---------------------------------------------------------
        # Subscribers (토픽 구독)
        # ---------------------------------------------------------
        # 1. 보여줘
        self.create_subscription(String, '/part_n_bad_show', self.show_bad_part_callback, 10, callback_group=self.callback_group)
        # 2. 처리해
        self.create_subscription(String, '/part_n_bad_dispose', self.dispose_bad_part_callback, 10, callback_group=self.callback_group)
        # 3. YOLO 좌표
        self.create_subscription(Float32MultiArray, '/yolo_object_pos', self.coord_callback, 10, callback_group=self.callback_group)
        # 4. 정지
        self.create_subscription(String, '/robot_stop', self.stop_callback, 10, callback_group=self.callback_group)
        
        # [NEW] 5. 그리퍼 제어 (추가됨!)
        self.create_subscription(String, '/gripper_control', self.gripper_callback, 10, callback_group=self.callback_group)

        self.get_logger().info("🤖 Robot Control Ready (Like v4 + Gripper)")
        self.init_thread()

    def init_thread(self):
        movej(self.POS_HOME, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()

    # =============================================================
    # Callback Functions
    # =============================================================
    def coord_callback(self, msg):
        self.latest_camera_coords = msg.data

    # [NEW] 그리퍼만 움직이는 콜백 함수
    def gripper_callback(self, msg):
        cmd = msg.data
        if cmd == "open":
            self.get_logger().info("🖐 그리퍼 열기 명령 수행")
            gripper.open_gripper()
        elif cmd == "close":
            self.get_logger().info("✊ 그리퍼 닫기 명령 수행")
            gripper.close_gripper()

    def get_robot_pose_safe(self):
        max_retries = 5
        for i in range(max_retries):
            try:
                pose_list = get_current_posx(ref=DR_BASE)
                if pose_list and len(pose_list) > 0:
                    return pose_list[0]
            except Exception: pass
            time.sleep(0.1)
        return None

    def show_bad_part_callback(self, msg):
        self.get_logger().info(f"📢 Show Command: {msg.data}")
        self.execute_move(msg, action="show")

    def dispose_bad_part_callback(self, msg):
        self.get_logger().info(f"📢 Dispose Command: {msg.data}")
        self.execute_move(msg, action="dispose")

    # =============================================================
    # 이동 로직 (Robot Control 4 기반)
    # =============================================================
    def execute_move(self, msg, action="show"):
        self.stop_flag = False 

        # 1. 홈 이동
        if msg.data == "home":
            self.get_logger().info("🏠 홈 이동")
            movej(self.POS_HOME, vel=VELOCITY, acc=ACC)
            mwait()
            return 

        # 2. 타겟 좌표 대기
        time.sleep(0.5) 
        self.latest_camera_coords = None 
        self.get_logger().info(f"⏳ '{msg.data}' 찾는 중...")

        wait_start_time = time.time()
        while self.latest_camera_coords is None:
            if self.stop_flag: 
                self.get_logger().warn("🛑 대기 중 정지")
                return
            if time.time() - wait_start_time > 5.0:
                self.get_logger().error("❌ Timeout: YOLO가 좌표를 보내지 않습니다.")
                return
            time.sleep(0.1)

        self.get_logger().info(f"✅ 좌표 수신 완료")
        if self.stop_flag: return

        # 3. 좌표 변환
        capture_robot_pos = self.get_robot_pose_safe()
        if capture_robot_pos is None: return

        try:
            td_coord = self.transform_to_base(self.latest_camera_coords[:3], self.calib_path, capture_robot_pos)
        except Exception: return

        if td_coord[2] and sum(td_coord) != 0:
            td_coord[2] += DEPTH_OFFSET
            td_coord[2] = max(td_coord[2], MIN_DEPTH)
        
        object_yaw = 0.0
        if len(self.latest_camera_coords) > 3:
            object_yaw = self.latest_camera_coords[3]

        # [중요] 각도 설정
        # 1. 최적화 루프(while) 삭제됨 (회전 문제 방지)
        # 2. +90도 삭제함 (YOLO가 파란선/짧은축 각도를 보내주므로 그대로 사용)
        final_target_pos = [td_coord[0], td_coord[1], td_coord[2], 0.0, 180.0, object_yaw + 90.0]
        
        # 4. 동작 수행
        if action == "show":
            target_pos = list(final_target_pos)
            target_pos[2] += 70.0  
            if self.stop_flag: return
            gripper.close_gripper() 
            if self.stop_flag: return
            movel(target_pos, vel=VELOCITY, acc=ACC)
            
        elif action == "dispose":
            # 접근
            if self.stop_flag: return
            approach_pos = list(final_target_pos)
            approach_pos[2] += 100.0 
            gripper.open_gripper()
            movel(approach_pos, vel=VELOCITY, acc=ACC)
            mwait()
            
            # 잡기 (높이 조절 포함)
            if self.stop_flag: return
            pick_pos = list(final_target_pos)
            pick_pos[2] -= 50.0 # [높이] 5mm 위에서 잡기
            
            movel(pick_pos, vel=VELOCITY, acc=ACC)
            mwait()

            if self.stop_flag: return
            gripper.close_gripper()
            time.sleep(1.5)

            # 들기
            if self.stop_flag: return
            movel(approach_pos, vel=VELOCITY, acc=ACC)
            mwait()

            # 버리기
            if self.stop_flag: return
            movej(self.POS_DISPOSE, vel=VELOCITY, acc=ACC)
            mwait()
            
            if self.stop_flag: return
            gripper.open_gripper()
            time.sleep(1.0)
            
            # 복귀
            if self.stop_flag: return
            movej(self.POS_HOME, vel=VELOCITY, acc=ACC)
            mwait()

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R_mat = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4); T[:3, :3] = R_mat; T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        if not os.path.exists(gripper2cam_path): return np.array([0,0,0])
        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)
        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)
        target = base2gripper @ gripper2cam @ coord
        return target[:3]
    
    def stop_callback(self, msg):
        self.get_logger().warn(f"🚨 비상 정지! ({msg.data})")
        self.stop_flag = True  
        try:
            from DSR_ROBOT2 import stop
            stop(2) 
        except: pass

def main(args=None):
    node = RobotController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("종료")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()