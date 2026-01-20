#!/usr/bin/env python3
import os
import time
import sys
import numpy as np
import json
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup 

from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory

import DR_init
from robot_control.onrobot import RG

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -10.0
MIN_DEPTH = 2.0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
dsr_node = rclpy.create_node("dsr_control_node", namespace=ROBOT_ID)
DR_init.__dsr__node = dsr_node

try:
    from DSR_ROBOT2 import movej, movel, get_current_posx, mwait, DR_BASE
except ImportError as e:
    sys.exit(1)

gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

class RobotController(Node):
    def __init__(self):
        super().__init__("robot_control_v12")
        self.callback_group = ReentrantCallbackGroup()

        try:
            self.package_path = get_package_share_directory("pick_and_place_voice")
            self.calib_path = os.path.join(self.package_path, "resource", "T_gripper2camera.npy")
        except Exception:
            self.calib_path = r'/home/wook/ros2_ws/src/dsr_rokey2/resource/T_gripper2camera.npy'
        
        self.object_memory = {}
        self.scan_base_pose = None
        self.is_at_home = False
        self.is_paused = False 
        
        self.POS_HOME = [0.094, -13.665, 59.737, -0.854, 116.752, 90.0]
        self.POS_DISPOSE = [-88.692, 17.798, 86.462, 0.326, 72.262, 90.0]
        self.POS_DISPOSE_PART3 = [-61.888, 31.102, 51.029, 0.012, 97.602, 90.0]
        
        # [NEW] Pass/Non-pass 전용 폐기 위치
        self.POS_DISPOSE_ASSEMBLY = [-40.835, 67.1, 11.204, 3.641, 99.871, 90.0]

        self.create_subscription(String, '/part_n_bad_show', self.show_bad_part_callback, 10, callback_group=self.callback_group)
        self.create_subscription(String, '/part_n_bad_dispose', self.dispose_bad_part_callback, 10, callback_group=self.callback_group)
        self.create_subscription(String, '/gripper_control', self.gripper_callback, 10, callback_group=self.callback_group)
        self.create_subscription(String, '/robot_stop', self.stop_callback, 10, callback_group=self.callback_group)
        self.create_subscription(String, '/robot_resume', self.resume_callback, 10, callback_group=self.callback_group)
        self.create_subscription(String, '/yolo_all_detect', self.update_memory_callback, 10, callback_group=self.callback_group)
        
        self.get_logger().info("🤖 Robot Control Ready (Pass/Non-pass Sorting Added)")
        self.init_thread()

    def init_thread(self):
        self.go_home_and_scan()

    def go_home_and_scan(self):
        movej(self.POS_HOME, vel=VELOCITY, acc=ACC)
        self.custom_gripper_open()
        mwait()
        self.scan_base_pose = self.get_robot_pose_safe()
        if self.scan_base_pose is not None:
            self.is_at_home = True
            self.object_memory.clear()
            self.get_logger().info("🏠 홈 도착 & 스캔 준비 완료")

    def update_memory_callback(self, msg):
        if not self.is_at_home: return
        try:
            data = json.loads(msg.data)
            for obj in data:
                self.object_memory[obj['name']] = obj['coords']
        except: pass

    def gripper_callback(self, msg):
        self.check_pause() 
        if msg.data == "open": self.custom_gripper_open() 
        elif msg.data == "close": gripper.close_gripper()

    def custom_gripper_open(self):
        try: gripper.move_gripper(60.0)
        except: gripper.open_gripper()

    def check_pause(self):
        if self.is_paused:
            while self.is_paused and rclpy.ok(): time.sleep(0.5)

    def stop_callback(self, msg):
        self.is_paused = True
        try: 
            from DSR_ROBOT2 import stop
            stop(2)
        except: pass

    def resume_callback(self, msg):
        self.is_paused = False

    def show_bad_part_callback(self, msg):
        self.execute_move(msg, action="show")

    def dispose_bad_part_callback(self, msg):
        self.execute_move(msg, action="dispose")

    def execute_move(self, msg, action="show"):
        self.is_paused = False 
        target_name = msg.data 

        if target_name == "home":
            self.check_pause()
            self.go_home_and_scan()
            return 

        self.is_at_home = False 
        
        # 메모리에서 좌표 찾기
        self.get_logger().info(f"🔍 '{target_name}' 위치 검색 중...")
        wait_start = time.time()
        found_coords = None
        while time.time() - wait_start < 3.0:
            if target_name in self.object_memory:
                found_coords = self.object_memory[target_name]
                break
            time.sleep(0.1)
        
        if found_coords is None:
            self.get_logger().error(f"❌ '{target_name}'를 찾을 수 없습니다!")
            return

        self.check_pause()
        if self.scan_base_pose is None: return

        try:
            td_coord = self.transform_to_base(found_coords[:3], self.calib_path, self.scan_base_pose)
        except: return

        if td_coord[2] and sum(td_coord) != 0:
            td_coord[2] += DEPTH_OFFSET
            td_coord[2] = max(td_coord[2], MIN_DEPTH)
        
        object_yaw = 0.0
        if len(found_coords) > 3: object_yaw = found_coords[3]

        final_target_pos = [td_coord[0], td_coord[1], td_coord[2], 0.0, 180.0, object_yaw]
        
        # 동작 수행
        if action == "show":
            target_pos = list(final_target_pos)
            target_pos[2] += 80.0  
            target_pos[5] += 90.0
            self.check_pause()
            movel(target_pos, vel=VELOCITY, acc=ACC)
            self.get_logger().info(f"📍 {target_name} 위로 이동 완료.")
            
        elif action == "dispose":
            # 접근 및 잡기
            approach_pos = list(final_target_pos)
            approach_pos[2] += 80.0 
            approach_pos[5] += 90.0

            self.custom_gripper_open()
            movel(approach_pos, vel=VELOCITY, acc=ACC)
            mwait()

            pick_pos = list(final_target_pos)
            pick_pos[2] -= 50.0 # Grip Depth
            pick_pos[5] += 90.0
            
            movel(pick_pos, vel=VELOCITY, acc=ACC)
            mwait()

            gripper.close_gripper()
            time.sleep(1.5)
            movel(approach_pos, vel=VELOCITY, acc=ACC)
            mwait()

            # [NEW] 타겟별 분리 배출 로직
            self.check_pause()
            
            if "pass" in target_name or "non_pass" in target_name:
                self.get_logger().info("🚛 조립품(pass/non_pass) 전용 위치로 이동")
                movej(self.POS_DISPOSE_ASSEMBLY, vel=VELOCITY, acc=ACC)
            elif "part_3" in target_name:
                self.get_logger().info("🚛 3번 부품 전용 폐기 위치로 이동")
                movej(self.POS_DISPOSE_PART3, vel=VELOCITY, acc=ACC)
            else:
                self.get_logger().info("🗑️ 일반 폐기 위치로 이동")
                movej(self.POS_DISPOSE, vel=VELOCITY, acc=ACC)
                
            mwait()
            self.custom_gripper_open()
            time.sleep(1.0)
            self.go_home_and_scan()

    def get_robot_pose_safe(self):
        for i in range(5):
            try:
                pose_list = get_current_posx(ref=DR_BASE)
                if pose_list: return pose_list[0]
            except: pass
            time.sleep(0.1)
        return None

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

def main(args=None):
    node = RobotController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try: executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()