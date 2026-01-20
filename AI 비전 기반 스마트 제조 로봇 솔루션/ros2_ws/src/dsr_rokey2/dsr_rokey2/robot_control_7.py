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

from std_msgs.msg import String, Float32MultiArray
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory

import DR_init
from robot_control.onrobot import RG

ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60, 60

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -5.0
MIN_DEPTH = 2.0

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

gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

class RobotController(Node):
    def __init__(self):
        super().__init__("robot_control_v10")
        self.callback_group = ReentrantCallbackGroup()

        try:
            self.package_path = get_package_share_directory("pick_and_place_voice")
            self.calib_path = os.path.join(self.package_path, "resource", "T_gripper2camera.npy")
        except Exception:
            self.calib_path = r'/home/wook/ros2_ws/src/dsr_rokey2/resource/T_gripper2camera.npy'

        self.latest_camera_coords = None 
        
        # [NEW] 1. 물체 메모리
        self.object_memory = {}
        # [NEW] 2. 홈 위치에서의 로봇 자세 (좌표 계산의 기준점)
        self.scan_base_pose = None
        
        self.is_at_home = False
        self.is_paused = False 
        
        self.POS_HOME = [0.094, -13.665, 59.737, -0.854, 116.752, 90.0]
        self.POS_DISPOSE = [-88.692, 17.798, 86.462, 0.326, 72.262, 90.0]

        # Subscribers
        self.create_subscription(String, '/part_n_bad_show', self.show_bad_part_callback, 10, callback_group=self.callback_group)
        self.create_subscription(String, '/part_n_bad_dispose', self.dispose_bad_part_callback, 10, callback_group=self.callback_group)
        self.create_subscription(String, '/gripper_control', self.gripper_callback, 10, callback_group=self.callback_group)
        self.create_subscription(String, '/robot_stop', self.stop_callback, 10, callback_group=self.callback_group)
        self.create_subscription(String, '/robot_resume', self.resume_callback, 10, callback_group=self.callback_group)
        
        # 전체 감지 토픽 구독
        self.create_subscription(String, '/yolo_all_detect', self.update_memory_callback, 10, callback_group=self.callback_group)
        
        self.get_logger().info("🤖 Robot Control v10 Ready (Smart Direct Move Mode)")
        self.init_thread()

    def init_thread(self):
        # 시작하자마자 홈으로 이동 후 기준 좌표 저장
        self.go_home_and_scan()

    def go_home_and_scan(self):
        """홈으로 이동하고 현재 자세를 스캔 기준으로 등록"""
        movej(self.POS_HOME, vel=VELOCITY, acc=ACC)
        self.custom_gripper_open()
        mwait()
        
        # [중요] 홈에 도착했으니 지금 로봇의 자세(Base Pose)를 저장합니다.
        # 앞으로 모든 물체 위치 계산은 이 'scan_base_pose'를 기준으로 합니다.
        self.scan_base_pose = self.get_robot_pose_safe()
        
        if self.scan_base_pose is not None:
            self.is_at_home = True
            self.object_memory.clear() # 기존 기억 삭제
            self.get_logger().info("🏠 홈 도착 & 기준 좌표 설정 완료. 스캔 중...")
        else:
            self.get_logger().error("❌ 홈 좌표를 읽어올 수 없습니다.")

    def update_memory_callback(self, msg):
        # 홈에 있을 때만 메모리 갱신 (좌표 꼬임 방지)
        if not self.is_at_home: return

        try:
            data = json.loads(msg.data)
            for obj in data:
                self.object_memory[obj['name']] = obj['coords']
        except Exception: pass

    def gripper_callback(self, msg):
        self.check_pause() 
        if msg.data == "open": self.custom_gripper_open() 
        elif msg.data == "close": gripper.close_gripper()

    def custom_gripper_open(self):
        try: gripper.move_gripper(60.0)
        except: gripper.open_gripper()

    def check_pause(self):
        if self.is_paused:
            self.get_logger().warn("⏸️ 일시 정지")
            while self.is_paused and rclpy.ok(): time.sleep(0.5)
            self.get_logger().info("▶️ 재개")

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
        target_name = msg.data # "part_2_bad"

        # 1. 홈 명령 처리
        if target_name == "home":
            self.check_pause()
            self.go_home_and_scan()
            return 

        # 2. 이동 시작 (이제 로봇은 홈을 떠납니다)
        self.is_at_home = False 
        
        # 3. 메모리 검색
        self.get_logger().info(f"🔍 '{target_name}' 위치 검색 중...")
        
        # (혹시 홈에 막 도착해서 데이터가 들어오는 중일 수 있으니 3초 대기)
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

        print(f"\n[메모리 좌표 확인] {target_name}: {found_coords}")

        # 4. 좌표 변환 (핵심)
        self.check_pause()
        
        # [매우 중요] 현재 로봇 위치(get_robot_pose_safe)를 쓰지 않습니다!
        # 아까 홈에서 저장해둔 'self.scan_base_pose'를 사용합니다.
        # 그래야 2번에 가있든 3번에 가있든 계산이 정확합니다.
        if self.scan_base_pose is None:
            self.get_logger().error("⚠️ 기준 좌표(Scan Pose)가 없습니다. 홈으로 먼저 가세요.")
            return

        try:
            # 기준 자세(scan_base_pose)를 넣어 변환
            td_coord = self.transform_to_base(found_coords[:3], self.calib_path, self.scan_base_pose)
        except Exception as e: 
            self.get_logger().error(f"좌표 변환 실패: {e}")
            return

        # 높이 안전장치
        if td_coord[2] and sum(td_coord) != 0:
            td_coord[2] += DEPTH_OFFSET
            td_coord[2] = max(td_coord[2], MIN_DEPTH)
        
        object_yaw = 0.0
        if len(found_coords) > 3:
            object_yaw = found_coords[3]

        final_target_pos = [td_coord[0], td_coord[1], td_coord[2], 0.0, 180.0, object_yaw]
        
        # 5. 동작 수행
        # (show 동작 시에는 홈으로 복귀 안 함 -> 바로 다음 명령 대기 가능)
        if action == "show":
            # 안전 높이 이동 (물체 위로)
            target_pos = list(final_target_pos)
            target_pos[2] += 80.0  
            target_pos[5] += 90.0

            self.check_pause()
            movel(target_pos, vel=VELOCITY, acc=ACC)
            self.get_logger().info(f"📍 {target_name} 위로 이동 완료. 다음 명령 대기.")
            
        elif action == "dispose":
            
            offset_map = {
                "part_1_bad": 45.0, 
                "part_2_bad": 60.0,  # 더 깊게 잡기
                "part_3_bad": 38.0,  # 덜 깊게 잡기
            }
            
            # 목록에 없으면 기본값 40.0 사용
            target_offset = offset_map.get(target_name, 40.0)
            
            self.get_logger().info(f"📏 {target_name} 맞춤 높이 적용: -{target_offset}mm")
            
            # 접근 -> 잡기 -> 버리기 -> 홈
            self.check_pause()
            approach_pos = list(final_target_pos)
            approach_pos[2] += 80.0 
            approach_pos[5] += 90.0
            self.custom_gripper_open()
            movel(approach_pos, vel=VELOCITY, acc=ACC)
            mwait()
            
            self.check_pause()
            pick_pos = list(final_target_pos)
            pick_pos[2] -= target_offset
            pick_pos[5] += 90.0

            movel(pick_pos, vel=VELOCITY, acc=ACC)
            mwait()

            self.check_pause()
            gripper.close_gripper()
            time.sleep(1.5)

            self.check_pause()
            movel(approach_pos, vel=VELOCITY, acc=ACC)
            mwait()

            self.check_pause()
            movej(self.POS_DISPOSE, vel=VELOCITY, acc=ACC)
            mwait()
            
            self.check_pause()
            self.custom_gripper_open()
            time.sleep(1.0)
            
            self.check_pause()
            # 버리고 나서는 홈으로 복귀 (다음 스캔 준비)
            self.go_home_and_scan()

    def get_robot_pose_safe(self):
        max_retries = 5
        for i in range(max_retries):
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
    try:
        executor.spin()
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()