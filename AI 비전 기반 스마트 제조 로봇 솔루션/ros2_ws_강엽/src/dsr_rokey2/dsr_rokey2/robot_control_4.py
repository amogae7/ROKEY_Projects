#!/usr/bin/env python3
import os
import time
import sys
import threading
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup 

from std_msgs.msg import String, Float32MultiArray
from std_srvs.srv import Trigger
from od_msg.srv import SrvDepthPosition
from ament_index_python.packages import get_package_share_directory
import DR_init
from robot_control.onrobot import RG

# =================================================================
# 1. 전역 설정
# =================================================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 90, 100

GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"
DEPTH_OFFSET = -5.0
MIN_DEPTH = 2.0

DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

# [1차 호출]
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
        super().__init__("robot_control_v4")
        
        try:
            self.package_path = get_package_share_directory("pick_and_place_voice")
            self.calib_path = os.path.join(self.package_path, "resource", "T_gripper2camera.npy")
        except Exception:
            print('cannot find package -> hard coding path used')
            self.calib_path = r'/home/wook/ros2_ws/src/dsr_rokey2/resource/T_gripper2camera.npy'

        self.POS_HOME = [0.094, -13.665, 59.737, -0.854, 116.752, 90.0]
        
        # [폐기 위치]
        self.POS_DISPOSE = [-88.692, 17.798, 86.462, 0.326, 72.262, 90.0]
        self.POS_DISPOSE_2 = [-88.644, 20.005, 90.878, 0.328, 65.737, 90.001]
        self.POS_DISPOSE_PART3 = [-61.888, 31.102, 51.029, 0.012, 97.602, -61.626]
        self.POS_DISPOSE_PART3_2 = [-61.837, 34.56, 75.384, 0.013, 69.938, -61.625]
        
        self.detected_coords = {}
        self.scan_base_pose = None
        self.is_processing = False 
        
        self.callback_group = ReentrantCallbackGroup()
        self.move_lock = threading.Lock() 

        # ---------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------
        # [Show]
        self.create_subscription(
            String, '/part_n_bad_show', self.show_bad_part_callback, 10, callback_group=self.callback_group)
        
        # [Dispose] 
        self.create_subscription(
            String, '/part_1_bad_dispose', lambda msg: self.dispose_bad_part_callback(msg, 1), 10, callback_group=self.callback_group)
        self.create_subscription(
            String, '/part_2_bad_dispose', lambda msg: self.dispose_bad_part_callback(msg, 2), 10, callback_group=self.callback_group)
        self.create_subscription(
            String, '/part_3_bad_dispose', lambda msg: self.dispose_bad_part_callback(msg, 3), 10, callback_group=self.callback_group)
        
        # [Coord]
        self.create_subscription(
            Float32MultiArray, '/part_1_bad_coord', lambda msg: self.coord_callback(msg, 1), 10, callback_group=self.callback_group)
        self.create_subscription(
            Float32MultiArray, '/part_2_bad_coord', lambda msg: self.coord_callback(msg, 2), 10, callback_group=self.callback_group)
        self.create_subscription(
            Float32MultiArray, '/part_3_bad_coord', lambda msg: self.coord_callback(msg, 3), 10, callback_group=self.callback_group)
        
        self.get_logger().info("🤖 Robot Control V4 Ready (Sequence Fix)")
        self.init_robot()

    def coord_callback(self, msg, part_index):
        self.detected_coords[part_index] = msg.data
        self.get_logger().info(f"📥 Received Coord for Part {part_index} (Stored)")

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
        self.execute_sequence(action="show", target_id=None)

    def dispose_bad_part_callback(self, msg, part_index):
        if self.is_processing:
            self.get_logger().warn(f"⚠️ Robot is busy! Ignoring dispose trigger from Part {part_index}")
            return
        self.get_logger().info(f"📢 Dispose Triggered by Part {part_index} (Batch Processing Started)")
        self.execute_sequence(action="dispose", target_id=None)

    def execute_sequence(self, action, target_id=None):
        self.is_processing = True
        wait_time = 0.0
        max_wait = 5.0
        
        try:
            # 1. 좌표 대기
            while True:
                if not rclpy.ok(): return
                
                if self.detected_coords:
                    self.get_logger().info("⏳ Buffering commands for 1.0s...")
                    time.sleep(1.0) 
                    break
                
                if wait_time >= max_wait:
                    self.get_logger().warn(f"⚠️ Timeout waiting for coords.")
                    self.is_processing = False
                    return

                time.sleep(0.1)
                wait_time += 0.1

            if self.scan_base_pose is None:
                self.get_logger().error("❌ Scan Pose NOT SET.")
                self.is_processing = False
                return

            # 2. 로봇 움직임 실행
            with self.move_lock:
                if not self.detected_coords:
                    self.get_logger().info("ℹ️ Queue is already empty.")
                    self.is_processing = False
                    return

                job_list = sorted(self.detected_coords.keys())
                self.get_logger().info(f"📋 Batch Job Start: {job_list} | Action: {action}")

                for idx in job_list:
                    if not rclpy.ok(): break
                    
                    if idx not in self.detected_coords: continue

                    coords = self.detected_coords[idx]
                    self.get_logger().info(f"▶️ Processing Part {idx}...")
                    
                    success = self.move_to_target(coords, action, self.scan_base_pose, idx)
                    
                    if success:
                        self.get_logger().info(f"✅ Part {idx} Done.")
                        if idx in self.detected_coords:
                            del self.detected_coords[idx]
                        time.sleep(0.5)
                
                self.get_logger().info("🎉 All Batch Jobs Completed.")
                
                if rclpy.ok() and (action == "show" or action == "dispose"):
                    self.get_logger().info("🏠 Returning to HOME Position...")
                    movej(self.POS_HOME, vel=VELOCITY, acc=ACC)
                    mwait()

        except Exception as e:
            self.get_logger().error(f"Execution Error: {e}")
        finally:
            self.is_processing = False

    def move_to_target(self, camera_coords, action, reference_pose, part_id):
        capture_robot_pos = reference_pose 

        try:
            td_coord, object_angle, object_width = self.transform_to_base(camera_coords, self.calib_path, capture_robot_pos)
        except Exception as e:
            self.get_logger().error(f"❌ Transform Error: {e}")
            return False

        if td_coord[2] and sum(td_coord) != 0:
            td_coord[2] += DEPTH_OFFSET
            td_coord[2] = max(td_coord[2], MIN_DEPTH)
        
        VERTICAL_ORIENTATION = [0.0, 180.0, object_angle]
        final_target_pos = list(td_coord) + VERTICAL_ORIENTATION
        
        # [Offset]
        if part_id == 2 or part_id == 1 or part_id == 3:
            self.get_logger().info(f"🔧 Part {part_id} detected: Applying X Offset")
            final_target_pos[0] += 20.0
        
        self.get_logger().info(f"🎯 Target(Part {part_id}): XYZ={final_target_pos[:3]}, Angle={object_angle:.1f}, Width={object_width:.1f}")

        if action == "show":
            target_pos = list(final_target_pos)
            target_pos[2] += 70.0  
            gripper.close_gripper()
            movel(target_pos, vel=VELOCITY, acc=ACC)
            mwait()
            
        elif action == "dispose":
            # [순서 변경] 1. 접근 (Approach)
            self.get_logger().info("🚀 Approaching target...")
            target_pos = list(final_target_pos)
            movel(target_pos, vel=VELOCITY, acc=ACC)
            mwait() # 도착 확인

            # [순서 변경] 2. 그리퍼 열기 (폭 계산)
            if part_id == 1:
                target_open_width = object_width - 9.0
            elif part_id == 3:
                target_open_width = object_width - 5.0
            else: 
                target_open_width = object_width + 5.0
                
            target_open_width = min(target_open_width, 110.0) 
            
            self.get_logger().info(f"👐 Opening Gripper to {target_open_width:.1f}mm")
            
            try:
                gripper.move_gripper(int(target_open_width * 10))
                # [수정] mwait(1.0) 대신 time.sleep(1.5) 사용
                # mwait은 로봇 모션 대기용이라 그리퍼 동작을 기다려주지 않을 수 있음
                time.sleep(1.5)
                # mwait()
            except AttributeError:
                gripper.open_gripper()
            except Exception as e:
                self.get_logger().error(f"Gripper Error: {e}")
                gripper.open_gripper()
            
            # [순서 변경] 3. 하강 (Descend)
            self.get_logger().info("⬇️ Descending to grasp...")
            target_pos[2] -= 45.0 
            movel(target_pos, vel=VELOCITY, acc=ACC)
            mwait() # 도착 확인

            # 4. 잡기
            gripper.close_gripper()
            mwait(1.0) 

            # 5. 들어올리기
            self.get_logger().info("⬆️ Lifting...")
            lift_pos = list(final_target_pos)
            lift_pos[2] += 150.0
            # movel(lift_pos, vel=VELOCITY, acc=ACC)
            # [핵심 수정] 리프트가 씹히지 않도록 확실한 대기 추가
            time.sleep(1.0) # 로봇이 움직이기 시작할 때까지 대기
            mwait() # [핵심 수정] (인자) 없이 호출하여 동작 완료까지 확실히 대기
            
            # 6. 버리기 이동
            if part_id == 3:
                self.get_logger().info("🗑️ Moving to Part 3 Disposal Area")
                movel(lift_pos, vel=VELOCITY, acc=ACC)
                self.get_logger().info("⬆️ part3 올라갑니다!")
                mwait(1.0)
                movej(self.POS_DISPOSE_PART3, vel=VELOCITY, acc=ACC)
                movej(self.POS_DISPOSE_PART3_2, vel=VELOCITY, acc=ACC)
            else:
                self.get_logger().info("🗑️ Moving to Default Disposal Area")
                movel(lift_pos, vel=VELOCITY, acc=ACC)
                mwait(1.0)
                movej(self.POS_DISPOSE, vel=VELOCITY, acc=ACC)
                movej(self.POS_DISPOSE_2, vel=VELOCITY, acc=ACC)

            # time.sleep(0.5)
            mwait() # 이동 완료 대기
            
            # 7. 놓기
            gripper.open_gripper()
            mwait(1.0)

        return True

    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R_mat = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4)
        T[:3, :3] = R_mat
        T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        if not os.path.exists(gripper2cam_path):
            self.get_logger().error(f"Calibration file not found: {gripper2cam_path}")
            return np.array([0,0,0]), 0.0, 0.0

        gripper2cam = np.load(gripper2cam_path)
        
        cam_data = list(camera_coords)
        if len(cam_data) == 5:
            xyz = np.array(cam_data[:3])
            angle = cam_data[3]
            width = cam_data[4]
        elif len(cam_data) == 4:
            xyz = np.array(cam_data[:3])
            angle = cam_data[3]
            width = 50.0 
        else:
            xyz = np.array(cam_data[:3])
            angle = 0.0
            width = 50.0

        coord = np.append(xyz, 1)
        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)
        base2cam = base2gripper @ gripper2cam
        td_coord = np.dot(base2cam, coord)

        return td_coord[:3], angle, width

    def init_robot(self):
        self.get_logger().info("🛫 Moving to HOME Position...")
        movej(self.POS_HOME, vel=VELOCITY, acc=ACC)
        gripper.open_gripper()
        mwait()
        
        time.sleep(0.5)
        self.scan_base_pose = self.get_robot_pose_safe()
        
        if self.scan_base_pose is not None:
            self.get_logger().info(f"🏠 Scan Pose Saved: {self.scan_base_pose}")
        else:
            self.get_logger().error("❌ Failed to save Scan Pose!")

def main(args=None):
    node = RobotController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()