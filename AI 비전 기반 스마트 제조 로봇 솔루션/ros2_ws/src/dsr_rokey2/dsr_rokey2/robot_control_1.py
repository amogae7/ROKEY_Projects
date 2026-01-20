#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String, Float32MultiArray
import numpy as np
import time
import os
import threading
from scipy.spatial.transform import Rotation

# [중요] Service는 동작용, Topic(RobotState)은 상태 확인용으로 분리
from dsr_msgs2.srv import MoveJoint, MoveLine
from dsr_msgs2.msg import RobotState 
from ament_index_python.packages import get_package_share_directory

# 그리퍼 라이브러리
from robot_control.onrobot import RG

# =================================================================
# 1. 전역 설정
# =================================================================
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 60.0, 60.0 

# 그리퍼 설정
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = "502"

# 오프셋 설정
DEPTH_OFFSET = -5.0
MIN_DEPTH = 2.0

class RobotController(Node):
    def __init__(self):
        super().__init__("robot_control_v6_topic")
        
        self.callback_group = ReentrantCallbackGroup()

        try:
            self.package_path = get_package_share_directory("pick_and_place_voice")
            self.calib_path = os.path.join(self.package_path, "resource", "T_gripper2camera.npy")
        except Exception:
            self.calib_path = r'/home/wook/ros2_ws/src/dsr_rokey2/resource/T_gripper2camera.npy'

        self.latest_camera_coords = None 
        self.current_robot_pos = None # [NEW] 로봇 실시간 위치 저장용
        
        # 좌표 설정
        self.POS_HOME = [0.094, -13.665, 59.737, -0.854, 116.752, 90.0]
        self.POS_DISPOSE = [-88.692, 17.798, 86.462, 0.326, 72.262, -358.097]

        # [1] 동작 서비스 (MoveJ, MoveL) - 이건 서비스가 맞음
        self.cli_movej = self.create_client(MoveJoint, '/dsr01/motion/move_joint', callback_group=self.callback_group)
        self.cli_movel = self.create_client(MoveLine, '/dsr01/motion/move_line', callback_group=self.callback_group)

        # [2] 그리퍼 초기화
        self.gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)

        self.get_logger().info("⏳ 서비스 연결 대기 중...")
        self.cli_movej.wait_for_service(timeout_sec=10.0)
        self.cli_movel.wait_for_service(timeout_sec=10.0)

        # [3] Subscribers
        self.create_subscription(String, '/part_n_bad_show', self.show_bad_part_callback, 10, callback_group=self.callback_group)
        self.create_subscription(String, '/part_n_bad_dispose', self.dispose_bad_part_callback, 10, callback_group=self.callback_group)
        self.create_subscription(Float32MultiArray, '/yolo_object_pos', self.coord_callback, 10, callback_group=self.callback_group)
        
        # [핵심 변경] 로봇 상태를 '서비스 요청'이 아니라 '구독(Topic)'으로 받습니다. (Timeout 해결)
        self.create_subscription(RobotState, '/dsr01/state', self.robot_state_callback, 10, callback_group=self.callback_group)

        self.get_logger().info("🤖 Robot Control V6 Ready (Topic State Mode)")
        
        threading.Thread(target=self.init_thread).start()

    def init_thread(self):
        self.get_logger().info("🚀 초기화 동작 시작 (홈 이동)")
        success = self.movej_safe(self.POS_HOME)
        if success:
            self.get_logger().info("✅ 홈 이동 완료")
        else:
            self.get_logger().error("❌ 홈 이동 실패 (Check Services)")
        self.gripper.open_gripper()

    # =============================================================
    # Callbacks
    # =============================================================
    def robot_state_callback(self, msg):
        # [핵심] 로봇이 보내주는 위치 정보를 실시간으로 저장 (대기 시간 0초)
        if msg is not None:
            self.current_robot_pos = list(msg.current_posx) # [x, y, z, rx, ry, rz]

    def coord_callback(self, msg):
        self.latest_camera_coords = msg.data

    def show_bad_part_callback(self, msg):
        self.get_logger().info(f"📢 Show Command: {msg.data}")
        threading.Thread(target=self.execute_move, args=(msg, "show")).start()

    def dispose_bad_part_callback(self, msg):
        self.get_logger().info(f"📢 Dispose Command: {msg.data}")
        threading.Thread(target=self.execute_move, args=(msg, "dispose")).start()

    # =============================================================
    # 안전한 이동 함수들
    # =============================================================
    def movej_safe(self, pos):
        req = MoveJoint.Request()
        req.pos = [float(x) for x in pos]
        req.vel = float(VELOCITY); req.acc = float(ACC)
        req.mode = 0; req.blend_type = 0; req.sync_type = 0
        future = self.cli_movej.call_async(req)
        while not future.done(): time.sleep(0.05)
        return future.result().success

    def movel_safe(self, pos):
        req = MoveLine.Request()
        req.pos = [float(x) for x in pos]
        req.vel = float(VELOCITY); req.acc = float(ACC)
        req.mode = 0; req.blend_type = 0; req.sync_type = 0
        future = self.cli_movel.call_async(req)
        while not future.done(): time.sleep(0.05)
        return future.result().success

    def get_robot_pose_safe(self):
        # [핵심] 서비스 호출 대신, 이미 받아놓은 변수(Topic) 리턴
        # 대기 시간 없이 즉시 리턴하므로 Timeout 에러가 날 수 없음
        if self.current_robot_pos is not None:
            return self.current_robot_pos
        
        # 만약 아직 데이터가 안 왔으면 2초만 기다려봄
        start = time.time()
        while self.current_robot_pos is None:
            if time.time() - start > 2.0: return None
            time.sleep(0.05)
        return self.current_robot_pos

    # =============================================================
    # 메인 로직
    # =============================================================
    def execute_move(self, msg, action="show"):
        if msg.data == "home":
            self.get_logger().info("🏠 홈 명령 수신! 홈 위치로 이동합니다.")
            self.movej_safe(self.POS_HOME)
            return

        time.sleep(0.5) 
        self.latest_camera_coords = None 
        self.get_logger().info(f"⏳ '{msg.data}' 찾는 중... (과거 데이터 삭제 완료)")

        wait_start_time = time.time()
        while self.latest_camera_coords is None:
            if time.time() - wait_start_time > 5.0:
                self.get_logger().error("❌ 타겟을 찾을 수 없습니다. (Timeout)")
                return
            time.sleep(0.1)

        self.get_logger().info(f"✅ 좌표 수신 완료! 이동 시작.")

        # [수정] 이제 여기서 Timeout 에러 안 남! (Topic 값 사용)
        self.get_logger().info("🤖 로봇 현재 위치 조회 중...")
        capture_robot_pos = self.get_robot_pose_safe()
        
        if capture_robot_pos is None: 
            self.get_logger().error("❌ 로봇 위치 정보가 없습니다 (Topic 미수신)")
            return

        self.get_logger().info(f"📍 로봇 위치 확보 완료. 변환 시작")

        try:
            td_coord = self.transform_to_base(self.latest_camera_coords[:3], self.calib_path, capture_robot_pos)
        except Exception as e: 
            self.get_logger().error(f"❌ 좌표 변환 에러: {e}")
            return

        # Z축 보정
        if td_coord[2] and sum(td_coord) != 0:
            td_coord[2] += DEPTH_OFFSET
            td_coord[2] = max(td_coord[2], MIN_DEPTH)
        
        # 회전 각도 최적화
        object_yaw = 0.0
        if len(self.latest_camera_coords) > 3:
            object_yaw = self.latest_camera_coords[3]

        while object_yaw > 90.0: object_yaw -= 180.0
        while object_yaw < -90.0: object_yaw += 180.0
        final_yaw = object_yaw + 90.0

        final_target_pos = [td_coord[0], td_coord[1], td_coord[2], 0.0, 180.0, final_yaw]
        self.get_logger().info(f"🚀 최종 목표 이동: {final_target_pos}")
        
        try:
            if action == "show":
                target_pos = list(final_target_pos)
                target_pos[2] += 70.0  
                self.gripper.close_gripper() 
                self.movel_safe(target_pos)
                
            elif action == "dispose":
                approach_pos = list(final_target_pos)
                approach_pos[2] += 100.0 
                self.gripper.open_gripper()
                self.movel_safe(approach_pos)
                
                pick_pos = list(final_target_pos)
                pick_pos[2] -= 50.0 
                self.movel_safe(pick_pos)
                self.gripper.close_gripper()
                time.sleep(1.5)

                self.movel_safe(approach_pos)
                self.movej_safe(self.POS_DISPOSE)
                self.gripper.open_gripper()
                time.sleep(1.0)
                self.movej_safe(self.POS_HOME)
                
        except Exception as e:
            self.get_logger().error(f"이동 중 에러 발생: {e}")

    # Helper Functions
    def get_robot_pose_matrix(self, x, y, z, rx, ry, rz):
        R_mat = Rotation.from_euler("ZYZ", [rx, ry, rz], degrees=True).as_matrix()
        T = np.eye(4); T[:3, :3] = R_mat; T[:3, 3] = [x, y, z]
        return T

    def transform_to_base(self, camera_coords, gripper2cam_path, robot_pos):
        if not os.path.exists(gripper2cam_path): 
            self.get_logger().error(f"캘리브레이션 파일 없음: {gripper2cam_path}")
            return np.array([0,0,0])
        gripper2cam = np.load(gripper2cam_path)
        coord = np.append(np.array(camera_coords), 1)
        x, y, z, rx, ry, rz = robot_pos
        base2gripper = self.get_robot_pose_matrix(x, y, z, rx, ry, rz)
        target = base2gripper @ gripper2cam @ coord
        return target[:3]

def main(args=None):
    rclpy.init(args=args)
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