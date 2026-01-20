import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor 
from std_msgs.msg import String
import numpy as np
import time
import os
import threading
from gtts import gTTS 

from dsr_msgs2.msg import *
from dsr_msgs2.srv import MoveJoint, MoveLine, Robotiq2FOpen, Robotiq2FClose, GetCurrentPosx, SetRobotMode, SetRobotControl, MoveStop
from od_msg.srv import SrvDepthPosition 

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        
        # --- 설정 ---
        self.POS_HOME = [0.094, -13.665, 59.737, -0.854, 116.752, 90]
        self.POS_DISPOSE = [-88.692, 17.798, 86.462, 0.326, 72.262, -358.097]
        self.calib_path = r'/home/rokey/ros2_ws/src/dsr_rokey2/resource/T_gripper2camera.npy'
        
        self.stop_event = False      
        self.is_moving = False       
        
        # --- 서비스 클라이언트 ---
        self.cli_movej = self.create_client(MoveJoint, '/dsr01/motion/move_joint')
        self.cli_movel = self.create_client(MoveLine, '/dsr01/motion/move_line')
        self.cli_stop = self.create_client(MoveStop, '/dsr01/motion/move_stop') 
        self.cli_open = self.create_client(Robotiq2FOpen, '/dsr01/gripper/robotiq_2f_open')
        self.cli_close = self.create_client(Robotiq2FClose, '/dsr01/gripper/robotiq_2f_close')
        self.cli_get_pose = self.create_client(GetCurrentPosx, '/dsr01/system/get_current_posx') 
        self.cli_set_mode = self.create_client(SetRobotMode, '/dsr01/system/set_robot_mode')
        self.cli_set_servo = self.create_client(SetRobotControl, '/dsr01/system/set_robot_control')
        self.cli_get_depth = self.create_client(SrvDepthPosition, 'get_3d_position')

        self.get_logger().info("Waiting for Robot Services...")
        self.cli_movej.wait_for_service(timeout_sec=10.0)
        self.cli_movel.wait_for_service(timeout_sec=10.0)
        self.cli_get_depth.wait_for_service(timeout_sec=10.0)

        # --- 구독 ---
        self.create_subscription(String, '/search_for_part_n_bad', self.move_to_home_callback, 10)
        self.create_subscription(String, '/part_n_bad_dispose', self.dispose_bad_part_callback, 10)
        self.create_subscription(String, '/part_n_bad_show', self.show_bad_part_callback, 10)
        self.create_subscription(String, '/robot_stop', self.stop_callback, 10)

        self.get_logger().info("🤖 Real Robot Control Ready!")
        
        # 시작 쓰레드
        self.init_thread = threading.Thread(target=self.initial_sequence)
        self.init_thread.start()

    def initial_sequence(self):
        self.set_robot_ready()
        self.speak("시스템 준비 완료.")
        time.sleep(1.0)
        
        # [디버깅] 여기서 멈추는지 확인
        self.get_logger().info("🚀 [Step 1] 홈 위치 이동 명령 전송 시작...")
        
        if self.movej_safe(self.POS_HOME, 40.0, 30.0):
            self.get_logger().info("🏁 [Step 1] 홈 위치 도착 완료!")
        else:
            self.get_logger().error("❌ [Step 1] 홈 위치 이동 실패 (로봇 응답 없음)")

    # =============================================================
    # [수정] 무한 대기 방지 (Timeout 추가)
    # =============================================================
    def movej_safe(self, pos, vel, acc):
        try:
            self.check_stop() 
            self.is_moving = True
            req = MoveJoint.Request()
            req.pos = pos; req.vel = vel; req.acc = acc; req.mode=0; req.blend_type=0; req.sync_type=0
            
            future = self.cli_movej.call_async(req)
            
            # [수정] 5초 동안 응답 없으면 포기 (무한 대기 방지)
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > 5.0:
                    self.get_logger().error("⏳ [Timeout] 로봇이 5초 동안 응답하지 않습니다. (물리 스위치 확인 필요)")
                    self.is_moving = False
                    return False
                self.check_stop()
                time.sleep(0.05)
            
            res = future.result()
            if not res.success: 
                self.get_logger().error(f"❌ 로봇 이동 거부 (State Error)")
                return False
                
            # 성공했다면 실제로 움직이는 시간 대기 (간단히 sleep 처리)
            # 실제로는 현재 위치가 목표와 같아질 때까지 기다려야 하지만, 일단 0.5초 대기
            time.sleep(0.5)
            
        except Exception as e:
            self.get_logger().error(f"MoveJ 에러: {e}")
            self.is_moving = False
            return False
            
        self.is_moving = False
        return True

    def movel_safe(self, pos, vel, acc):
        try:
            self.check_stop()
            self.is_moving = True
            req = MoveLine.Request()
            req.pos = pos; req.vel = vel; req.acc = acc; req.mode=0; req.blend_type=0; req.sync_type=0
            
            future = self.cli_movel.call_async(req)
            
            start_time = time.time()
            while not future.done():
                if time.time() - start_time > 5.0:
                    self.get_logger().error("⏳ [Timeout] MoveL 응답 없음")
                    self.is_moving = False
                    return False
                self.check_stop()
                time.sleep(0.05)
                
            res = future.result()
            if not res.success: 
                self.get_logger().error("MoveL 명령 실패")
                return False
            time.sleep(0.1)
        except Exception as e:
            self.get_logger().error(f"MoveL 에러: {e}")
            self.is_moving = False
            return False
        self.is_moving = False
        return True

    # =============================================================
    # Callbacks & Sequences (좌표 출력 부분)
    # =============================================================
    def stop_callback(self, msg):
        self.get_logger().warn("🚨 외부 정지 명령 수신!")
        self.stop_event = True
        req = MoveStop.Request(); req.stop_mode = 2
        self.cli_stop.call_async(req)

    def move_to_home_callback(self, msg):
        threading.Thread(target=self.run_home_sequence).start()
        
    def dispose_bad_part_callback(self, msg):
        # 매니저가 보내준 메시지(msg.data)에 진짜 이름이 들어있음
        target_name = msg.data
        self.get_logger().info(f"📥 불량품({target_name}) 처리 요청 받음! 시퀀스 시작...") 
        
        # 쓰레드에 이름(target_name)을 전달함
        threading.Thread(target=self.run_dispose_sequence, args=(target_name,)).start()

    def show_bad_part_callback(self, msg):
        self.speak("위치를 확인합니다.")

    def run_home_sequence(self):
        self.stop_event = False
        self.speak("홈 위치로 이동합니다.")
        self.movej_safe(self.POS_HOME, 40.0, 30.0)

    # 함수 정의에 target_name 추가
    def run_dispose_sequence(self, target_name="part_1_bad"): 
        self.stop_event = False
        self.speak(f"{target_name} 처리를 시작합니다.") # (선택) 이름 말해주기
        
        # 1. 좌표 요청 (여기에 전달받은 이름을 넣음!)
        req = SrvDepthPosition.Request()
        req.target = target_name  # <--- [핵심] 여기가 "bad"에서 변수명으로 바뀜
        
        future = self.cli_get_depth.call_async(req)
        
        # ... (이후 코드는 그대로 유지) ...
        
        # 1. 좌표 요청
        req = SrvDepthPosition.Request(); req.target = "bad"
        future = self.cli_get_depth.call_async(req)
        
        start_wait = time.time()
        while not future.done():
            if time.time() - start_wait > 3.0: break 
            if self.stop_event: return 
            time.sleep(0.1)
            
        if not future.done(): 
            self.get_logger().error("❌ 카메라 서비스 응답 없음")
            self.speak("카메라 응답이 없습니다."); return
            
        response = future.result()
        if response is None or sum(response.depth_position) == 0:
            self.get_logger().warn("⚠️ 카메라는 연결됐지만, 물체 깊이를 못 쟀음 (0.0)")
            self.speak("좌표를 받지 못했습니다."); return

        # =====================================================
        # [출력 1] 여기가 원하시는 카메라 좌표 출력 부분입니다!
        # =====================================================
        cam_x, cam_y, cam_z = response.depth_position
        self.get_logger().info(f"\n📸 [Camera Coords] X={cam_x:.3f}, Y={cam_y:.3f}, Z={cam_z:.3f}")

        # 2. 좌표 변환
        robot_coords = self.transform_to_base([cam_x, cam_y, cam_z])
        if robot_coords is None: 
            self.speak("좌표 변환 실패"); return

        # =====================================================
        # [출력 2] 변환된 로봇 좌표 출력
        # =====================================================
        self.get_logger().info(f"🤖 [Robot Base Coords] X={robot_coords[0]:.3f}, Y={robot_coords[1]:.3f}, Z={robot_coords[2]:.3f}\n")

        current_pose = self.get_current_robot_pose()
        if current_pose is None: return
        
        target_z = max(robot_coords[2], 0.05)
        target_pos = [robot_coords[0], robot_coords[1], target_z, current_pose[3], current_pose[4], current_pose[5]]
        
        # 3. 이동 시작
        try:
            self.control_gripper("open")
            
            # 접근
            approach_pos = list(target_pos); approach_pos[2] += 0.1
            self.get_logger().info(f"🚀 접근 위치로 이동: {approach_pos}")
            if not self.movej_safe(approach_pos, 40.0, 30.0): return
            
            # 잡기
            if not self.movel_safe(target_pos, 20.0, 10.0): return
            self.control_gripper("close"); time.sleep(1.0)
            if not self.movel_safe(approach_pos, 40.0, 30.0): return
            
            # 버리기
            if not self.movej_safe(self.POS_DISPOSE, 60.0, 40.0): return
            self.speak("폐기 완료.")
            self.control_gripper("open"); time.sleep(1.0)
            
            self.movej_safe(self.POS_HOME, 50.0, 30.0)
            
        except Exception as e:
            self.get_logger().error(f"시퀀스 실행 중 에러: {e}")

    # =============================================================
    # Helper Functions
    # =============================================================
    def set_robot_ready(self):
        try:
            self.get_logger().info("⚙️ 로봇 초기화 중...")
            # 2번씩 보내서 확실하게 처리
            req_mode = SetRobotMode.Request(); req_mode.robot_mode = 1 
            self.cli_set_mode.call_async(req_mode)
            time.sleep(0.2)
            
            req_servo = SetRobotControl.Request(); req_servo.robot_control = 1 
            self.cli_set_servo.call_async(req_servo)
            time.sleep(0.2)
            self.get_logger().info("✅ 서보 ON 신호 전송 완료")
        except: pass
    
    def check_stop(self):
        if self.stop_event:
            self.get_logger().warn("🛑 정지 신호 감지됨! 동작을 중단합니다.")
            req = MoveStop.Request(); req.stop_mode = 2
            self.cli_stop.call_async(req)
            raise Exception("STOP_EVENT_TRIGGERED")

    def speak(self, text):
        try:
            filename = 'robot_voice.mp3'
            if os.path.exists(filename): os.remove(filename)
            tts = gTTS(text=text, lang='ko')
            tts.save(filename)
            os.system(f"mpg123 -q {filename} > /dev/null 2>&1 &")
        except: pass

    def control_gripper(self, action):
        if action == "open": self.cli_open.call_async(Robotiq2FOpen.Request())
        else: self.cli_close.call_async(Robotiq2FClose.Request())

    def transform_to_base(self, camera_coords):
        try:
            if not os.path.exists(self.calib_path): return None
            gripper2cam = np.load(self.calib_path)
            if abs(gripper2cam[0, 3]) > 1.0 or abs(gripper2cam[2, 3]) > 1.0: gripper2cam[:3, 3] /= 1000.0
            
            current_pose = self.get_current_robot_pose()
            if current_pose is None: return None
            
            base2gripper = self.pose_to_matrix(current_pose)
            cam_point = np.array([camera_coords[0], camera_coords[1], camera_coords[2], 1.0])
            base2cam = np.dot(base2gripper, gripper2cam)
            target_base = np.dot(base2cam, cam_point)
            return target_base[:3] 
        except: return None

    def get_current_robot_pose(self):
        req = GetCurrentPosx.Request(); req.ref = 0 
        future = self.cli_get_pose.call_async(req)
        start_t = time.time()
        while not future.done(): 
             if time.time() - start_t > 2.0: return None
             time.sleep(0.01)
        try:
            res = future.result()
            p = res.task_pos_info[0].data
            return [p[0]/1000.0, p[1]/1000.0, p[2]/1000.0, p[3], p[4], p[5]]
        except: return None

    def pose_to_matrix(self, pose):
        x, y, z, rx, ry, rz = pose
        rx, ry, rz = np.radians([rx, ry, rz])
        cx, sx = np.cos(rx), np.sin(rx); cy, sy = np.cos(ry), np.sin(ry); cz, sz = np.cos(rz), np.sin(rz)
        R = np.array([[cy*cz, cz*sx*sy - cx*sz, cx*cz*sy + sx*sz],[cy*sz, cx*cz + sx*sy*sz, -cz*sx + cx*sy*sz],[-sy, cy*sx, cx*cy]])
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = [x, y, z]
        return T

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()