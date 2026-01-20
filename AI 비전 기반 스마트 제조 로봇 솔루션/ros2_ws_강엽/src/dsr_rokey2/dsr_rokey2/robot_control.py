# 파일명: robot_control.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

# --- 두산 로봇 제어 메시지/서비스 Import ---
from dsr_msgs2.msg import *
from dsr_msgs2.srv import MoveJoint, MoveLine, Robotiq2FOpen, Robotiq2FClose

class RobotControlNode(Node):
    def __init__(self):
        super().__init__('robot_control_node')
        
        # ==========================================================
        # 여기에 mini_jog로 확인한 실제 각도를 적으세요!
        # ==========================================================
        # 1. 홈 위치 (모든 시야가 확보되는 위치) -> [수정 필수!]
        # 아까 말씀하신 그 좌표입니다.
        self.POS_HOME = [0.094, -13.665, 59.737, -0.854, 116.752, -270.851]

        # 2. 불량품 집는 위치 (Pick) -> [수정 필수!]
        # 방금 알려주신 좌표입니다. (posj 괄호 제외)
        self.POS_PICK = [2.015, 23.263, 77.157, -4.051, 79.769, -353.736]

        # 3. 버리는 위치 (Dispose) -> [필요 시 수정]
        # 일단 기존 값 유지 또는 안전한 곳으로 변경
        self.POS_DISPOSE = [-179.5, 2.6, 92.4, -0.8, 81.0, -267.9]
        
        # --- Service Clients ---
        self.cli_movej = self.create_client(MoveJoint, '/dsr01/motion/move_joint')
        self.cli_movel = self.create_client(MoveLine, '/dsr01/motion/move_line')
        self.cli_open = self.create_client(Robotiq2FOpen, '/dsr01/gripper/robotiq_2f_open')
        self.cli_close = self.create_client(Robotiq2FClose, '/dsr01/gripper/robotiq_2f_close')

        self.get_logger().info("Waiting for Robot Services...")
        # 로봇 서비스가 뜰 때까지 대기
        self.cli_movej.wait_for_service(timeout_sec=10.0)
            
        # --- Subscribers ---
        self.create_subscription(String, '/search_for_part_n_bad', self.move_to_home_callback, 10)
        self.create_subscription(String, '/part_n_bad_show', self.show_bad_part_callback, 10)
        self.create_subscription(String, '/part_n_bad_dispose', self.dispose_bad_part_callback, 10)

        self.get_logger().info("🤖 Real Robot Control Ready!")

    # --- 1. 홈 위치 이동 ---
    def move_to_home_callback(self, msg):
        self.get_logger().info("Moving to HOME...")
        self.movej(self.POS_HOME, 40.0, 30.0)

    # --- 2. 불량 부품 가리키기 ---
    def show_bad_part_callback(self, msg):
        self.get_logger().info("Approaching BAD part...")
        # 집는 위치로 이동 (안전을 위해 천천히)
        self.movej(self.POS_PICK, 30.0, 20.0) 

    # --- 3. 불량 부품 버리기 (Pick & Place) ---
    def dispose_bad_part_callback(self, msg):
        self.get_logger().info("Disposing BAD part...")
        
        # 1. 그리퍼 열기
        self.control_gripper("open")
        
        # 2. 물건 집는 위치로 이동
        self.get_logger().info("Moving to PICK position...")
        self.movej(self.POS_PICK, 40.0, 30.0) 
        time.sleep(0.5) 
        
        # 3. 그리퍼 닫기 (집기)
        self.control_gripper("close")
        time.sleep(1.0) 

        # 4. 버리는 위치로 이동
        self.get_logger().info("Moving to DISPOSE position...")
        self.movej(self.POS_DISPOSE, 60.0, 40.0)
        
        # 5. 그리퍼 열기 (버리기)
        self.control_gripper("open")
        time.sleep(1.0)
        
        # 6. 홈으로 복귀
        self.movej(self.POS_HOME, 50.0, 30.0)

    # --- 실제 로봇 구동 함수들 ---
    def movej(self, pos, vel, acc):
        req = MoveJoint.Request()
        req.pos = pos; req.vel = vel; req.acc = acc
        req.time = 0.0; req.radius = 0.0; req.mode = 0; req.blend_type = 0; req.sync_type = 0
        self.cli_movej.call_async(req)

    def control_gripper(self, action):
        if action == "open":
            self.cli_open.call_async(Robotiq2FOpen.Request())
        elif action == "close":
            self.cli_close.call_async(Robotiq2FClose.Request())

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()