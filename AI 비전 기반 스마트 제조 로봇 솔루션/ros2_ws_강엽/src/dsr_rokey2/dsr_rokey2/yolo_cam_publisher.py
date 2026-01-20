import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from rclpy.qos import qos_profile_sensor_data

class CamNode(Node):
    def __init__(self):
        super().__init__('cam_node')
        
        # 1. 퍼블리셔 생성 (토픽 이름 중요!)
        # YOLO 노드가 듣고 있는 토픽 이름과 똑같이 맞춰줍니다.
        self.publisher_ = self.create_publisher(
            Image, 
            '/camera/camera/color/image_raw', 
            qos_profile_sensor_data
        )
        
        # 2. 타이머 설정 (0.033초 = 약 30 FPS)
        self.timer = self.create_timer(0.033, self.timer_callback)
        
        # 3. OpenCV로 웹캠 열기 (Device Index 0번)
        # 노트북 캠이 있다면 0번, 추가 연결된 USB 캠은 1번이나 2번일 수 있습니다.
        self.cap = cv2.VideoCapture(6)
        
        # 카메라 해상도 설정 (선택 사항, 속도를 위해 640x480 추천)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.bridge = CvBridge()
        
        if not self.cap.isOpened():
            self.get_logger().error("Could not open webcam!")
        else:
            self.get_logger().info("Webcam Node Started. Publishing to /camera/camera/color/image_raw")

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            # OpenCV(BGR) 이미지를 ROS 메시지로 변환
            msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            
            # 메시지 발행
            self.publisher_.publish(msg)
            # self.get_logger().info('Publishing video frame') # 로그가 너무 많으면 주석 처리
        else:
            self.get_logger().warn("Failed to capture image")

    def destroy_node(self):
        # 노드 종료 시 카메라 자원 해제
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()