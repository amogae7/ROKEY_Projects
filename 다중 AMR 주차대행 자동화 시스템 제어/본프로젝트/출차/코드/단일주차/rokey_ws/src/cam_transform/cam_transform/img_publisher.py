import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):
    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, '/image_raw', 10)
        
        # 0.033초마다 실행 (30 FPS)
        self.timer = self.create_timer(0.033, self.timer_callback)
        
        self.camera_device_id = 2  # 사용 환경에 맞게 0 또는 2
        self.cap = cv2.VideoCapture(self.camera_device_id)
        
        # [중요] 카메라 호환성을 위한 포맷 강제 설정 (MJPG)
        # 웹캠이 고해상도에서 버벅이거나 화면이 안 나오면 이 설정이 필수입니다.
        fourcc = cv2.VideoWriter_fourcc(*'MJPG') 
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

        self.cv_bridge = CvBridge()
        
        if not self.cap.isOpened():
            self.get_logger().error(f"카메라({self.camera_device_id})를 열 수 없습니다!")
        else:
            self.get_logger().info(f"카메라({self.camera_device_id}) 연결 성공! 데이터 읽기 시도 중...")

    def timer_callback(self):
        ret, frame = self.cap.read()
        
        if ret:
            msg = self.cv_bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            self.publisher_.publish(msg)
            # 디버깅용 로그: 성공 시 점(.) 하나 출력 (너무 시끄럽지 않게)
            print(".", end="", flush=True) 
        else:
            # 실패 시 명확하게 경고 출력
            self.get_logger().warn("⚠️ 프레임 읽기 실패 (ret=False). 장치 번호나 포맷을 확인하세요.")

    def __del__(self):
        # 노드 종료 시 카메라 자원 해제
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
