import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  # [중요] 문자열 메시지 사용
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from rclpy.qos import qos_profile_sensor_data

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        # 1. 모델 불러오기 (경로를 본인 컴퓨터에 맞게 꼭 수정하세요!)
        # 예: /home/rokey/ros2_ws/src/obb_detector/runs/detect/train/weights/best.pt
        self.model = YOLO('/home/wook/Downloads/please/rokey_obb_project/run_1/weights/best.pt') 

        # 2. 퍼블리셔 만들기 (결과를 보낼 토픽: /yolo_results)
        self.result_publisher = self.create_publisher(String, '/yolo_results', 10)
        
        # 3. 카메라 구독 (카메라 토픽 이름 확인 필요, 보통 /camera/image_raw 또는 /image_raw)
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            qos_profile_sensor_data) # <-- 10 대신 이걸로 변경!
        
        self.bridge = CvBridge()
        self.get_logger().info("YOLO Node has been started.")

    def image_callback(self, msg):
        try:
            # ROS 이미지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # YOLO 추론
            results = self.model(cv_image, verbose=False)
            
            detected_classes = []
            
            for r in results:
                # [중요] OBB 모델일 경우 r.boxes 대신 r.obb를 사용해야 함
                if r.obb is not None:
                    for c in r.obb.cls:
                        class_name = self.model.names[int(c)]
                        detected_classes.append(class_name)
                
                # 혹시 일반 모델로 바꿀 경우를 대비해 r.boxes도 체크 (선택사항)
                elif r.boxes is not None:
                    for c in r.boxes.cls:
                        class_name = self.model.names[int(c)]
                        detected_classes.append(class_name)

            # 결과 문자열 만들기
            result_str = ",".join(detected_classes)
            
            # 메시지 발행
            msg = String()
            msg.data = result_str
            self.result_publisher.publish(msg)
            
            # 화면 표시
            annotated_frame = results[0].plot()
            cv2.imshow("YOLO Inference", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error in processing image: {e}')

            
def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()