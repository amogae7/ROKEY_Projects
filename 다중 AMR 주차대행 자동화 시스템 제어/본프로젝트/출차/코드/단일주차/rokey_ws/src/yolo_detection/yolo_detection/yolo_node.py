import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        # YOLO 모델 로드
        self.model = YOLO('yolov8n.pt')  # nano 모델 (빠름)
        # self.model = YOLO('yolov8s.pt')  # small 모델 (정확함)
        # CvBridge (ROS Image :양방향_화살표: OpenCV)
        self.bridge = CvBridge()
        # 구독자: 원본 이미지
        self.image_sub = self.create_subscription(
            Image,
            '/robot1/oakd/rgb/preview/image_raw',
            self.image_callback,
            10
        )
        # 발행자: Bounding box 그린 이미지
        self.image_pub = self.create_publisher(
            Image,
            '/yolo/detection_image',
            10
        )
        self.get_logger().info("YOLO Detection Node 시작")
    def image_callback(self, msg):
        try:
            # ROS Image → OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # YOLO 추론
            results = self.model(cv_image, verbose=False)
            # Bounding box 그리기
            annotated_image = results[0].plot()
            # 검출된 객체 정보 로그
            detections = results[0].boxes
            if len(detections) > 0:
                for box in detections:
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    label = self.model.names[cls]
                    self.get_logger().info(
                        f"검출: {label} (신뢰도: {conf:.2f})"
                    )
            # OpenCV → ROS Image
            detection_msg = self.bridge.cv2_to_imgmsg(
                annotated_image,
                encoding='bgr8'
            )
            detection_msg.header = msg.header  # 타임스탬프 유지
            # 발행
            self.image_pub.publish(detection_msg)
        except Exception as e:
            self.get_logger().error(f"에러: {str(e)}")
def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()