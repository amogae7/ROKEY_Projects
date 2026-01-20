import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import torch
import os

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        
        # 1. 모델 경로 설정
        home_dir = os.path.expanduser('~')
        model_path = os.path.join(home_dir, 'rokey_ws/src/cam_transform/best_yolo8s.pt')
        
        if os.path.exists(model_path):
            self.get_logger().info(f"모델 파일 로드 중: {model_path}")
            self.model = YOLO(model_path)
        else:
            self.get_logger().warn(f"파일을 찾을 수 없음: {model_path}")
            self.get_logger().warn("기본 모델(yolov8n.pt)을 대신 사용합니다.")
            self.model = YOLO('yolov8n.pt') 

        self.frame_count = 0
        self.skip_frames = 1
        
        if torch.cuda.is_available():
            self.model.to('cuda')
            self.get_logger().info("CUDA 가속 사용 가능 (GPU)")
        else:
            self.get_logger().info("CUDA 사용 불가 (CPU로 동작)")

        self.bridge = CvBridge()

        # 2. 구독 (Subscription)
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )
        self.enable_sub = self.create_subscription(
            Bool,
            '/yolo/enable',
            self.enable_callback,
            10
        )
        self.enabled = True

        # 3. 발행 (Publisher)
        self.image_pub = self.create_publisher(Image, '/yolo_sub/detection_image', 10)
        self.center_pub = self.create_publisher(Point, '/yolo_sub/detection_centers', 10)
        self.label_pub = self.create_publisher(String, '/yolo_sub/detection_labels', 10)
        
        # ==========================================
        # [추가] Car 감지 여부 발행 퍼블리셔
        # ==========================================
        self.car_detected_pub = self.create_publisher(Bool, '/yolo/car_detected', 10)
        
        self.get_logger().info("YOLO Detection Node (Webcam + Car Detection) 시작")

    def image_callback(self, msg):
        if not self.enabled:
            return

        self.frame_count += 1
        if self.frame_count % self.skip_frames != 0:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # YOLO 추론
            results = self.model(cv_image, verbose=False, conf=0.5)
            annotated_image = results[0].plot()
            detections = results[0].boxes

            # [추가] 이번 프레임에서 차가 발견되었는지 체크하는 플래그
            car_found_in_current_frame = False

            if len(detections) > 0:
                for box in detections:
                    # 좌표 및 정보 추출
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    label = self.model.names[cls]
                    
                    # ==========================================
                    # [추가] 라벨이 'car'인지 확인
                    # ==========================================
                    if label == 'car':
                        car_found_in_current_frame = True

                    # 중심점 계산
                    center_x = float((x1 + x2) / 2)
                    center_y = float((y1 + y2) / 2)

                    # 기존 퍼블리셔 발행
                    center_msg = Point()
                    center_msg.x = center_x
                    center_msg.y = center_y
                    center_msg.z = conf
                    self.center_pub.publish(center_msg)

                    label_msg = String()
                    label_msg.data = f"{label}|{center_x:.0f}|{center_y:.0f}|{conf:.2f}"
                    self.label_pub.publish(label_msg)

                    self.get_logger().info(
                        f"검출: {label} | 신뢰도: {conf:.2f} | 중심: ({center_x:.0f}, {center_y:.0f})"
                    )

            # ==========================================
            # [추가] Car 감지 결과 발행 (True / False)
            # ==========================================
            car_bool_msg = Bool()
            car_bool_msg.data = car_found_in_current_frame
            self.car_detected_pub.publish(car_bool_msg)

            # 4. 이미지 발행
            detection_img_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            detection_img_msg.header = msg.header
            self.image_pub.publish(detection_img_msg)

        except Exception as e:
            self.get_logger().error(f"이미지 처리 중 에러 발생: {str(e)}")

    def enable_callback(self, msg: Bool):
        self.enabled = msg.data
        status = "ON" if self.enabled else "OFF"
        self.get_logger().info(f"YOLO Detection 상태 변경: {status}")

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