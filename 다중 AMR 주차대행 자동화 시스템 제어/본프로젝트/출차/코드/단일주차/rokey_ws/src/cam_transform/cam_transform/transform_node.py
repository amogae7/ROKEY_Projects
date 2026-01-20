import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import os

class YoloMapTransformNode(Node):
    def __init__(self):
        super().__init__('yolo_map_transform_node')
        
        # ==========================================
        # [핵심] 좌표 매칭 설정 (사용자가 직접 입력해야 함!)
        # ==========================================
        
        # 1. 소스 좌표 (웹캠 이미지 픽셀: u, v) - 2단계에서 구한 값
        self.src_points = np.float32([
            [125, 80],   # 좌상 (Top-Left Pixel)
            [510, 85],   # 우상 (Top-Right Pixel)
            [90,  410],  # 좌하 (Bottom-Left Pixel)
            [580, 400]   # 우하 (Bottom-Right Pixel)
        ])

        # 2. 목적 좌표 (SLAM 맵 좌표: x, y 미터) - 1단계에서 구한 값
        # 주의: RViz에서 찍은 좌표 그대로 넣으세요. (음수 포함)
        self.dst_points = np.float32([
            [ 2.0,  1.5],  # 좌상 (Map X, Map Y)
            [ 2.0, -1.5],  # 우상
            [-2.0,  1.5],  # 좌하
            [-2.0, -1.5]   # 우하
        ])

        # 3. 변환 행렬 계산 (픽셀 -> 맵 좌표 직행 행렬)
        self.homography_matrix = cv2.getPerspectiveTransform(self.src_points, self.dst_points)
        self.get_logger().info("✅ 픽셀 -> 맵 좌표 변환 행렬 계산 완료!")

        # ------------------------------------------
        # 모델 및 ROS 설정 (기존과 동일)
        home_dir = os.path.expanduser('~')
        model_path = os.path.join(home_dir, 'rokey_ws/src/cam_transform/best_yolo8s.pt')
        if os.path.exists(model_path):
            self.model = YOLO(model_path)
        else:
            self.model = YOLO('yolov8n.pt') 
        
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        
        # 좌표 발행 (Point)
        self.center_pub = self.create_publisher(Point, '/yolo/map_coordinates', 10)
        self.image_pub = self.create_publisher(Image, '/yolo/detection_image', 10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_image, verbose=False, conf=0.5)
            annotated_image = results[0].plot()
            
            for box in results[0].boxes:
                # 1. 객체 중심 픽셀 좌표 구하기
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cx_pixel = (x1 + x2) / 2
                cy_pixel = (y1 + y2) / 2
                
                # 2. 맵 좌표로 변환 (핵심 함수 호출)
                map_x, map_y = self.pixel_to_map(cx_pixel, cy_pixel)
                
                # 3. 결과 발행
                point_msg = Point()
                point_msg.x = float(map_x)
                point_msg.y = float(map_y)
                point_msg.z = 0.0
                self.center_pub.publish(point_msg)
                
                # 로그 출력
                label = self.model.names[int(box.cls[0])]
                self.get_logger().info(f"검출: {label} -> Map 좌표: ({map_x:.2f}m, {map_y:.2f}m)")
                
                # 이미지에 맵 좌표 그려주기 (시각화)
                cv2.putText(annotated_image, f"({map_x:.1f},{map_y:.1f})m", 
                           (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2)

            self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_image, "bgr8"))

        except Exception as e:
            self.get_logger().error(f"Error: {e}")

    def pixel_to_map(self, u, v):
        """ 픽셀(u, v)를 입력받아 맵 좌표(x, y)를 리턴하는 함수 """
        # 행렬 연산을 위한 차원 확장
        point = np.array([[[u, v]]], dtype=np.float32)
        # 변환 수행
        transformed = cv2.perspectiveTransform(point, self.homography_matrix)
        # 결과 추출
        return transformed[0][0][0], transformed[0][0][1]

def main(args=None):
    rclpy.init(args=args)
    node = YoloMapTransformNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()