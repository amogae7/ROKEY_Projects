import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import String, Bool
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import torch
import os
import sys

# Windows OS에서는 CV2.imshow와 ROS spin이 잘 작동하지 않을 수 있으므로,
# Linux 환경을 가정하고 코드를 작성합니다.

class YoloMapTransformNode(Node):
    def __init__(self):
        super().__init__('yolo_map_transform_node')
        
        # ==========================================
        # [설정 1] 모델 로드 및 CUDA 확인
        # ==========================================
        home_dir = os.path.expanduser('~')
        model_path = os.path.join(home_dir, 'rokey_ws/src/cam_transform/best_yolo8s.pt')
        
        if os.path.exists(model_path):
            self.model = YOLO(model_path)
        else:
            self.model = YOLO('yolov8n.pt') 
            self.get_logger().warn("⚠️ 모델 파일 없음. 기본 yolov8n.pt 사용")

        if torch.cuda.is_available():
            self.model.to('cuda')
            self.get_logger().info("CUDA 가속 사용 가능 (GPU)")
        
        # ==========================================
        # [설정 2] 목적 좌표 (SLAM 맵 실제 좌표 - 기둥 바닥면)
        # ==========================================
        # 맵에서 찍은 기둥의 4개 코너 좌표 [앞, 좌, 우, 뒤]
        self.dst_points = np.float32([
            [-2.00,  0.4790],   # 앞 (Front)
            [-1.56,  0.4990],   # 좌 (Left)
            [-1.98,  0.0161],   # 우 (Right)
            [-1.51,  0.0266]    # 뒤 (Back)
        ])

        # ==========================================
        # [변수 1] 캘리브레이션 상태 관리
        # ==========================================
        self.calib_step = 0              # 0: 높이측정, 1: 4점클릭, 2: 완료
        self.height_vector = np.array([0, 0]) # [dx, dy]
        self.temp_pt = None              # 임시 저장용 (높이 측정 시 윗면 좌표)
        self.clicked_points = []         # 사용자가 클릭한 윗면 4점 저장용
        self.homography_matrix = None
        self.click_order = ["앞(Front)", "좌(Left)", "우(Right)", "뒤(Back)"] 
        self.cv_window_name = "YOLO & Calibration"
        
        # ==========================================
        # [변수 2] ROS 통신 및 기타 설정
        # ==========================================
        self.bridge = CvBridge()
        self.enabled = True
        self.frame_count = 0
        self.skip_frames = 1
        
        # ROS 통신 설정 (Subscriber)
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.enable_sub = self.create_subscription(Bool, '/yolo/enable', self.enable_callback, 10)

        # Publisher (7개 토픽)
        self.image_pub = self.create_publisher(Image, '/yolo_sub/detection_image', 10)
        self.center_pub = self.create_publisher(Point, '/yolo_sub/detection_centers', 10)
        self.label_pub = self.create_publisher(String, '/yolo_sub/detection_labels', 10)
        self.car_pub = self.create_publisher(Bool, '/yolo/car_detected', 10)
        self.map_point_pub = self.create_publisher(PointStamped, '/yolo/map_point', 10)
        self.marker_pub = self.create_publisher(Marker, '/yolo/marker', 10)

        self.get_logger().info("🚀 노드 시작! [Step 1] 기둥의 높이 차이를 측정하세요.")

    def enable_callback(self, msg: Bool):
        self.enabled = msg.data

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # Step 1: 높이 측정 (Top -> Bottom 클릭)
            if self.calib_step == 0:
                if self.temp_pt is None:
                    self.temp_pt = (x, y) # 윗면 클릭
                    self.get_logger().info("☝️ 윗면 클릭됨. 이제 수직 아래 바닥을 클릭하세요.")
                else:
                    # 바닥 클릭 -> 높이 벡터 계산
                    dx = x - self.temp_pt[0]
                    dy = y - self.temp_pt[1]
                    self.height_vector = np.array([dx, dy], dtype=np.float32)
                    self.calib_step = 1
                    self.temp_pt = None
                    self.get_logger().info(f"📏 높이 보정값 계산 완료: x={dx}, y={dy}. 4점 클릭 시작.")

            # Step 2: 윗면 4점 클릭
            elif self.calib_step == 1:
                if len(self.clicked_points) < 4:
                    self.clicked_points.append([x, y])
                    self.get_logger().info(f"🖱️ 클릭 {len(self.clicked_points)}/4: ({x}, {y})")

    def image_callback(self, msg):
        if not self.enabled:
            return
        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # ==========================================
            # 모드 1: 캘리브레이션 (Step 0, 1)
            # ==========================================
            if self.calib_step < 2:
                # 캘리브레이션 모드 화면 표시 및 처리
                if self.calib_step == 0:
                    msg_text = "Click TOP edge" if self.temp_pt is None else "Click BOTTOM edge"
                    cv2.putText(cv_image, "[STEP 1] Height Vector", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
                    cv2.putText(cv_image, msg_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

                elif self.calib_step == 1:
                    # 4점 클릭 단계
                    msg_text = f"Click TOP: {self.click_order[len(self.clicked_points)]}" if len(self.clicked_points) < 4 else "Computing..."
                    cv2.putText(cv_image, "[STEP 2] 4-Point Homography", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
                    cv2.putText(cv_image, msg_text, (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                    
                    # 4점 찍었으면 행렬 계산 ( Assertion Error 방지 로직 포함)
                    if len(self.clicked_points) == 4:
                        src_pts_list = np.float32(self.clicked_points)
                        
                        # [ASSERTION CHECK] 행렬 크기 검증 (가장 중요)
                        if src_pts_list.shape == (4, 2) and self.dst_points.shape == (4, 2):
                            # [핵심] 높이 보정 적용 (바닥 좌표 계산)
                            bottom_pts = src_pts_list + self.height_vector
                            
                            self.homography_matrix = cv2.getPerspectiveTransform(bottom_pts, self.dst_points)
                            self.calib_step = 2
                            self.get_logger().info("✅ 캘리브레이션 완료! YOLO 모드로 전환합니다.")
                        else:
                            self.get_logger().error(f"❌ 좌표 개수 오류! {src_pts_list.shape[0]}점. 리셋합니다.")
                            self.clicked_points = []
                            self.temp_pt = None
                            self.calib_step = 0 # 처음부터 다시 시작

                    # 찍은 점과 예상 바닥 위치 시각화
                    for pt in self.clicked_points:
                        cv2.circle(cv_image, (pt[0], pt[1]), 5, (0, 255, 255), -1) # 윗면 (노랑)
                        bottom_x = int(pt[0] + self.height_vector[0])
                        bottom_y = int(pt[1] + self.height_vector[1])
                        cv2.circle(cv_image, (bottom_x, bottom_y), 3, (0, 255, 0), -1) # 바닥 (초록)

                # 화면 출력 및 마우스 콜백 연결
                cv2.imshow(self.cv_window_name, cv_image)
                cv2.setMouseCallback(self.cv_window_name, self.mouse_callback)
                
                # 키 입력 확인 (R 누르면 리셋)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('r') or key == ord('R'):
                    self.clicked_points = []
                    self.temp_pt = None
                    self.calib_step = 0
                    self.get_logger().info("🔄 리셋되었습니다. 다시 찍어주세요.")
                return # 캘리브레이션 중에는 YOLO 실행 안 함

            # ==========================================
            # 모드 2: YOLO 실행 및 좌표 변환 (calib_step == 2)
            # ==========================================
            else:
                self.frame_count += 1
                if self.frame_count % self.skip_frames != 0:
                    return
                
                results = self.model(cv_image, verbose=False, conf=0.5, imgsz=320)
                annotated_image = results[0].plot()
                car_detected = False

                for i, box in enumerate(results[0].boxes):
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    label = self.model.names[int(box.cls[0])]
                    if label == 'car': car_detected = True

                    # 중심점 (픽셀)
                    cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
                    
                    # 맵 좌표 변환 (Homography)
                    px = np.array([[[cx, cy]]], dtype=np.float32)
                    dst = cv2.perspectiveTransform(px, self.homography_matrix)
                    map_x, map_y = dst[0][0][0], dst[0][0][1]

                    # 1. PointStamped (RViz 지도 표시)
                    ps_msg = PointStamped()
                    ps_msg.header.stamp = self.get_clock().now().to_msg()
                    ps_msg.header.frame_id = "map"  
                    ps_msg.point.x, ps_msg.point.y = float(map_x), float(map_y)
                    self.map_point_pub.publish(ps_msg)

                    # 2. Marker (RViz 시각화)
                    marker = Marker()
                    marker.header = ps_msg.header
                    marker.ns = "yolo_objects"
                    marker.id = i
                    marker.type = Marker.SPHERE
                    marker.action = Marker.ADD
                    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = float(map_x), float(map_y), 0.0
                    marker.scale.x, marker.scale.y, marker.scale.z = 0.2, 0.2, 0.2
                    marker.color.a = 1.0; marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0
                    self.marker_pub.publish(marker)

                    # 3. 로그 및 이미지 시각화
                    cv2.putText(annotated_image, f"({map_x:.2f}, {map_y:.2f})m", 
                               (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
                    self.get_logger().info(f"📍 {label} -> Map({map_x:.2f}, {map_y:.2f})")

                self.car_pub.publish(Bool(data=car_detected))
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(annotated_image, "bgr8"))
                
                cv2.imshow(self.cv_window_name, annotated_image)
                
                if cv2.waitKey(1) & 0xFF == ord('r'):
                    self.calib_step = 0
                    self.clicked_points = []
                    self.temp_pt = None

        except Exception as e:
            self.get_logger().error(f"FATAL ERROR: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloMapTransformNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    # 캘리브레이션 윈도우가 메인 함수에서 생성되어야 하므로 이 코드를 사용합니다.
    main()