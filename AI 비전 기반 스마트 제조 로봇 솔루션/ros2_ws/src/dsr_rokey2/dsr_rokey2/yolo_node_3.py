import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
from rclpy.qos import qos_profile_sensor_data

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        # 1. 모델 설정
        try:
            self.model = YOLO(r'/home/rokey/ros2_ws/src/Tutorial/Calibration_Tutorial/data/please/rokey_obb_project/run_1/weights/best.pt', task='obb')
        except Exception as e:
            self.get_logger().error(f"모델 로드 실패: {e}")
            raise e

        # 2. 퍼블리셔
        self.result_publisher = self.create_publisher(String, '/yolo_results', 10)
        self.pose_publisher = self.create_publisher(Float32MultiArray, '/yolo_object_pos', 10)
        
        # 3. 서브스크라이버 (Color, Depth, CameraInfo)
        self.sub_color = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, qos_profile_sensor_data)
        self.sub_depth = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, qos_profile_sensor_data)
        self.sub_info = self.create_subscription(
            CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.info_callback, qos_profile_sensor_data)
        
        self.bridge = CvBridge()
        self.depth_image = None
        
        # 카메라 내부 파라미터 (초기값)
        self.fx = 605.0
        self.fy = 605.0
        self.cx = 320.0
        self.cy = 240.0
        self.is_intrinsics_received = False

        self.get_logger().info("YOLO Node (Robust Depth & Auto-Intrinsics) Started.")

    def info_callback(self, msg):
        """카메라 내부 파라미터 실시간 업데이트"""
        if not self.is_intrinsics_received:
            # K matrix: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.fy = msg.k[4]
            self.cy = msg.k[5]
            self.is_intrinsics_received = True
            self.get_logger().info(f"✅ Camera Intrinsics Updated: fx={self.fx:.1f}, fy={self.fy:.1f}, cx={self.cx:.1f}, cy={self.cy:.1f}")

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Depth error: {e}")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_image, verbose=False)
            
            detected_classes = []
            target_coords_camera = None 

            for r in results:
                if r.obb is not None:
                    for i, c in enumerate(r.obb.cls):
                        # 신뢰도 60% 미만 무시
                        if r.obb.conf[i] < 0.6:
                            continue

                        class_name = self.model.names[int(c)]
                        detected_classes.append(class_name)
                        
                        # 타겟 물체("bad" 또는 "part") 발견 시 좌표 계산
                        if "bad" in class_name or "part" in class_name: 
                            obb_box = r.obb.xywhr[i].cpu().numpy()
                            # 중심점 좌표 (float -> int 반올림)
                            u, v = int(round(obb_box[0])), int(round(obb_box[1]))
                            
                            # [핵심] 개선된 좌표 계산 함수 호출
                            target_coords_camera = self.get_camera_coordinates(u, v)

            # 결과 문자열 발행
            self.result_publisher.publish(String(data=",".join(detected_classes)))

            # 좌표 발행
            if target_coords_camera is not None:
                pos_msg = Float32MultiArray()
                pos_msg.data = target_coords_camera
                self.pose_publisher.publish(pos_msg)
                
                # 화면에 거리 표시
                text = f"X:{target_coords_camera[0]:.0f} Y:{target_coords_camera[1]:.0f} Z:{target_coords_camera[2]:.0f}"
                cv2.putText(cv_image, text, (u, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.circle(cv_image, (u, v), 5, (0, 0, 255), -1)

            # 결과 화면 출력
            annotated_frame = results[0].plot(conf=0.6)
            cv2.imshow("YOLO Inference", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Processing Error: {e}')

    def get_camera_coordinates(self, u, v):
        """
        픽셀(u, v)의 Depth를 안전하게 가져와 3D 좌표로 변환
        Detection.py의 _pixel_to_camera_coords 로직 적용 + 노이즈 필터링
        """
        if self.depth_image is None: return None

        # 1. 이미지 범위 체크
        h, w = self.depth_image.shape
        if u < 0 or u >= w or v < 0 or v >= h:
            return None

        # 2. [개선] 주변 픽셀(ROI)의 중앙값(Median) 사용 (노이즈 제거)
        roi_size = 5  # 5x5 영역 확인
        half_size = roi_size // 2
        
        u_min = max(0, u - half_size)
        u_max = min(w, u + half_size + 1)
        v_min = max(0, v - half_size)
        v_max = min(h, v + half_size + 1)

        roi = self.depth_image[v_min:v_max, u_min:u_max]
        
        # 0(유효하지 않은 값)을 제외한 값들만 추출
        valid_depths = roi[roi > 0]
        
        if len(valid_depths) == 0:
            return None # 유효한 깊이 정보가 없음

        # 중앙값 사용 (튀는 값 제거에 탁월)
        z_depth = np.median(valid_depths)

        # 3. 카메라 좌표 변환 공식 (detection.py 참조)
        # X = (u - cx) * Z / fx
        # Y = (v - cy) * Z / fy
        x_cam = (u - self.cx) * z_depth / self.fx
        y_cam = (v - self.cy) * z_depth / self.fy
        z_cam = z_depth 
        
        return [float(x_cam), float(y_cam), float(z_cam)]

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()