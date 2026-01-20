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
            # [경로 확인] 본인의 모델 경로로 수정하세요
            self.model = YOLO(r'/home/rokey/ros2_ws/src/Tutorial/Calibration_Tutorial/data/please/rokey_obb_project/run_1/weights/best.pt', task='obb')
        except Exception as e:
            self.get_logger().error(f"모델 로드 실패: {e}")
            raise e

        # 2. 퍼블리셔 & 서브스크라이버
        self.result_publisher = self.create_publisher(String, '/yolo_results', 10)
        self.pose_publisher = self.create_publisher(Float32MultiArray, '/yolo_object_pos', 10)
        
        # [NEW] YOLO도 이제 "뭐 찾을지" 명령을 듣습니다.
        self.create_subscription(String, '/part_n_bad_dispose', self.command_callback, 10)
        self.create_subscription(String, '/part_n_bad_show', self.command_callback, 10)

        # 카메라 구독
        self.sub_color = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, qos_profile_sensor_data)
        self.sub_depth = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, qos_profile_sensor_data)
        self.sub_info = self.create_subscription(
            CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.info_callback, qos_profile_sensor_data)
        
        self.bridge = CvBridge()
        self.depth_image = None
        
        self.fx = 605.0; self.fy = 605.0; self.cx = 320.0; self.cy = 240.0
        self.is_intrinsics_received = False
        
        # [핵심] 현재 찾고 있는 목표물 (기본값: 1번)
        self.target_object = "part_1_bad"

        self.get_logger().info("✅ YOLO Node Started (Selective Mode)")

    def command_callback(self, msg):
        """Smart Manager가 '2번 찾아'라고 하면 타겟을 바꿉니다"""
        self.target_object = msg.data
        self.get_logger().info(f"🎯 타겟 변경됨: {self.target_object}")

    def info_callback(self, msg):
        if not self.is_intrinsics_received:
            self.fx = msg.k[0]; self.cx = msg.k[2]
            self.fy = msg.k[4]; self.cy = msg.k[5]
            self.is_intrinsics_received = True

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except: pass

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_image, verbose=False)
            
            detected_classes = []
            final_coords = None 
            
            # [디버깅용] 화면에 현재 타겟 표시
            cv2.putText(cv_image, f"Target: {self.target_object}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

            for r in results:
                if r.obb is not None:
                    for i, c in enumerate(r.obb.cls):
                        if r.obb.conf[i] < 0.6: continue

                        class_name = self.model.names[int(c)]
                        detected_classes.append(class_name)
                        
                        # [핵심 수정] "지금 찾고 있는 타겟"과 이름이 같을 때만 좌표를 계산합니다!
                        # (부분 일치 허용: target="part_1_bad" 이면 "part_1_bad" 찾음)
                        if self.target_object in class_name: 
                            obb_box = r.obb.xywhr[i].cpu().numpy()
                            u, v = int(round(obb_box[0])), int(round(obb_box[1]))
                            angle_rad = obb_box[4]
                            
                            # 좌표 계산 (XYZ + Angle)
                            final_coords = self.get_camera_coordinates(u, v, angle_rad)
                            
                            # 화면에 박스 그리기 (찾은 놈만 초록색)
                            if final_coords:
                                cv2.circle(cv_image, (u, v), 8, (0, 255, 0), -1)
                                cv2.putText(cv_image, "FOUND!", (u, v-20), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            self.result_publisher.publish(String(data=",".join(detected_classes)))

            # 좌표 발행 (찾은 놈이 있을 때만!)
            if final_coords is not None:
                pos_msg = Float32MultiArray()
                pos_msg.data = final_coords
                self.pose_publisher.publish(pos_msg)
            
            # 화면 출력
            annotated_frame = results[0].plot(conf=0.6)
            # 타겟 표시가 덮어씌워질 수 있어서 다시 그림
            cv2.imshow("YOLO Inference", cv_image) # annotated_frame 대신 직접 그린 cv_image 사용 권장
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Processing Error: {e}')

    def get_camera_coordinates(self, u, v, angle_rad=0.0):
        if self.depth_image is None: return None
        h, w = self.depth_image.shape
        if u < 0 or u >= w or v < 0 or v >= h: return None

        roi_size = 5
        u_min = max(0, u - roi_size//2); u_max = min(w, u + roi_size//2 + 1)
        v_min = max(0, v - roi_size//2); v_max = min(h, v + roi_size//2 + 1)
        roi = self.depth_image[v_min:v_max, u_min:u_max]
        valid_depths = roi[roi > 0]
        
        if len(valid_depths) == 0: return None
        z_depth = np.median(valid_depths)

        x_cam = (u - self.cx) * z_depth / self.fx
        y_cam = (v - self.cy) * z_depth / self.fy
        
        # Radian -> Degree 변환
        angle_deg = np.degrees(angle_rad)
        
        return [float(x_cam), float(y_cam), float(z_depth), float(angle_deg)]

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()