import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from ultralytics import YOLO
from rclpy.qos import qos_profile_sensor_data

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        # 1. 모델 설정
        try:
            # [경로 확인] 본인의 모델 경로로 유지하세요
            self.model = YOLO(r'/home/rokey/ros2_ws/src/Tutorial/Calibration_Tutorial/data/please/rokey_obb_project/run_1/weights/best.pt', task='obb')
        except Exception as e:
            self.get_logger().error(f"모델 로드 실패: {e}")
            raise e

        # 2. 퍼블리셔 & 서브스크라이버
        self.result_publisher = self.create_publisher(String, '/yolo_results', 10)
        self.pose_publisher = self.create_publisher(Float32MultiArray, '/yolo_object_pos', 10)
        
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
        
        self.target_object = "part_1_bad"

        self.get_logger().info("✅ YOLO Node Started (Angle: Short Axis Mode)")

    def command_callback(self, msg):
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

    # [헬퍼 함수] 선 끝점 계산
    def get_axis_point(self, cx, cy, length, angle_rad):
        dx = (length / 2) * math.cos(angle_rad)
        dy = (length / 2) * math.sin(angle_rad)
        return (int(cx + dx), int(cy + dy)), (int(cx - dx), int(cy - dy))

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_image, verbose=False)
            
            detected_classes = []
            final_coords = None 
            
            cv2.putText(cv_image, f"Target: {self.target_object}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

            for r in results:
                if r.obb is not None:
                    for i, c in enumerate(r.obb.cls):
                        if r.obb.conf[i] < 0.6: continue

                        class_name = self.model.names[int(c)]
                        detected_classes.append(class_name)
                        
                        if self.target_object in class_name: 
                            obb_box = r.obb.xywhr[i].cpu().numpy()
                            
                            cx, cy = int(round(obb_box[0])), int(round(obb_box[1]))
                            w, h = obb_box[2], obb_box[3]
                            angle_rad = obb_box[4]

                            # ---------------------------------------------------------
                            # [핵심 로직] 긴 축 / 짧은 축 계산
                            # ---------------------------------------------------------
                            if w > h:
                                long_len, short_len = w, h
                                long_angle, short_angle = angle_rad, angle_rad + (math.pi / 2)
                            else:
                                long_len, short_len = h, w
                                long_angle, short_angle = angle_rad + (math.pi / 2), angle_rad

                            # 시각화용 선 그리기
                            l_p1, l_p2 = self.get_axis_point(cx, cy, long_len, long_angle)
                            s_p1, s_p2 = self.get_axis_point(cx, cy, short_len, short_angle)

                            cv2.line(cv_image, l_p1, l_p2, (0, 0, 255), 2)   # 빨강: 긴 축
                            cv2.line(cv_image, s_p1, s_p2, (255, 0, 0), 2)   # 파랑: 짧은 축 (그리퍼 방향)
                            # ---------------------------------------------------------

                            # [수정됨] 로봇에게 '파란 선(short_angle)'의 각도를 보냅니다!
                            final_coords = self.get_camera_coordinates(cx, cy, short_angle)
                            
                            if final_coords:
                                cv2.circle(cv_image, (cx, cy), 8, (0, 255, 0), -1)
                                cv2.putText(cv_image, "FOUND!", (cx, cy-20), 
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

            self.result_publisher.publish(String(data=",".join(detected_classes)))

            if final_coords is not None:
                pos_msg = Float32MultiArray()
                pos_msg.data = final_coords
                self.pose_publisher.publish(pos_msg)
            
            cv2.imshow("YOLO Inference", cv_image) 
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