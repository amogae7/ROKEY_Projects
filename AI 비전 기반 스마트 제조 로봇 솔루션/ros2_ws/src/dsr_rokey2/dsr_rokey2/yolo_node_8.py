import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import json
from collections import deque  # [NEW] 3프레임 저장을 위한 큐
from ultralytics import YOLO
from rclpy.qos import qos_profile_sensor_data

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        try:
            self.model = YOLO(r'/home/rokey/ros2_ws/src/Tutorial/Calibration_Tutorial/data/please/rokey_obb_project/run_1/weights/best.pt', task='obb')
        except Exception as e:
            self.get_logger().error(f"모델 로드 실패: {e}")
            raise e

        self.result_publisher = self.create_publisher(String, '/yolo_results', 10)
        self.pose_publisher = self.create_publisher(Float32MultiArray, '/yolo_object_pos', 10)
        self.all_objects_publisher = self.create_publisher(String, '/yolo_all_detect', 10)
        
        self.create_subscription(String, '/part_n_bad_dispose', self.command_callback, 10)
        self.create_subscription(String, '/part_n_bad_show', self.command_callback, 10)
        self.create_subscription(String, '/target_update', self.command_callback, 10)

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
        
        # [NEW] 물체별로 최근 3프레임의 Depth 평균을 저장할 딕셔너리
        # 구조: {'part_1_bad': deque([z1, z2, z3], maxlen=3), ...}
        self.depth_buffers = {}
        
        self.get_logger().info("✅ YOLO Node v8 Ready (5-Point Spatial & 3-Frame Temporal Average)")

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

    # [수정] 좌표만 계산해서 반환하는 함수 (그리기용)
    def calculate_axis_points(self, cx, cy, length, angle_rad):
        dx = (length / 2) * math.cos(angle_rad)
        dy = (length / 2) * math.sin(angle_rad)
        p1 = (int(cx + dx), int(cy + dy))
        p2 = (int(cx - dx), int(cy - dy))
        return p1, p2

    # [NEW] 특정 (u,v) 좌표의 깊이값 하나만 쏙 빼오는 함수
    def get_pixel_depth(self, u, v):
        if self.depth_image is None: return None
        h, w = self.depth_image.shape
        
        # 이미지 범위 벗어나면 무시
        if u < 0 or u >= w or v < 0 or v >= h: return None
        
        # 해당 픽셀의 깊이값 (0이면 노이즈로 간주하고 무시)
        d = self.depth_image[v, u]
        if d > 0: return float(d)
        return None

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_image, verbose=False)
            
            detected_classes = []
            all_objects_data = [] 
            
            cv2.putText(cv_image, f"Target: {self.target_object}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

            for r in results:
                if r.obb is not None:
                    for i, c in enumerate(r.obb.cls):
                        if r.obb.conf[i] < 0.6: continue

                        class_name = self.model.names[int(c)]
                        detected_classes.append(class_name)
                        
                        # OBB 계산
                        obb_box = r.obb.xywhr[i].cpu().numpy()
                        cx, cy = int(round(obb_box[0])), int(round(obb_box[1]))
                        w, h = obb_box[2], obb_box[3]
                        angle_rad = obb_box[4]

                        if w > h:
                            long_len, short_len = w, h
                            long_angle, short_angle = angle_rad, angle_rad + (math.pi / 2)
                        else:
                            long_len, short_len = h, w
                            long_angle, short_angle = angle_rad + (math.pi / 2), angle_rad

                        # 1. 5개의 점 좌표 구하기 (중심, 긴축 양끝, 짧은축 양끝)
                        l_p1, l_p2 = self.calculate_axis_points(cx, cy, long_len * 0.8, long_angle) # 0.8은 끝부분 노이즈 방지용 안쪽
                        s_p1, s_p2 = self.calculate_axis_points(cx, cy, short_len * 0.8, short_angle)
                        
                        # (시각화)
                        cv2.line(cv_image, l_p1, l_p2, (0, 0, 255), 2)
                        cv2.line(cv_image, s_p1, s_p2, (255, 0, 0), 2)
                        # 5개 점 찍어보기
                        points_to_check = [(cx, cy), l_p1, l_p2, s_p1, s_p2]
                        for pt in points_to_check:
                            cv2.circle(cv_image, pt, 3, (0, 255, 255), -1)

                        # 2. 5개 점의 Depth 수집
                        valid_depths = []
                        for pt in points_to_check:
                            d = self.get_pixel_depth(pt[0], pt[1])
                            if d: valid_depths.append(d)
                        
                        if not valid_depths: continue # 깊이 정보가 하나도 없으면 패스

                        # 3. 이번 프레임의 평균 Depth (공간 평균)
                        current_frame_avg_depth = np.mean(valid_depths)

                        # 4. 3프레임 버퍼에 저장 (시간 평균)
                        if class_name not in self.depth_buffers:
                            self.depth_buffers[class_name] = deque(maxlen=3)
                        
                        self.depth_buffers[class_name].append(current_frame_avg_depth)
                        
                        # 5. 최종 Depth 계산 (버퍼 내 평균)
                        final_stable_depth = np.mean(self.depth_buffers[class_name])
                        
                        # 6. 3D 좌표 변환 (Z값은 위에서 구한 안정화된 값 사용)
                        x_cam = (cx - self.cx) * final_stable_depth / self.fx
                        y_cam = (cy - self.cy) * final_stable_depth / self.fy
                        angle_deg = np.degrees(short_angle)
                        
                        # 최종 좌표: [x, y, stabilized_z, angle]
                        final_coords = [float(x_cam), float(y_cam), float(final_stable_depth), float(angle_deg)]

                        # 데이터 패키징
                        all_objects_data.append({
                            "name": class_name,
                            "coords": final_coords
                        })
                        
                        if self.target_object in class_name:
                            # 디버깅: 화면에 Depth 표시
                            cv2.putText(cv_image, f"Z: {int(final_stable_depth)}mm", (cx, cy+20), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                            
                            pos_msg = Float32MultiArray()
                            pos_msg.data = final_coords
                            self.pose_publisher.publish(pos_msg)

            if all_objects_data:
                json_msg = String()
                json_msg.data = json.dumps(all_objects_data)
                self.all_objects_publisher.publish(json_msg)

            self.result_publisher.publish(String(data=",".join(detected_classes)))
            cv2.imshow("YOLO Inference", cv_image) 
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Processing Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()