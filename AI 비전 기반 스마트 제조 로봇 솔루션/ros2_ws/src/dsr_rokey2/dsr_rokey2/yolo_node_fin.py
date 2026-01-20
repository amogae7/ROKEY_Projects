import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Float32MultiArray
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
import json
from collections import deque
from ultralytics import YOLO
from rclpy.qos import qos_profile_sensor_data

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')
        
        # 1. 모델 로드
        try:
            self.model = YOLO(r'/home/rokey/ros2_ws/src/Tutorial/Calibration_Tutorial/data/assemble/rokey_obb_project/run_1/weights/best.pt', task='obb')
        except Exception as e:
            self.get_logger().error(f"모델 로드 실패: {e}")
            raise e
        
        # 2. 퍼블리셔 설정
        # [A] 기존 로봇 제어용 (개별 토픽) - Smart Manager가 이걸 구독함
        self.target_list = [
            'part_1_bad', 'part_1_good',
            'part_2_bad', 'part_2_good',
            'part_3_bad', 'part_3_good',
            'non_pass', 'pass'
        ]
        self.pubs = {}
        for name in self.target_list:
            topic_name = f'/{name}'
            self.pubs[name] = self.create_publisher(Float32MultiArray, topic_name, 10)

        # [B] 시각화 및 디버깅용 (통합 JSON & 타겟)
        self.pose_publisher = self.create_publisher(Float32MultiArray, '/yolo_object_pos', 10)
        self.all_objects_publisher = self.create_publisher(String, '/yolo_all_detect', 10)
        self.result_publisher = self.create_publisher(String, '/yolo_results', 10)

        # 3. 서브스크라이버
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
        
        self.target_object = "part_1_bad" # 디버깅용 타겟 표시
        self.depth_buffers = {} # 시간 평균용 버퍼

        self.get_logger().info("✅ YOLO Node Final Ready (Dual Publish Mode)")

    def info_callback(self, msg):
        if not self.is_intrinsics_received:
            self.fx = msg.k[0]; self.cx = msg.k[2]
            self.fy = msg.k[4]; self.cy = msg.k[5]
            self.is_intrinsics_received = True

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            pass

    # 축의 양 끝점 계산
    def calculate_axis_points(self, cx, cy, length, angle_rad):
        dx = (length / 2) * math.cos(angle_rad)
        dy = (length / 2) * math.sin(angle_rad)
        p1 = (int(cx + dx), int(cy + dy))
        p2 = (int(cx - dx), int(cy - dy))
        return p1, p2

    # 특정 픽셀 Depth 가져오기
    def get_pixel_depth(self, u, v):
        if self.depth_image is None: return None
        h, w = self.depth_image.shape
        if u < 0 or u >= w or v < 0 or v >= h: return None
        d = self.depth_image[v, u]
        if d > 0: return float(d)
        return None

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            results = self.model(cv_image, verbose=False)
            
            detected_classes = []
            all_objects_data = [] 

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

                        # 짧은 변 기준 정렬
                        if w > h:
                            long_len, short_len = w, h
                            long_angle, short_angle = angle_rad, angle_rad + (math.pi / 2)
                        else:
                            long_len, short_len = h, w
                            long_angle, short_angle = angle_rad + (math.pi / 2), angle_rad

                        # 1. 5-Point Sampling (중심 + 긴축 끝 + 짧은축 끝)
                        l_p1, l_p2 = self.calculate_axis_points(cx, cy, long_len * 0.8, long_angle)
                        s_p1, s_p2 = self.calculate_axis_points(cx, cy, short_len * 0.8, short_angle)
                        points_to_check = [(cx, cy), l_p1, l_p2, s_p1, s_p2]

                        # 2. Depth 수집 및 공간 평균
                        valid_depths = []
                        for pt in points_to_check:
                            d = self.get_pixel_depth(pt[0], pt[1])
                            if d: valid_depths.append(d)
                        
                        if not valid_depths: continue
                        current_frame_avg_depth = np.mean(valid_depths)

                        # 3. 시간 평균 (3프레임 버퍼)
                        if class_name not in self.depth_buffers:
                            self.depth_buffers[class_name] = deque(maxlen=5)
                        self.depth_buffers[class_name].append(current_frame_avg_depth)
                        final_stable_depth = np.mean(self.depth_buffers[class_name])
                        
                        # 4. 좌표 변환 (mm 단위 너비 포함)
                        x_cam = (cx - self.cx) * final_stable_depth / self.fx
                        y_cam = (cy - self.cy) * final_stable_depth / self.fy
                        angle_deg = np.degrees(short_angle) # 정욱 그리퍼 각도
                        anlge_deg_2 = np.degrees(long_angle) # 강엽 그리퍼 각도
                        width_mm = (short_len * final_stable_depth) / self.fx

                        # [x, y, z, angle, width] - 5개 데이터 패키징
                        final_coords = [float(x_cam), float(y_cam), float(final_stable_depth), float(angle_deg), float(width_mm)]
                        final_coords_2 = [float(x_cam), float(y_cam), float(final_stable_depth), float(anlge_deg_2), float(width_mm)]

                        # -------------------------------------------------------------
                        # [핵심] 두 가지 방식으로 모두 발행 (호환성 유지)
                        # -------------------------------------------------------------
                        # 강엽 
                        if class_name in self.pubs:
                            msg_data = Float32MultiArray()
                            msg_data.data = final_coords_2
                            self.pubs[class_name].publish(msg_data)

                        # 정욱
                        all_objects_data.append({
                            "name": class_name,
                            "coords": final_coords
                        })
                        
                        # -------------------------------------------------------------
                        # 시각화
                        # -------------------------------------------------------------
                        cv2.line(cv_image, l_p1, l_p2, (0, 0, 255), 2)
                        cv2.line(cv_image, s_p1, s_p2, (255, 0, 0), 2)
                        for pt in points_to_check:
                            cv2.circle(cv_image, pt, 3, (0, 255, 255), -1)

                        info_text = f"{class_name} Z:{int(final_stable_depth)} W:{int(width_mm)}"
                        cv2.putText(cv_image, info_text, (cx - 40, cy - 20), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

            # JSON 발행
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