import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Float32MultiArray
from visualization_msgs.msg import Marker # [추가됨] 마커 메시지
from geometry_msgs.msg import Point, Quaternion # [추가됨] 위치/회전 메시지
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
        
        # 1. 모델 로드 (경로는 본인 환경에 맞게 유지)
        try:
            self.model = YOLO(r'/home/rokey/ros2_ws/src/Tutorial/Calibration_Tutorial/data/please/rokey_obb_project/run_1/weights/best.pt', task='obb')
        except Exception as e:
            self.get_logger().error(f"모델 로드 실패: {e}")
            raise e
        
        # 2. 퍼블리셔 설정
        self.target_list = [
            'part_1_bad', 'part_1_good',
            'part_2_bad', 'part_2_good',
            'part_3_bad', 'part_3_good'
        ]
        self.pubs = {}
        for name in self.target_list:
            topic_name = f'/{name}'
            self.pubs[name] = self.create_publisher(Float32MultiArray, topic_name, 10)

        self.pose_publisher = self.create_publisher(Float32MultiArray, '/yolo_object_pos', 10)
        self.all_objects_publisher = self.create_publisher(String, '/yolo_all_detect', 10)
        self.result_publisher = self.create_publisher(String, '/yolo_results', 10)

        # [추가됨] Rviz 시각화용 마커 퍼블리셔
        self.marker_pub = self.create_publisher(Marker, '/yolo_marker', 10)

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
        self.depth_buffers = {}
        self.get_logger().info("✅ YOLO Node Ready with Visualization!")

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

    def calculate_axis_points(self, cx, cy, length, angle_rad):
        dx = (length / 2) * math.cos(angle_rad)
        dy = (length / 2) * math.sin(angle_rad)
        p1 = (int(cx + dx), int(cy + dy))
        p2 = (int(cx - dx), int(cy - dy))
        return p1, p2

    def get_pixel_depth(self, u, v):
        if self.depth_image is None: return None
        h, w = self.depth_image.shape
        if u < 0 or u >= w or v < 0 or v >= h: return None
        d = self.depth_image[v, u]
        if d > 0: return float(d)
        return None

    # [추가됨] 각도(Euler)를 쿼터니언으로 변환하는 함수
    def get_quaternion_from_euler(self, roll, pitch, yaw):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    # [추가됨] 마커 발행 함수
    def publish_marker(self, x_mm, y_mm, z_mm, angle_rad, class_name, id_num):
        marker = Marker()
        # 중요: 카메라는 optical frame 기준으로 좌표를 계산하므로 frame_id를 카메라로 설정
        marker.header.frame_id = "camera_color_optical_frame" 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "detected_objects"
        marker.id = id_num
        marker.type = Marker.CUBE # 상자 모양
        marker.action = Marker.ADD

        # 위치 (mm -> meter 변환 필수!)
        marker.pose.position.x = x_mm / 1000.0
        marker.pose.position.y = y_mm / 1000.0
        marker.pose.position.z = z_mm / 1000.0

        # 회전 (YOLO 각도 반영)
        # 카메라는 Z가 앞, X가 오른쪽, Y가 아래입니다.
        # 물체의 회전은 카메라 축 기준 Z축 회전으로 표현됩니다.
        marker.pose.orientation = self.get_quaternion_from_euler(0, 0, angle_rad)

        # 크기 (미터 단위, 적당히 5cm 박스로 설정)
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.02 # 두께

        # 색상 (Good: 초록, Bad: 빨강)
        marker.color.a = 0.8 # 투명도
        if "bad" in class_name:
            marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0
        else:
            marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0

        self.marker_pub.publish(marker)

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

                        # ... (Depth 계산 부분은 기존과 동일) ...
                        l_p1, l_p2 = self.calculate_axis_points(cx, cy, long_len * 0.8, long_angle)
                        s_p1, s_p2 = self.calculate_axis_points(cx, cy, short_len * 0.8, short_angle)
                        points_to_check = [(cx, cy), l_p1, l_p2, s_p1, s_p2]

                        valid_depths = []
                        for pt in points_to_check:
                            d = self.get_pixel_depth(pt[0], pt[1])
                            if d: valid_depths.append(d)
                        
                        if not valid_depths: continue
                        current_frame_avg_depth = np.mean(valid_depths)

                        if class_name not in self.depth_buffers:
                            self.depth_buffers[class_name] = deque(maxlen=3)
                        self.depth_buffers[class_name].append(current_frame_avg_depth)
                        final_stable_depth = np.mean(self.depth_buffers[class_name])
                        
                        # 좌표 변환 (Camera Coordinate in mm)
                        x_cam = (cx - self.cx) * final_stable_depth / self.fx
                        y_cam = (cy - self.cy) * final_stable_depth / self.fy
                        angle_deg = np.degrees(short_angle)
                        anlge_deg_2 = np.degrees(long_angle)
                        width_mm = (short_len * final_stable_depth) / self.fx

                        final_coords = [float(x_cam), float(y_cam), float(final_stable_depth), float(angle_deg), float(width_mm)]
                        final_coords_2 = [float(x_cam), float(y_cam), float(final_stable_depth), float(anlge_deg_2), float(width_mm)]

                        # [발행 1] 로봇 제어용
                        if class_name in self.pubs:
                            msg_data = Float32MultiArray()
                            msg_data.data = final_coords_2
                            self.pubs[class_name].publish(msg_data)

                        # [발행 2] 통합 데이터
                        all_objects_data.append({"name": class_name, "coords": final_coords})

                        # [발행 3] ★★★ Rviz 시각화 마커 발행 ★★★
                        # YOLO 인식 결과가 나오면 바로 Rviz 화면에 3D 박스를 띄웁니다.
                        self.publish_marker(x_cam, y_cam, final_stable_depth, short_angle, class_name, i)

                        # OpenCV 시각화 (기존)
                        cv2.line(cv_image, l_p1, l_p2, (0, 0, 255), 2)
                        cv2.line(cv_image, s_p1, s_p2, (255, 0, 0), 2)
                        cv2.putText(cv_image, f"{class_name}", (cx - 40, cy - 20), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

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