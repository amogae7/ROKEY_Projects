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
            self.model = YOLO('/home/wook/ros2_ws/assemble/rokey_obb_project/run_1/weights/best.pt', task='obb')
        except Exception as e:
            self.get_logger().error(f"모델 로드 실패: {e}")
            raise e

        # 2. 퍼블리셔 설정 (요청하신 6개 토픽 고정 생성)
        # 전체 결과 문자열 발행용
        self.result_publisher = self.create_publisher(String, '/yolo_results', 10)
        
        # [NEW] 물체별 전용 퍼블리셔 생성
        self.target_list = [
            'part_1_bad', 'part_1_good',
            'part_2_bad', 'part_2_good',
            'part_3_bad', 'part_3_good'
        ]
        
        self.pubs = {}
        for name in self.target_list:
            topic_name = f'/{name}'  # 예: /part_1_bad
            self.pubs[name] = self.create_publisher(Float32MultiArray, topic_name, 10)
            self.get_logger().info(f"Topic Created: {topic_name}")

        # 3. 서브스크라이버
        self.sub_color = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, qos_profile_sensor_data)
        self.sub_depth = self.create_subscription(
            Image, '/camera/camera/aligned_depth_to_color/image_raw', self.depth_callback, qos_profile_sensor_data)
        self.sub_info = self.create_subscription(
            CameraInfo, '/camera/camera/aligned_depth_to_color/camera_info', self.info_callback, qos_profile_sensor_data)
        
        self.bridge = CvBridge()
        self.depth_image = None
        
        # 카메라 파라미터 초기값
        self.fx = 605.0
        self.fy = 605.0
        self.cx = 320.0
        self.cy = 240.0
        self.is_intrinsics_received = False

        self.get_logger().info("YOLO Node (Fixed Topics) Started.")

    def info_callback(self, msg):
        if not self.is_intrinsics_received:
            self.fx = msg.k[0]
            self.cx = msg.k[2]
            self.fy = msg.k[4]
            self.cy = msg.k[5]
            self.is_intrinsics_received = True
            self.get_logger().info(f"✅ Intrinsics Updated: fx={self.fx:.1f}, cx={self.cx:.1f}")

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

            for r in results:
                if r.obb is not None:
                    for i, c in enumerate(r.obb.cls):
                        conf = r.obb.conf[i]
                        # 신뢰도 60% 미만 무시
                        if conf < 0.6:
                            continue

                        class_name = self.model.names[int(c)]
                        detected_classes.append(class_name)
                        
                        # [핵심] 타겟 리스트에 있는 물체인지 확인
                        if class_name in self.pubs:
                            
                            # 좌표 계산
                            obb_box = r.obb.xywhr[i].cpu().numpy()
                            u, v = int(round(obb_box[0])), int(round(obb_box[1]))
                            
                            coords = self.get_camera_coordinates(u, v)
                            
                            if coords is not None:
                                # [핵심] 해당 물체의 전용 토픽으로 발행
                                msg_data = Float32MultiArray()
                                msg_data.data = coords # [x, y, z]
                                self.pubs[class_name].publish(msg_data)
                                
                                # 화면 표시
                                text = f"{class_name} ({coords[0]:.0f}, {coords[1]:.0f})"
                                cv2.putText(cv_image, text, (u, v - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                cv2.circle(cv_image, (u, v), 5, (0, 0, 255), -1)

            # 전체 감지 목록 발행
            self.result_publisher.publish(String(data=",".join(detected_classes)))

            # 결과 화면 출력
            annotated_frame = results[0].plot(conf=0.6)
            cv2.imshow("YOLO Inference", annotated_frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Processing Error: {e}')

    def get_camera_coordinates(self, u, v):
        if self.depth_image is None: return None

        h, w = self.depth_image.shape
        if u < 0 or u >= w or v < 0 or v >= h:
            return None

        # Median Depth Filter (노이즈 제거)
        roi_size = 5
        half_size = roi_size // 2
        u_min = max(0, u - half_size)
        u_max = min(w, u + half_size + 1)
        v_min = max(0, v - half_size)
        v_max = min(h, v + half_size + 1)

        roi = self.depth_image[v_min:v_max, u_min:u_max]
        valid_depths = roi[roi > 0]
        
        if len(valid_depths) == 0:
            return None 

        z_depth = np.median(valid_depths)

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