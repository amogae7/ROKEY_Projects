import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from od_msg.srv import SrvDepthPosition
import cv2
import numpy as np
import time

class ArucoDetectionNode(Node):
    def __init__(self):
        super().__init__('aruco_detection_node')
        
        # ==========================================
        # [설정] 마커 크기 (미터 단위) - 꼭 수정하세요!
        # ==========================================
        self.MARKER_SIZE = 0.1  # 예: 5cm = 0.05m
        
        # 아루코 사전 설정 (4x4_50, 5x5_100 등 본인 마커에 맞게)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()

        # 카메라 매트릭스 (Intrinsic) - 처음엔 비어있음
        self.camera_matrix = None
        self.dist_coeffs = None
        
        self.bridge = CvBridge()
        self.cv_image = None

        # 구독 & 서비스
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        
        # 기존 detection_node와 똑같은 이름의 서비스 제공 (로봇 코드를 안 고쳐도 됨!)
        self.create_service(SrvDepthPosition, 'get_3d_position', self.handle_get_position)
        
        self.get_logger().info("🏁 Aruco Detection Node Started! (Waiting for Camera Info...)")

    def info_callback(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info("✅ 카메라 정보 수신 완료 (Intrinsics Loaded)")

    def image_callback(self, msg):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Image Error: {e}")

    def handle_get_position(self, request, response):
        # 로봇이 "좌표 내놔" 하면 실행됨
        target_name = request.target # (사실 아루코라 이름은 상관없지만 로그용)
        self.get_logger().info(f"요청 받음: {target_name} -> 아루코 마커 찾는 중...")

        if self.cv_image is None or self.camera_matrix is None:
            self.get_logger().warn("아직 카메라 데이터가 없습니다.")
            return response

        # 1. 아루코 감지
        corners, ids, rejected = cv2.aruco.detectMarkers(
            self.cv_image, self.aruco_dict, parameters=self.aruco_params
        )

        if ids is not None and len(ids) > 0:
            # 2. 포즈 추정 (SolvePnP) -> 깊이 센서 없이 수학으로 거리 계산
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, self.MARKER_SIZE, self.camera_matrix, self.dist_coeffs
            )
            
            # 첫 번째 마커만 사용 (ID 0번 등 특정 ID를 원하면 조건문 추가 가능)
            tvec = tvecs[0][0] # [x, y, z]
            
            # 3. 좌표 반환
            # tvec[0] = x (오른쪽), tvec[1] = y (아래), tvec[2] = z (앞, 거리)
            x, y, z = tvec[0], tvec[1], tvec[2]
            
            self.get_logger().info(f"📍 마커 발견! (ID: {ids[0][0]})")
            self.get_logger().info(f"   => Camera Coords: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")
            
            response.depth_position = [float(x), float(y), float(z)]
            
            # (디버깅용) 화면에 축 그리기
            cv2.drawFrameAxes(self.cv_image, self.camera_matrix, self.dist_coeffs, rvecs[0], tvecs[0], 0.03)
            cv2.aruco.drawDetectedMarkers(self.cv_image, corners, ids)
            cv2.imshow("Aruco Debug", self.cv_image)
            cv2.waitKey(1)
            
        else:
            self.get_logger().warn("❌ 마커를 못 찾았습니다.")
            response.depth_position = [0.0, 0.0, 0.0]

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()