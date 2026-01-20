import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import cv2
from cv_bridge import CvBridge
from std_msgs.msg import String, Float32
import json

# ================================
# 설정 상수
# ================================
DEPTH_TOPIC = '/robot5/oakd/stereo/image_raw'  # Depth 이미지 토픽
CAMERA_INFO_TOPIC = '/robot5/oakd/stereo/camera_info'  # CameraInfo 토픽
MAX_DEPTH_METERS = 5.0                 # 시각화 시 최대 깊이 값 (m)
NORMALIZE_DEPTH_RANGE = 3.0            # 시각화 정규화 범위 (m)
WINDOW_NAME = 'Depth Image (Click to get distance)'
# ================================

class DepthChecker(Node):
    def __init__(self):
        super().__init__('depth_checker')
        self.bridge = CvBridge()
        self.K = None
        self.should_exit = False
        self.depth_mm = None  # 최신 depth 이미지 저장
        self.depth_colored = None  # 시각화 이미지 저장

        self.subscription = self.create_subscription(
            Image,
            DEPTH_TOPIC,
            self.depth_callback,
            10)

        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            CAMERA_INFO_TOPIC,
            self.camera_info_callback,
            10)

                # YOLO에서 보내는 bbox 중심 좌표(JSON) 구독
        self.bbox_subscription = self.create_subscription(
            String,
            'yolo/bbox_centers',   # yolo_oakd_subscriber에서 publish 하는 토픽 이름
            self.bbox_callback,
            10
        )
        self.last_detections = []  # 최근 YOLO 탐지 결과 저장

        self.distance_pub = self.create_publisher(Float32, 'car_distance_m', 10)
        self.get_logger().info("📏 /car_distance_m: center depth (m) will be published.")



        # OpenCV 마우스 콜백 설정
        cv2.namedWindow(WINDOW_NAME)
        cv2.setMouseCallback(WINDOW_NAME, self.mouse_callback)

    def depth_callback(self, msg: Image):
        if self.should_exit:
            return

        if self.K is None:
            self.get_logger().warn('Waiting for CameraInfo...')
            return

        # ROS Image → depth(mm) numpy
        self.depth_mm = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        height, width = self.depth_mm.shape

        # 시각화용 이미지 생성
        depth_vis = np.nan_to_num(self.depth_mm, nan=0.0)
        depth_vis = np.clip(depth_vis, 0, NORMALIZE_DEPTH_RANGE * 1000)
        depth_vis = (depth_vis / (NORMALIZE_DEPTH_RANGE * 1000) * 255).astype(np.uint8)
        self.depth_colored = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)

        # 카메라 중심점 표시
        cx = int(self.K[0, 2])
        cy = int(self.K[1, 2])
        cv2.circle(self.depth_colored, (cx, cy), 5, (0, 0, 0), -1)
        cv2.line(self.depth_colored, (0, cy), (width, cy), (0, 0, 0), 1)
        cv2.line(self.depth_colored, (cx, 0), (cx, height), (0, 0, 0), 1)

        cv2.imshow(WINDOW_NAME, self.depth_colored)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.should_exit = True

    # ----------------------------

    def camera_info_callback(self, msg):
        if self.K is None:
            self.K = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(f"CameraInfo received: fx={self.K[0,0]:.2f}, fy={self.K[1,1]:.2f}, cx={self.K[0,2]:.2f}, cy={self.K[1,2]:.2f}")

    def bbox_callback(self, msg: String):
        # YOLO에서 넘어오는 JSON 문자열 파싱
        try:
            detections = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse bbox_centers JSON: {e}")
            return

        if self.depth_mm is None:
            self.get_logger().warn("Depth frame not ready yet, cannot compute distance.")
            return

        if not detections:
            return

        # 일단 첫 번째 물체로 예시 (여러개 처리 가능)
        det = detections[0]

        cx_preview = float(det['center']['x'])
        cy_preview = float(det['center']['y'])

        x1_preview = float(det['bbox']['x1'])
        y1_preview = float(det['bbox']['y1'])
        x2_preview = float(det['bbox']['x2'])
        y2_preview = float(det['bbox']['y2'])


        class_name = det['class']
        confidence = det['confidence']

        # Depth 이미지 해상도
        h_d, w_d = self.depth_mm.shape

        # 🔹 프리뷰(352) → 뎁스(704) 스케일 변환
        #   정확히 2배라면 w_d/352, h_d/352 가 2가 됨
        scale_x = w_d / 352.0
        scale_y = h_d / 352.0

        cx = int(cx_preview * scale_x)
        cy = int(cy_preview * scale_y)
        x1 = int(x1_preview * scale_x)
        x2 = int(x2_preview * scale_x)
        y1 = int(y1_preview * scale_y)
        y2 = int(y2_preview * scale_y)

        # 범위 체크
        if not (0 <= cx < w_d and 0 <= cy < h_d):
            self.get_logger().warn(
                f"Scaled center ({cx},{cy}) is outside depth image size ({w_d},{h_d})."
            )
            return
        # 범위 체크
        # h, w = self.depth_mm.shape
        # if not (0 <= cx < w and 0 <= cy < h):
        #     self.get_logger().warn("Center is outside the depth image.")
        #     return

        # ① 중심 깊이
        center_depth_mm = float(self.depth_mm[cy, cx])
        center_depth_m = center_depth_mm / 1000.0

        if center_depth_mm > 0:
            msg = Float32()
            msg.data = center_depth_m
            self.distance_pub.publish(msg)
            self.get_logger().info(f"📤 Published car_distance_m = {center_depth_m:.3f} m")
        else:
            self.get_logger().warn("Center depth is 0 or invalid, not publishing car_distance_m.")


        # ② 바운딩박스 최소/평균 깊이
        y1_clamped = max(0, min(h_d - 1, y1))
        y2_clamped = max(0, min(h_d - 1, y2))
        x1_clamped = max(0, min(w_d - 1, x1))
        x2_clamped = max(0, min(w_d - 1, x2))

        roi = self.depth_mm[y1_clamped:y2_clamped, x1_clamped:x2_clamped]
        roi_valid = roi[roi > 0]

        if roi_valid.size > 0:
            min_depth_mm = float(roi_valid.min())
            min_depth_m = min_depth_mm / 1000.0

            mean_depth_mm = float(roi_valid.mean())
            mean_depth_m = mean_depth_mm / 1000.0
        else:
            min_depth_m = float('nan')
            mean_depth_m = float('nan')

        # 출력
        self.get_logger().info(
            f"\n[YOLO + DEPTH]"
            f"\n - Object: {class_name} ({confidence})"
            f"\n - Center (@{cx},{cy}) depth: {center_depth_m:.3f} m"
            f"\n - Min depth in bbox: {min_depth_m:.3f} m"
            f"\n - Mean depth in bbox: {mean_depth_m:.3f} m\n"
        )

            # ----------------------------
            # 마우스로 클릭해서 거리 확인
            # ----------------------------
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.depth_mm is not None:
            distance_mm = float(self.depth_mm[y, x])
            distance_m = distance_mm / 1000.0  # mm → m
            self.get_logger().info(
                f"Clicked at (u={x}, v={y}) → Distance = {distance_m:.3f} meters"
            )





def main():
    rclpy.init()
    node = DepthChecker()

    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
