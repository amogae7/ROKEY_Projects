#!/usr/bin/env python3
from ultralytics import YOLO
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


class YoloCameraPublisher(Node):
    def __init__(self):
        super().__init__('yolo_camera_publisher')

        # --- ROS 파라미터 ---
        self.declare_parameter('camera_index', 1)
        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('fps', 20.0)
        self.declare_parameter('topic_raw', 'camera/image_raw')
        self.declare_parameter('topic_annotated', 'camera/image_annotated')
        self.declare_parameter('topic_result', 'yolo/result_text')

        camera_index = self.get_parameter('camera_index').value
        model_path = self.get_parameter('model_path').value
        fps = self.get_parameter('fps').value
        topic_raw = self.get_parameter('topic_raw').value
        topic_annotated = self.get_parameter('topic_annotated').value
        topic_result = self.get_parameter('topic_result').value

        self.get_logger().info(f'Using camera index: {camera_index}')
        self.get_logger().info(f'Loading YOLO model: {model_path}')

        # YOLO 모델 로드
        self.model = YOLO(model_path)

        # 카메라 오픈
        self.cap = cv2.VideoCapture(camera_index)
        if not self.cap.isOpened():
            self.get_logger().error(
                f'Cannot open camera index {camera_index}. Check device!'
            )
            raise RuntimeError('Camera open failed')

        # Publisher 설정
        self.bridge = CvBridge()
        self.pub_raw = self.create_publisher(Image, topic_raw, 10)
        self.pub_annotated = self.create_publisher(Image, topic_annotated, 10)
        self.pub_result = self.create_publisher(String, topic_result, 10)
        self.pub_car_detect = self.create_publisher(String, '/car_detected', 10)

        # car / dummy 중심 좌표 publish
        self.pub_car_center = self.create_publisher(String, '/car/center_pixel', 10)
        self.pub_dummy_center = self.create_publisher(String, '/dummy/center_pixel', 10)

        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame from camera')
            return

        # -----------------------------
        # 0) 프레임 정보 및 중앙 사각형 설정
        # -----------------------------
        h, w, _ = frame.shape
        cx_frame = w // 2
        cy_frame = h // 2

        box_size = 400  # 임의의 크기 (원하면 조절 가능)
        half = box_size // 2

        roi_x1 = max(cx_frame - half, 0)
        roi_y1 = max(cy_frame - half, 0)
        roi_x2 = min(cx_frame + half, w - 1)
        roi_y2 = min(cy_frame + half, h - 1)

        # 기본은 파란색(BGR)
        rect_color = (255, 0, 0)

        # 1) 원본 이미지 publish
        msg_raw = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.pub_raw.publish(msg_raw)

        # 2) YOLO 추론
        results = self.model(frame)  # list[Results]
        res = results[0]

        names = res.names  # dict: {class_id: class_name}
        boxes = res.boxes

        class_ids = boxes.cls.tolist() if boxes is not None else []
        class_ids = [int(c) for c in class_ids]

        msg_car_detect = String()
        msg_car_detect.data = 'detected'

        car_centers = []
        dummy_centers = []

        car_in_roi = False  # ROI 안에 car가 들어왔는지 여부

        if boxes is not None and len(boxes) > 0:
            # xyxy: [x1, y1, x2, y2] 형태
            xyxy_list = boxes.xyxy.cpu().tolist()

            for cls_id, (x1, y1, x2, y2) in zip(class_ids, xyxy_list):
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                # 0: car, 1: dummy 라고 가정
                if cls_id == 0:  # car
                    car_centers.append((cx, cy))
                    # car 중심이 중앙 사각형(ROI) 안에 들어왔는지 체크
                    if roi_x1 <= cx <= roi_x2 and roi_y1 <= cy <= roi_y2:
                        car_in_roi = True
                elif cls_id == 1:  # dummy
                    dummy_centers.append((cx, cy))

        # ROI 안에 car가 하나라도 있으면:
        if car_in_roi:
            # 사각형 색을 빨간색으로 변경
            rect_color = (0, 0, 255)
            # /car_detected publish
            self.pub_car_detect.publish(msg_car_detect)
            self.get_logger().info('in the box!')
        # 3) annotated frame 만들기 (YOLO가 그려주는 박스 포함)
        annotated_frame = res.plot()

        # 4) 중앙 사각형 그리기 (car_in_roi 여부에 따라 색 다름)
        cv2.rectangle(
            annotated_frame,
            (roi_x1, roi_y1),
            (roi_x2, roi_y2),
            rect_color,
            2
        )

        # 5) car / dummy 중심 좌표 publish (첫 번째 것만 사용)
        if car_centers:
            cx, cy = car_centers[0]
            msg = String()
            msg.data = f"{cx}, {cy}"
            self.pub_car_center.publish(msg)

        if dummy_centers:
            cx, cy = dummy_centers[0]
            msg = String()
            msg.data = f"{cx}, {cy}"
            self.pub_dummy_center.publish(msg)

        # 6) 텍스트 결과 (Detected: ...)
        if len(class_ids) > 0:
            class_names = [names[c] for c in class_ids]
            unique_names = sorted(set(class_names))
            text = f"Detected: {', '.join(unique_names)}"
        else:
            text = "Detected: none"

        msg_result = String()
        msg_result.data = text
        self.pub_result.publish(msg_result)

        # 7) annotated 이미지 publish
        msg_annotated = self.bridge.cv2_to_imgmsg(
            annotated_frame, encoding='bgr8'
        )
        self.pub_annotated.publish(msg_annotated)

    def destroy_node(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
