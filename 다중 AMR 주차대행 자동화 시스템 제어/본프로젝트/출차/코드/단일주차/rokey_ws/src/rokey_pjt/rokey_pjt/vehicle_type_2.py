#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch

class YoloVehicleDetectionNode(Node):
    def __init__(self):
        super().__init__('vehicle_detection_node')

        # YOLO 모델 경로 설정
        model_path = "/home/rokey/rokey_ws/src/rokey_pjt/rokey_pjt/(yolov8n_640,100)best.pt"
        self.model = YOLO(model_path)

        # 레이블 맵 설정
        self.label_map = {
            'big': 'C',
            'mid': 'B',
            'small': 'A'
        }

        # 상태 관리 변수
        self.last_detected_label = None

        # CUDA 설정
        if torch.cuda.is_available():
            self.model.to('cuda')
            self.get_logger().info("✅ CUDA 사용")
        else:
            self.get_logger().info("⚠️ CPU 사용")

        self.bridge = CvBridge()

        # 활성화/비활성화 토픽
        self.enabled = True

        # 발행: 차량 중심 좌표, 라벨, 디버그 이미지
        self.center_pub = self.create_publisher(Point, '/vehicle_type', 10)
        self.label_pub = self.create_publisher(String, '/vehicle_label', 10)
        self.image_pub = self.create_publisher(Image, '/yolo/debug_image', 10)

        self.get_logger().info("=" * 60)
        self.get_logger().info("🎯 YOLO Vehicle Detection Node Started")
        self.get_logger().info(f"   - 매핑 규칙: {self.label_map}")
        self.get_logger().info("=" * 60)

        # 웹캠 열기 (기본 웹캠 0번)
        self.cap = cv2.VideoCapture(2)
        if not self.cap.isOpened():
            self.get_logger().error("웹캠을 열 수 없습니다.")
            return

        # 구독: 활성화/비활성화 토픽
        self.enable_sub = self.create_subscription(
            Bool,
            '/yolo/enable',
            self.enable_callback,
            10
        )

        self.get_logger().info("웹캠 준비 완료")

    def enable_callback(self, msg: Bool):
        self.enabled = msg.data
        if not self.enabled:
            self.last_detected_label = None
        status = "활성화" if self.enabled else "비활성화"
        self.get_logger().info(f"🔄 YOLO {status}")

    def run_detection(self):
        if not self.enabled:
            return

        ret, frame = self.cap.read()  # 웹캠에서 이미지 캡처
        if not ret:
            self.get_logger().error("웹캠 이미지 캡처 실패")
            return
        
        # YOLO 추론
        results = self.model(frame, verbose=False, conf=0.5)
        annotated_image = results[0].plot()

        detections = results[0].boxes
        
        current_best_label = None
        best_box = None
        max_conf = 0.0

        # 1. 가장 신뢰도 높은 객체 선택
        if len(detections) > 0:
            for box in detections:
                conf = float(box.conf[0])
                
                # 가장 높은 신뢰도를 가진 객체 선택
                if conf > max_conf:
                    max_conf = conf
                    best_box = box
                    cls = int(box.cls[0])
                    raw_label = self.model.names[cls]  # 모델 출력 (예: large)
                    
                    # 변환된 레이블 맵을 사용
                    current_best_label = self.label_map.get(raw_label, raw_label)

        # 2. 상태 변경 감지 (새로운 객체가 감지되었을 때)
        if current_best_label is not None and current_best_label != self.last_detected_label:
            # 중심 좌표 계산
            x1, y1, x2, y2 = best_box.xyxy[0].cpu().numpy()
            center_x = float((x1 + x2) / 2)
            center_y = float((y1 + y2) / 2)

            # 중심 좌표 발행
            center_msg = Point()
            center_msg.x = center_x
            center_msg.y = center_y
            center_msg.z = max_conf
            self.center_pub.publish(center_msg)
            
            # 라벨 발행 (변환된 라벨)
            label_msg = String()
            label_msg.data = f"{current_best_label}|{center_x:.0f}|{center_y:.0f}|{max_conf:.2f}"
            self.label_pub.publish(label_msg)
            
            self.get_logger().info(f"🔔 [변환됨] {raw_label} -> {current_best_label} (Conf: {max_conf:.2f})")
            
            # 상태 업데이트
            self.last_detected_label = current_best_label

        elif current_best_label is None:
            if self.last_detected_label is not None:
                self.get_logger().info("💨 객체 사라짐")
            self.last_detected_label = None

        # 시각화된 이미지 발행
        detection_img_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
        self.image_pub.publish(detection_img_msg)

    def destroy(self):
        self.cap.release()  # 웹캠 리소스 해제

def main(args=None):
    rclpy.init(args=args)
    node = YoloVehicleDetectionNode()

    try:
        while rclpy.ok():
            node.run_detection()  # 실시간으로 웹캠 이미지 처리
            rclpy.spin_once(node, timeout_sec=0.1)  # 주기적으로 콜백 실행
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("🛑 종료")
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
