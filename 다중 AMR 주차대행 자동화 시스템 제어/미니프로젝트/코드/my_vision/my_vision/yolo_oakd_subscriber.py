#!/usr/bin/env python3
"""
TurtleBot Oak-D 카메라 이미지를 구독하여 YOLO 객체 탐지를 수행하는 노드
바운딩 박스의 중심 좌표를 계산하고 publish
"""

from ultralytics import YOLO
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import json


class YoloOakdSubscriber(Node):
    def __init__(self):
        super().__init__('yolo_oakd_subscriber')

        # --- ROS 파라미터 ---
        self.declare_parameter('robot_name', 'robot5')
        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('use_preview', True)
        self.declare_parameter('topic_annotated', 'yolo/image_annotated')
        self.declare_parameter('topic_result', 'yolo/result_text')
        self.declare_parameter('topic_centers', 'yolo/bbox_centers')  # 새로운 토픽
        self.declare_parameter('confidence', 0.5)
        self.declare_parameter('draw_center', True)  # 중심점 그리기 옵션

        robot_name = self.get_parameter('robot_name').value
        model_path = self.get_parameter('model_path').value
        use_preview = self.get_parameter('use_preview').value
        topic_annotated = self.get_parameter('topic_annotated').value
        topic_result = self.get_parameter('topic_result').value
        topic_centers = self.get_parameter('topic_centers').value
        self.confidence = self.get_parameter('confidence').value
        self.draw_center = self.get_parameter('draw_center').value
        self.declare_parameter('topic_trigger', 'yolo/center_trigger')
        topic_trigger = self.get_parameter('topic_trigger').value

        self.pub_trigger = self.create_publisher(Bool, topic_trigger, 10)
        self.get_logger().info(f'🔔 Center trigger will be published to: {topic_trigger}')
        self.pub_amr_detected = self.create_publisher(String, '/amr_car_detected', 10)
        self.get_logger().info("📡 /amr_car_detected: center aligned car signal will be published when centered.")

        # 🔹 x축 오차 튜닝용 퍼블리셔 (왼쪽 + / 오른쪽 -)
        self.pub_x_error = self.create_publisher(Float32, 'x_error_for_tuning', 10)
        self.get_logger().info("📏 /x_error_for_tuning: x-axis error (pixels) will be published for tuning.")

        # 🔹 car 박스 가로길이 퍼블리셔 (px)
        self.pub_bbox_width = self.create_publisher(Float32, 'bbox_width_for_tuning', 10)
        self.get_logger().info("📏 /bbox_width_for_tuning: car bbox width (pixels) will be published.")

        



        # 카메라 토픽 선택
        if use_preview:
            camera_topic = f'/{robot_name}/oakd/rgb/preview/image_raw' 
            self.get_logger().info('Using preview image (낮은 해상도, 빠름)')
        else:
            camera_topic = f'/{robot_name}/oakd/rgb/image_raw'
            self.get_logger().info('Using full resolution image (고해상도, 느림)')

        self.get_logger().info(f'Subscribing to: {camera_topic}')
        self.get_logger().info(f'Loading YOLO model: {model_path}')

        # --- YOLO 모델 로드 ---
        try:
            self.model = YOLO(model_path)
            self.get_logger().info('✅ YOLO model loaded successfully!')
        except Exception as e:
            self.get_logger().error(f'❌ Failed to load YOLO model: {e}')
            raise

        # --- Subscriber 설정 ---
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )

        # --- Publisher 설정 ---
        self.pub_annotated = self.create_publisher(Image, topic_annotated, 10)
        self.pub_result = self.create_publisher(String, topic_result, 10)
        self.pub_centers = self.create_publisher(String, topic_centers, 10)  # 중심 좌표

        self.get_logger().info('🚀 Node started! Waiting for images...')
        self.get_logger().info(f'📍 Publishing bbox centers to: {topic_centers}')

    def image_callback(self, msg):
        """이미지 토픽을 받아서 YOLO 추론 실행"""
        try:
            # 1. ROS Image → OpenCV 이미지 변환
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            h, w = frame.shape[:2]
            image_center_x = w // 2

            # 중앙 라인 주변 허용 범위 (예: 전체 폭의 5%)
            center_band = int(w * 0.10) 

            # 🔹 위쪽 무시 비율 (예: 상단 30%는 무시)
            ignore_top_ratio = 0.4
            ignore_top = int(h * ignore_top_ratio)


            # 2. YOLO 추론
            results = self.model(frame, conf=self.confidence, verbose=False)
            res = results[0]

            # 3. 탐지 결과가 그려진 이미지 생성
            annotated_frame = res.plot()
            
            # 4. 바운딩 박스 중심 좌표 계산 및 그리기
            names = res.names
            boxes = res.boxes
            
            detection_data = []  # 탐지 결과 저장
            trigger = False

            # 🔹 car 중에서 가장 가운데에 가까운 것 기준으로 x_error 계산
            best_x_error = None     # 실제 오차 값 (왼쪽 + / 오른쪽 -)
            best_abs_error = None   # |오차| (가운데에서 얼마나 먼지)
            best_width = None
            # ===========================================


            h, w = frame.shape[:2]
            image_center_x = w // 2

            

            # ====== 🔍 센터 밴드 시각화 추가 부분 ======
            left_bound  = image_center_x - center_band
            right_bound = image_center_x + center_band

            # 인덱스 범위 보정
            left_bound  = max(0, left_bound)
            right_bound = min(w - 1, right_bound)

            # 반투명 박스로 가운데 영역 표시
            overlay = annotated_frame.copy()
            cv2.rectangle(
                overlay,
                (left_bound, 0),
                (right_bound, h - 1),
                (255, 0, 0),     # 파란색 영역 (BGR)
                -1               # 채우기
            )
            alpha = 0.2  # 투명도
            annotated_frame = cv2.addWeighted(overlay, alpha, annotated_frame, 1 - alpha, 0)

            # 중앙선(정확한 center_x)도 흰색 선으로 한번 더 표시
            cv2.line(
                annotated_frame,
                (image_center_x, 0),
                (image_center_x, h - 1),
                (255, 255, 255),  # 흰색
                1
            )

            # if boxes is not None and len(boxes) > 0:
            #     for box in boxes:
            #         # 바운딩 박스 좌표 (x1, y1, x2, y2)
            #         x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    
            #         # 중심 좌표 계산
            #         center_x = int((x1 + x2) / 2)
            #         center_y = int((y1 + y2) / 2)
                    
            #         # 클래스 정보
            #         class_id = int(box.cls[0])
            #         class_name = names[class_id]
            #         confidence = float(box.conf[0])
                    
            #         # 데이터 저장
            #         detection_data.append({
            #             'class': class_name,
            #             'confidence': round(confidence, 2),
            #             'bbox': {
            #                 'x1': int(x1), 'y1': int(y1),
            #                 'x2': int(x2), 'y2': int(y2)
            #             },
            #             'center': {
            #                 'x': center_x,
            #                 'y': center_y
            #             }
            #         })
                    
            #         # 이미지에 중심점 그리기
            #         if self.draw_center:
            #             # 중심점 표시 (빨간 원)
            #             cv2.circle(annotated_frame, (center_x, center_y), 5, (0, 0, 255), -1)
                        
            #             # 십자선 표시
            #             cv2.line(annotated_frame, 
            #                     (center_x - 10, center_y), 
            #                     (center_x + 10, center_y), 
            #                     (0, 0, 255), 2)
            #             cv2.line(annotated_frame, 
            #                     (center_x, center_y - 10), 
            #                     (center_x, center_y + 10), 
            #                     (0, 0, 255), 2)
                        
            #             # 좌표 텍스트 표시
            #             coord_text = f"({center_x}, {center_y})"
            #             cv2.putText(annotated_frame, coord_text, 
            #                        (center_x + 15, center_y - 15),
            #                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            if boxes is not None and len(boxes) > 0:
                for box in boxes:
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)

                    class_id = int(box.cls[0])
                    class_name = names[class_id]
                    confidence = float(box.conf[0])

                    if class_name != 'car':
                        continue

                    # 🔹 위쪽 영역(상단 30%)에 있는 박스는 무시
                    if center_y < ignore_top:
                        continue

                    width_px = float(x2 - x1)

                    # 중앙선에 들어왔는지 판단 (x 좌표 기준)
                    if abs(center_x - image_center_x) <= center_band:
                        trigger = True
                        self.get_logger().info(
                            f"🎯 {class_name} center aligned with vertical center line: "
                            f"center_x={center_x}, image_center_x={image_center_x}"
                        )

                    x_error = image_center_x - center_x          # px 단위
                    abs_error = abs(x_error)

                    # car 중에서 가장 가운데에 가까운 것 선택
                    if best_abs_error is None or abs_error < best_abs_error:
                        best_abs_error = abs_error
                        best_x_error = x_error
                        best_width = width_px

                    detection_data.append({
                        'class': class_name,
                        'confidence': round(confidence, 2),
                        'bbox': {
                            'x1': int(x1), 'y1': int(y1),
                            'x2': int(x2), 'y2': int(y2)
                        },
                        'center': {
                            'x': center_x,
                            'y': center_y
                        }
                    })

                    # (기존 중심점 그리기 코드 그대로 유지)
                    if self.draw_center:
                        cv2.circle(annotated_frame, (center_x, center_y), 5, (0, 0, 255), -1)
                        cv2.line(annotated_frame, (center_x - 10, center_y),
                                (center_x + 10, center_y), (0, 0, 255), 2)
                        cv2.line(annotated_frame, (center_x, center_y - 10),
                                (center_x, center_y + 10), (0, 0, 255), 2)
                        coord_text = f"({center_x}, {center_y})"
                        cv2.putText(annotated_frame, coord_text,
                                    (center_x + 15, center_y - 15),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                
                # 로그 출력
                for det in detection_data:
                    self.get_logger().info(
                        f"📍 {det['class']}({det['confidence']}) - "
                        f"Center: ({det['center']['x']}, {det['center']['y']})"
                    )

                # 🔹 car 기준 x_error_for_tuning publish
                if best_x_error is not None:
                    err_msg = Float32()
                    err_msg.data = float(best_x_error)
                    self.pub_x_error.publish(err_msg)

                    self.get_logger().info(
                        f"📏 x_error_for_tuning: {err_msg.data:.1f} px (left + / right -)"
                    )

                # 🔹 bbox_width_for_tuning publish (car 중 가장 가운데에 가까운 박스의 가로길이)
                if best_width is not None:
                    w_msg = Float32()
                    w_msg.data = float(best_width)
                    self.pub_bbox_width.publish(w_msg)
                    self.get_logger().info(
                        f"📏 bbox_width_for_tuning: {w_msg.data:.1f} px (preview frame 기준)"
                    )
            
            # 5. 중심 좌표 데이터 publish (JSON 형태)
            centers_msg = String()
            centers_msg.data = json.dumps(detection_data, indent=2)
            self.pub_centers.publish(centers_msg)
            
            # 6. 탐지 결과 이미지 publish
            msg_annotated = self.bridge.cv2_to_imgmsg(
                annotated_frame, encoding='bgr8'
            )
            msg_annotated.header = msg.header
            self.pub_annotated.publish(msg_annotated)

            # 7. 간단한 텍스트 결과 publish
            if detection_data:
                detection_list = [f"{d['class']}({d['confidence']})" for d in detection_data]
                text = f"Detected: {', '.join(detection_list)}"
            else:
                text = "Detected: none"

            msg_result = String()
            msg_result.data = text
            self.pub_result.publish(msg_result)

            # 8. 중앙에 있을 때만 /amr_car_detected 토픽 송신 (없으면 아무 것도 안 보냄)
            if trigger:
                trigger_msg = String()
                trigger_msg.data = "center_detected"
                self.pub_amr_detected.publish(trigger_msg)
            

        except Exception as e:
            self.get_logger().error(f'Error in image_callback: {e}')

    def destroy_node(self):
        self.get_logger().info('Shutting down YOLO node...')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloOakdSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()