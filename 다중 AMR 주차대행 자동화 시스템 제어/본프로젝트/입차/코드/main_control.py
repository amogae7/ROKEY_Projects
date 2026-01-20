
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool

from cv_bridge import CvBridge
import cv2
import numpy as np

import math
import time
import json
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

# TurtleBot4 Nav2 helper
from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Navigator,
    TurtleBot4Directions,
)


class OrientationConverter:
    """방향 변환 유틸리티"""
    DIRECTION_MAP = {
        'NORTH': TurtleBot4Directions.NORTH,
        'SOUTH': TurtleBot4Directions.SOUTH,
        'EAST': TurtleBot4Directions.EAST,
        'WEST': TurtleBot4Directions.WEST,
    }
    
    @classmethod
    def to_turtlebot_direction(cls, orientation_str: str) -> float:
        return cls.DIRECTION_MAP.get(orientation_str.upper(), TurtleBot4Directions.NORTH)


class ParkingOrchestrator(Node):
    def __init__(self):
        super().__init__("parking_orchestrator")

        self.bridge = CvBridge()
        self.reentrant_group = ReentrantCallbackGroup()
        # ==================== 상태 관리 ====================
        self.state = "WAITING_ALLOCATION"  # WAITING_ALLOCATION, NAVIGATING, LINE_DETECT, DONE
        self.current_allocation = None
        self.enable_line_detect = False
        
        # ==================== Parameters ====================
        self.declare_parameter("image_topic", "/robot1/oakd/rgb/image_raw/compressed")
        self.declare_parameter("cmd_vel_topic", "/robot1/cmd_vel")
        self.declare_parameter("kp_angular", 0.003)
        self.declare_parameter("align_linear_speed", 0.05)
        self.declare_parameter("final_forward_speed", 0.08)
        self.declare_parameter("final_forward_distance", 0.2)

        image_topic = self.get_parameter("image_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.kp_ang = self.get_parameter("kp_angular").value
        self.align_linear_speed = self.get_parameter("align_linear_speed").value
        self.final_forward_speed = self.get_parameter("final_forward_speed").value
        self.final_forward_distance = self.get_parameter("final_forward_distance").value

        if self.final_forward_speed > 0.0:
            self.final_forward_duration = self.final_forward_distance / self.final_forward_speed
        else:
            self.final_forward_duration = 0.0

        # ==================== QoS ====================
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # 주차 할당 결과 구독
        self.allocation_sub = self.create_subscription(
            String,
            '/parking_allocation',
            self.allocation_callback,
            10,
            callback_group=self.reentrant_group
        )
        
        # 이미지 구독
        self.image_sub = self.create_subscription(
            CompressedImage,
            image_topic,
            self.image_callback,
            sensor_qos
        )

        # cmd_vel 발행
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # 주차 완료 신호 발행
        self.parking_done_pub = self.create_publisher(
            Bool,
            '/parking_done',
            10
        )

        # 디버그 이미지
        self.debug_image_pub = self.create_publisher(
            Image,
            "/line_center/debug_image",
            reliable_qos
        )

        # ==================== Navigator ====================
        self.navigator = TurtleBot4Navigator()

        # ==================== 라인 검출 변수 ====================
        # 디버그 용
        self.callback_count = 0
        self.last_callback_time = None

        # 정렬 튜닝 파라미터
        self.dead_band_px = 5.0
        self.center_tolerance_px = 15.0
        self.max_angular_z = 0.5
        self.stable_needed = 5
        self.stable_count = 0

        # Phase 상태
        self.phase = "ALIGN_MOVE"
        self.phase3_start_time = None
        self.scan_start_time = None

        self.get_logger().info("🚗 Parking Orchestrator 시작!")
        self.get_logger().info("⏳ 주차 공간 할당 대기 중...")
        
        # 타이머
        self.create_timer(2.0, self.status_check)

    def allocation_callback(self, msg: String):

        if self.state != "WAITING_ALLOCATION":
            return
        
        try:
            data = json.loads(msg.data)
            
            if not data['success']:
                self.get_logger().error(f"❌ 주차 할당 실패: {data['message']}")
                return
            
            self.current_allocation = data
            
            self.get_logger().info(f"\n{'='*60}")
            self.get_logger().info(f"✅ 주차 공간 할당됨!")
            self.get_logger().info(f"   위치: {data['location_id']}")
            self.get_logger().info(f"   Zone: {data['zone_id']}")
            self.get_logger().info(f"   좌표: ({data['x']}, {data['y']})")
            self.get_logger().info(f"   방향: {data['orientation']}")
            self.get_logger().info(f"{'='*60}\n")
            
            # Navigation 시작
            self.state = "NAVIGATING"
            self.navigate_to_zone()
            
        except Exception as e:
            self.get_logger().error(f"❌ 할당 결과 처리 실패: {e}")

    def navigate_to_zone(self):
        """Nav2로 Zone 좌표 이동"""
        if not self.current_allocation:
            return
        
        allocation = self.current_allocation
        
        target_x = allocation['x']
        target_y = allocation['y']
        
        # Orientation 변환
        orientation_str = allocation['orientation']
        target_yaw = OrientationConverter.to_turtlebot_direction(orientation_str)
        
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"🚀 Navigation 시작")
        self.get_logger().info(f"   목표: ({target_x:.2f}, {target_y:.2f})")
        self.get_logger().info(f"   방향: {orientation_str}")
        self.get_logger().info(f"{'='*60}\n")
        
        # Undocking (필요시)
        if self.navigator.getDockedStatus():
            self.get_logger().info("📍 Undocking...")
            self.navigator.undock()
            time.sleep(2)
        
        # Navigation 실행
        success = self.navigate_to_goal(target_x, target_y, target_yaw)
        
        if success:
            self.get_logger().info("✅ Zone 도착! 라인 정렬 시작")
            self.state = "LINE_DETECT"
            self.enable_line_detect = True
        else:
            self.get_logger().error("❌ Navigation 실패")
            self.state = "WAITING_ALLOCATION"

    def navigate_to_goal(self, target_x: float, target_y: float, target_yaw: float) -> bool:
        """Nav2로 목표 지점 이동"""
        try:
            self.get_logger().info("⏳ Nav2 활성화 대기...")
            self.navigator.waitUntilNav2Active()
            
            goal_pose = self.navigator.getPoseStamped([target_x, target_y], target_yaw)
            
            self.get_logger().info(f"🎯 목표: ({target_x:.2f}, {target_y:.2f})")
            self.navigator.goToPose(goal_pose)
            
            start_time = time.time()
            timeout = 180
            
            while not self.navigator.isTaskComplete():
                elapsed = time.time() - start_time
                if elapsed > timeout:
                    self.get_logger().error("❌ 타임아웃!")
                    self.navigator.cancelTask()
                    return False
                time.sleep(0.5)
            
            self.get_logger().info("✅ 이동 완료!")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Navigation 오류: {e}")
            return False

    def status_check(self):
        """주기적 상태 체크"""
        current_time = self.get_clock().now()
        
        if self.last_callback_time is not None:
            interval = (current_time - self.last_callback_time).nanoseconds / 1e9
            fps = 1.0 / interval if interval > 0 else 0.0
        else:
            fps = 0.0
        
        self.get_logger().info(
            f"[상태] State={self.state}, Phase={self.phase}, "
            f"콜백={self.callback_count}회, FPS={fps:.1f}"
        )

    def image_callback(self, msg: CompressedImage):
        """이미지 콜백 - 라인 검출"""
        if not self.enable_line_detect:
            return
        
        self.callback_count += 1
        self.last_callback_time = self.get_clock().now()
        
        if self.callback_count <= 3:
            self.get_logger().info(f"✅ image_callback 호출! (#{self.callback_count})")
        
        cmd = Twist()

        if self.phase == "DONE":
            self.cmd_pub.publish(cmd)
            return

        # 1) CompressedImage -> OpenCV BGR
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"이미지 디코딩 실패: {e}")
            return

        h, w, _ = frame.shape

        # 2) ROI 설정
        roi_y_start = int(h * 2 / 3)
        roi = frame[roi_y_start:h, :]

        # 3) 흰 선 검출 (HSV 필터)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        lower_white = np.array([0, 0, 200], dtype=np.uint8)
        upper_white = np.array([179, 30, 255], dtype=np.uint8)
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # 4) 노이즈 제거
        kernel = np.ones((5, 5), np.uint8)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, kernel)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, kernel)

        # 5) 컨투어 찾기
        contours, _ = cv2.findContours(
            mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        line_found = False
        lane_center_x = None

        if contours:
            min_area = 300.0
            min_aspect = 2.0

            candidates = []
            for c in contours:
                area = cv2.contourArea(c)
                if area < min_area: continue

                x, y, cw, ch = cv2.boundingRect(c)
                aspect = ch / float(cw + 1e-3)
                if aspect < min_aspect: continue

                candidates.append((area, c))

            if candidates:
                image_center_x = w / 2.0
                best_c = None
                min_distance_to_center = float('inf')

                for area, c in candidates:
                    M = cv2.moments(c)
                    if M["m00"] > 0:
                        cx = M["m10"] / M["m00"]
                        dist = abs(cx - image_center_x)
                        if dist < min_distance_to_center:
                            min_distance_to_center = dist
                            best_c = c

                if best_c is not None:
                    M = cv2.moments(best_c)
                    if M["m00"] > 0:
                        cx_global = (M["m10"] / M["m00"])
                        cy_global = (M["m01"] / M["m00"]) + roi_y_start

                        x, y, cw, ch = cv2.boundingRect(best_c)
                        cv2.rectangle(frame, (x, y + roi_y_start), (x + cw, y + ch + roi_y_start), (0, 255, 0), 2)
                        cv2.circle(frame, (int(cx_global), int(cy_global)), 5, (0, 0, 255), -1)

                        lane_center_x = cx_global
                        line_found = True

        # ---- 제어 로직 ----
        if line_found and lane_center_x is not None:
            self.scan_start_time = None

            cv2.line(frame, (int(lane_center_x), roi_y_start), (int(lane_center_x), h), (255, 0, 0), 2)

            image_center_x = w / 2.0
            error_px = lane_center_x - image_center_x

            cv2.putText(frame, f"Phase: {self.phase}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Error: {error_px:.1f}px", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Stable: {self.stable_count}/{self.stable_needed}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            raw_omega = -self.kp_ang * error_px
            if raw_omega > self.max_angular_z: raw_omega = self.max_angular_z
            elif raw_omega < -self.max_angular_z: raw_omega = -self.max_angular_z

            if self.phase == "ALIGN_MOVE":
                cmd.linear.x = self.align_linear_speed

                if abs(error_px) < self.dead_band_px:
                    cmd.angular.z = 0.0
                else:
                    cmd.angular.z = float(raw_omega)

                if abs(error_px) < self.center_tolerance_px:
                    self.stable_count += 1
                else:
                    self.stable_count = 0

                if self.stable_count >= self.stable_needed:
                    self.phase = "FINAL_FORWARD"
                    self.phase3_start_time = time.time()
                    self.get_logger().info("✅ 정렬 완료! 최종 직진 시작")

            elif self.phase == "FINAL_FORWARD":
                elapsed = time.time() - self.phase3_start_time
                cmd.linear.x = self.final_forward_speed
                cmd.angular.z = float(raw_omega)

                if elapsed >= self.final_forward_duration:
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.phase = "DONE"
                    self.get_logger().info("🏁 주차 완료!")
                    
                    # 주차 완료 신호 발행
                    self.parking_done_pub.publish(Bool(data=True))
                    
                    # 상태 변경
                    self.state = "DONE"
                    self.enable_line_detect = False

        else:
            # 라인 미검출
            if self.phase == "ALIGN_MOVE":
                if self.scan_start_time is None:
                    self.scan_start_time = time.time()
                
                elapsed_scan = time.time() - self.scan_start_time
                
                status_text = "SCANNING..."
                if elapsed_scan < 2.0:
                    cmd.angular.z = -0.3
                    status_text = "SCAN: LEFT"
                elif elapsed_scan < 5.0:
                    cmd.angular.z = 0.3
                    status_text = "SCAN: RIGHT"
                elif elapsed_scan < 7.0:
                    cmd.angular.z = -0.3
                    status_text = "SCAN: CENTER"
                else:
                    cmd.angular.z = 0.0
                    status_text = "SCAN: GAVE UP"

                cmd.linear.x = 0.0
                cv2.putText(frame, status_text, (10, 150), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            else:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
            
            cv2.putText(frame, "NO LINE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        self.cmd_pub.publish(cmd)

        # 디버그 이미지
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            if self.callback_count <= 3:
                self.get_logger().error(f"디버그 이미지 실패: {e}")


def main(args=None):
    print("=== Parking Orchestrator 시작 ===")
    rclpy.init(args=args)
    
    orchestrator = ParkingOrchestrator()
    
    # 기존: rclpy.spin(orchestrator)
    
    # 변경: MultiThreadedExecutor 사용
    executor = MultiThreadedExecutor()
    executor.add_node(orchestrator)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        orchestrator.get_logger().info("종료")
    finally:
        orchestrator.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == "__main__":
    main()