#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage, Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String, Bool

from cv_bridge import CvBridge
import cv2
import numpy as np
import time


class LineAlignNode(Node):
    """
    역할:
    1) /robot5/waypoint_done(String)을 받으면 → 라인 정렬 모드 ON
    2) 카메라 이미지에서 흰 선을 찾아 중앙에 맞추면서 전진
    3) 충분히 정렬되면, 일정 거리 직진 후 정지
       (옵션) /robot5/line_align_done 및 /robot5/tuning_done 발행
    """

    def __init__(self):
        super().__init__('line_align_node')

        self.bridge = CvBridge()

        # ====== 상태 ======
        self.phase = "IDLE"
        self.enable_line_detect = False

        # ====== 파라미터 ======
        self.declare_parameter("image_topic", "/robot5/oakd/rgb/image_raw/compressed")
        self.declare_parameter("cmd_vel_topic", "/robot5/cmd_vel")
        self.declare_parameter("kp_angular", 0.003)
        self.declare_parameter("align_linear_speed", 0.05)
        self.declare_parameter("final_forward_speed", 0.08)
        self.declare_parameter("final_forward_distance", 0.2)

        image_topic = self.get_parameter("image_topic").value
        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.kp_ang = float(self.get_parameter("kp_angular").value)
        self.align_linear_speed = float(self.get_parameter("align_linear_speed").value)
        self.final_forward_speed = float(self.get_parameter("final_forward_speed").value)
        self.final_forward_distance = float(self.get_parameter("final_forward_distance").value)

        if self.final_forward_speed > 0.0:
            self.final_forward_duration = self.final_forward_distance / self.final_forward_speed
        else:
            self.final_forward_duration = 0.0

        # 🔸 최종 직진 단계 최대 대기 시간(초) – 이 시간이 지나면 강제 완료
        self.final_forward_timeout = 5.0

        # ====== 정렬 튜닝 값 ======
        self.dead_band_px = 5.0
        self.center_tolerance_px = 15.0
        self.max_angular_z = 0.5
        self.stable_needed = 5
        self.stable_count = 0

        # ====== 기타 ======
        self.callback_count = 0
        self.phase3_start_time = None
        self.scan_start_time = None
        self.last_callback_time = None

        # ====== 구독/발행 ======
        self.waypoint_done_sub = self.create_subscription(
            String,
            '/robot5/waypoint_done',
            self.waypoint_done_callback,
            10
        )

        self.image_sub = self.create_subscription(
            CompressedImage,
            image_topic,
            self.image_callback,
            10
        )

        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        self.line_done_pub = self.create_publisher(
            Bool,
            '/robot5/line_align_done',
            10
        )

        # === 튜닝 완료 토픽 ===
        self.tuning_done_pub = self.create_publisher(
            String,
            '/robot5/tuning_done',
            10
        )

        self.debug_image_pub = self.create_publisher(
            Image,
            '/line_center/debug_image',
            10
        )

        self.get_logger().info("✅ LineAlignNode 시작 (waypoint_done → 흰선 정렬 모드)")

    # --------------------------
    # waypoint_done 트리거
    # --------------------------
    def waypoint_done_callback(self, msg: String):
        self.get_logger().info(
            f"[waypoint_done] '{msg.data}' 수신 → 흰선 정렬 시작"
        )

        self.phase = "ALIGN_MOVE"
        self.enable_line_detect = True
        self.stable_count = 0
        self.scan_start_time = None
        self.phase3_start_time = None
        self.callback_count = 0
         
    # --------------------------
    # 이미지 콜백
    # --------------------------
    def image_callback(self, msg: CompressedImage):
        self.get_logger().info('이미지 콜백 실행')
        # 아직 정렬 모드 아님
        if not self.enable_line_detect:
            return

        self.callback_count += 1
        cmd = Twist()

        # 정렬 끝난 상태면 정지
        if self.phase == "DONE":
            self.cmd_pub.publish(cmd)
            return

        # 1) Image 변환
        try:
            frame = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"[image_callback] 이미지 디코딩 실패: {e}")
            return

        h, w, _ = frame.shape

        # -----------------------------
        # 2) ROI 설정
        # -----------------------------
        roi_y_start = int(h * 2 / 3)
        roi = frame[roi_y_start:h, :]

        # -----------------------------
        # 3) 흰색 검출 (HSV)
        # -----------------------------
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200], dtype=np.uint8)
        upper_white = np.array([179, 30, 255], dtype=np.uint8)
        mask_white = cv2.inRange(hsv, lower_white, upper_white)

        # 노이즈 제거
        kernel = np.ones((5, 5), np.uint8)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_OPEN, kernel)
        mask_white = cv2.morphologyEx(mask_white, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        line_found = False
        lane_center_x = None

        # -----------------------------
        # 4) 선 후보 검출
        # -----------------------------
        if contours:
            min_area = 300.0
            min_aspect = 2.0

            candidates = []
            for c in contours:
                area = cv2.contourArea(c)
                if area < min_area:
                    continue

                x, y, cw, ch = cv2.boundingRect(c)
                aspect = ch / float(cw + 1e-3)
                if aspect < min_aspect:
                    continue

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

                        lane_center_x = cx_global
                        line_found = True

        # -----------------------------
        # 5) 정렬 제어
        # -----------------------------
        if line_found and lane_center_x is not None:

            image_center_x = w / 2.0
            error_px = lane_center_x - image_center_x

            # P 제어
            raw_omega = -self.kp_ang * error_px
            raw_omega = max(min(raw_omega, self.max_angular_z), -self.max_angular_z)

            # ===== Phase 1: 정렬 단계 =====
            if self.phase == "ALIGN_MOVE":
                cmd.linear.x = self.align_linear_speed
                cmd.angular.z = 0.0 if abs(error_px) < self.dead_band_px else float(raw_omega)

                if abs(error_px) < self.center_tolerance_px:
                    self.stable_count += 1
                else:
                    self.stable_count = 0

                if self.stable_count >= self.stable_needed:
                    self.phase = "FINAL_FORWARD"
                    self.phase3_start_time = time.time()
                    self.get_logger().info("✅ 정렬 완료! 최종 직진 시작")

            # ===== Phase 2: 최종 직진 =====
            elif self.phase == "FINAL_FORWARD":
                elapsed = time.time() - self.phase3_start_time
                cmd.linear.x = self.final_forward_speed
                cmd.angular.z = float(raw_omega)

                # ⏱ 기준 1: 설정된 거리만큼 직진 완료
                # ⏱ 기준 2: FINAL_FORWARD 들어온 뒤 5초가 지나면 강제 완료
                if (self.final_forward_duration > 0.0 and elapsed >= self.final_forward_duration) \
                        or elapsed >= self.final_forward_timeout:
                    # ---- 완료 조건 ----
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.phase = "DONE"
                    self.enable_line_detect = False

                    if elapsed >= self.final_forward_timeout:
                        self.get_logger().warn(
                            f"⏰ FINAL_FORWARD 타임아웃({self.final_forward_timeout}s) 도달 → 강제 완료 처리"
                        )
                    else:
                        self.get_logger().info("🏁 라인 정렬 + 최종 직진 완료!")

                    # 완료 신호
                    self.line_done_pub.publish(Bool(data=True))

                    # tuning_done 발행
                    tuning_msg = String()
                    tuning_msg.data = "done"
                    self.tuning_done_pub.publish(tuning_msg)
                    self.get_logger().info("📢 /robot5/tuning_done 발행됨: done")

        else:
            # 선을 못 보면 스캔 동작 대신 여기선 정지(단순화)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        # 속도 명령 발행
        self.cmd_pub.publish(cmd)

        # 디버그 이미지 발행
        try:
            debug_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
            debug_msg.header = msg.header
            self.debug_image_pub.publish(debug_msg)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = LineAlignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 LineAlignNode 종료")
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
