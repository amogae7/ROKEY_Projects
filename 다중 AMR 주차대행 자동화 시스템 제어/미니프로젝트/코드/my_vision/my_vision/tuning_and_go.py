#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient

from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist

from irobot_create_msgs.action import DriveDistance


class AmrTuningNode(Node):
    def __init__(self):
        super().__init__('amr_tuning_node')

        # /amr_tuning: 튜닝 트리거용 String 토픽
        self.sub_amr_tuning = self.create_subscription(
            String,
            '/amr_tuning',
            self.amr_tuning_callback,
            10
        )

        # /x_error_for_tuning: 튜닝 기준이 되는 float 에러값
        self.sub_x_error = self.create_subscription(
            Float32,
            '/x_error_for_tuning',
            self.x_error_callback,
            10
        )

        # /car_distance_m: 물체까지의 거리 (depth 기반)
        self.sub_depth_dist = self.create_subscription(
            Float32,
            '/car_distance_m',
            self.depth_dist_callback,
            10
        )

        # /cmd_vel: 회전 명령 발행 (정렬용)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/robot5/cmd_vel',
            10
        )

        # /drive_distance 액션 클라이언트 (거리 기반 전진)
        self.drive_client = ActionClient(self, DriveDistance, '/robot5/drive_distance')
        self.drive_goal_sent = False

        # --- 상태 변수들 ---

        # 거리 관련
        self.initial_dist = None          # 평균으로 결정된 초기 거리
        self.initial_dist_fixed = False   # 한 번 세팅되면 True
        self.current_dist = 0.0
        self.dist_to_go = 0.0            # self.initial_dist - 0.05

        # 멈춘 이후 평균 계산을 위한 상태
        self.stop_time = None            # stop_and_finish에서 세팅
        self.collect_depth = False       # 현재 depth 평균 수집 중인지
        self.depth_samples = []          # 평균을 위한 샘플 버퍼

        # 정렬 튜닝 관련 상태
        self.latest_x_error = None       # 최근 에러 값
        self.tuning_requested = False    # /amr_tuning 들어왔는지
        self.tuning_active = False       # 실제 회전 중인지
        self.tuning_request_time = None  # /amr_tuning 받은 시각
        self.tuning_done = False         # 한 번 전체 튜닝/이동 끝났는지 (한 번만 실행용)

        # 주기적으로 상태를 확인할 타이머 (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info('AMR tuning node initialized.')

    # /amr_tuning 수신 콜백: 한 번만 실행됨
    def amr_tuning_callback(self, msg: String):
        self.get_logger().info(f'/amr_tuning received: "{msg.data}"')

        # 이미 한 번 튜닝/이동 다 끝났으면 추가 요청 무시
        if self.tuning_done or self.tuning_requested or self.tuning_active:
            self.get_logger().info('Tuning already processed or in progress. Ignoring /amr_tuning.')
            return

        # 새 튜닝 요청 등록
        self.tuning_requested = True
        self.tuning_active = False  # 이전 튜닝 중이면 리셋
        self.tuning_request_time = self.get_clock().now()
        self.get_logger().info('Tuning requested.')

    # /x_error_for_tuning 수신 콜백: 최근 에러값 저장
    def x_error_callback(self, msg: Float32):
        self.latest_x_error = msg.data
        self.get_logger().info(f"/x_error_for_tuning subbed! {self.latest_x_error}")

    # /car_distance_m 수신 콜백
    def depth_dist_callback(self, msg: Float32):
        now = self.get_clock().now()
        self.current_dist = msg.data   # 항상 최신 거리 저장

        # 멈춘 뒤 5초 후부터 10개 depth 값 평균 내기
        if self.collect_depth and self.stop_time is not None:
            elapsed = (now - self.stop_time).nanoseconds / 1e9

            # 5초 이후에만 샘플 수집 시작
            if elapsed >= 5.0:
                if len(self.depth_samples) < 10:
                    self.depth_samples.append(msg.data)
                    self.get_logger().info(
                        f"[depth_dist] sample {len(self.depth_samples)}/10 = {msg.data:.3f}"
                    )

                # 샘플 10개 다 모였으면 평균 내고 initial_dist 확정
                if len(self.depth_samples) == 10:
                    avg_dist = sum(self.depth_samples) / len(self.depth_samples)
                    self.initial_dist = avg_dist
                    self.dist_to_go = self.initial_dist - 0.7
                    self.initial_dist_fixed = True
                    self.collect_depth = False  # 수집 종료

                    self.get_logger().info(
                        f"[depth_dist] initial_dist (avg) = {self.initial_dist:.3f}, "
                        f"dist_to_go = {self.dist_to_go:.3f}"
                    )

                    # ✅ 여기서 바로 /drive_distance 액션으로 dist_to_go 만큼 전진
                    self.send_drive_distance_goal()

    def timer_callback(self):
        now = self.get_clock().now()

        # 1) 튜닝 요청만 들어온 상태면, 바로(혹은 필요시 지연 후) 튜닝 시작
        if self.tuning_requested and not self.tuning_active:
            self.get_logger().info("timer on (tuning_requested)")
            if self.tuning_request_time is not None:
                # 지연을 쓰고 싶다면 Duration 비교를 추가하면 됨
                self.get_logger().info('Starting tuning after delay.')
                self.tuning_active = True
                self.tuning_requested = False
            return

        # 2) 튜닝이 활성화된 상태에서 회전 제어
        if self.tuning_active:
            if self.latest_x_error is None:
                self.get_logger().warn('No x_error_for_tuning received yet.')
                return

            x_err = self.latest_x_error
            twist = Twist()

            if x_err > 0.0:
                if x_err >= 50.0:
                    twist.angular.z = 0.01
                    self.cmd_vel_pub.publish(twist)
                    self.get_logger().info(f'x_error={x_err:.2f} → turning left.')
                else:
                    self.get_logger().info(
                        f'x_error={x_err:.2f} → stop turning (tuning done).'
                    )
                    self.stop_and_finish()

            elif x_err < 0.0:
                if x_err <= -50.0:
                    twist.angular.z = -0.01
                    self.cmd_vel_pub.publish(twist)
                    self.get_logger().info(f'x_error={x_err:.2f} → turning right.')
                else:
                    self.get_logger().info(
                        f'x_error={x_err:.2f} → stop turning (tuning done).'
                    )
                    self.stop_and_finish()
            else:
                self.get_logger().info(
                    'x_error is 0 → stop turning (tuning done).'
                )
                self.stop_and_finish()

            # 회전 중일 땐 전진/액션 제어는 하지 않음
            return

        # 3) 튜닝도 끝났고, forward 제어는 /drive_distance 액션이 담당하므로
        #    여기서는 추가 동작 없음.
        return

    def stop_and_finish(self):
        """회전 멈추고 튜닝 종료 상태로 전환 + 이후 거리 평균 수집 준비."""
        twist = Twist()  # 모두 0 → 정지
        self.cmd_vel_pub.publish(twist)
        self.tuning_active = False
        self.get_logger().info('Tuning finished and robot stopped.')

        # ✅ 멈춘 시각 저장 + 이후 depth 평균 계산 준비
        self.stop_time = self.get_clock().now()
        self.collect_depth = True
        self.depth_samples = []
        self.initial_dist_fixed = False  # 새 초기 거리로 갱신할 예정

        self.get_logger().info(
            "Will start collecting depth samples 5 seconds after stop."
        )

        # 한 번 튜닝 과정 들어갔다가 끝난 뒤에는,
        # /amr_tuning 재요청도 무시하도록 플래그 설정
        self.tuning_done = True

    # ========== /drive_distance 액션 관련 메서드들 ==========

    def send_drive_distance_goal(self):
        """dist_to_go 만큼 /drive_distance 액션을 사용해 전진."""
        if self.dist_to_go is None:
            self.get_logger().warn("dist_to_go is None, cannot send DriveDistance goal.")
            return

        if self.dist_to_go <= 0.0:
            self.get_logger().warn(
                f"dist_to_go ({self.dist_to_go:.3f}) <= 0, skipping DriveDistance."
            )
            return

        # 액션 서버 준비 대기
        self.get_logger().info("Waiting for /drive_distance action server...")
        if not self.drive_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("DriveDistance action server not available.")
            return

        goal_msg = DriveDistance.Goal()
        goal_msg.distance = float(self.dist_to_go)          # ✅ 이동할 거리 (m)
        goal_msg.max_translation_speed = 0.3                # 원하는 속도

        self.get_logger().info(
            f"Sending DriveDistance goal: distance={goal_msg.distance:.3f}, "
            f"max_speed={goal_msg.max_translation_speed:.2f}"
        )

        send_future = self.drive_client.send_goal_async(
            goal_msg,
            feedback_callback=self.drive_feedback_callback
        )
        send_future.add_done_callback(self.drive_goal_response_callback)
        self.drive_goal_sent = True

    def drive_goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("DriveDistance goal rejected.")
            return

        self.get_logger().info("DriveDistance goal accepted.")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.drive_result_callback)

    def drive_feedback_callback(self, feedback_msg):
        # DriveDistance.Feedback에 맞춰서 실제 필드를 보고 싶으면
        #   ros2 interface show irobot_create_msgs/action/DriveDistance
        # 로 확인해서 로그 수정해도 됨
        self.get_logger().info(f"DriveDistance feedback: {feedback_msg.feedback}")

    def drive_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"DriveDistance finished with result: {result}")
        # 여기서 필요하면 노드 종료나 다음 상태 전환도 가능


def main(args=None):
    rclpy.init(args=args)
    node = AmrTuningNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down amr_tuning_node...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
