#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator,
)


class CarTriggeredNavigator(Node):
    def __init__(self):
        super().__init__('car_triggered_navigator')

        # TurtleBot4 네비게이터 객체 생성
        self.navigator = TurtleBot4Navigator()
        self.get_logger().info('Navigator node initialized')

        # --- 상태 플래그들 ---
        self._triggered = False          # /car_detected 한 번만 처리
        self._amr_detected = False       # /amr_car_detected 수신 여부
        self._state = 'idle'             # idle -> going_to_goal -> sweeping -> done

        # 회전 관련 상태
        self._angles_deg = []            # SOUTH_EAST -> WEST 까지 각도 리스트
        self._angle_idx = 0              # 현재 각도 인덱스
        self._wait_start_time = None     # 각도 도달 후 6초 대기 시작 시각

        self.get_logger().info('fuck0')

        # --- 1) 도킹 상태 확인 & 도킹 ---
        if not self.navigator.getDockedStatus():
            self.navigator.info('Docking before initializing pose')
            self.navigator.dock()
        self.get_logger().info('도킹 상태 확인 & 도킹')

        # --- 2) 초기 포즈 설정 ---
        initial_pose = self.navigator.getPoseStamped(
            [0.0, 0.0], TurtleBot4Directions.NORTH
        )
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info('초기 포즈 설정')

        # --- 3) Nav2 활성화 대기 ---
        self.navigator.waitUntilNav2Active()
        self.navigator.info('Nav2 is active. Waiting for /car_detected...')   

        # --- 4) 목표 위치 & 포즈 ---
        self.goal_xy = [-1.46, -1.55]
        self.goal_pose = self.navigator.getPoseStamped(
            self.goal_xy,
            TurtleBot4Directions.SOUTH
        )
        self.get_logger().info('목표 위치 & 포즈 설정')

        # SOUTH_EAST(225°) → WEST(90°) 로 20도씩 감소하는 리스트 생성
        start_deg = int(TurtleBot4Directions.SOUTH_EAST)  # 225
        end_deg = int(TurtleBot4Directions.WEST)          # 90
        # 225, 205, 185, ..., 105 에 90도(정확한 WEST)까지 포함
        self._angles_deg = list(range(start_deg, end_deg, -20)) + [end_deg]
        # 예: [225, 205, 185, 165, 145, 125, 105, 90]

        # --- 5) /car_detected 구독 ---
        self.sub_car = self.create_subscription(
            String,
            '/car_detected',
            self.car_detected_callback,
            10
        )

        # --- 6) /amr_car_detected 구독 ---
        self.sub_amr = self.create_subscription(
            String,
            '/amr_car_detected',
            self.amr_detected_callback,
            10
        )

        # --- 7) 상태를 돌려줄 타이머 (0.1초마다) ---
        self.timer = self.create_timer(0.1, self.timer_callback)

        # --- 8) /amr_tuning 발행 ---
        self.amr_tuning_pub = self.create_publisher(
            String,
            '/amr_tuning',
            10
        )
    # /car_detected 들어왔을 때: undock + goal_pose로 이동 시작
    def car_detected_callback(self, msg: String):
        self.get_logger().info(msg.data)
        if self._triggered:
            return  # 이미 한 번 처리했으면 무시

        self._triggered = True
        self.get_logger().info(f'Received /car_detected: "{msg.data}"')
        self.navigator.info('Car detected! Undocking and going to goal pose.')

        # Undock
        self.navigator.undock()

        # goal_pose로 이동 시작
        self.navigator.startToPose(self.goal_pose)

        # 상태 전환: 목표 위치로 가는 중
        self._state = 'going_to_goal'
        self.navigator.info('Goal pose command sent. (going_to_goal)')

    # /amr_car_detected 들어왔을 때 
    def amr_detected_callback(self, msg: String):
        # # 이미 처리된 후라면 다시 안 함 (한 번만 실행)
        # if self._amr_detected:
        #     return
        self.get_logger().info("amr_car_detected callback!")
        self._amr_detected = True
        self.get_logger().info(f'Received /amr_car_detected: "{msg.data}"')
        self.navigator.info('AMR detected! (flag set)')

        tuning_msg = String()
        tuning_msg.data = 'amr_tuning_start!'
        self.amr_tuning_pub.publish(tuning_msg)
        self.get_logger().info(f'Published /amr_tuning: "{tuning_msg.data}"')

    # 주기적으로 상태에 따라 동작시키는 타이머 콜백
    def timer_callback(self):
        # idle 상태: 아직 /car_detected 안 온 상태 → 아무것도 안 함
        if self._state == 'idle':
            return

        # 목표 위치로 이동 중일 때
        if self._state == 'going_to_goal':
            # Nav2에서 현재 goal(초기 goal_pose)이 끝났는지 확인
            if self.navigator.isTaskComplete():
                self.navigator.info('Reached goal pose. Start single SOUTH_EAST -> WEST sweep.')

                # 회전 시퀀스 초기화
                self._angle_idx = 0
                self._wait_start_time = None

                first_angle = self._angles_deg[self._angle_idx]
                self.navigator.info(f'Starting sweep at {first_angle} deg (SOUTH_EAST)')
                self.send_sweep_goal(first_angle)

                # 상태 전환
                self._state = 'sweeping'

        # SOUTH_EAST → WEST 한 번만 회전 (20도 step, 각 단계 사이 6초 대기)
        elif self._state == 'sweeping':
            # AMR가 이미 감지된 경우 → 스윕 종료
            if self._amr_detected:
                self.navigator.info('AMR detected during sweep. Stopping sweep.')
                self._state = 'done'
                return

            # 현재 목표 각도까지의 네비게이션이 완료되었는지 확인
            if self.navigator.isTaskComplete():
                now = self.get_clock().now()

                # 아직 대기 시작 안 했으면, 이제부터 6초 대기 시작
                if self._wait_start_time is None:
                    self._wait_start_time = now
                    return

                # 6초 지났는지 확인
                elapsed = (now - self._wait_start_time).nanoseconds / 1e9
                if elapsed < 3.0:
                    return  # 6초 될 때까지 대기

                # 6초가 지났고, 다음 각도가 남아 있으면 다음 각도로 이동
                if self._angle_idx < len(self._angles_deg) - 1:
                    self._angle_idx += 1
                    next_angle = self._angles_deg[self._angle_idx]
                    self.navigator.info(f'Moving to next angle: {next_angle} deg')
                    self.send_sweep_goal(next_angle)
                    # 다음 각도에 도달하면 다시 6초 대기를 위해 초기화
                    self._wait_start_time = None
                else:
                    # 마지막 각도까지 모두 완료 → 스윕 종료
                    self.navigator.info('Finished single sweep from SOUTH_EAST to WEST.')
                    self._state = 'done'

        elif self._state == 'done':
            # 이후에는 추가 동작 없음 (원하면 dock() 호출 가능)
            self.get_logger().info("detected and stopped!")
            return

    def send_sweep_goal(self, direction_deg):
        """
        같은 위치(self.goal_xy)에 방향만 바꿔서 pose goal 전송.
        direction_deg: 남북 기준 도(degree), 예: 225, 205, ..., 90
        """
        sweep_pose = self.navigator.getPoseStamped(self.goal_xy, direction_deg)
        self.navigator.startToPose(sweep_pose)


def main(args=None):
    rclpy.init(args=args)
    node = CarTriggeredNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down car_triggered_navigator...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
