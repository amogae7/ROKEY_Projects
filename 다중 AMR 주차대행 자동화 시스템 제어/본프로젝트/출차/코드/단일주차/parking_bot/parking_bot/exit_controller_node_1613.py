#!/usr/bin/env python3
import rclpy
import threading
from rclpy.node import Node
from std_msgs.msg import String

from parking_bot.exit_actions_robot5 import ExitActions


class ExitController(Node):
    def __init__(self):
        super().__init__("exit_controller", namespace="robot5")

        # 출차 동작 모듈
        self.actions = ExitActions(self)

        # waypoint_done 수신
        self.create_subscription(
            String,
            "waypoint_done",     # => /robot5/waypoint_done
            self.cb_waypoint_done,
            10
        )

        self.get_logger().info("[ExitController] READY — waiting for waypoint_done...")

        # self.is_pickup_done = False  # 픽업 완료 여부
        # self.is_exit_done = False    # 출구 이동 완료 여부

    # ===========================================================
    # waypoint_done → Thread에서 pickup + exit 실행
    # ===========================================================
    def cb_waypoint_done(self, msg):
        car_type = msg.data if msg.data else "mid"
        self.get_logger().info(
            f"[ExitController] waypoint_done received → 수행 시작 (car_type={car_type})"
        )

        # # 순차적으로 작업 실행
        # self._run_exit_sequence(car_type)

        # Thread에서 전체 시퀀스 실행 (deadlock 방지) 
        worker = threading.Thread(
            target=self._run_exit_sequence,
            args=(car_type,),
            daemon=True
            )
        worker.start()


    # 실제 출차 시퀀스
    def _run_exit_sequence(self, car_type):
        # 1) pickup
        self.get_logger().info("[ExitController] pickup_car 시작")
        self.actions.pickup_car(car_type) # 픽업 작업이 끝나면 그 다음 작업을 진행
        # self.is_pickup_done = True # 픽업 완료 상태로 설정
        self.get_logger().info("[ExitController] pickup_car 완료")

        # 2) exit
        self.get_logger().info("[ExitController] exit_to_goal 시작")
        self.actions.exit_to_goal()
        # self.is_exit_done = True  # 출구 이동 완료 상태로 설정
        self.get_logger().info("[ExitController] EXIT SEQUENCE COMPLETED")
    
    # def check_and_trigger_next(self):
    #     """현재 상태를 체크하고, 모든 작업이 끝났는지 확인"""
    #     if self.is_pickup_done and not self.is_exit_done:
    #         self.get_logger().info("[ExitController] 출구 이동 시작")
    #         self.actions.exit_to_goal()

    #     elif self.is_pickup_done and self.is_exit_done:
    #         self.get_logger().info("[ExitController] 모든 동작 완료!")


def main(args=None):
    rclpy.init(args=args)
    node = ExitController()

    # *** 반드시 MultiThreadedExecutor 사용해야 deadlock 없음 ***
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
