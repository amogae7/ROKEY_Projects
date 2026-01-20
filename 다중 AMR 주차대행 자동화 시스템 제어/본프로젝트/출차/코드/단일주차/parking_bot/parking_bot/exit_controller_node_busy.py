#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from parking_bot.exit_actions_robot5 import ExitActions


class ExitController(Node):
    def __init__(self):
        super().__init__("exit_controller", namespace="robot5")

        # 출차 동작 모듈
        self.actions = ExitActions(self, "robot5")

        # Controller 자체도 중복 실행 방지
        self.busy = False

        self.create_subscription(
            String,
            "tuning_done",  
            self.cb_tuning_done,
            10
        )

        self.get_logger().info("[ExitController] READY — waiting for /robot5/tuning_done ...")


    def cb_tuning_done(self, msg):
        if self.busy:
            self.get_logger().warn("[ExitController] 이미 실행 중 → 요청 무시")
            return

        self.busy = True

        car_type = msg.data if msg.data else "mid"
        self.get_logger().info(f"[ExitController] tuning_done → 출차 시퀀스 시작 (car_type={car_type})")
        self._run_exit_sequence(car_type)

    def _run_exit_sequence(self, car_type):
        self.actions.pickup_car(car_type)

        # ExitActions에서 busy가 False로 바뀌면 Controller도 해제
        def check_actions_busy():
            if not self.actions.busy:
                self.get_logger().info("[ExitController] ExitActions 완료 → busy=False")
                self.busy = False
                timer.cancel()

        timer = self.create_timer(0.2, check_actions_busy)


def main(args=None):
    rclpy.init(args=args)
    node = ExitController()

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
