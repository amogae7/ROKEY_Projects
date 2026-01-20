import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from parking_bot.exit_actions_robot5 import ExitActions


class ExitController(Node):
    def __init__(self):
        super().__init__("exit_controller", namespace="robot5")

        # 출차 동작 모듈
        self.actions = ExitActions(self, "robot5")  # "robot5" 네임스페이스를 전달

        # 완료 토픽 수신
        self.create_subscription(
            String,
            "waypoint_done",     # => /robot5/waypoint_done
            self.cb_waypoint_done,
            10
        )

        self.get_logger().info("[ExitController] READY — waiting for waypoint_done...")

    # ===========================================================
    # waypoint_done → pickup + exit 실행
    # ===========================================================
    def cb_waypoint_done(self, msg):
        car_type = msg.data if msg.data else "mid"
        self.get_logger().info(f"[ExitController] waypoint_done received → 수행 시작 (car_type={car_type})")

        # 순차적으로 작업 실행
        self._run_exit_sequence(car_type)

    # 실제 출차 시퀀스
    def _run_exit_sequence(self, car_type):
        # 1) 차량 픽업
        self.get_logger().info("[ExitController] pickup_car 시작")
        self.actions.pickup_car(car_type)
        self.get_logger().info("[ExitController] pickup_car 완료")

        # 2) 출구로 이동
        if self.is_pickup_done:
            self.get_logger().info("[ExitController] exit_to_goal 시작")
            self.actions.exit_to_goal()
            self.get_logger().info("[ExitController] EXIT SEQUENCE COMPLETED")

    def check_and_trigger_next(self):
        """현재 상태를 체크하고, 모든 작업이 끝났는지 확인"""
        if self.is_pickup_done and not self.is_exit_done:
            self.get_logger().info("[ExitController] 출구 이동 시작")
            self.actions.exit_to_goal()

        elif self.is_pickup_done and self.is_exit_done:
            self.get_logger().info("[ExitController] 모든 동작 완료!")

def main(args=None):
    rclpy.init(args=args)
    node = ExitController()

    # MultiThreadedExecutor
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
