#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# from exit_actions import ExitActions
from parking_bot.exit_actions import ExitActions


class ExitController(Node):
    def __init__(self):
        super().__init__("exit_controller_robot5", namespace="robot5")

        self.actions = ExitActions(self)

        # 명령 토픽 4종
        self.create_subscription(String, "cmd_go_front", self.cb_go_front, 10)
        self.create_subscription(String, "cmd_tune", self.cb_tune, 10)
        self.create_subscription(String, "cmd_pickup", self.cb_pickup, 10)
        self.create_subscription(String, "cmd_go_exit", self.cb_exit, 10)

        self.get_logger().info("ExitController(robot5) started.")

    def cb_go_front(self, msg):
        self.actions.go_to_slot_front(msg.data)

    def cb_tune(self, msg):
        self.actions.tune_orientation(msg.data)

    def cb_pickup(self, msg):
        self.get_logger().info("recieved pickup topic")
        self.actions.pickup_car(msg.data)

    def cb_exit(self, msg):
        self.actions.go_to_exit()


def main():
    rclpy.init()
    node = ExitController()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
