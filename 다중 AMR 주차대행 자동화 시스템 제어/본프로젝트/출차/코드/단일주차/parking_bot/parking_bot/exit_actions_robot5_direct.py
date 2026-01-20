import math
import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.action import ActionClient
from irobot_create_msgs.action import DriveDistance, RotateAngle 


def yaw_to_quat(yaw):
    return (
        0.0,
        0.0,
        math.sin(yaw / 2),
        math.cos(yaw / 2)
    )


class ExitActions:
    def __init__(self, node):
        self.node = node
        self.ns = "robot5"

        # cmd_vel publisher
        self.cmd_pub = node.create_publisher(
            Twist,
            f"/{self.ns}/cmd_vel",
            10
        )

        # 거리 테이블
        self.pickup_distances = {
            "small": 0.1,
            "mid": 0.3,
            "suv": 0.5
        }

        # waypoint & 출구 위치
        self.exit_waypoint = (-1.14, 0.551, math.pi)
        self.exit_pose = (-3.222, 4.3177, math.pi)

        # Nav2 Navigator
        from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
        self.navigator = TurtleBot4Navigator(namespace=self.ns)

    # ===========================================================
    # 1) 차량 픽업 단계 (전진 + 회전)
    # ===========================================================
    def pickup_car(self, car_type):
        d = self.pickup_distances.get(car_type, 0.6)
        self.node.get_logger().info(f"[픽업] {d}m 전진 시작")

        # ------------------------------------------------------
        # 1) DriveDistance 전진
        # ------------------------------------------------------
        drive_client = ActionClient(
            self.node,
            DriveDistance,
            f"/{self.ns}/drive_distance"
        )

        if not drive_client.wait_for_server(timeout_sec=3.0):
            self.node.get_logger().error("drive_distance 서버 없음")
            return

        goal = DriveDistance.Goal()
        goal.distance = float(d)
        goal.max_translation_speed = 0.3

        send_future = drive_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future)
        goal_handle = send_future.result()


        if not goal_handle.accepted:
            self.node.get_logger().error("drive_distance goal rejected")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        self.node.get_logger().info("[픽업] 전진 완료 → 360° 회전 시작")

        # ------------------------------------------------------
        # 2) RotateAngle 액션 기반 회전
        # ------------------------------------------------------
        rotate_client = ActionClient(
            self.node,
            RotateAngle,
            f"/{self.ns}/rotate_angle"
        )

        if not rotate_client.wait_for_server(timeout_sec=3.0):
            self.node.get_logger().error("rotate_angle 서버 없음")
            return

        rotate_goal = RotateAngle.Goal()
        rotate_goal.angle = 2 * math.pi          # 360도
        rotate_goal.max_rotation_speed = 1.5     # rad/s

        self.node.get_logger().info("[픽업] rotate_angle goal 전송...")

        send_future2 = rotate_client.send_goal_async(rotate_goal)
        rclpy.spin_until_future_complete(self.node, send_future2)
        goal_handle2 = send_future2.result()

        if not goal_handle2.accepted:
            self.node.get_logger().error("rotate_angle goal rejected")
            return

        result_future2 = goal_handle2.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future2)

        self.node.get_logger().info("[픽업] 360° 회전 완료!")

    # ===========================================================
    # 2) 출구 이동
    # ===========================================================
    def exit_to_goal(self):
        self.node.get_logger().info("[출차] 180도 회전")

        rotate_client = ActionClient(
            self.node,
            RotateAngle,
            f"/{self.ns}/rotate_angle"
        )

        if not rotate_client.wait_for_server(timeout_sec=3.0):
            self.node.get_logger().error("rotate_angle 서버 없음 (출차용 180° 회전)")
            # rotate 실패해도 그냥 진행할지 말지는 상황에 따라
            # 여기서 return 해도 되고, 그냥 아래 waypoint로 진행해도 됨.
            # 일단은 그냥 진행하지 않고 return 시킴.
            return

        half_turn = RotateAngle.Goal()
        half_turn.angle = math.pi              # 180도
        half_turn.max_rotation_speed = 0.5

        self.node.get_logger().info("[출차] rotate_angle(180°) goal 전송...")

        send_future = rotate_client.send_goal_async(half_turn)
        rclpy.spin_until_future_complete(self.node, send_future)
        goal_handle = send_future.result()

        if not goal_handle.accepted:
            self.node.get_logger().error("rotate_angle(180°) goal rejected")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        self.node.get_logger().info("[출차] 180° 회전 완료 → waypoint로 이동 시작")

        # 1) waypoint 이동
        wx, wy, wyaw = self.exit_waypoint

        wp = PoseStamped()
        wp.header.frame_id = "map"
        wp.pose.position.x = wx
        wp.pose.position.y = wy
        q = yaw_to_quat(wyaw)
        wp.pose.orientation.z = q[2]
        wp.pose.orientation.w = q[3]

        self.node.get_logger().info(
            f"[출차] waypoint 이동 → ({wx:.2f}, {wy:.2f})"
        )

        self.navigator.goToPose(wp)
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.get_logger().info("[출차] waypoint 도착 완료")

        # 2) exit 이동
        ex, ey, eyaw = self.exit_pose

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = ex
        goal.pose.position.y = ey
        q = yaw_to_quat(eyaw)
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        self.node.get_logger().info(
            f"[출차] exit 이동 → ({ex:.2f}, {ey:.2f})"
        )

        self.navigator.goToPose(goal)
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.get_logger().info("[출차] 출구 도착 완료")
