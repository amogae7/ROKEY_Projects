import math
import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.action import ActionClient
from irobot_create_msgs.action import DriveDistance, RotateAngle 
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration
from rclpy.node import Node

def yaw_to_quat(yaw):
    return (
        0.0,
        0.0,
        math.sin(yaw / 2),
        math.cos(yaw / 2)
    )


class BeepNode(Node):
    def __init__(self, namespace):
        super().__init__('beep_node')
        self.pub = self.create_publisher(AudioNoteVector, f'/{namespace}/cmd_audio', 10)

    def play_beep(self):
        msg = AudioNoteVector()
        msg.append = False
        msg.notes = [
            AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # 삐
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # 뽀
            AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # 삐
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # 뽀
        ]
        
        self.get_logger().info('삐뽀삐뽀 소리 전송 중...')
        self.pub.publish(msg)
        self.get_logger().info('삐뽀삐뽀 소리 전송 완료...')


class ExitActions:
    def __init__(self, node, namespace):
        self.node = node
        self.ns = namespace

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
        self.exit_waypoint = (-1.14, 0.551, math.pi)       # 기존 waypoint
        self.exit_pose = (-3.222, 4.3177, math.pi)         # 출구 위치
        self.middle_waypoint = [-1.6231178733012308, 4.21895869581359]  # 중간 지점

        # Nav2 Navigator
        from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
        self.navigator = TurtleBot4Navigator(namespace=self.ns)

        # BeepNode 초기화 (삐뽀 소리 전송)
        self.beep_node = BeepNode(self.ns)

    # ===========================================================
    # 1) 차량 픽업 단계 (전진)
    # ===========================================================
    def pickup_car(self, car_type):
        d = self.pickup_distances.get(car_type, 0.6)
        self.node.get_logger().info(f"[픽업] {d}m 전진 시작")

        # ---- DriveDistance 전진 ----
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

        self.node.get_logger().info(f"[픽업] 전진1")

        send_future = drive_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, send_future)
        goal_handle = send_future.result()
        self.node.get_logger().info(f"[픽업] 전진2")
        
        if not goal_handle.accepted:
            self.node.get_logger().error("drive_distance goal rejected")
            return

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)

        self.node.get_logger().info("[픽업] 전진 완료 → 삐뽀 소리 시작")

        # 삐뽀 소리 전송
        self.beep_node.play_beep()

    # ===========================================================
    # 2) 출구 이동 (중간 waypoint 추가)
    # ===========================================================
    def exit_to_goal(self):
        self.node.get_logger().info("[출차] 180도 회전")

        # 180도 회전
        rotate_client = ActionClient(
            self.node,
            RotateAngle,
            f"/{self.ns}/rotate_angle"
        )

        if not rotate_client.wait_for_server(timeout_sec=3.0):
            self.node.get_logger().error("rotate_angle 서버 없음 (출차용 180° 회전)")
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

        self.node.get_logger().info("[출차] 180° 회전 완료 → 중간 waypoint로 이동 시작")

        # 1) 중간 waypoint 이동 (goToPose 호출)
        wx, wy = self.middle_waypoint
        wp = PoseStamped()
        wp.header.frame_id = "map"
        wp.pose.position.x = wx
        wp.pose.position.y = wy
        q = yaw_to_quat(0)  # 0으로 회전 설정
        wp.pose.orientation.z = q[2]
        wp.pose.orientation.w = q[3]

        self.node.get_logger().info(f"[출차] 중간 waypoint 이동 → ({wx:.2f}, {wy:.2f})")

        self.navigator.goToPose(wp)
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.get_logger().info("[출차] 중간 waypoint 도달 완료")

        # 2) exit 이동 (출구로 이동)
        ex, ey, eyaw = self.exit_pose
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = ex
        goal.pose.position.y = ey
        q = yaw_to_quat(eyaw)
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        self.node.get_logger().info(f"[출차] exit 이동 → ({ex:.2f}, {ey:.2f})")

        self.navigator.goToPose(goal)
        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.get_logger().info("[출차] 출구 도달 완료")
