import math
import time

from geometry_msgs.msg import Twist, PoseStamped
from rclpy.action import ActionClient
from irobot_create_msgs.action import DriveDistance, RotateAngle 
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator, TurtleBot4Directions


def yaw_to_quat(yaw: float):
    """yaw(라디안) → quaternion (x, y, z, w)"""
    return (
        0.0,
        0.0,
        math.sin(yaw / 2.0),
        math.cos(yaw / 2.0),
    )


class BeepNode(Node):
    def __init__(self, namespace: str):
        super().__init__("beep_node")
        self.pub = self.create_publisher(AudioNoteVector, f"/{namespace}/cmd_audio", 10)

    def play_beep(self):
        msg = AudioNoteVector()
        msg.append = False
        msg.notes = [
            AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # 삐
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # 뽀
            AudioNote(frequency=880, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # 삐
            AudioNote(frequency=440, max_runtime=Duration(sec=0, nanosec=300_000_000)),  # 뽀
        ]

        self.get_logger().info("삐뽀삐뽀 소리 전송 중...")
        self.pub.publish(msg)
        self.get_logger().info("삐뽀삐뽀 소리 전송 완료...")


class ExitActions:
    """
    출차 시퀀스:
      1) pickup_car(car_type)  : 전진 → 삐뽀 → exit_to_goal()
      2) exit_to_goal()        : 180° 회전 → 중간 WP → 출구
    """

    def __init__(self, node: Node, namespace: str):
        self.node = node
        self.ns = namespace

        # 중복 실행 방지
        self.busy = False

        # 차량별 전진 거리
        self.pickup_distances = {
            "small": 0.3,
            "mid": 0.5,
            "suv": 0.7,
        }

        # waypoint & 출구 위치
        self.exit_pose = (-3.222, 4.3177, math.pi)    # 출구 위치
        self.middle_waypoint = [-1.6231178733012308, 4.21895869581359]  # 중간 지점

        # Nav2 Navigator
        self.navigator = TurtleBot4Navigator(namespace=self.ns)

        # BeepNode (삐뽀 소리용)
        self.beep_node = BeepNode(self.ns)

        # 액션 클라이언트들
        self.drive_client = ActionClient(
            self.node,
            DriveDistance,
            f"/{self.ns}/drive_distance",
        )

        self.rotate_client = ActionClient(
            self.node,
            RotateAngle,
            f"/{self.ns}/rotate_angle",
        )

        # Nav2 상태 체크용 타이머 핸들
        self._middle_timer = None
        self._exit_timer = None

    # ===========================================================
    # 1) 차량 픽업 단계 (전진)
    # ===========================================================
    def pickup_car(self, car_type: str):

        if self.busy:
            self.node.get_logger().warn("[픽업] 이미 출차 시퀀스 실행 중 → 요청 무시")
            return
        self.busy = True

        d = self.pickup_distances.get(car_type, 0.6)
        self.node.get_logger().info(f"[픽업] {d:.2f} m 전진 시작 (car_type={car_type})")

        if not self.drive_client.wait_for_server(timeout_sec=3.0):
            self.node.get_logger().error("[픽업] drive_distance 서버 없음")
            self.busy = False
            return

        goal = DriveDistance.Goal()
        goal.distance = float(d)
        goal.max_translation_speed = 0.3

        self.node.get_logger().info("[픽업] drive_distance goal 전송...")
        send_future = self.drive_client.send_goal_async(goal)
        send_future.add_done_callback(self._pickup_goal_response)

    def _pickup_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.node.get_logger().error(f"[픽업] drive_distance goal 전송 실패: {e}")
            self.busy = False
            return

        if not goal_handle.accepted:
            self.node.get_logger().error("[픽업] drive_distance goal rejected")
            self.busy = False
            return

        self.node.get_logger().info("[픽업] drive_distance goal accepted → 결과 대기")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._pickup_done)

    def _pickup_done(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.node.get_logger().error(f"[픽업] drive_distance 결과 수신 실패: {e}")
            self.busy = False
            return

        self.node.get_logger().info(f"[픽업] 전진 완료 (status={result.status}) → 삐뽀 소리 시작")

        self.beep_node.play_beep()
        time.sleep(1.2)
        self.node.get_logger().info("[픽업] 삐뽀 재생 완료 → exit_to_goal 실행")

        self.exit_to_goal()

    # ===========================================================
    # 2) 출구 이동 (180° 회전 → 중간 waypoint → 출구)
    # ===========================================================
    def exit_to_goal(self):
        self.node.get_logger().info("[출차] 180° 회전 시작")

        if not self.rotate_client.wait_for_server(timeout_sec=3.0):
            self.node.get_logger().error("[출차] rotate_angle 서버 없음 (출차용 180° 회전)")
            self.busy = False
            return

        half_turn = RotateAngle.Goal()
        half_turn.angle = math.pi           # 180도
        half_turn.max_rotation_speed = 0.5

        self.node.get_logger().info("[출차] rotate_angle(180°) goal 전송...")
        send_future = self.rotate_client.send_goal_async(half_turn)
        send_future.add_done_callback(self._rotate_goal_response)

    def _rotate_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.node.get_logger().error(f"[출차] rotate_angle goal 전송 실패: {e}")
            self.busy = False
            return

        if not goal_handle.accepted:
            self.node.get_logger().error("[출차] rotate_angle goal rejected")
            self.busy = False
            return

        self.node.get_logger().info("[출차] rotate_angle goal accepted → 결과 대기")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._rotate_done)

    def _rotate_done(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.node.get_logger().error(f"[출차] rotate_angle 결과 수신 실패: {e}")
            self.busy = False
            return

        self.node.get_logger().info(f"[출차] 180° 회전 완료 (status={result.status})")
        self.node.get_logger().info("[출차] → 중간 waypoint로 이동 시작")

        self._start_nav_to_middle()

    def _start_nav_to_middle(self):
        wx, wy = self.middle_waypoint

        wp = self.navigator.getPoseStamped(
            [wx, wy],
            TurtleBot4Directions.SOUTH
        )
        # wp = PoseStamped()
        # wp.header.frame_id = "map"
        # wp.header.stamp = self.node.get_clock().now().to_msg()
        # wp.pose.position.x = wx
        # wp.pose.position.y = wy
        # q = yaw_to_quat(0.0)
        # wp.pose.orientation.z = q[2]
        # wp.pose.orientation.w = q[3]

        self.node.get_logger().info(f"[출차] 중간 waypoint 이동 → ({wx:.2f}, {wy:.2f})")
        self.navigator.goToPose(wp)

        if self._middle_timer is not None:
            self._middle_timer.cancel()
            self._middle_timer = None

        self._middle_timer = self.node.create_timer(
            0.1, self._check_middle_reached
        )

    def _check_middle_reached(self):
        if not self.navigator.isTaskComplete():
            return

        if self._middle_timer is not None:
            self._middle_timer.cancel()
            self._middle_timer = None

        result = self.navigator.getResult()
        self.node.get_logger().info(f"[출차] 중간 waypoint 도달 완료 (status={result})")

        self.node.get_logger().info("[출차] → 출구로 이동 시작")
        self._start_nav_to_exit()

    def _start_nav_to_exit(self):
        ex, ey, eyaw = self.exit_pose

        # eyaw = π rad 이므로 WEST 방향
        goal = self.navigator.getPoseStamped(
            [ex, ey],
            TurtleBot4Directions.SOUTH
        )

        # goal = PoseStamped()
        # goal.header.frame_id = "map"
        # goal.header.stamp = self.node.get_clock().now().to_msg()
        # goal.pose.position.x = ex
        # goal.pose.position.y = ey
        # q = yaw_to_quat(eyaw)
        # goal.pose.orientation.z = q[2]
        # goal.pose.orientation.w = q[3]

        self.node.get_logger().info(f"[출차] exit 이동 → ({ex:.2f}, {ey:.2f}, yaw={eyaw:.2f})")
        self.navigator.goToPose(goal)

        if self._exit_timer is not None:
            self._exit_timer.cancel()
            self._exit_timer = None

        self._exit_timer = self.node.create_timer(
            0.1, self._check_exit_reached
        )

    def _check_exit_reached(self):
        if not self.navigator.isTaskComplete():
            return

        if self._exit_timer is not None:
            self._exit_timer.cancel()
            self._exit_timer = None

        result = self.navigator.getResult()
        self.node.get_logger().info(f"[출차] 출구 도달 완료 (status={result})")

        # 시퀀스 끝났으니 busy 해제
        self.busy = False
        self.node.get_logger().info("[출차] 출차 시퀀스 종료 → busy=False")

        # 나중에 여기서 도킹이나 다음 작업 큐 연동하면 됨
