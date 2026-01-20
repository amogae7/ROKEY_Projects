import math
import rclpy
from geometry_msgs.msg import Twist, PoseStamped
from rclpy.action import ActionClient
from irobot_create_msgs.action import DriveDistance, RotateAngle 
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator
import time

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
        # 절대 토픽 경로 사용
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
    출차 시퀀스 전체를 '비동기'로 처리하는 클래스.

    흐름:
      1) pickup_car(car_type)
         - DriveDistance 액션 전송 (전진)
         - 결과 수신 후 → 삐뽀 소리 → exit_to_goal() 호출

      2) exit_to_goal()
         - RotateAngle(180도) 액션 전송
         - 결과 수신 후 → 중간 waypoint로 Nav2 이동 시작
         - navigator.isTaskComplete()는 타이머 콜백으로 주기적으로 확인
         - 중간 waypoint 도달 후 → exit pose로 Nav2 이동
    """

    def __init__(self, node: Node, namespace: str):
        self.node = node
        self.ns = namespace

        # cmd_vel publisher (필요 시 수동 제어용 – 지금은 사용 X)
        # self.cmd_pub = node.create_publisher(Twist, f"/{self.ns}/cmd_vel", 10)

        # 차량별 전진 거리
        self.pickup_distances = {
            "small": 0.3,
            "mid": 0.5,
            "suv": 0.7,
        }

        # waypoint & 출구 위치
        self.exit_waypoint = (-1.14, 0.551, math.pi)  # 기존 waypoint (필요시 사용)
        self.exit_pose = (-3.222, 4.3177, math.pi)    # 출구 위치
        self.middle_waypoint = [-1.6231178733012308, 4.21895869581359]  # 중간 지점

        # Nav2 Navigator
        from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Navigator
        self.navigator = TurtleBot4Navigator(namespace=self.ns)

        # BeepNode (삐뽀 소리용)
        self.beep_node = BeepNode(self.ns)

        # 액션 클라이언트들 (한 번 만들어서 계속 사용)
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

        self.test_queue_exists = False

    # ===========================================================
    # 1) 차량 픽업 단계 (전진)
    # ===========================================================
    def pickup_car(self, car_type: str):
        d = self.pickup_distances.get(car_type, 0.6)
        self.node.get_logger().info(f"[픽업] {d:.2f} m 전진 시작 (car_type={car_type})")

        if not self.drive_client.wait_for_server(timeout_sec=3.0):
            self.node.get_logger().error("[픽업] drive_distance 서버 없음")
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
            return

        if not goal_handle.accepted:
            self.node.get_logger().error("[픽업] drive_distance goal rejected")
            return

        self.node.get_logger().info("[픽업] drive_distance goal accepted → 결과 대기")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._pickup_done)

    def _pickup_done(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.node.get_logger().error(f"[픽업] drive_distance 결과 수신 실패: {e}")
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
            return

        if not goal_handle.accepted:
            self.node.get_logger().error("[출차] rotate_angle goal rejected")
            return

        self.node.get_logger().info("[출차] rotate_angle goal accepted → 결과 대기")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._rotate_done)

    def _rotate_done(self, future):
        try:
            result = future.result()
        except Exception as e:
            self.node.get_logger().error(f"[출차] rotate_angle 결과 수신 실패: {e}")
            return

        self.node.get_logger().info(f"[출차] 180° 회전 완료 (status={result.status})")
        self.node.get_logger().info("[출차] → 중간 waypoint로 이동 시작")

        self._start_nav_to_middle()

    # ---------------- 중간 waypoint Nav2 ----------------
    def _start_nav_to_middle(self):
        wx, wy = self.middle_waypoint
        wp = PoseStamped()
        wp.header.frame_id = "map"
        wp.header.stamp = self.node.get_clock().now().to_msg()
        wp.pose.position.x = wx
        wp.pose.position.y = wy
        q = yaw_to_quat(0.0)  # 방향은 일단 0 rad
        wp.pose.orientation.z = q[2]
        wp.pose.orientation.w = q[3]
        # wp = self.navigator.getPoseStamped(
        #     [wx, wy],
        #     TurtleBot4Directions.WEST  
        # )
        # self.navigator.goToPose(wp)


        self.node.get_logger().info(f"[출차] 중간 waypoint 이동 → ({wx:.2f}, {wy:.2f})")
        self.navigator.goToPose(wp)

        # 이전 타이머 있으면 정리
        if self._middle_timer is not None:
            self._middle_timer.cancel()
            self._middle_timer = None

        # 주기적으로 isTaskComplete() 확인
        self._middle_timer = self.node.create_timer(
            0.1, self._check_middle_reached
        )

    def _check_middle_reached(self):
        if not self.navigator.isTaskComplete():
            return

        # 타이머 정리
        if self._middle_timer is not None:
            self._middle_timer.cancel()
            self._middle_timer = None

        result = self.navigator.getResult()
        self.node.get_logger().info(f"[출차] 중간 waypoint 도달 완료 (status={result})")

        self.node.get_logger().info("[출차] → 출구로 이동 시작")
        self._start_nav_to_exit()

    # ---------------- 출구 Nav2 ----------------
    def _start_nav_to_exit(self):
        ex, ey, eyaw = self.exit_pose
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.pose.position.x = ex
        goal.pose.position.y = ey
        q = yaw_to_quat(eyaw)
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]

        self.node.get_logger().info(f"[출차] exit 이동 → ({ex:.2f}, {ey:.2f}, yaw={eyaw:.2f})")
        self.navigator.goToPose(goal)

        # 이전 타이머 있으면 정리
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

        # ========================= 테스트용 큐 분기점 ===================================
        queue_exists = self.test_queue_exists

        if queue_exists:
            self.node.get_logger().info("[출차] 큐가 있음 → 다음 명령 실행")
            self._run_next_task()
        else:
            self.node.get_logger().info("[출차] 큐 없음 → 도킹 시작")
            self._start_docking()
    
    def _run_next_task(self):
        self.node.get_logger().warn("[다음 작업] 테스트 모드: 다음 작업 실행됨!")
        # 여기에 다음 Nav2 goal 또는 drive_distance 등 넣으면 됨.


    def _start_docking(self):
        self.node.get_logger().warn("[도킹] 테스트 모드: 도킹 절차 시작됨!")
        self.navigator.dock()
        # 여기에 도킹 시퀀스(예: rotate, goToPose 등) 넣으면 됨.