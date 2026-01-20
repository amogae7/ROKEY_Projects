# exit_actions_robot5_test.py
import math
import time
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from irobot_create_msgs.action import RotateAngle
from irobot_create_msgs.msg import AudioNoteVector, AudioNote
from builtin_interfaces.msg import Duration
from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Navigator, TurtleBot4Directions
)
from std_msgs.msg import String


def direction_from_string(s: str):
    mapping = {
        "NORTH": TurtleBot4Directions.NORTH,
        "SOUTH": TurtleBot4Directions.SOUTH,
        "EAST":  TurtleBot4Directions.EAST,
        "WEST":  TurtleBot4Directions.WEST,
    }
    return mapping.get(s.upper(), TurtleBot4Directions.EAST)


class BeepNode:
    def __init__(self, node, namespace):
        self.node = node
        self.pub = node.create_publisher(
            AudioNoteVector, f"/{namespace}/cmd_audio", 10
        )

    def play_beep(self):
        msg = AudioNoteVector()
        msg.append = False
        msg.notes = [
            AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
            AudioNote(frequency=440, max_runtime=Duration(nanosec=300_000_000)),
            AudioNote(frequency=880, max_runtime=Duration(nanosec=300_000_000)),
            AudioNote(frequency=440, max_runtime=Duration(nanosec=300_000_000)),
        ]
        self.node.get_logger().info("[삐뽀] 재생")
        self.pub.publish(msg)


class ExitActions:

    def __init__(self, node, namespace):
        self.node = node
        self.ns = namespace

        self.busy = False
        self.task = None
        self.on_finish = None

        self.navigator = TurtleBot4Navigator(namespace=self.ns)
        self.beep = BeepNode(node, namespace)
        self.rotate_client = ActionClient(node, RotateAngle, f"/{self.ns}/rotate_angle")

        # 출차 완료 알림 토픽
        self.exit_done_pub = self.node.create_publisher(
            String,
            f"/{self.ns}/exit_done",
            10
        )

        # 중간 웨이포인트 (직각 주행용 + 대기 위치)
        self.middle_waypoint = [-1.6231178, 4.35]
        self.waiting_point = [-0.671, 4.35]

        self.timer = None
        self.nav_started = False

    # =========================================================
    # 외부에서 Task JSON 설정
    # =========================================================
    def set_task(self, task_json):
        self.task = task_json

    # =========================================================
    # 출차 시퀀스 시작
    # =========================================================
    def start_exit_sequence(self, car_type):
        if self.busy:
            self.node.get_logger().warn("[출차] 이미 실행 중")
            return
        if self.task is None:
            self.node.get_logger().error("[출차] Task 없음")
            return

        self.busy = True
        start = self.task["start_coords"]

        # 1) start_coords로 이동
        self._go_to_pose(start, "[출차] start_coords 도착", next_step=self._after_start)

        

    # =========================================================
    # 2) start_coords 도착 → 삐뽀 + 180도 회전
    # =========================================================
    def _after_start(self):
        # 픽업 완료 알림 삐뽀
        self.beep.play_beep()
        time.sleep(1.2)

        goal = RotateAngle.Goal()
        goal.angle = math.pi
        goal.max_rotation_speed = 1.5

        if not self.rotate_client.wait_for_server(timeout_sec=3.0):
            self.node.get_logger().error("[출차] rotate 서버 없음")
            self._finish_failed()
            return

        future = self.rotate_client.send_goal_async(goal)
        future.add_done_callback(self._rotate_done)

    # =========================================================
    # 3) 회전 완료 → start_waypoint(SOUTH로 강제) 이동
    # =========================================================
    def _rotate_done(self, future):
        self.node.get_logger().info("[출차] 회전 완료 → start waypoint 이동")

        # JSON 기반 start_waypoint_coords 복사 후 orientation만 SOUTH로 강제 변경
        wp = dict(self.task["start_waypoint_coords"])
        wp["orientation"] = "WEST"

        self._go_to_pose(wp, "[출차] start waypoint 도착", next_step=self._after_waypoint)


    # =========================================================
    # 4) start waypoint → middle waypoint 이동
    # =========================================================
    def _after_waypoint(self):
        wp = {
            "x": self.middle_waypoint[0],
            "y": self.middle_waypoint[1],
            "orientation": "SOUTH"
        }
        self._go_to_pose(wp, "[출차] middle waypoint 도착", next_step=self._after_middle)


    # =========================================================
    # 5) middle waypoint → 출구(target_coords) 이동
    # =========================================================
    def _after_middle(self):
        exit_pose = self.task["target_coords"]
        self._go_to_pose(exit_pose, "[출차] 출구 도착", next_step=self._finish_success)


    # =========================================================
    # 6) 출구 도착 → 삐뽀 + exit_done publish + middle 복귀 후 완료
    # =========================================================
    def _finish_success(self):
        self.node.get_logger().info("[출차] 출구 도착 → 출차 완료")

        # 6-1) 출구에서 삐뽀 (출차 완료 알림)
        self.beep.play_beep()
        time.sleep(1.5)

        # 6-2) 출차 완료 토픽 발행 (task_id 포함)
        msg = String()
        msg.data = self.task["task_id"] if self.task else "UNKNOWN"
        self.exit_done_pub.publish(msg)
        self.node.get_logger().info(f"[출차] 출차 완료 토픽 발행: {msg.data}")

        self.busy = False
        if self.on_finish:
            self.on_finish(success=True)

    def _finish_failed(self):
        self.busy = False
        if self.on_finish:
            self.on_finish(success=False)

    # =========================================================
    # Nav2 이동 공통 함수
    # =========================================================
    def _go_to_pose(self, coord, log_msg, next_step):

        self.nav_started = False

        x, y = coord["x"], coord["y"]
        direction = direction_from_string(coord["orientation"])

        goal = self.navigator.getPoseStamped([x, y], direction)
        self.navigator.goToPose(goal)

        self.timer = self.node.create_timer(
            0.1, lambda: self._check_nav(log_msg, next_step)
        )

    def _check_nav(self, log_msg, next_step):

        # 첫 콜백은 무조건 한 번 버려서 isTaskComplete가 너무 빨리 True 되는 것을 방지
        if not self.nav_started:
            self.nav_started = True
            return

        if not self.navigator.isTaskComplete():
            return

        self.timer.cancel()
        self.timer = None

        self.node.get_logger().info(log_msg)
        next_step()
