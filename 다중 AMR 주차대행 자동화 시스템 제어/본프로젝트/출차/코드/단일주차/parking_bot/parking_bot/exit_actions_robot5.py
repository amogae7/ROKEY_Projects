# exit_actions_robot5.py
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
    """출차 동작 전담. DB는 전혀 모름. 완료 시 on_finish() 호출만 수행."""
    
    def __init__(self, node, namespace: str):
        self.node = node
        self.ns = namespace

        self.busy = False
        self.task = None
        self.on_finish = None  # ExitController가 넣어줄 콜백

        self.navigator = TurtleBot4Navigator(namespace=self.ns)
        self.beep = BeepNode(node, namespace)

        self.rotate_client = ActionClient(node, RotateAngle, f"/{self.ns}/rotate_angle")

        # 고정된 middle waypoint
        self.middle_waypoint = [-1.6231178, 4.2189586]
        self.timer = None

    # Task 저장(ExitController가 호출)
    def set_task(self, task_json):
        self.task = task_json

    # 출차 시작
    def start_exit_sequence(self, car_type):
        if self.busy:
            self.node.get_logger().warn("[출차] 이미 실행 중")
            return
        if self.task is None:
            self.node.get_logger().error("[출차] Task 없음")
            return

        self.busy = True
        start = self.task["start_coords"]

        self._go_to_pose(start, "[출차] start_coords 도착", next_step=self._after_start)

    def _after_start(self):
        self.beep.play_beep()
        time.sleep(1.2)

        goal = RotateAngle.Goal()
        goal.angle = math.pi
        goal.max_rotation_speed = 1.0

        if not self.rotate_client.wait_for_server(timeout_sec=3.0):
            self.node.get_logger().error("[출차] rotate 서버 없음")
            self._finish_failed()
            return

        future = self.rotate_client.send_goal_async(goal)
        future.add_done_callback(self._rotate_done)

    def _rotate_done(self, future):
        self.node.get_logger().info("[출차] 회전 완료 → DB waypoint 이동")

        wp = self.task["waypoint_coords"]
        self._go_to_pose(
            wp, "[출차] DB waypoint 도착", next_step=self._after_waypoint
        )

    def _after_waypoint(self):
        wp = {
            "x": self.middle_waypoint[0],
            "y": self.middle_waypoint[1],
            "orientation": "SOUTH"
        }
        self._go_to_pose(
            wp, "[출차] middle waypoint 도착", next_step=self._after_middle
        )

    def _after_middle(self):
        exit_pose = self.task["target_coords"]
        self._go_to_pose(
            exit_pose, "[출차] 출구 도착", next_step=self._finish_success
        )

    def _finish_success(self):
        self.node.get_logger().info("[출차] 출차 완료")
        self.busy = False

        # === ExitController에게 알림(여기서 DB 업데이트 안 함) ===
        if self.on_finish:
            self.on_finish(success=True)

    def _finish_failed(self):
        self.busy = False
        if self.on_finish:
            self.on_finish(success=False)

    # NAV2 공통 처리
    def _go_to_pose(self, coord, log_msg, next_step):
        x, y = coord["x"], coord["y"]
        direction = direction_from_string(coord["orientation"])

        goal = self.navigator.getPoseStamped([x, y], direction)
        self.navigator.goToPose(goal)

        self.timer = self.node.create_timer(
            0.1, lambda: self._check_nav(log_msg, next_step)
        )

    def _check_nav(self, log_msg, next_step):
        if not self.navigator.isTaskComplete():
            return

        self.timer.cancel()
        self.timer = None

        self.node.get_logger().info(log_msg)
        next_step()

    def go_to_middle_after_finish(self):
        """Exit 완료 후 다음 Task 있을 때 middle waypoint로 이동."""
        wp = {
            "x": self.middle_waypoint[0],
            "y": self.middle_waypoint[1],
            "orientation": "SOUTH"
        }

        self._go_to_pose(
            wp, "[출차] 다음 Task 처리 전에 middle waypoint로 이동 완료",
            next_step=lambda: self.node.get_logger().info("[출차] middle 도착 후 다음 Task 대기")
        )

