#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from parking_bot.exit_actions_robot5_test import ExitActions
from supabase import create_client, Client
import time


class ExitController(Node):
    def __init__(self):
        super().__init__("exit_controller", namespace="robot5")

        self.callback_group = ReentrantCallbackGroup()

        self.actions = ExitActions(self, "robot5")
        self.actions.on_finish = self._on_exit_finished

        self.received_task = None

        # 출차 끝나고 다음 Task가 없을 때 이동할 대기 지점
        self.waiting_point = {
            "x": -0.671,
            "y": 4.2189586,
            "orientation": "NORTH"
        }

        # 다음 Task 감지용 flag
        self.waiting_for_next_task = False
        self.next_task_arrived = False

        # nav_to_pose 에 "이제 다음 Task 실행해도 된다" 신호를 보내는 토픽
        self.exit_ready_pub = self.create_publisher(
            String,
            "/robot5/exit_ready",
            10
        )

        # -------------------------------
        # 🚀 Supabase 연결
        # -------------------------------
        url = "https://shmqecsymzygxatjsqid.supabase.co"
        key = "sb_publishable_imLQmNJH4atY59EnnbqLuw_8P-3HPH_"
        self.supabase: Client = create_client(url, key)
        self.get_logger().info("[DB] Supabase 연결 완료")

        # Task 구독 (/task_command/robot5 는 절대 경로)
        self.create_subscription(
            String,
            "/task_command/robot5",
            self.cb_task_received,
            10,
            callback_group=self.callback_group
        )

        # tuning_done 구독 (실제 토픽: /robot5/tuning_done)
        self.create_subscription(
            String,
            "waypoint_done",
            self.cb_tuning_done,
            10,
            callback_group=self.callback_group
        )

        self.get_logger().info("[ExitController] READY")

    # =========================================================
    # Task(JSON) 수신
    # =========================================================
    def cb_task_received(self, msg):
        task = json.loads(msg.data)
        task_id = task["task_id"]
        task_location = task["start_location"]

        self.get_logger().info(f"[ExitController] Task 수신: {task_location}")

        # 🚨 1) 출차가 아직 시작되지 않았고, 이미 Task를 가진 상태라면 새로운 Task 무시
        # (단, 출차 완료 후 '다음 Task 대기 상태'일 때는 허용해야 함)
        if (
            self.received_task is not None
            and not self.actions.busy
            and not self.waiting_for_next_task      # ★ 패치: 다음 Task 대기 상태가 아닐 때만 막기
        ):
            self.get_logger().warn("[ExitController] 이미 대기 중인 Task가 있어 새로운 Task 무시")
            return

        # 🚨 2) 출차 실행 중이면 무조건 무시
        if self.actions.busy:
            self.get_logger().warn("작업 중 Task 무시")
            return

        # 🚨 3) 같은 Task ID 반복 수신 무시
        if self.received_task and self.received_task["task_id"] == task_id:
            self.get_logger().info("같은 Task 반복 수신 → 무시")
            return

        # 정상적으로 Task 수신
        self.received_task = task
        self.actions.set_task(task)
        self.get_logger().info(f"[ExitController] 새로운 Task 저장: {task_id}")

        # 출차 종료 후 waiting 상태였다면 next_task_arrived 처리
        if self.waiting_for_next_task:
            self.next_task_arrived = True

    # =========================================================
    # 튜닝 완료 → 출차 시퀀스 시작
    # =========================================================
    def cb_tuning_done(self, msg):
        if self.received_task is None:
            self.get_logger().error("Task 없이 tuning_done 수신됨")
            return

        car_type = msg.data if msg.data else "mid"
        self.actions.start_exit_sequence(car_type)

    # =========================================================
    # 🚀 Supabase DB: Task 완료 업데이트
    # =========================================================
    def mark_task_done(self, task_id: str):
        try:
            self.supabase.table("tasks").update({
                "done": True,
                "status": "done"
            }).eq("task_id", task_id).execute()

            self.get_logger().info(f"[DB] Task DONE 처리 완료: {task_id}")
        except Exception as e:
            self.get_logger().error(f"[DB] Task DONE 처리 실패: {e}")

    # =========================================================
    # ExitActions → 출차 완료 알림
    # =========================================================
    def _on_exit_finished(self, success):
        if success:
            self.get_logger().info("[ExitController] 출차 성공 → DB 업데이트 & 다음 Task 판단 시작")

            # DB 업데이트 수행
            if self.received_task:
                task_id = self.received_task["task_id"]
                self.mark_task_done(task_id)

            # 현재 Task는 완전히 종료
            self.received_task = None

        else:
            self.get_logger().error("[ExitController] 출차 실패 → 대기")

        # 다음 Task를 기다리는 상태
        self.waiting_for_next_task = True
        self.next_task_arrived = False

        # 일정 시간(예: 5초) 동안 다음 Task를 기다림
        self.timer = self.create_timer(5.0, self._after_wait_for_next_task)

    # =========================================================
    # 일정 시간 후 다음 Task 여부 확인
    # =========================================================
    def _after_wait_for_next_task(self):
        self.timer.cancel()
        self.waiting_for_next_task = False

        # ✅ 다음 Task가 들어온 경우 → middle waypoint로 이동 후 nav_to_pose 에 ready 알림
        if self.next_task_arrived:
            self.get_logger().info("[ExitController] 다음 Task 감지됨 → middle waypoint 이동")

            wp = {
                "x": self.actions.middle_waypoint[0],
                "y": self.actions.middle_waypoint[1],
                "orientation": "EAST"
            }
            self.actions._go_to_pose(
                wp,
                "[ExitController] middle waypoint 도착 → nav_to_pose 에 ready 발행",
                next_step=self._publish_exit_ready
            )

        # ❌ 다음 Task가 없는 경우 → waiting_point로 이동 후 도킹
        else:
            self.get_logger().info("[ExitController] 다음 Task 없음 → waiting_point로 이동 후 dock 수행")

            wp = self.waiting_point
            self.actions._go_to_pose(
                wp,
                "[ExitController] waiting_point 도착 → 도킹 수행",
                next_step=self._dock_robot
            )

    # =========================================================
    # middle waypoint 도착 후 → nav_to_pose 에 "ready" 신호 발행
    # =========================================================
    def _publish_exit_ready(self):
        msg = String()
        msg.data = "ready"
        self.exit_ready_pub.publish(msg)
        self.get_logger().info("[ExitController] middle waypoint 도착 → nav_to_pose 에 ready 신호 발행")

        # 이제 다음 Task 출차를 위해 ExitActions busy 해제
        self.actions.busy = False

    # =========================================================
    # 도킹 수행
    # =========================================================
    def _dock_robot(self):
        try:
            self.actions.navigator.dock()
            self.get_logger().info("docking 완료")
        except Exception as e:
            self.get_logger().error(f"dock 실패: {e}")
        finally:
            # 도킹 후에는 더 이상 출차 동작 없음 → busy 해제
            self.actions.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = ExitController()

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
