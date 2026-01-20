# exit_controller_node.py
#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from supabase import create_client, Client
from parking_bot.exit_actions_robot5 import ExitActions


class ExitController(Node):
    def __init__(self):
        super().__init__("exit_controller", namespace="robot5")

        self.callback_group = ReentrantCallbackGroup()

        # ===============================
        # DB 연결 (입차팀과 동일)
        # ===============================
        self.declare_parameter('supabase_url', 'https://shmqecsymzygxatjsqid.supabase.co')
        self.declare_parameter('supabase_key', 'sb_publishable_imLQmNJH4atY59EnnbqLuw_8P-3HPH_')

        url = self.get_parameter('supabase_url').value
        key = self.get_parameter('supabase_key').value
        self.supabase: Client = create_client(url, key)
        self.get_logger().info("✅ Supabase 연결 성공")

        # ExitActions 동작 모듈
        self.actions = ExitActions(self, "robot5")
        self.actions.on_finish = self._on_exit_finished

        # 현재 Task 저장
        self.received_task = None

        # 다음 Task 감지용 플래그
        self.waiting_for_next_task = False
        self.next_task_arrived = False

        # Task 구독
        self.create_subscription(
            String,
            "/task_command/robot5",
            self.cb_task_received,
            10,
            callback_group=self.callback_group
        )

        # tuning_done 구독
        self.create_subscription(
            String,
            "tuning_done",
            self.cb_tuning_done,
            10,
            callback_group=self.callback_group
        )

        self.get_logger().info("[ExitController] READY")

    # -------------------------------
    # Task 수신
    # -------------------------------
    def cb_task_received(self, msg):
        task = json.loads(msg.data)
        task_id = task["task_id"]

        # 새 Task가 왔음을 표시
        if self.waiting_for_next_task:
            self.next_task_arrived = True

        # 현재 Task 저장
        self.received_task = task

        # DB: assigned
        self.mark_task_assigned(task_id)

        # ExitActions에 전달
        self.actions.set_task(task)

        self.get_logger().info(f"[ExitController] Task 수신: {task_id}")

    # -------------------------------
    # tuning_done → exit 시작
    # -------------------------------
    def cb_tuning_done(self, msg):
        car_type = msg.data if msg.data else "mid"

        if self.received_task is None:
            self.get_logger().error("[ExitController] Task 없이 tuning_done 수신")
            return

        self.actions.start_exit_sequence(car_type)

    # -------------------------------
    # ExitActions 종료 콜백
    # -------------------------------
    def _on_exit_finished(self, success: bool):
        task_id = self.received_task["task_id"]

        if success:
            self.mark_task_done(task_id)
        else:
            self.mark_task_failed(task_id)

        self.get_logger().info("[ExitController] Task 종료 후 다음 상태 판단 시작")

        # 다음 Task 기다릴 준비
        self.waiting_for_next_task = True
        self.next_task_arrived = False

        # 2초 기다리며, 새로운 Task가 오는지 감지
        self.timer = self.create_timer(2.0, self._after_wait_for_next_task)

    # -------------------------------
    # 2초 후 다음 Task 여부 판단
    # -------------------------------
    def _after_wait_for_next_task(self):
        self.timer.cancel()
        self.waiting_for_next_task = False

        if self.next_task_arrived:
            self.get_logger().info("📌 [ExitController] 다음 Task 감지됨 → middle waypoint 이동")

            # middle waypoint로 이동 요청
            self.actions.go_to_middle_after_finish()

        else:
            self.get_logger().info("📌 [ExitController] 다음 Task 없음 → docking 수행")

            try:
                self.actions.navigator.dock()
            except Exception as e:
                self.get_logger().error(f"dock 실패: {e}")

    # -------------------------------
    # DB 상태 업데이트 (입차팀 동일)
    # -------------------------------
    def mark_task_assigned(self, task_id):
        try:
            self.supabase.table("tasks").update({"status": "assigned"}).eq("task_id", task_id).execute()
        except Exception as e:
            self.get_logger().error(f"assigned 실패: {e}")

    def mark_task_done(self, task_id):
        try:
            self.supabase.table("tasks").update({"status": "done", "done": True}).eq("task_id", task_id).execute()
        except Exception as e:
            self.get_logger().error(f"done 실패: {e}")

    def mark_task_failed(self, task_id):
        try:
            self.supabase.table("tasks").update({"status": "failed"}).eq("task_id", task_id).execute()
        except Exception as e:
            self.get_logger().error(f"failed 실패: {e}")


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
