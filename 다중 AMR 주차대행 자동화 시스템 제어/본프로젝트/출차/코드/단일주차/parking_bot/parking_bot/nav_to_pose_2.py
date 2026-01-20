#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Navigator,
    TurtleBot4Directions,
)
from nav2_simple_commander.robot_navigator import TaskResult

from supabase import create_client

# JSON orientation 문자열 -> Nav2용 yaw(deg)
ORIENTATION_TO_YAW = {
    "NORTH": TurtleBot4Directions.NORTH,
    "EAST": TurtleBot4Directions.EAST,
    "SOUTH": TurtleBot4Directions.SOUTH,
    "WEST": TurtleBot4Directions.WEST,
}


class JsonNavToPoseWithPre(Node):
    def __init__(self):
        super().__init__('json_nav_to_pose_with_pre')

        # ✅ (필요하다면 여기서 self.supabase 초기화)
        # from supabase import create_client
        # 🔹 Supabase 초기화 (환경변수나 하드코딩 중 택1)
        SUPABASE_URL = "https://shmqecsymzygxatjsqid.supabase.co"
        SUPABASE_KEY = "sb_publishable_imLQmNJH4atY59EnnbqLuw_8P-3HPH_"  # 실제 키
        self.supabase = create_client(SUPABASE_URL, SUPABASE_KEY)

        # TurtleBot4 Navigator
        self.navigator = TurtleBot4Navigator()
        self.get_logger().info("json_nav_to_pose_with_pre node initialized")

        # --- 중간 포즈 (먼저 이동할 위치) ---
        self.pre_pose_xy = [-1.6231178733012308, 4.21895869581359]
        self.pre_pose_yaw_deg = TurtleBot4Directions.EAST  # 필요하면 수정 가능

        # 상태 변수들
        self.pre_pose_reached = False    # pre_pose 이동 완료 여부
        self.undocked_once = False       # undock 후 pre_pose 한 번만 이동
        self.ready_for_task = True       # ✅ EXIT_DONE 받을 때까지는 False로 막음

        # --- 1) 도킹 상태 확인 & 도킹 ---
        if not self.navigator.getDockedStatus():
            self.navigator.info('Docking before initializing pose')
            self.navigator.dock()
        self.get_logger().info('도킹 상태 확인 & 도킹 완료')

        # --- 2) 초기 포즈 설정 ---
        initial_pose = self.navigator.getPoseStamped(
            [-0.09684580061153468, 4.317650642336148],
            0.0  # NORTH
        )
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info('초기 포즈 설정 (-0.243, 4.31765, NORTH)')

        # --- 3) Nav2 활성화 대기 ---
        self.navigator.waitUntilNav2Active()
        self.navigator.info('Nav2 is active. Waiting for /task_command/robot5...')

        # --- 4) Task(JSON) 구독 ---
        self.task_sub = self.create_subscription(
            String,
            '/task_command/robot5',
            self.task_callback,
            10
        )

        # --- 5) waypoint 완료 토픽 퍼블리셔 ---
        self.waypoint_done_pub = self.create_publisher(
            String,
            '/robot5/waypoint_done',
            10
        )

        # --- 6) EXIT_DONE 구독: 이걸 받아야 다시 Task 수락 가능 ---
        self.exit_done_sub = self.create_subscription(
            String,
            '/robot5/exit_done',
            self.exit_done_callback,
            10
        )

    # ========================================
    # 🔹 Task를 'assigned' 상태로 변경하는 함수
    # ========================================
    def mark_task_assigned(self, task_id: str):
        """Task를 'assigned' 상태로 변경"""
        try:
            # self.supabase 는 네가 쓰던 Supabase 클라이언트 객체라고 가정
            self.supabase.table('tasks').update({
                'status': 'assigned'
            }).eq('task_id', task_id).execute()
            self.get_logger().info(f":흰색_확인_표시: Task 할당 완료: {task_id}")
        except Exception as e:
            self.get_logger().error(f":x: Task 할당 실패: {e}")

    # ========================================
    # 🔹 EXIT_DONE 콜백: 여기서 다음 Task를 받을 수 있게 풀어줌
    # ========================================
    def exit_done_callback(self, msg: String):
        """
        /robot5/exit_done 수신 시:
        - 다음 /task_command/robot5 를 다시 받을 수 있도록 ready_for_task=True
        """
        self.get_logger().info(f"[exit_done] '{msg.data}' 수신 → 다음 Task 수락 가능")
        self.ready_for_task = True

    # ========================================
    # JSON Task 콜백
    # ========================================
    def task_callback(self, msg: String):
        self.get_logger().info('task_callback subbed!')

        # ✅ 이미 하나 처리 중이면, exit_done 올 때까지 무시
        if not self.ready_for_task:
            self.get_logger().warn(
                "[task_callback] 현재 EXIT 작업 진행 중 → /robot5/exit_done "
                "수신 전까지 새 Task 무시"
            )
            return

        try:
            task = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"[task_callback] JSON 파싱 실패: {e}")
            return

        # 이 Task를 수락하기로 했으니, 바로 ready_for_task를 False로 막음
        self.ready_for_task = False

        # ✅ 여기서 먼저 DB에 'assigned'로 표시 → 그 다음에 undock + 이동
        task_id = task.get("task_id", None)
        if task_id:
            self.mark_task_assigned(task_id)
            self.get_logger().info('DB에 assigned 송신')

        # 여기서는 EXIT_SINGLE 기준으로 waypoint 사용한다고 가정
        waypoint_name = task.get("start_waypoint_location", None)
        waypoint_coords = task.get("start_waypoint_coords", None)

        if waypoint_coords is None:
            self.get_logger().warn("Task에 start_waypoint_coords가 없음. 무시함.")
            return

        if waypoint_name is None:
            waypoint_name = "UNKNOWN_WAYPOINT"

        try:
            x = float(waypoint_coords["x"])
            y = float(waypoint_coords["y"])
        except Exception as e:
            self.get_logger().error(f"start_waypoint_coords 안에 x,y 형식이 잘못됨: {e}")
            return

        ori_str = waypoint_coords.get("orientation", "NORTH")
        ori_str = ori_str.upper()

        if ori_str not in ORIENTATION_TO_YAW:
            self.get_logger().warn(
                f"알 수 없는 orientation '{ori_str}' → NORTH로 대체"
            )
            ori_str = "NORTH"

        yaw_deg = float(ORIENTATION_TO_YAW[ori_str])

        self.get_logger().info(
            f"📌 Task 수신 → waypoint '{waypoint_name}' 로 이동 요청 "
            f"(x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.1f}deg)"
        )

        # 실제 Nav2 이동 로직 호출 (여기서 undock() 이 실행됨)
        self.move_with_pre_pose(waypoint_name, x, y, yaw_deg)

    # ========================================
    # pre_pose + 최종 goal 이동 로직
    # ========================================
    def move_with_pre_pose(self, waypoint_name: str, x: float, y: float, yaw_deg: float):
        """
        1) 아직 undock 안 했으면 한 번만 undock
        2) 아직 pre_pose_reached=False 이면 pre_pose로 먼저 이동
        3) 그 다음 최종 goal_pose로 이동
        4) SUCCEEDED 시 /robot5/waypoint_done 발행
        """
        # --- 도킹되어 있다면 undock 먼저 (한번만) ---
        if self.navigator.getDockedStatus() and not self.undocked_once:
            self.navigator.info('Robot is docked. Undocking before moving.')
            self.navigator.undock()
            self.undocked_once = True  # 첫 번째만 undock

        # --- 1단계: pre_pose로 이동 (아직 안 갔으면) ---
        if not self.pre_pose_reached:
            pre_goal = self.navigator.getPoseStamped(
                self.pre_pose_xy,
                self.pre_pose_yaw_deg
            )
            self.navigator.info(
                f"Step 1: Going to pre-pose at "
                f"(x={self.pre_pose_xy[0]:.3f}, y={self.pre_pose_xy[1]:.3f}, "
                f"yaw={self.pre_pose_yaw_deg:.1f}deg)"
            )

            self.navigator.goToPose(pre_goal)

            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.1)

            pre_result = self.navigator.getResult()

            if pre_result != TaskResult.SUCCEEDED:
                self.navigator.warn(
                    f"Pre-pose navigation FAILED or CANCELED (result={pre_result}). "
                    f"Skipping final waypoint navigation."
                )
                return

            self.navigator.info("Pre-pose reached successfully. Proceeding to final waypoint.")
            self.pre_pose_reached = True  # 이후로는 pre_pose 스킵

        # --- 2단계: 최종 waypoint(goal_pose)로 이동 ---
        goal_pose = self.navigator.getPoseStamped([x, y], yaw_deg)
        self.navigator.info(
            f"Step 2: Going to waypoint '{waypoint_name}' at "
            f"(x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.1f}deg)"
        )

        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.navigator.info(f"Navigation to '{waypoint_name}' SUCCEEDED.")
            done_msg = String()
            done_msg.data = f"{waypoint_name}_done"

            # --- waypoint 이동 완료 토픽 발행 ----> 튜닝 ---> 픽업 ---
            self.waypoint_done_pub.publish(done_msg)
            self.get_logger().info(f"Published /robot5/waypoint_done: '{done_msg.data}'")

        elif result == TaskResult.CANCELED:
            self.navigator.warn(f"Navigation to '{waypoint_name}' was CANCELED.")
        elif result == TaskResult.FAILED:
            self.navigator.error(f"Navigation to '{waypoint_name}' FAILED.")
        else:
            self.navigator.warn(
                f"Navigation to '{waypoint_name}' returned unknown status: {result}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = JsonNavToPoseWithPre()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down json_nav_to_pose_with_pre...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
