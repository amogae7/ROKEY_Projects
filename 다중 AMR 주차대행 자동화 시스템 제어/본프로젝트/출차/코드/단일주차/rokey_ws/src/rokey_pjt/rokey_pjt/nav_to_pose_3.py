#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtlebot4_navigation.turtlebot4_navigator import (
    TurtleBot4Directions,
    TurtleBot4Navigator,
)
from nav2_simple_commander.robot_navigator import TaskResult


class CmdGoFrontNavigator(Node):
    def __init__(self):
        super().__init__('cmd_go_front_navigator')

        # TurtleBot4 네비게이터 객체 생성
        self.navigator = TurtleBot4Navigator()
        self.get_logger().info('CmdGoFrontNavigator node initialized')

        # 슬롯 이름 → (x, y, yaw[rad]) 매핑
        self.slot_poses = {
            "way_point_A_1": (-1.4, 0.551, 0.0), 'A_1_1': (-1.4, 0.551, 0.0), 'A_1_2': (-1.4, 0.551, 0.0),
            'way_point_A_2': (-1.19, 1.16, 0.0), 'A_2_1': (-1.19, 1.16, 0.0), 'A_2_2': (-1.19, 1.16, 0.0),
            'way_point_A_3': (-1.76, 0.624, TurtleBot4Directions.SOUTH), 'A_3_1': (-1.76, 0.624, TurtleBot4Directions.SOUTH),
            'A_3_2': (-1.76, 0.624, TurtleBot4Directions.SOUTH),
            'way_point_A_4': (-1.79, 1.17, TurtleBot4Directions.SOUTH), 'A_3_1': (-1.79, 1.17, TurtleBot4Directions.SOUTH),
            'A_3_2': (-1.79, 1.17, TurtleBot4Directions.SOUTH),
        }

        # --- 중간 포즈 (먼저 이동할 위치) ---
        self.pre_pose_xy = [-1.6231178733012308, 4.21895869581359]
        self.pre_pose_yaw_deg = TurtleBot4Directions.EAST  # 편하게 EAST로 설정
        # self.pre_pose_yaw_deg = 0.2752  # 각도수정해봄

        # 상태 변수들
        self.pre_pose_reached = False  # pre_pose 이동 완료 여부
        self.undocked_once = False  # undock 후 pre_pose만 한 번만 이동하도록 설정

        # --- 1) 도킹 상태 확인 & 도킹 ---
        if not self.navigator.getDockedStatus():
            self.navigator.info('Docking before initializing pose')
            self.navigator.dock()
        self.get_logger().info('도킹 상태 확인 & 도킹 완료')

        # --- 2) 초기 포즈 설정 ---
        initial_pose = self.navigator.getPoseStamped(
            [-0.243, 4.277650642336148],
            0.0  # NORTH
        )
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info('초기 포즈 설정 (-0.243, 4.31765, NORTH)')

        # --- 3) Nav2 활성화 대기 ---
        self.navigator.waitUntilNav2Active()
        self.navigator.info('Nav2 is active. Waiting for /cmd_go_front...')

        # --- 4) /출차 토픽 구독 대기---
        self.cmd_sub = self.create_subscription(
            String,
            '/cmd_go_front',
            self.to_waypoint_callback,
            10
        )

        # --- waypoint 이동 완료 토픽 퍼블리셔 (/robot5/waypoint_done) ---
        self.waypoint_done_pub = self.create_publisher(
            String,
            '/robot5/waypoint_done',
            10
        )

    def to_waypoint_callback(self, msg: String):
        """
        /cmd_go_front에 들어온 문자열(data)이 slot_poses의 키면

        1) 먼저 pre_pose_xy로 이동
        2) 그 다음 해당 slot pose로 이동
        3) 최종 waypoint 성공 시 /robot5/waypoint_done 에 "<slot_name>_done" 발행
        """
        slot_name = msg.data.strip()
        self.get_logger().info(f"Received /cmd_go_front: '{slot_name}'")

        if slot_name not in self.slot_poses:
            self.get_logger().warn(
                f"Unknown slot name '{slot_name}'. "
                f"Available keys: {list(self.slot_poses.keys())}"
            )
            return

        # 최종 목표 좌표/자세
        x, y, yaw_rad = self.slot_poses[slot_name]
        yaw_deg = math.degrees(yaw_rad)

        # --- 도킹되어 있다면 undock 먼저 ---
        if self.navigator.getDockedStatus() and not self.undocked_once:
            self.navigator.info('Robot is docked. Undocking before moving.')
            self.navigator.undock()
            self.undocked_once = True  # 첫 번째로만 undock 처리

        # --- 1단계: pre_pose로 이동 ---
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
                    f"Skipping final slot navigation."
                )
                return

            self.navigator.info("Pre-pose reached successfully. Proceeding to final slot.")
            self.pre_pose_reached = True  # pre_pose 도달 후에만 final 목표로 이동

        # --- 2단계: 최종 slot pose로 이동 ---
        goal_pose = self.navigator.getPoseStamped([x, y], yaw_deg)
        self.navigator.info(
            f"Step 2: Going to slot '{slot_name}' at "
            f"(x={x:.3f}, y={y:.3f}, yaw={yaw_deg:.1f}deg)"
        )

        self.navigator.goToPose(goal_pose)

        while not self.navigator.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.navigator.getResult()

        if result == TaskResult.SUCCEEDED:
            self.navigator.info(f"Navigation to '{slot_name}' SUCCEEDED.")
            done_msg = String()
            done_msg.data = f"{slot_name}_done"

            # --- waypoint 이동 완료 토픽 발행 ----> 튜닝 ---> 픽업 ---
            self.waypoint_done_pub.publish(done_msg)
            self.get_logger().info(f"Published /robot5/waypoint_done: '{done_msg.data}'")

        elif result == TaskResult.CANCELED:
            self.navigator.warn(f"Navigation to '{slot_name}' was CANCELED.")
        elif result == TaskResult.FAILED:
            self.navigator.error(f"Navigation to '{slot_name}' FAILED.")
        else:
            self.navigator.warn(
                f"Navigation to '{slot_name}' returned unknown status: {result}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = CmdGoFrontNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down cmd_go_front_navigator...')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
