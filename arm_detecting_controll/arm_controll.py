# SPDX-FileCopyrightText: 2025 Kenta ishizeki<a.w.g.d0201@icloud.com>
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class ArmControll(Node):
    def __init__(self):
        super().__init__('arm_controll')

        self.create_subscription(Point, '/ball_position_closest', self.point_callback, 10)
        self.create_subscription(String, '/ball_position_closest_color', self.color_callback, 10)

        self.trajectory_pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)

        self.latest_position = None
        self.latest_color = None
        self.reached = False
        self.motion_stage = 0
        self.timer = None
        self.wait_duration = 2.0  # 各段階の待ち時間（秒）

        self.get_logger().info("ArmControll node initialized.")

    def point_callback(self, msg: Point):
        if msg.z > 0.22:
            self.get_logger().info("Ball too far. Ignoring.")
            return
        self.latest_position = msg
        self.try_activate_motion()

    def color_callback(self, msg: String):
        self.latest_color = msg.data
        self.try_activate_motion()

    def try_activate_motion(self):
        if self.reached:
            return
        if self.latest_position is not None and self.latest_color is not None:
            self.reached = True
            self.motion_stage = 0
            self.execute_motion()

    def execute_motion(self):
        traj = JointTrajectory()
        traj.joint_names = ['sholder_arm', 'elbow_arm']
        point = JointTrajectoryPoint()

        if self.motion_stage == 0:
            shoulder = 0.0
            elbow = math.radians(-110)
            self.get_logger().info("Stage 0: Moving to initial pose 1")
            self.motion_stage = 1

        elif self.motion_stage == 1:
            shoulder = 0.0
            elbow = math.radians(-110)

            if self.latest_color == 'blue':
                shoulder = math.radians(0)
                elbow = math.radians(60)
            elif self.latest_color == 'yellow':
                shoulder = math.radians(20)
                elbow = math.radians(60)
            elif self.latest_color == 'red':
                shoulder = math.radians(-20)
                elbow = math.radians(60)
            else:
                self.get_logger().info(f"Unknown color '{self.latest_color}', using default pose.")

            self.get_logger().info(f"Stage 1: Moving to color-specific pose: {self.latest_color}")
            self.motion_stage = 2

        elif self.motion_stage == 2:
            shoulder = math.radians(0)
            elbow = math.radians(0)
            self.get_logger().info("Stage 2: Moving to initial pose 2")

            # 動作完了 → 次の入力待ちに戻す
            self.reached = False
            self.latest_position = None
            self.latest_color = None

            if self.timer:
                self.timer.cancel()
                self.timer = None

            self.get_logger().info("Motion cycle finished.")
            return

        else:
            self.get_logger().warn(f"Invalid motion stage: {self.motion_stage}")
            return

        # trajectory の送信処理
        point.positions = [shoulder, elbow]
        point.time_from_start.sec = 2
        traj.points.append(point)
        self.trajectory_pub.publish(traj)

        # タイマーを1回だけ設定
        if self.timer:
            self.timer.cancel()
        self.timer = self.create_timer(self.wait_duration, self.execute_motion)

    def destroy_node(self):
        if self.timer:
            self.timer.cancel()
        super().destroy_node()


def main():
    rclpy.init()
    node = ArmControll()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
