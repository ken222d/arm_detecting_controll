# SPDX-FileCopyrightText: 2025 Kenta ishizeki<a.w.g.d0201@icloud.com>
# SPDX-License-Identifier: BSD-3-Clause


import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_ros_link_attacher.srv import Attach
from std_msgs.msg import Bool


class ArmControll(Node):
    def __init__(self):
        super().__init__('arm_controll')

        self.create_subscription(Point, '/ball_position_closest', self.point_callback, 10)
        self.create_subscription(String, '/ball_position_closest_color', self.color_callback, 10)

        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )
        # ライントレース有効化用のPublisher
        self.line_trace_pub = self.create_publisher(Bool, '/line_trace_mode', 10)

        self.latest_position = None
        self.latest_color = None
        self.reached = False
        self.timer = None
        self.final_wait_timer = None
        self.wait_duration = 3.0

        self.attach_cli = self.create_client(Attach, '/attach')
        while not self.attach_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/attach service not available, waiting...')

        self.get_logger().info("ArmControll node initialized.")

    def point_callback(self, msg: Point):
        if msg.z > 0.192:
            self.get_logger().info("Ball too far. Ignoring.")
            return
        self.latest_position = msg
        self.try_activate()

    def color_callback(self, msg: String):
        self.latest_color = msg.data
        self.try_activate()

    def try_activate(self):
        if self.reached:
            return
        if self.latest_position and self.latest_color:
            self.reached = True
            self.get_logger().info("Starting sequence")
            self.step_initial_pose()

    # 各ステップを順に呼び出す
    def step_initial_pose(self):
        self.move_arm(shoulder=0.0, elbow=math.radians(-110))
        self.get_logger().info("Step 1: Move to initial pose")
        self.timer = self.create_timer(self.wait_duration, self.step_attach)


    def step_attach(self):
        self.timer.cancel()
        self.get_logger().info("Step 2: Attach object")

        try:
            req = Attach.Request()
            req.model_name_1 = 'arm_detecting_controll'
            req.link_name_1 = 'fan_link'

            color_to_model = {
                "blue": ("test_ball1", "ball_link1"),
                "red":  ("test_ball2", "ball_link2"),
                "yellow": ("test_ball3", "ball_link3")
            }
            model2, link2 = color_to_model.get(self.latest_color, ("test_ball1", "ball_link1"))
            req.model_name_2 = model2
            req.link_name_2 = link2

            future = self.attach_cli.call_async(req)

        # コールバックで処理を継続
            def attach_done_callback(fut):
                try:
                    result = fut.result()
                    if result and result.ok:
                        self.get_logger().info(f"[✓] Attached {model2} to fan_link")
                    else:
                        self.get_logger().warn(f"[×] Failed to attach {model2} to fan_link")
                except Exception as e:
                    self.get_logger().error(f"Attach service failed: {e}")

            # アタッチ完了後に次のステップへ
                self.timer = self.create_timer(self.wait_duration, self.step_return_pose)

            future.add_done_callback(attach_done_callback)

        except Exception as e:
            self.get_logger().error(f"Attach service exception: {e}")
        # 次に進めるための保険
            self.timer = self.create_timer(self.wait_duration, self.step_return_pose)

    def step_return_pose(self):
        self.timer.cancel()
        self.get_logger().info("Step 3: Return to initial pose")

        self.move_arm(shoulder=0.0, elbow=0.0)
        self.final_wait_timer = self.create_timer(5.0, self.reset_motion)

    def move_arm(self, shoulder: float, elbow: float):
        traj = JointTrajectory()
        traj.joint_names = ['sholder_arm', 'elbow_arm']
        point = JointTrajectoryPoint()
        point.positions = [shoulder, elbow]
        point.time_from_start.sec = 2
        traj.points.append(point)
        self.trajectory_pub.publish(traj)

    def reset_motion(self):
        if self.final_wait_timer:
            self.final_wait_timer.cancel()
        self.latest_position = None
        self.latest_color = None
        self.reached = False
        self.get_logger().info("Sequence complete. Ready for next ball.")

    # ライントレース再開のためTrueを送信
        msg = Bool()
        msg.data = True
        self.line_trace_pub.publish(msg)
        self.get_logger().info("Line trace re-enabled.")

    def destroy_node(self):
        if self.timer:
            self.timer.cancel()
        if self.final_wait_timer:
            self.final_wait_timer.cancel()
        super().destroy_node()


def main():
    rclpy.init()
    node = ArmControll()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

