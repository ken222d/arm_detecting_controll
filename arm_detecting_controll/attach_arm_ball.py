import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from gazebo_ros2_link_attacher.srv import Attach

class ArmControll(Node):
    def __init__(self):
        super().__init__('attach_arm_ball')

        self.create_subscription(Point, '/ball_position_closest', self.point_callback, 10)
        self.create_subscription(String, '/ball_position_closest_color', self.color_callback, 10)

        self.trajectory_pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory',
            10
        )

        self.latest_position = None
        self.latest_color = None
        self.reached = False
        self.motion_stage = 0
        self.timer = None
        self.wait_duration = 5.0  # [秒] ステージ間の待機時間

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
            point.positions = [shoulder, elbow]
            point.time_from_start.sec = 2
            traj.points.append(point)
            self.trajectory_pub.publish(traj)

            self.get_logger().info("Stage 0: Moving to initial pose 1")

            self.motion_stage = 1

            # タイマーを使って数秒後に次ステージへ
            if self.timer:
                self.timer.cancel()
            self.timer = self.create_timer(self.wait_duration, self.execute_motion)
            return

        elif self.motion_stage == 1:
            shoulder = 0.0
            elbow = math.radians(-110)

            if self.latest_color == 'blue':
                shoulder = math.radians(0)
                elbow = math.radians(60)
                self.attach('test_ball1', 'ball_link1')
            elif self.latest_color == 'yellow':
                shoulder = math.radians(20)
                elbow = math.radians(60)
                self.attach('test_ball3', 'ball_link3')
            elif self.latest_color == 'red':
                shoulder = math.radians(-20)
                elbow = math.radians(60)
                self.attach('test_ball2', 'ball_link2')
            else:
                self.get_logger().info(f"Unknown color '{self.latest_color}', using default pose.")
                self.motion_stage = 2
                self.execute_motion()
                return

            point.positions = [shoulder, elbow]
            point.time_from_start.sec = 2
            traj.points.append(point)
            self.trajectory_pub.publish(traj)

            self.get_logger().info(f"Stage 1: Moving to color-specific pose: {self.latest_color}")
            # Attachが完了したら次ステージへ（attach_callbackで呼ぶ）
            return

        elif self.motion_stage == 2:
            shoulder = math.radians(0)
            elbow = math.radians(0)
            point.positions = [shoulder, elbow]
            point.time_from_start.sec = 2
            traj.points.append(point)
            self.trajectory_pub.publish(traj)

            self.get_logger().info("Stage 2: Moving to initial pose 2")

            # 動作完了 → 状態初期化
            self.reached = False
            self.latest_position = None
            self.latest_color = None
            self.motion_stage = 0

            if self.timer:
                self.timer.cancel()
                self.timer = None

            self.get_logger().info("Motion cycle finished.")
            return

        else:
            self.get_logger().warn(f"Invalid motion stage: {self.motion_stage}")

    def attach(self, ball_model, ball_link):
        self.attach_client = self.create_client(Attach, '/link_attacher_plugin/attach')
        if not self.attach_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error('Attach service not available')
            self.motion_stage = 2
            self.execute_motion()
            return

        req = Attach.Request()
        req.model_name_1 = ball_model
        req.link_name_1 = ball_link
        req.model_name_2 = 'wheel_robot_simple'
        req.link_name_2 = 'elbow_link'

        future = self.attach_client.call_async(req)
        future.add_done_callback(self.attach_callback)

    def attach_callback(self, future):
        if future.result():
            self.get_logger().info("Attach success")
        else:
            self.get_logger().warn("Attach failed")

        # Attach完了後に次ステージへ進む
        self.motion_stage = 2
        self.execute_motion()

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

