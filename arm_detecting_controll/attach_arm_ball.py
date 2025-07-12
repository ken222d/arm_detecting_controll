import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty

class ArmControll(Node):
    def __init__(self):
        super().__init__('vacuum_arm_controller')

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
        self.wait_duration = 5.0  # 秒

        self.get_logger().info("ArmControll node initialized.")

    def point_callback(self, msg: Point):
        if msg.z > 0.3:
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
        if self.latest_position and self.latest_color:
            self.reached = True
            self.motion_stage = 0
            self.execute_motion()

    def execute_motion(self):
        traj = JointTrajectory()
        traj.joint_names = ['sholder_arm', 'elbow_arm']
        point = JointTrajectoryPoint()
        self.vacuum_on()  # 吸着開始
        if self.motion_stage == 0:
            point.positions = [0.0, math.radians(-110)]
            point.time_from_start.sec = 2
            traj.points.append(point)
            self.trajectory_pub.publish(traj)

            self.get_logger().info("Stage 0: Moving to initial pose 1")

            self.motion_stage = 1
            if self.timer:
                self.timer.cancel()
            self.timer = self.create_timer(self.wait_duration, self.execute_motion)
            return

        elif self.motion_stage == 1:
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
                self.get_logger().warn(f"Unknown color '{self.latest_color}', skipping attach")
                self.motion_stage = 2
                self.execute_motion()
                return


            point.positions = [shoulder, elbow]
            point.time_from_start.sec = 2
            traj.points.append(point)
            self.trajectory_pub.publish(traj)

            self.get_logger().info(f"Stage 1: Moving to pose for {self.latest_color} ball")

            self.motion_stage = 2
            if self.timer:
                self.timer.cancel()
            self.timer = self.create_timer(self.wait_duration, self.execute_motion)
            return

        elif self.motion_stage == 2:
            point.positions = [math.radians(0), math.radians(0)]
            point.time_from_start.sec = 2
            traj.points.append(point)
            self.trajectory_pub.publish(traj)

            self.get_logger().info("Stage 2: Returning to initial pose 2")

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

    def vacuum_on(self):
        client = self.create_client(Empty, '/vacuum_gripper/on')
        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("Vacuum ON service not available")
            return
        req = Empty.Request()
        client.call_async(req)
        self.get_logger().info("Vacuum ON called")

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

