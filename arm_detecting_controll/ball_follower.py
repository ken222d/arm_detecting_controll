import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from std_msgs.msg import Bool

class BallFollower(Node):
    def __init__(self):
        super().__init__('ball_follower')

        self.cmd_pub = self.create_publisher(Twist, '/wheel_robot_simple/cmd_vel', 10)
        self.mode_pub = self.create_publisher(Bool, '/line_trace_mode', 10)

        self.sub = self.create_subscription(Point, '/ball_position_closest', self.ball_callback, 10)
        self.create_subscription(Bool, '/line_trace_mode', self.line_trace_mode_callback, 10)  # ★追加

        self.timer = self.create_timer(0.1, self.control_loop)

        self.last_ball_time = self.get_clock().now()
        self.latest_msg = None

        self.fx = 554.5
        self.image_width = 640

        self.target_reached = False
        self.reenable_sent = False
        self.reenable_threshold_sec = 2.0

        self.line_trace_enabled = True  # ★初期状態（ライントレース中）

    def line_trace_mode_callback(self, msg: Bool):
        self.line_trace_enabled = msg.data  # True = ライントレース中、False = 停止中（このノード有効）

    def ball_callback(self, msg):
        self.last_ball_time = self.get_clock().now()
        self.latest_msg = msg
        self.reenable_sent = False

    def control_loop(self):
        if self.line_trace_enabled:
            return  # ★ ライントレース中はボール追従無効化

        twist = Twist()
        now = self.get_clock().now()
        elapsed = (now - self.last_ball_time).nanoseconds / 1e9

        if self.target_reached:
            self.cmd_pub.publish(twist)
            return

        if elapsed > self.reenable_threshold_sec and not self.reenable_sent:
            self.get_logger().info("ボール見失い → ライントレース再開")
            self.mode_pub.publish(Bool(data=True))
            self.reenable_sent = True
            self.cmd_pub.publish(twist)
            return

        if self.latest_msg and elapsed < self.reenable_threshold_sec:
            image_center = self.image_width / 2
            x_offset = (self.latest_msg.x - image_center) / self.fx
            distance = self.latest_msg.z

            if distance <= 0.22:
                self.get_logger().info("ボールを取れ！")
                self.target_reached = True
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                
            else:
                if abs(x_offset) > 0.05:
                    twist.angular.z = -20.0 * x_offset
                else:
                    twist.linear.x = 0.5
        else:
            twist.angular.z = 0.4

        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = BallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

