# SPDX-FileCopyrightText: 2025 Kenta ishizeki<a.w.g.d0201@icloud.com>
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point

class BallFollower(Node):
    def __init__(self):
        super().__init__('ball_follower')
        self.cmd_pub = self.create_publisher(Twist, '/wheel_robot_simple/cmd_vel', 10)
        self.sub = self.create_subscription(Point, '/ball_position_closest', self.ball_callback, 10)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.last_ball_time = self.get_clock().now()
        self.latest_msg = None

        self.fx = 380.0
        self.image_width = 640

        self.target_reached = False

    def ball_callback(self, msg):
        self.last_ball_time = self.get_clock().now()
        self.latest_msg = msg

    def control_loop(self):
        twist = Twist()

        if self.target_reached:
            self.cmd_pub.publish(twist)
            return

        now = self.get_clock().now()

        if self.latest_msg and (now - self.last_ball_time).nanoseconds < 1e9:
            image_center = self.image_width / 2
            x_offset = (self.latest_msg.x - image_center) / self.fx
            distance = self.latest_msg.z

            self.get_logger().info(f"x_offset: {x_offset:.3f}, distance: {distance:.3f}")

            if distance <= 0.222:
                self.get_logger().info("ボールを取れ！")
                self.target_reached = True
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            else:
                if abs(x_offset) > 0.05:
                    # まだ中心に入ってない → 回転のみ
                    twist.angular.z = -1.0 * x_offset
                else:
                    # 中心に入った → 直進のみ
                    twist.linear.x = 0.1

        else:
            # 探索モード
            twist.angular.z = 0.4

        self.cmd_pub.publish(twist)

def main():
    rclpy.init()
    node = BallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
