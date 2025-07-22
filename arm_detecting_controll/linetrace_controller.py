# SPDX-FileCopyrightText: 2025 Kenta ishizeki<a.w.g.d0201@icloud.com>
# SPDX-License-Identifier: BSD-3-Clause


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np


class LineTraceController(Node):
    def __init__(self):
        super().__init__('line_trace_controller')

        self.bridge = CvBridge()
        self.cmd_pub = self.create_publisher(Twist, '/wheel_robot_simple/cmd_vel', 10)
        self.create_subscription(Image, '/wheel_robot_simple/linetrace3/image_raw', self.left2_image_callback, 10)
        self.create_subscription(Image, '/wheel_robot_simple/linetrace4/image_raw', self.left1_image_callback, 10)
        self.create_subscription(Image, '/wheel_robot_simple/linetrace5/image_raw', self.right1_image_callback, 10)
        self.create_subscription(Image, '/wheel_robot_simple/linetrace6/image_raw', self.right2_image_callback, 10)

        self.line_trace_enabled = True
        self.create_subscription(Bool, '/line_trace_mode', self.mode_callback, 10)

        self.left2_color = None
        self.left1_color = None
        self.right1_color = None
        self.right2_color = None

        self.timer = self.create_timer(0.1, self.control_loop)

    def mode_callback(self, msg: Bool):
        self.line_trace_enabled = msg.data
        self.get_logger().info(f"Line trace {'enabled' if msg.data else 'disabled'}")

    def left2_image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.left2_color = self.detect_black_or_white(frame)

    def left1_image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.left1_color = self.detect_black_or_white(frame)

    def right1_image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.right1_color = self.detect_black_or_white(frame)

    def right2_image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.right2_color = self.detect_black_or_white(frame)

    def detect_black_or_white(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 黒のしきい値（低明度）
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 70])
        black_mask = cv2.inRange(hsv, lower_black, upper_black)

        # 白のしきい値（低彩度・高明度）
        lower_white = np.array([0, 0, 150])
        upper_white = np.array([180, 60, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        black_ratio = cv2.countNonZero(black_mask) / (frame.shape[0] * frame.shape[1])
        white_ratio = cv2.countNonZero(white_mask) / (frame.shape[0] * frame.shape[1])

        if white_ratio > 0.05:
            return 'white'
        elif black_ratio > 0.05:
            return 'black'
        else:
            return None

    def control_loop(self):
        twist = Twist()

        if not self.line_trace_enabled:
            self.cmd_pub.publish(twist)
            return

        base_speed = 0.1
        adjust_speed = 2.0

        self.get_logger().info(f"[LineTrace] 2: {self.left2_color}, 3: {self.left1_color}, 4: {self.right1_color}, 5: {self.right2_color}")

        # 以下、"gray" → "white" に置換
        if self.left1_color == 'white' and self.right1_color == 'black' and self.left2_color == 'white' and self.right2_color == 'white':
            twist.linear.x = base_speed * 0.1
            twist.angular.z = -adjust_speed * 1.0

        elif self.left1_color == 'black' and self.right1_color == 'white' and self.left2_color == 'white' and self.right2_color == 'white':
            twist.linear.x = base_speed * 0.05
            twist.angular.z = adjust_speed * 1.0

        elif self.left1_color == 'black' and self.right1_color == 'white' and self.left2_color == 'black' and self.right2_color == 'white':
            twist.linear.x = base_speed * 0.005
            twist.angular.z = adjust_speed * 1.0

        elif self.left1_color == 'white' and self.right1_color == 'black' and self.left2_color == 'white' and self.right2_color == 'black':
            twist.linear.x = base_speed * 0.025
            twist.angular.z = -adjust_speed * 1.0

        elif self.left1_color == 'white' and self.right1_color == 'white' and self.left2_color == 'white' and self.right2_color == 'black':
            twist.linear.x = base_speed * 0.0
            twist.angular.z = -adjust_speed * 2.0

        elif self.left1_color == 'white' and self.right1_color == 'white' and self.left2_color == 'black' and self.right2_color == 'white':
            twist.linear.x = base_speed * 0.00025
            twist.angular.z = adjust_speed * 2.0

        elif self.left1_color == 'black' and self.right1_color == 'black' and self.left2_color == 'white' and self.right2_color == 'black':
            twist.linear.x = base_speed * 0.0005
            twist.angular.z = -adjust_speed * 0.5

        elif self.left1_color == 'black' and self.right1_color == 'black' and self.left2_color == 'black' and self.right2_color == 'white':
            twist.linear.x = base_speed * 0.00025
            twist.angular.z = adjust_speed * 0.5

        elif self.left1_color == 'black' and self.right1_color == 'white' and self.left2_color == 'black' and self.right2_color == 'black':
            twist.linear.x = base_speed * 0.0
            twist.angular.z = -adjust_speed * 1.0

        elif self.left1_color == 'white' and self.right1_color == 'black' and self.left2_color == 'black' and self.right2_color == 'black':
            twist.linear.x = base_speed * 0.0
            twist.angular.z = adjust_speed * 1.0

        elif self.left1_color == 'black' and self.right1_color == 'white' and self.left2_color == 'white' and self.right2_color == 'black':
            twist.linear.x = base_speed * 0.0
            twist.angular.z = -adjust_speed * 1.0

        elif self.left1_color == 'white' and self.right1_color == 'black' and self.left2_color == 'black' and self.right2_color == 'white':
            twist.linear.x = base_speed * 0.0
            twist.angular.z = adjust_speed * 1.0

        elif self.left1_color == 'black' and self.right1_color == 'black' and self.left2_color == 'white' and self.right2_color == 'white':
            twist.linear.x = base_speed
            twist.angular.z = 0.0

        elif self.left1_color == 'white' and self.right1_color == 'white' and self.left2_color == 'white' and self.right2_color == 'white':
            twist.linear.x = base_speed * 2
            twist.angular.z = 0.0

        elif self.left1_color == 'black' and self.right1_color == 'black' and self.left2_color == 'black' and self.right2_color == 'black':
            twist.linear.x = base_speed * 2
            twist.angular.z = 0.0

        elif self.left1_color == 'white' and self.right1_color == 'white' and self.left2_color == 'black' and self.right2_color == 'black':
            twist.linear.x = -base_speed
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)


def main():
    rclpy.init()
    node = LineTraceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

