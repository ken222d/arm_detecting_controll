# SPDX-FileCopyrightText: 2025 Kenta ishizeki<a.w.g.d0201@icloud.com>
# SPDX-License-Identifier: BSD-3-Clause


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class LineTraceIdentifier(Node):
    def __init__(self):
        super().__init__('line_trace_identifier')

        self.bridge = CvBridge()
        self.latest_status = {
            "3": "wait",
            "4": "wait",
            "5": "wait",
            "6": "wait"
        }

        self.create_subscription(Image, '/wheel_robot_simple/linetrace1/image_raw', self.left1_callback, 10)
        self.create_subscription(Image, '/wheel_robot_simple/linetrace2/image_raw', self.left2_callback, 10)
        self.create_subscription(Image, '/wheel_robot_simple/linetrace3/image_raw', self.left3_callback, 10)
        self.create_subscription(Image, '/wheel_robot_simple/linetrace4/image_raw', self.left4_callback, 10)
        self.create_subscription(Image, '/wheel_robot_simple/linetrace5/image_raw', self.right4_callback, 10)
        self.create_subscription(Image, '/wheel_robot_simple/linetrace6/image_raw', self.right3_callback, 10)
        self.create_subscription(Image, '/wheel_robot_simple/linetrace7/image_raw', self.right2_callback, 10)
        self.create_subscription(Image, '/wheel_robot_simple/linetrace8/image_raw', self.right1_callback, 10)

        # 0.5秒ごとにログをまとめて表示
        self.create_timer(0.5, self.print_status)

    def detect_black_or_white(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 黒のHSV範囲
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 70])
        black_mask = cv2.inRange(hsv, lower_black, upper_black)

        # 白のHSV範囲
        lower_white = np.array([0, 0, 200])
        upper_white = np.array([180, 30, 255])
        white_mask = cv2.inRange(hsv, lower_white, upper_white)

        black_ratio = cv2.countNonZero(black_mask) / (frame.shape[0] * frame.shape[1])
        white_ratio = cv2.countNonZero(white_mask) / (frame.shape[0] * frame.shape[1])

        if white_ratio > 0.05:
            return f"white"
        elif black_ratio > 0.05:
            return f"black"
        else:
            return f"unknown"

    def update_status(self, label, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        result = self.detect_black_or_white(frame)
        self.latest_status[label] = result

    def left1_callback(self, msg):
        self.update_status("1", msg)

    def left2_callback(self, msg):
        self.update_status("2", msg)

    def left3_callback(self, msg):
        self.update_status("3", msg)

    def left4_callback(self, msg):
        self.update_status("4", msg)

    def right4_callback(self, msg):
        self.update_status("5", msg)

    def right3_callback(self, msg):
        self.update_status("6", msg)

    def right2_callback(self, msg):
        self.update_status("7", msg)

    def right1_callback(self, msg):
        self.update_status("8", msg)


    def print_status(self):
    # ラベルの順番を固定
        order = ['8', '7', '6', '5', '4', '3', '2', '1']
        status_str = ' | '.join(
            [f"{label}: {self.latest_status.get(label, 'N/A')}" for label in order]
        )
        self.get_logger().info(status_str)


def main():
    rclpy.init()
    node = LineTraceIdentifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

