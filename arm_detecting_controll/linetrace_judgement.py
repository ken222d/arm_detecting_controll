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

        self.create_subscription(Image, '/wheel_robot_simple/linetrace3/image_raw', self.left3_callback, 10)
        self.create_subscription(Image, '/wheel_robot_simple/linetrace4/image_raw', self.left4_callback, 10)
        self.create_subscription(Image, '/wheel_robot_simple/linetrace5/image_raw', self.right4_callback, 10)
        self.create_subscription(Image, '/wheel_robot_simple/linetrace6/image_raw', self.right3_callback, 10)

        # 0.5秒ごとにログをまとめて表示
        self.create_timer(0.5, self.print_status)

    def detect_gray_or_black(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_gray = np.array([0, 0, 50])
        upper_gray = np.array([180, 30, 200])
        gray_mask = cv2.inRange(hsv, lower_gray, upper_gray)

        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])
        black_mask = cv2.inRange(hsv, lower_black, upper_black)

        gray_ratio = cv2.countNonZero(gray_mask) / (frame.shape[0] * frame.shape[1])
        black_ratio = cv2.countNonZero(black_mask) / (frame.shape[0] * frame.shape[1])

        if gray_ratio > 0.05:
            return f"white（{gray_ratio:.1%}）"
        elif black_ratio > 0.05:
            return f"black（{black_ratio:.1%}）"
        else:
            return f"unknown（Gray: {gray_ratio:.1%}, Black: {black_ratio:.1%}）"

    def update_status(self, label, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        result = self.detect_gray_or_black(frame)
        self.latest_status[label] = result

    def left3_callback(self, msg):
        self.update_status("3:", msg)

    def left4_callback(self, msg):
        self.update_status("4:", msg)

    def right4_callback(self, msg):
        self.update_status("5:", msg)

    def right3_callback(self, msg):
        self.update_status("6:", msg)

    def print_status(self):
        status_str = ' | '.join(
            [f"{label}: {self.latest_status[label]}" for label in reversed(self.latest_status)]
        )
        self.get_logger().info(status_str)


def main():
    rclpy.init()
    node = LineTraceIdentifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

