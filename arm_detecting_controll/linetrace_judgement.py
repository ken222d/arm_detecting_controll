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

        # 2つのセンサ画像を購読
        self.create_subscription(Image, '/wheel_robot_simple/linetrace3/image_raw', self.left_callback, 10)
        self.create_subscription(Image, '/wheel_robot_simple/linetrace4/image_raw', self.right_callback, 10)

    def detect_gray_or_black(self, frame, label):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # グレー判定：低彩度 + 中明度
        lower_gray = np.array([0, 0, 50])
        upper_gray = np.array([180, 30, 200])
        gray_mask = cv2.inRange(hsv, lower_gray, upper_gray)

        # 黒判定：低彩度 + 低明度
        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 50])
        black_mask = cv2.inRange(hsv, lower_black, upper_black)

        gray_ratio = cv2.countNonZero(gray_mask) / (frame.shape[0] * frame.shape[1])
        black_ratio = cv2.countNonZero(black_mask) / (frame.shape[0] * frame.shape[1])

        if gray_ratio > 0.05:
            self.get_logger().info(f"{label}: グレーを検出（{gray_ratio:.1%}）")
        elif black_ratio > 0.05:
            self.get_logger().info(f"{label}: 黒を検出（{black_ratio:.1%}）")
        else:
            self.get_logger().info(f"{label}: 判別不可（Gray: {gray_ratio:.1%}, Black: {black_ratio:.1%}）")

    def left_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.detect_gray_or_black(frame, "センサ3（左）")

    def right_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.detect_gray_or_black(frame, "センサ4（右）")


def main():
    rclpy.init()
    node = LineTraceIdentifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

