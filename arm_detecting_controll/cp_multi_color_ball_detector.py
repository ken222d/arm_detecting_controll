# SPDX-FileCopyrightText: 2025 Kenta ishizeki<a.w.g.d0201@icloud.com>
# SPDX-License-Identifier: BSD-3-Clause


import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String, Bool
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class MultiColorBallDetector(Node):
    def __init__(self):
        super().__init__('multi_color_ball_detector')

        # パブリッシャ
        self.publisher_ = self.create_publisher(Point, '/ball_position_closest', 10)
        self.color_pub = self.create_publisher(String, '/ball_position_closest_color', 10)
        self.mode_pub = self.create_publisher(Bool, '/line_trace_mode', 10)

        self.bridge = CvBridge()

        # 状態管理
        self.line_trace_stopped = False
        self.last_detect_time = self.get_clock().now()

        # パラメータ
        self.fx = 600.0
        self.ball_diameter = 0.066
        self.reenable_duration_sec = 3.0  # 再開までの待機秒数

        # タイマー：一定周期で検出状態をチェック
        self.timer = self.create_timer(0.5, self.check_lost_ball)

        # カメラ購読
        self.subscription = self.create_subscription(
            Image, '/wheel_robot_simple/camera/image_raw', self.image_callback, 10
        )

        # 色検出定義
        self.color_ranges = {
            'blue': ([100, 150, 50], [140, 255, 255]),
            'red1': ([0, 150, 50], [10, 255, 255]),
            'red2': ([170, 150, 50], [180, 255, 255]),
            'yellow': ([20, 150, 50], [35, 255, 255])
        }
        self.color_bgr = {
            'blue': (255, 0, 0), 'red': (0, 0, 255),
            'yellow': (0, 255, 255), 'closest': (255, 255, 255)
        }

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            detected_balls = []

            for color, (lower, upper) in self.color_ranges.items():
                if color in ['red1', 'red2']:
                    continue
                mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
                detected_balls.extend(self.extract_balls(mask, color, frame))

            # 赤色結合
            red_mask1 = cv2.inRange(hsv, np.array(self.color_ranges['red1'][0]), np.array(self.color_ranges['red1'][1]))
            red_mask2 = cv2.inRange(hsv, np.array(self.color_ranges['red2'][0]), np.array(self.color_ranges['red2'][1]))
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)
            detected_balls.extend(self.extract_balls(red_mask, 'red', frame))

            if detected_balls:
                # 最も近いボールを処理
                closest = min(detected_balls, key=lambda b: b['depth'])

                # 最終検出時間を更新
                self.last_detect_time = self.get_clock().now()

                # 一度だけ停止命令を送る
                if not self.line_trace_stopped:
                    self.get_logger().info("Ball found! Stopping line trace.")
                    self.mode_pub.publish(Bool(data=False))
                    self.line_trace_stopped = True

                # パブリッシュ
                x, y, r = closest['center']
                point_msg = Point(x=float(x), y=float(y), z=float(closest['depth']))
                self.publisher_.publish(point_msg)

                color_msg = String()
                color_msg.data = closest['color']
                self.color_pub.publish(color_msg)

                # 可視化
                cv2.circle(frame, (int(x), int(y)), int(r), self.color_bgr['closest'], 2)

            cv2.imshow("Ball Detection", frame)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

    def extract_balls(self, mask, color, frame):
        balls = []
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            if radius > 5:
                depth = (self.ball_diameter * self.fx) / (2.0 * radius)
                balls.append({'color': color, 'depth': depth, 'center': (x, y, radius)})
                cv2.circle(frame, (int(x), int(y)), int(radius), self.color_bgr[color], 2)
        return balls

    def check_lost_ball(self):
        now = self.get_clock().now()
        duration = (now - self.last_detect_time).nanoseconds / 1e9  # 秒に変換

        if self.line_trace_stopped and duration > self.reenable_duration_sec:
            self.get_logger().info("No ball detected recently. Resuming line trace.")
            self.mode_pub.publish(Bool(data=True))
            self.line_trace_stopped = False

def main():
    rclpy.init()
    node = MultiColorBallDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

