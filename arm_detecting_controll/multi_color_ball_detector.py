# SPDX-FileCopyrightText: 2025 Kenta ishizeki<a.w.g.d0201@icloud.com>
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
class MultiColorBallDetector(Node):
    def __init__(self):
        super().__init__('multi_color_ball_detector')
        # パブリッシャ（座標 + 色）
        self.publisher_ = self.create_publisher(Point, '/ball_position_closest', 10)
        self.color_pub = self.create_publisher(String, '/ball_position_closest_color', 10)
        # カメラ画像のサブスクライバ
        self.subscription = self.create_subscription(
            Image,
            '/wheel_robot_simple/camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.fx = 600.0  # 仮の焦点距離 [pixel]
        self.ball_diameter = 0.066  # ボール直径 [m]
        # HSV 範囲定義（色ごとに上下限）
        self.color_ranges = {
            'blue': ([100, 150, 50], [140, 255, 255]),
            'red1': ([0, 150, 50], [10, 255, 255]),
            'red2': ([170, 150, 50], [180, 255, 255]),
            'yellow': ([20, 150, 50], [35, 255, 255])
        }
        self.color_bgr = {
            'blue': (255, 0, 0),
            'red': (0, 0, 255),
            'yellow': (0, 255, 255),
            'closest': (255, 255, 255)
        }
    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            detected_balls = []
            # 各色のボールを検出
            for color, (lower, upper) in self.color_ranges.items():
                if color in ['red1', 'red2']:
                    continue  # 赤は後で統合
                mask = cv2.inRange(hsv, np.array(lower), np.array(upper))
                detected_balls.extend(self.extract_balls(mask, color, frame))
            # 赤色だけ2つの範囲を結合
            red_mask1 = cv2.inRange(hsv, np.array(self.color_ranges['red1'][0]), np.array(self.color_ranges['red1'][1]))
            red_mask2 = cv2.inRange(hsv, np.array(self.color_ranges['red2'][0]), np.array(self.color_ranges['red2'][1]))
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)
            detected_balls.extend(self.extract_balls(red_mask, 'red', frame))
            # 最も近いボールを決定
            if detected_balls:
                closest = min(detected_balls, key=lambda b: b['depth'])
                # 白で強調表示
                x, y, r = closest['center']
                cv2.circle(frame, (int(x), int(y)), int(r), self.color_bgr['closest'], 2)
                # 座標を送信
                point_msg = Point()
                point_msg.x = float(x)
                point_msg.y = float(y)
                point_msg.z = float(closest['depth'])
                self.publisher_.publish(point_msg)
                # 色を送信
                color_msg = String()
                color_msg.data = closest['color']
                self.color_pub.publish(color_msg)
                self.get_logger().info(f"Published color: {color_msg.data}")
                self.get_logger().info(f"Closest ball: color={closest['color']}, depth={closest['depth']:.3f}m")
                # 検出した全ボールの情報をログ出力
                for ball in detected_balls:
                    x, y, _ = ball['center']
                    color = ball['color']
                    depth = ball['depth']
                    self.get_logger().info(f"{color}: x={x:.1f}, y={y:.1f}, depth={depth:.3f}m")
            # 可視化
            cv2.imshow("Multi Color Ball Detection", frame)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")
    def extract_balls(self, mask, color, frame):
        balls = []
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for cnt in contours:
            (x, y), radius = cv2.minEnclosingCircle(cnt)
            if radius > 5:  # ノイズ除去
                depth = (self.ball_diameter * self.fx) / (2.0 * radius)
                balls.append({
                    'color': color,
                    'depth': depth,
                    'center': (x, y, radius)
                })
                # 通常色で描画
                cv2.circle(frame, (int(x), int(y)), int(radius), self.color_bgr[color], 2)
        return balls
def main():
    rclpy.init()
    node = MultiColorBallDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
