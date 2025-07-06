import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty


class TeleopMotor(Node):
    def __init__(self):
        super().__init__('teleop_motor')
        self.publisher_ = self.create_publisher(Twist, '/wheel_robot_simple/cmd_vel', 10)
        self.get_logger().info("Use W/A/S/D to control, Q to quit.")
        self.speed = 0.1  # 線速度[m/s]
        self.turn = 0.5   # 角速度[rad/s]

    def run(self):
        settings = termios.tcgetattr(sys.stdin)
        try:
            while rclpy.ok():
                key = self.get_key()
                twist = Twist()

                if key == 'w':
                    twist.linear.x = self.speed
                elif key == 's':
                    twist.linear.x = -self.speed
                elif key == 'a':
                    twist.angular.z = self.turn
                elif key == 'd':
                    twist.angular.z = -self.turn
                elif key == ' ':
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                elif key == 'q':
                    break

                self.publisher_.publish(twist)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcflush(sys.stdin, termios.TCIFLUSH)
        return key


def main():
    rclpy.init()
    node = TeleopMotor()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

