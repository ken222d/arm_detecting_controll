import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class StraightDriver(Node):
    def __init__(self):
        super().__init__('straight_driver')
        self.left_pub = self.create_publisher(Float64MultiArray, '/left_wheel_controller/commands', 10)
        self.right_pub = self.create_publisher(Float64MultiArray, '/right_wheel_controller/commands', 10)
        self.timer = self.create_timer(0.1, self.drive_forward)

    def drive_forward(self):
        speed = 1.0  # [rad/s] ここを変えれば速度調整可能
        msg = Float64MultiArray()
        msg.data = [speed]
        self.left_pub.publish(msg)
        self.right_pub.publish(msg)
        self.get_logger().info(f"Driving forward at {speed} rad/s")

def main():
    rclpy.init()
    node = StraightDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
