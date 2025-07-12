import rclpy
from rclpy.node import Node
from gazebo_ros2_link_attacher.srv import Attach


class AttachSLS(Node):
    def __init__(self):
        super().__init__('demo_attach_links')

        # Create service client
        self.cli = self.create_client(Attach, '/gazebo_ros2_link_attacher/attach')

        # Wait for service
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo_ros2_link_attacher/attach service...')

        self.get_logger().info('Attach service available.')

        # --- First Attach ---
        self.get_logger().info("Attaching px4vision_0 to ball_hinge_0")
        req = Attach.Request()
        req.model_name_1 = "ball_model"
        req.link_name_1 = "ball_link"
        req.model_name_2 = "slung_load"
        req.link_name_2 = "ball_hinge_0::base_link"
        self.call_service(req)

        # --- Second Attach ---
        self.get_logger().info("Attaching px4vision_1 to ball_hinge_1")
        req = Attach.Request()
        req.model_name_1 = "px4vision_1"
        req.link_name_1 = "base_link"
        req.model_name_2 = "slung_load"
        req.link_name_2 = "ball_hinge_1::base_link"
        self.call_service(req)

    def call_service(self, req):
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            if future.result().ok:
                self.get_logger().info("Attach successful.")
            else:
                self.get_logger().error("Attach failed.")
        else:
            self.get_logger().error('Service call failed.')


def main():
    rclpy.init()
    node = AttachSLS()
    rclpy.shutdown()

