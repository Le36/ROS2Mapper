import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from .submodules.data_repository import data_repository


class MemoryNode(Node):

    def __init__(self):
        super().__init__("memory_node")
        self.add_data_subscription = self.create_subscription(
            String,
            "/add_data",
            self.add_data_callback,
            10)

    def add_data_callback(self, msg):
        self.get_logger().info(
            f"Adding QR code with data '{msg.data}' to the database")
        data_repository.add_data(msg.data)


def main(args=None):
    rclpy.init(args=args)
    adder = MemoryNode()
    rclpy.spin(adder)
    adder.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
