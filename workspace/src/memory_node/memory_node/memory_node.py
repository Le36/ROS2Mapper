import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from .submodules.data_repository import data_repository


class MemoryNode(Node):

    def __init__(self) -> None:
        super().__init__("memory_node")
        self.add_data_subscription = self.create_subscription(
            String,
            "/add_data",
            self.add_data_callback,
            10)

    def add_data_callback(self, msg: String) -> None:
        self.get_logger().info(
            f"Adding QR code with data '{msg.data}' to the database")
        data_repository.add_data(msg.data)

    def read_data_callback(self, data) -> None:
        self.get_logger().info(
            f"Reading data from the database with data:'{data}'"
        )
        return data_repository.read_data(data)

    def drop_table(self) -> None:
        data_repository.drop_table()


def main(args=None) -> None:
    rclpy.init(args=args)
    memory_node = MemoryNode()
    rclpy.spin(memory_node)
    memory_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
