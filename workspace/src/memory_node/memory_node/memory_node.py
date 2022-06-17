import rclpy
from interfaces.msg import QRCode
from rclpy.node import Node

from .submodules.data_repository import data_repository


class MemoryNode(Node):
    def __init__(self) -> None:
        super().__init__("memory_node")
        self.add_data_subscription = self.create_subscription(
            QRCode, "/add_data", self.add_data_callback, 10
        )

    def add_data_callback(self, msg: QRCode) -> None:
        self.get_logger().info(f"Adding QR code with data '{msg.data}' to the database")
        data_repository.add_data(msg.data)


def main(args=None) -> None:  # pragma: no cover
    rclpy.init(args=args)
    memory_node = MemoryNode()
    rclpy.spin(memory_node)
    memory_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
