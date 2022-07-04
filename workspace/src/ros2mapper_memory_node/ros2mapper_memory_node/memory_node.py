import rclpy
from interfaces.msg import QRCode, QRCodeList
from rclpy.node import Node

from .submodules.data_repository import data_repository


class MemoryNode(Node):
    def __init__(self) -> None:
        super().__init__("ros2mapper_memory_node")
        self.qr_code_subscription = self.create_subscription(
            QRCode, "/qr_code", self.add_qr_code_callback, 10
        )
        self.publisher = self.create_publisher(QRCodeList, "/qr_code_list", 10)
        self.create_timer(5, self.publish_qr_codes)

    def add_qr_code_callback(self, qr_code: QRCode) -> None:
        """Add a QR code to the database"""
        self.get_logger().debug(f"Adding QR code with id {qr_code.id} to the database")
        data_repository.add_qr_code(qr_code)
        data_repository.add_qr_code_to_history(qr_code)
        self.publish_qr_codes()

    def publish_qr_codes(self) -> None:
        """Publish the list of QR codes"""
        self.get_logger().debug(f"Publishing the list of QR codes")
        qr_codes = data_repository.get_qr_codes()
        self.publisher.publish(QRCodeList(qr_codes=qr_codes))


def main(args=None) -> None:  # pragma: no cover
    rclpy.init(args=args)
    memory_node = MemoryNode()
    rclpy.spin(memory_node)
    memory_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
