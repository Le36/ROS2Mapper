import rclpy
from interfaces.msg import QRCode
from interfaces.srv import GetQRCodes
from rclpy.node import Node

from .submodules.data_repository import data_repository


class MemoryNode(Node):
    def __init__(self) -> None:
        super().__init__("memory_node")
        self.qr_code_subscription = self.create_subscription(
            QRCode, "/qr_code", self.add_qr_code_callback, 10
        )
        self.srv = self.create_service(
            GetQRCodes, "get_qr_codes", self.get_qr_codes_callback
        )

    def add_qr_code_callback(self, qr_code: QRCode) -> None:
        self.get_logger().info(f"Adding QR code with id {qr_code.id} to the database")
        data_repository.add_qr_code(qr_code)
        data_repository.add_qr_code_to_history(qr_code)

    def get_qr_codes_callback(
        self, request: GetQRCodes.Request, response: GetQRCodes.Response
    ) -> GetQRCodes.Response:
        self.get_logger().info(f"Returning the list of QR codes")
        response.qr_codes = data_repository.get_qr_codes()
        return response


def main(args=None) -> None:  # pragma: no cover
    rclpy.init(args=args)
    memory_node = MemoryNode()
    rclpy.spin(memory_node)
    memory_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
