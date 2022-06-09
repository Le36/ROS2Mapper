import rclpy
import cv2
from cv_bridge import CvBridge
from pyzbar import pyzbar
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class QRCodeReader(Node):
    def __init__(self) -> None:
        """Create the subscriber and the publisher"""
        super().__init__("qr_code_reader")

        self.found_codes = []
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, 10
        )

        self.publisher = self.create_publisher(String, "/add_data", 10)

    def image_callback(self, msg_image: Image) -> None:
        """Find and publish QR code data"""
        image = self.bridge.imgmsg_to_cv2(msg_image, "bgr8")
        codes = pyzbar.decode(image)
        for code in codes:
            data = code.data.decode()
            if data in self.found_codes:
                continue
            self.found_codes.append(data)
            self.get_logger().info(f"Found new a QR code with data '{data}'")
            self.publisher.publish(String(data=data))

    def reset_found_codes(self) -> None:
        self.found_codes = []


def main(args=None) -> None:  # pragma: no cover
    """Run the node"""
    rclpy.init(args=args)
    qr_code_reader = QRCodeReader()
    rclpy.spin(qr_code_reader)
    qr_code_reader.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
