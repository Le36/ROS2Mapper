from std_msgs.msg import String
from pyzbar import pyzbar

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class QRCodeReader(Node):
    def __init__(self):
        super().__init__('qr_code_reader')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10)
        self.publisher_ = self.create_publisher(String, '/qr_code_found', 10)

        self.bridge = CvBridge()

    def image_callback(self, msg_image: Image):
        """Convert image to correct format and find possible QR codes in it
        and then publish the data in the QR codes"""
        image = self.bridge.imgmsg_to_cv2(msg_image, "bgr8")
        codes = pyzbar.decode(image)
        for code in codes:
            self.get_logger().info(f"Found code with data \"{code.data}\"")
            self.publish(code.data)

    def publish(self, data: str):
        """Publish QR code data to /qr_code_found topic when QR codes are found"""
        msg = String()
        msg.data = str(data)
        self.publisher_.publish(msg)


def main(args=None):
    """Run the node"""
    rclpy.init(args=args)
    qr_code_reader = QRCodeReader()
    rclpy.spin(qr_code_reader)
    qr_code_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
