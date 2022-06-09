import rclpy
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraNode(Node):
    def __init__(self) -> None:
        """Create the publisher"""
        super().__init__("qr_code_reader")

        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().fatal("Could not open camera")

        self.publisher = self.create_publisher(Image, "/camera/image_raw", 10)
        self.image_publisher()

    def image_publisher(self) -> None:
        """Publish image from camera"""
        while True:
            _, image = self.cap.read()
            ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.publisher.publish(ros_image)


def main(args=None) -> None:  # pragma: no cover
    """Run the node"""
    rclpy.init(args=args)
    camera_node = CameraNode()
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":  # pragma: no cover
    main()
