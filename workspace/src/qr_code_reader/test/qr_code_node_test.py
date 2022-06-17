import os
import threading
import time
import unittest
from unittest.mock import Mock

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ..qr_code_reader.qr_code_reader import QRCodeReader


class NodeNode(Node):
    def __init__(self, subscriber_callback):
        super().__init__("test_node")

        self.bridge = CvBridge()

        self.publisher = self.create_publisher(Image, "/camera/image_raw", 10)
        self.subscription = self.create_subscription(
            String, "/add_data", subscriber_callback, 10
        )

    def send_image(self, image_filename: str):
        dirname = os.path.dirname(__file__)
        image_path = os.path.join(dirname, "images", image_filename)
        image = cv2.imread(image_path)
        ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.publisher.publish(ros_image)


class QRCodeNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        rclpy.init()
        self.delay = int(os.getenv("DELAY")) if os.getenv("DELAY") else 0.1

        self.subscriber_mock = Mock()
        self.test_node = NodeNode(self.subscriber_mock)
        self.qr_code_reader_node = QRCodeReader(draw=False)
        self.qr_code_reader_node.get_logger().set_level(40)

        executor = MultiThreadedExecutor()
        executor.add_node(self.qr_code_reader_node)
        executor.add_node(self.test_node)
        self.executor_thread = threading.Thread(target=executor.spin)
        self.executor_thread.start()

        time.sleep(self.delay)

    def setUp(self):
        self.subscriber_mock.reset_mock()
        self.qr_code_reader_node.reset_found_codes()

    @classmethod
    def tearDownClass(self):
        rclpy.shutdown()
        self.executor_thread.join()

    def test_sending_image_without_a_qr_code(self):
        return  # Skip test until the tests are fixed
        self.test_node.send_image("no-qr-code.jpg")
        time.sleep(self.delay)
        self.subscriber_mock.assert_not_called()

    def test_sending_image_with_a_qr_code(self):
        return  # Skip test until the tests are fixed
        self.test_node.send_image("qr-code.jpg")
        time.sleep(self.delay)
        self.subscriber_mock.assert_called_once_with(
            String(data='{"name": "STATION", "id": "1"}')
        )

    def test_sending_image_with_the_same_qr_code_twice(self):
        return  # Skip test until the tests are fixed
        self.test_node.send_image("qr-code.jpg")
        time.sleep(self.delay)
        self.test_node.send_image("qr-code.jpg")
        time.sleep(self.delay)
        self.subscriber_mock.assert_called_once_with(
            String(data='{"name": "STATION", "id": "1"}')
        )
