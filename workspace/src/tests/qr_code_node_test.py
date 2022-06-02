import os
import threading
import time
import unittest
from unittest.mock import Mock

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class NodeNode(Node):
    def __init__(self, subscriber_callback):
        super().__init__("test_node")

        self.publisher = self.create_publisher(Image, "/camera/image_raw", 10)
        self.subscription = self.create_subscription(
            String,
            "/add_data",
            subscriber_callback,
            10
        )


class QRCodeNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        self.bridge = CvBridge()

        self.subscriber_mock = Mock()
        self.test_node = NodeNode(self.subscriber_mock)

        self.node_thread = threading.Thread(
            target=lambda: rclpy.spin(self.test_node))
        self.node_thread.start()

    def setUp(self):
        self.subscriber_mock.reset_mock()

    @classmethod
    def tearDownClass(self):
        rclpy.shutdown()
        self.node_thread.join()

    def test_sending_image_without_a_qr_code(self):
        dirname = os.path.dirname(__file__)
        image_path = os.path.join(dirname, "images/no-qr-code.jpg")
        image = cv2.imread(image_path)
        ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.test_node.publisher.publish(ros_image)

        time.sleep(5)
        self.subscriber_mock.assert_not_called()

    def test_sending_image_with_a_qr_code(self):
        dirname = os.path.dirname(__file__)
        image_path = os.path.join(dirname, "images/qr-code.jpg")
        image = cv2.imread(image_path)
        ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.test_node.publisher.publish(ros_image)

        time.sleep(5)
        self.subscriber_mock.assert_called_once_with(String(
            data='{"name": "STATION", "id": "1"}'
        ))
