import os
import threading
import time
import unittest
from typing import Tuple
from unittest.mock import call

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion, Vector3
from interfaces.msg import QRCode
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image

from ..qr_code_reader.qr_code_reader import QRCodeReader


class NodeNode(Node):
    def __init__(self, subscriber_callback):
        super().__init__("test_node")

        self.bridge = CvBridge()

        self.publisher = self.create_publisher(Image, "/camera/image_raw", 10)
        self.subscription = self.create_subscription(
            QRCode, "/qr_code", subscriber_callback, 10
        )

    def send_image(self, image_filename: str):
        dirname = os.path.dirname(__file__)
        image_path = os.path.join(dirname, "images", image_filename)
        image = cv2.imread(image_path)
        ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.publisher.publish(ros_image)


class Subscriber:
    def __init__(self):
        self.calls = []

    def callback(self, qr_code: QRCode):
        self.calls.append(qr_code)


def get_position() -> Tuple[Vector3, Quaternion]:
    return (Vector3(x=0.0, y=0.0, z=0.0), Quaternion(w=0.0, x=0.0, y=0.0, z=0.0))


class QRCodeNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        rclpy.init()
        self.delay = int(os.getenv("DELAY")) if os.getenv("DELAY") else 0.1

        self.subscriber = Subscriber()
        self.test_node = NodeNode(self.subscriber.callback)
        self.qr_code_reader_node = QRCodeReader(threshold=-1, get_position=get_position)
        self.qr_code_reader_node.get_logger().set_level(40)

        executor = MultiThreadedExecutor()
        executor.add_node(self.qr_code_reader_node)
        executor.add_node(self.test_node)
        self.executor_thread = threading.Thread(target=executor.spin)
        self.executor_thread.start()

        time.sleep(self.delay)

    def setUp(self):
        self.subscriber.calls = []
        self.qr_code_reader_node.reset_found_codes()
        self.qr_code_reader_node.get_position = get_position
        self.qr_code = QRCode(
            id=1,
            center=np.array([1.08456, 0.341482, 0.010306]),
            normal_vector=np.array([-0.716183, 0.69788, -0.006722]),
            rotation=np.array([1.716183, -0.0, 0.006722, 0.69788]),
        )

    @classmethod
    def tearDownClass(self):
        rclpy.shutdown()
        self.executor_thread.join()

    def assert_qr_codes_equal(
        self,
        qr_code_1: QRCode,
        qr_code_2: QRCode,
    ):
        self.assertEqual(qr_code_1.id, qr_code_2.id)
        np.testing.assert_array_almost_equal(qr_code_1.center, qr_code_2.center)
        np.testing.assert_array_almost_equal(
            qr_code_1.normal_vector, qr_code_2.normal_vector
        )
        np.testing.assert_array_almost_equal(qr_code_1.rotation, qr_code_2.rotation)

    def test_sending_image_without_a_qr_code(self):
        self.test_node.send_image("no-aruco.jpg")
        time.sleep(self.delay)
        self.assertCountEqual(self.subscriber.calls, [])

    def test_sending_image_with_a_qr_code(self):
        self.test_node.send_image("aruco.png")
        time.sleep(self.delay)

        self.assertEqual(len(self.subscriber.calls), 1)
        self.assert_qr_codes_equal(self.subscriber.calls[0], self.qr_code)

    def test_sending_image_with_the_same_qr_code_twice(self):
        self.test_node.send_image("aruco.png")
        time.sleep(self.delay)
        self.test_node.send_image("aruco.png")
        time.sleep(self.delay)

        self.assertEqual(len(self.subscriber.calls), 1)
        self.assert_qr_codes_equal(self.subscriber.calls[0], self.qr_code)

    def test_sending_image_twice_but_changing_robot_position(self):
        self.test_node.send_image("aruco.png")
        time.sleep(self.delay)

        self.qr_code_reader_node.get_position = lambda: (
            Vector3(x=1.0, y=0.0, z=0.0),
            Quaternion(w=0.0, x=0.0, y=0.0, z=0.0),
        )
        self.test_node.send_image("aruco.png")
        time.sleep(self.delay)

        self.assertEqual(len(self.subscriber.calls), 2)
