import os
import threading
import time
import unittest
from unittest.mock import Mock

import numpy as np
import rclpy
from interfaces.msg import QRCode, QRCodeList
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from ..memory_node.memory_node import MemoryNode
from ..memory_node.submodules.data_repository import data_repository


class NodeNode(Node):
    def __init__(self, subscriber_callback):
        super().__init__("test_node")
        self.publisher = self.create_publisher(QRCode, "/qr_code", 10)
        self.subscriber = self.create_subscription(
            QRCodeList, "/qr_code_list", subscriber_callback, 10
        )

    def add_qr_code(self, qr_code: QRCode):
        self.publisher.publish(qr_code)


class MemoryNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        rclpy.init()
        self.delay = int(os.getenv("DELAY")) if os.getenv("DELAY") else 0.1

        self.mock = Mock()
        self.test_node = NodeNode(self.mock)
        self.memory_node = MemoryNode()
        self.memory_node.get_logger().set_level(40)

        executor = MultiThreadedExecutor()
        executor.add_node(self.memory_node)
        executor.add_node(self.test_node)
        self.executor_thread = threading.Thread(target=executor.spin)
        self.executor_thread.start()
        time.sleep(self.delay)

    def setUp(self):
        self.mock.reset_mock()
        data_repository.delete_all()

        self.qr_code = QRCode(
            id=1,
            center=np.array([1.08456, 0.341482, 0.010306]),
            normal_vector=np.array([-0.716183, 0.69788, -0.006722]),
            rotation=np.array(
                [2.838172e-01, 4.314446e-17, -6.722186e-03, -6.978803e-01]
            ),
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

    def test_getting_data_when_no_qr_codes_have_been_added(self):
        time.sleep(self.delay)
        self.mock.assert_not_called()

    def test_adding_data(self):
        self.test_node.add_qr_code(self.qr_code)
        time.sleep(self.delay)

        self.mock.assert_called_once_with(QRCodeList(qr_codes=[self.qr_code]))
