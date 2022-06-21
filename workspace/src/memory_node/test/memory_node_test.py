import os
import threading
import time
import unittest

import numpy as np
import rclpy
from interfaces.msg import QRCode
from interfaces.srv import GetQRCodes
from numpy import ndarray
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from ..memory_node.memory_node import MemoryNode
from ..memory_node.submodules.data_repository import data_repository


class NodeNode(Node):
    def __init__(self):
        super().__init__("test_node")
        self.publisher = self.create_publisher(QRCode, "/qr_code", 10)
        self.client = self.create_client(GetQRCodes, "get_qr_codes")

    def add_qr_code(self, qr_code: QRCode):
        self.publisher.publish(qr_code)

    def get_qr_codes(self):
        request = GetQRCodes.Request()
        self.future = self.client.call_async(request)


class MemoryNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        rclpy.init()
        self.delay = int(os.getenv("DELAY")) if os.getenv("DELAY") else 0.1

        self.test_node = NodeNode()
        self.memory_node = MemoryNode()
        # self.memory_node.get_logger().set_level(40)

        executor = MultiThreadedExecutor()
        executor.add_node(self.memory_node)
        executor.add_node(self.test_node)
        self.executor_thread = threading.Thread(target=executor.spin)
        self.executor_thread.start()
        time.sleep(self.delay)

    def setUp(self):
        data_repository.delete_all()

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

    def test_getting_data_when_no_qr_codes_have_been_added(self):
        self.test_node.get_qr_codes()
        time.sleep(self.delay)

        self.assertTrue(self.test_node.future.done())
        qr_codes = self.test_node.future.result().qr_codes
        self.assertCountEqual(qr_codes, [])

    def test_adding_data(self):
        self.test_node.add_qr_code(self.qr_code)
        time.sleep(self.delay)

        self.test_node.get_qr_codes()
        time.sleep(self.delay)

        self.assertTrue(self.test_node.future.done())
        qr_codes = self.test_node.future.result().qr_codes
        self.assertEqual(len(qr_codes), 1)
        self.assert_qr_codes_equal(qr_codes[0], self.qr_code)
