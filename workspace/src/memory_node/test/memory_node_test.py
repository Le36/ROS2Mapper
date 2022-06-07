import os
import threading
import time
import unittest

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

from ..memory_node.memory_node import MemoryNode
from ..memory_node.submodules.data_repository import data_repository


class NodeNode(Node):
    def __init__(self):
        super().__init__("test_node")
        self.publisher = self.create_publisher(String, "/add_data", 10)

    def send_data(self, data: str):
        self.publisher.publish(String(data=data))


class MemoryNodeTest(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        rclpy.init()
        self.delay = int(os.getenv("DELAY")) if os.getenv("DELAY") else 0.1

        self.test_node = NodeNode()
        self.memory_node = MemoryNode()
        self.memory_node.get_logger().set_level(40)

        executor = MultiThreadedExecutor()
        executor.add_node(self.memory_node)
        executor.add_node(self.test_node)
        self.executor_thread = threading.Thread(target=executor.spin)
        self.executor_thread.start()
        time.sleep(self.delay)

    def setUp(self):
        data_repository.delete_all()

    @classmethod
    def tearDownClass(self):
        rclpy.shutdown()
        self.executor_thread.join()

    def test_adding_data(self):
        self.test_node.send_data("1")
        time.sleep(self.delay)
