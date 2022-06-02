import unittest
from std_msgs.msg import String
from rclpy.node import Node
import rclpy


class MemoryNodeTest(unittest.TestCase, Node):
    def setUp(self) -> None:
        Node.__init__(self, "memory_node_test")
        self.publisher = self.create_publisher(String, "/add_data", 10)

    def test_adding_data(self):
        msg = String()
        msg.data = str("test")
        self.publisher.publish(msg)
