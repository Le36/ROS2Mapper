import unittest
from std_msgs.msg import String
import rclpy
from unittest.mock import Mock



from memory_node import memory_node



class MemoryNodeTest(unittest.TestCase):
    rclpy.init()
    test = memory_node.MemoryNode()

    @classmethod
    def tearDownClass(self):
        rclpy.shutdown()
        self.test.drop_table()

    def test_adding_data(self):
        msg = String()
        msg.data = str("test")
        self.test.add_data_callback(msg)
        inDatabase = self.test.read_data_callback("test")
        self.assertEqual(inDatabase, msg.data)
        


