import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from .submodules.crud import add_data
from .submodules.init import init


class AddData(Node):

    def __init__(self):
        super().__init__('add_data')
        self.subscription = self.create_subscription(
            String,
            'add_data',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
        init()
        add_data(msg.data)
        self.get_logger().info(f"Added to db:  '{msg.data}'")


def main(args=None):
    rclpy.init(args=args)
    adder = AddData()
    rclpy.spin(adder)
    adder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
