import os
import rclpy
from interfaces.msg import QRCode
from .submodules.manual_control import ManualControl
from .submodules.qr_menu import QRMenu
from .submodules.main_menu import MainMenu

from rclpy.node import Node
from std_msgs.msg import String

TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]


class IONode(Node):
    def __init__(self):
        super().__init__("io_node")

        """Create the autonomous exploration publisher"""
        self.main_menu_publisher_ = self.create_publisher(
            String, "/autonomous_exploration", 5
        )
        self.main_menu = MainMenu(self.main_menu_publisher_)

        """Create the QR navigator publisher"""
        self.qr_menu_publisher_ = self.create_publisher(String, "/qr_navigator", 5)
        self.qr_menu = QRMenu(
            lambda: self.load_view(self.main_menu), self.qr_menu_publisher_
        )
        self.qr_code_subscription_ = self.create_subscription(
            QRCode, "qr_list", self.qr_listener_callback, 10
        )

        self.manual_control = ManualControl(lambda: self.load_view(self.main_menu))

        self.main_menu.set_load_functions(
            lambda: self.load_view(self.manual_control),
            lambda: self.load_view(self.qr_menu),
        )

        self.view = self.main_menu
        self.view.open()

    def load_view(self, view):
        self.view.close()
        self.view = view
        self.view.open()

    def qr_listener_callback(self, qr_code: QRCode):
        """Listen for QR codes being found and add them to QR menu list"""
        print("I found", qr_code)


def main(args=None):
    """Run the node"""
    rclpy.init(args=args)
    io_publisher = IONode()
    rclpy.spin(io_publisher)
    io_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
