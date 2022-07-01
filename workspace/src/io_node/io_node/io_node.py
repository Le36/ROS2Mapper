import os
from typing import Union

import rclpy
from geometry_msgs.msg import Twist
from interfaces.msg import QRCode, QRCodeList
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

from .submodules.main_menu import MainMenu
from .submodules.manual_control import ManualControl
from .submodules.qr_menu import QRMenu

TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]


class IONode(Node):
    def __init__(self):
        super().__init__("io_node")

        # Publishers
        self.qr_menu_publisher = self.create_publisher(QRCode, "/qr_navigator", 5)
        self.exploration_publisher = self.create_publisher(
            String, "/autonomous_exploration", 5
        )
        self.manual_control_publisher = self.create_publisher(
            Twist, "/cmd_vel", QoSProfile(depth=10)
        )

        # Views
        self.main_menu = MainMenu(self.exploration_publisher)
        self.qr_menu = QRMenu(
            lambda: self.load_view(self.main_menu),
            lambda: self.exploration_publisher.publish(String(data="2")),
            self.qr_menu_publisher,
        )
        self.manual_control_menu = ManualControl(
            lambda: self.load_view(self.main_menu), self.manual_control_publisher
        )

        # Set main menu load functions
        self.main_menu.set_load_functions(
            lambda: self.load_view(self.manual_control_menu),
            lambda: self.load_view(self.qr_menu),
        )

        # Subscriptions
        self.log_subscription_ = self.create_subscription(String, "/log", self.log, 10)
        self.qr_code_subscription_ = self.create_subscription(
            QRCodeList, "/qr_code_list", self.qr_menu.qr_code_list_callback, 10
        )

        self.timer = self.create_timer(0.1, self.check_quit)

        self.view = self.main_menu
        self.view.open()

    def load_view(self, view: Union[MainMenu, ManualControl, QRMenu]) -> None:
        """Render the view given as a parameter

        Args:
            view (Union[MainMenu, ManualControl, QRMenu]): View to be rendered
        """
        self.view.close()
        self.view = view
        self.view.open()

    def check_quit(self) -> None:
        """Periodically (every 0.1s) checks if the program has been stopped"""
        if not self.view.running:
            exit(0)

    def log(self, msg: String) -> None:
        """Log data to the logger

        Args:
            msg (String): Data to log
        """
        self.main_menu.log(msg.data)
        self.qr_menu.log(msg.data)


def main(args=None):
    """Run the node"""
    rclpy.init(args=args)
    io_publisher = IONode()
    rclpy.spin(io_publisher)
    io_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
