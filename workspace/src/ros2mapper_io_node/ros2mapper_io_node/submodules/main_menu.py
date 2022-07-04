import os
import sys
import termios
import threading
from typing import Callable, List

from rclpy.node import Publisher
from std_msgs.msg import String

from .view import View

MAIN_MENU = """
Main menu
---------------------------
1: Start exploring
2: Stop exploring
3: Observed QR codes
4: Manual override
---------------------------
CTRL-C to quit
---------------------------

"""


class MainMenu(View):
    def __init__(self, publisher: Publisher) -> None:
        super().__init__()
        self._publisher = publisher
        self._load_manual_control_view = None
        self._load_qr_view = None
        self._data_to_log = None

    def set_load_functions(
        self,
        load_manual_control_view: Callable[[], None],
        load_qr_view: Callable[[], None],
    ) -> None:
        """Set the view load functions

        Args:
            load_manual_control_view (Callable[[], None]): Manual control view loader function
            load_qr_view (Callable[[], None]): QR view loader function
        """
        self._load_manual_control_view = load_manual_control_view
        self._load_qr_view = load_qr_view

    def log(self, data: str) -> None:
        """Log given data to logger

        Args:
            data (str): Data to log
        """
        if self.running:
            self._data_to_log = data

    def _exploration_callback(self, msg_command: String) -> None:
        """Publish user input for starting/stopping autonomous exploration."""
        self._publisher.publish(String(data=msg_command))

    def _print_menu(self) -> None:
        """Prints the main menu"""
        os.system("clear")
        print(MAIN_MENU)

    def _handle_io(self) -> None:
        """Handles the inputs of the user while in the menu"""
        self._print_menu()
        log_count = 0

        settings = termios.tcgetattr(sys.stdin)
        while self.running:
            key = self._get_key(settings)
            if self._data_to_log:
                if log_count == 10:
                    self._print_menu()
                    log_count = 0
                log_count += 1
                print(self._data_to_log)
                self._data_to_log = None

            if key == "\x03":
                self.running = False
                exit(0)
            elif key == "1":
                self._exploration_callback("1")
                self._print_menu()
                print("Autonomous exploration started")
            elif key == "2":
                self._exploration_callback("2")
                self._print_menu()
                print("Autonomous exploration stopped")
            elif key == "3":
                self._load_qr_view()
            elif key == "4":
                self._load_manual_control_view()
            elif key:
                self._print_menu()
                print("Input not recognized")

    def _main(self):
        self.thread = threading.Thread(target=self._handle_io)
        self.thread.start()
