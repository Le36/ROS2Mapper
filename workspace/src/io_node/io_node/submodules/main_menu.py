import os
import select
import sys
import termios
import threading
import tty
from typing import Callable, List

from std_msgs.msg import String

TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]

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


class MainMenu:
    def __init__(self, publisher: Callable[[String], None]) -> None:
        self._publisher = publisher
        self._load_manual_control_view = None
        self._load_qr_view = None
        self._data_to_log = None
        self.running = False

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

    def open(self) -> None:
        """Open the main menu view"""
        self.running = True
        self._main()

    def close(self) -> None:
        """Close the main menu view"""
        self.running = False

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

    def _get_key(self, settings: List) -> str:
        """Interpret the key input by the user

        Args:
            settings (List): Settings fetched with termios.tcgetattr(sys.stdin)

        Returns:
            str: The key input by the user
        """
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ""

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

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
        # self.thread = threading.Thread(target=self._handle_io)
        # self.thread.start()
        self._handle_io()
