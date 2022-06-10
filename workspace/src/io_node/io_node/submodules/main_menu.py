import os
import select
import sys
import rclpy

import termios
import tty

TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]

MAIN_MENU = """
Main menu
---------------------------
1: Start exploring
2: Stop exploring
3: Observed QR codes
4: Manual override

CTRL-C to quit
"""


class MainMenu:

    def __init__(self) -> None:
        self._load_manual_control_view = None
        self._load_qr_view = None
        self._running = False

    def set_load_functions(self, load_manual_control_view, load_qr_view):
        self._load_manual_control_view = load_manual_control_view
        self._load_qr_view = load_qr_view
        print(load_manual_control_view)
        print(load_qr_view)

    def _get_key(self, settings):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ""

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def open(self):
        self._running = True
        self._main()

    def close(self):
        self._running = False

    def _main(self):
        settings = termios.tcgetattr(sys.stdin)
        os.system("clear")
        print(MAIN_MENU)

        while self._running:
            key = self._get_key(settings)

            if key == "\x03":
                exit(0)
            elif key == "1":
                os.system("clear")
                print(MAIN_MENU)
                print("Autonomous exploration started")
            elif key == "2":
                os.system("clear")
                print(MAIN_MENU)
                print("Autonomous exploration stopped")
            elif key == "3":
                self._load_qr_view()
            elif key == "4":
                self._load_manual_control_view()
