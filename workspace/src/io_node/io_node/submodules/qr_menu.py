import os
import select
import sys
import rclpy

import termios
import tty

TURTLEBOT3_MODEL = os.environ["TURTLEBOT3_MODEL"]

QR_MENU = """
List of observed QR codes
---------------------------

m : return to main menu

CTRL-C to quit
"""


class QRMenu:

    def __init__(self, return_to_menu) -> None:
        self._return_to_menu = return_to_menu
        self._running = False

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
        print(QR_MENU)

        while(self._running):
            key = self._get_key(settings)

            if key == "\x03":
                exit(0)
            elif key == "m":
                self._return_to_menu()
