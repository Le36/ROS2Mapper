import select
import sys
import termios
import tty
from typing import List


class View:
    def __init__(self) -> None:
        self.running = False

    def open(self) -> None:
        """Open the view"""
        self.running = True
        self._main()

    def close(self) -> None:
        """Close the view"""
        self.running = False

    def _get_key(self, settings: List) -> str:
        """Interpret the key input by the user

        Args:
            settings (List): Settings fetched with termios.tcgetattr(sys.stdin)

        Returns:
            str: The key input by the user
        """
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
            if rlist:
                key = sys.stdin.read(1)
            else:
                key = ""

            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            return key
        except ValueError:
            exit(0)

    def _main(self):
        pass
