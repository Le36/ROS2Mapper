import os
import sys
import termios
import threading
from typing import List

from interfaces.msg import QRCode

from .view import View

QR_MENU = """
List of observed QR codes
---------------------------
%s
---------------------------
1-9 : navigate to a found QR code
0 : stop navigation

m : return to main menu

CTRL-C to quit
---------------------------

"""


class QRMenu(View):
    def __init__(self, return_to_menu, stop_exploring, publisher) -> None:
        super().__init__()
        self._return_to_menu = return_to_menu
        self._stop_exploring = stop_exploring
        self._publisher = publisher

        self._data_to_log = None
        self._reprint_menu = False
        self._qr_codes: List[QRCode] = []

    def log(self, data: str) -> None:
        """Log given data to logger

        Args:
            data (str): Data to log
        """
        if self.running:
            self._data_to_log = data

    def qr_listener_callback(self, qr_code: QRCode) -> None:
        """Listen for QR codes being found and add them to QR menu list

        Args:
            qr_code (QRCode): QR code to be added to the list
        """
        for local_qr_code in self._qr_codes:
            if qr_code.id == local_qr_code.id:
                return

        self._qr_codes.append(qr_code)
        if self.running:
            self._reprint_menu = True

    def _qr_navigation_callback(self, qr_code: QRCode) -> None:
        """Publish user input for QR code to be navigated to

        Args:
            qr_code (QRCode): QR code to be navigated to
        """
        self._publisher.publish(qr_code)

    def _print_menu(self) -> None:
        """Print the QR code menu with found QR codes"""
        os.system("clear")
        if self._qr_codes:
            formatted_list = [
                f"{i+1}: '{qr_code.id}'" for i, qr_code in enumerate(self._qr_codes)
            ]
            print(QR_MENU % "\n".join(formatted_list))
        else:
            print(QR_MENU % "No QR codes found")

    def _handle_io(self):
        """Handles the inputs of the user while in the menu"""
        self._print_menu()
        log_count = 0

        settings = termios.tcgetattr(sys.stdin)

        while self.running:
            key = self._get_key(settings)
            if self._reprint_menu:
                self._print_menu()
                log_count = 0
                self._reprint_menu = False
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
            elif key.isdigit():
                index = int(key)
                if index < 0 or index > 9:
                    self._print_menu()
                    print("QR code index error. Please input a number between 1-9.")
                elif index == 0:
                    self._stop_exploring()
                    self._print_menu()
                    print("Navigation stopped")
                elif index > len(self._qr_codes):
                    self._print_menu()
                    print("Index out of range")
                else:
                    self._stop_exploring()
                    self._qr_navigation_callback(self._qr_codes[index - 1])
                    self._print_menu()
                    print(
                        f"Navigating to QR code with id {self._qr_codes[index - 1].id}"
                    )
            elif key == "m":
                self._return_to_menu()
            elif key:
                self._print_menu()
                print("Input not recognized")

    def _main(self):
        self.thread = threading.Thread(target=self._handle_io)
        self.thread.start()
