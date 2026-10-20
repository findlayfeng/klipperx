#!/usr/bin/env python3
"""Ask a klipper USB-CDC board for its bootloader the Arduino way.

Sets the line coding to 1200 baud and then drops DTR: the firmware's
`usb_req_set_control_line_state` (src/generic/usb_cdc.c:524) calls
`bootloader_request()` on exactly that shape, which is the one way in when the
firmware is too wedged to answer `identify` and there is no debugger.

Usage: usb-bootloader-request.py /dev/ttyACM1
"""

import fcntl
import os
import struct
import sys
import termios


def main(path: str) -> int:
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        attrs = termios.tcgetattr(fd)
        attrs[4] = termios.B1200
        attrs[5] = termios.B1200
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        # TIOCMBIC = clear the given modem bits: DTR low is the half the
        # firmware waits for.
        fcntl.ioctl(fd, termios.TIOCMBIC, struct.pack("I", termios.TIOCM_DTR))
    finally:
        os.close(fd)
    print(f"bootloader request sent to {path}")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    sys.exit(main(sys.argv[1]))
