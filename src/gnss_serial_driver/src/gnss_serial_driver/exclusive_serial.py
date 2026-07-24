#!/usr/bin/env python3
import fcntl
import os
import struct
import termios
import tty


BAUD = {
    9600: termios.B9600,
    19200: termios.B19200,
    38400: termios.B38400,
    57600: termios.B57600,
    115200: termios.B115200,
}
for name in ("B230400", "B460800", "B921600"):
    if hasattr(termios, name):
        BAUD[int(name[1:])] = getattr(termios, name)


def open_locked(path: str, baud: int, dtr: bool = True, rts: bool = False) -> int:
    if baud not in BAUD:
        raise ValueError("unsupported baud {}".format(baud))
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        tty.setraw(fd)
        attrs = termios.tcgetattr(fd)
        attrs[4] = BAUD[baud]
        attrs[5] = BAUD[baud]
        attrs[6][termios.VMIN] = 0
        attrs[6][termios.VTIME] = 1
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        if hasattr(termios, "TIOCMGET"):
            bits = struct.unpack("I", fcntl.ioctl(fd, termios.TIOCMGET, struct.pack("I", 0)))[0]
            bits = bits | termios.TIOCM_DTR if dtr else bits & ~termios.TIOCM_DTR
            bits = bits | termios.TIOCM_RTS if rts else bits & ~termios.TIOCM_RTS
            fcntl.ioctl(fd, termios.TIOCMSET, struct.pack("I", bits))
        return fd
    except Exception:
        os.close(fd)
        raise
