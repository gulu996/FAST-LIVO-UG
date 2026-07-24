import errno
import fcntl
import os
import struct
import termios
import tty


DEFAULT_DTR = True
DEFAULT_RTS = False

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


def _set_control_lines(fd: int, dtr: bool, rts: bool) -> None:
    required = ("TIOCMGET", "TIOCMSET", "TIOCM_DTR", "TIOCM_RTS")
    if any(not hasattr(termios, name) for name in required):
        raise OSError(errno.ENOTSUP, "serial control-line ioctl is unavailable")
    bits = struct.unpack(
        "I", fcntl.ioctl(fd, termios.TIOCMGET, struct.pack("I", 0))
    )[0]
    bits = bits | termios.TIOCM_DTR if dtr else bits & ~termios.TIOCM_DTR
    bits = bits | termios.TIOCM_RTS if rts else bits & ~termios.TIOCM_RTS
    fcntl.ioctl(fd, termios.TIOCMSET, struct.pack("I", bits))


def open_locked(
    path: str,
    baud: int,
    dtr: bool = DEFAULT_DTR,
    rts: bool = DEFAULT_RTS,
) -> int:
    if baud not in BAUD:
        raise ValueError("unsupported baud {}".format(baud))
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        tty.setraw(fd)
        attrs = termios.tcgetattr(fd)
        attrs[4], attrs[5] = BAUD[baud], BAUD[baud]
        attrs[6][termios.VMIN], attrs[6][termios.VTIME] = 0, 1
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        try:
            _set_control_lines(fd, bool(dtr), bool(rts))
        except OSError as error:
            reason = error.strerror or str(error)
            raise OSError(
                error.errno or errno.EIO,
                "cannot set DTR={} RTS={} on {}: {}".format(
                    bool(dtr), bool(rts), path, reason
                ),
            ) from error
        return fd
    except Exception:
        os.close(fd)
        raise
