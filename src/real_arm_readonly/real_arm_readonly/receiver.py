"""Read-only fd; tty access requires physical isolation or explicit UART-open authorization.

O_RDONLY prevents application writes, NOT kernel-generated control-line changes.
UART-open authorization is a separate user decision, not a claim of physical isolation.
"""
import fcntl
import os
import stat
import termios


class Receiver:
    def __init__(self, path, rx_isolated=False, *, uart_open_authorized=False):
        if not (rx_isolated or uart_open_authorized):
            raise RuntimeError('Refusing device open: neither RX isolation nor UART-open authorization was supplied')
        resolved = os.path.realpath(path)
        if not (resolved.startswith('/dev/ttyUSB') or resolved.startswith('/dev/ttyACM')):
            raise ValueError('Only an explicitly selected USB serial device is supported')
        self.fd = None
        fd = os.open(resolved, os.O_RDONLY | os.O_NONBLOCK | os.O_NOCTTY | os.O_CLOEXEC)
        try:
            if not stat.S_ISCHR(os.fstat(fd).st_mode):
                raise ValueError('Expected character device')
            # Does not evict an existing holder. Operator must check ownership first.
            fcntl.ioctl(fd, termios.TIOCEXCL)
            attrs = termios.tcgetattr(fd)
            attrs[0] = 0  # no XON/XOFF, translations, or parity processing
            attrs[1] = 0  # no output processing
            attrs[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
            attrs[3] = 0  # no echo/canonical mode/signal generation
            attrs[4] = attrs[5] = termios.B921600
            attrs[6][termios.VMIN] = 0
            attrs[6][termios.VTIME] = 0
            termios.tcsetattr(fd, termios.TCSANOW, attrs)
        except BaseException:
            os.close(fd)
            raise
        self.fd = fd

    def read(self):
        try:
            return os.read(self.fd, 4096)
        except BlockingIOError:
            return b''

    def close(self):
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None
