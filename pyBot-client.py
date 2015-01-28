import socket
import sys
import select
import termios
import tty
import signal


class TimeoutException(Exception):
    pass


def getKey():
    def timeout_handler(signum, frame):
        raise TimeoutException()

    old_handler = signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(1)  # this is the watchdog timing
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    try:
        keyPressed = sys.stdin.read(1)
        # print "Read key"
    except TimeoutException:
        # print "Timeout"
        return "-"
    finally:
        signal.signal(signal.SIGALRM, old_handler)

    signal.alarm(0)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return keyPressed

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    host = '172.22.21.225'  # KUKA youBot IP address
    port = 12345  # The same port as used by the server
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    while True:
        key = getKey()
        if key == 'q':
            exit()
        else:
            s.sendall(key)
        print key
    data = s.recv(1024)
    s.close()
    print('Received', repr(data))