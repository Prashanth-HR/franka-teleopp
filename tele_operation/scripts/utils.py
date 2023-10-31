import sys
import termios
import tty
import select


old_settings = None


def set_up_terminal_for_key_check():
    global old_settings
    old_settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())


def reset_terminal():
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def check_for_key(target_key):
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        user_key = sys.stdin.read(1)
        if user_key == target_key:
            return True
        else:
            return False
    else:
        return False