import termios
import threading
import sys
import time

from dataclasses import dataclass
from typing import Optional, Literal, Tuple, Dict, Set, FrozenSet

@dataclass(frozen=True)
class COMMAND_CONFIG:
    key_list: Tuple[str, ...]
    activation_switch: bool
    activation_switch_key: Optional[str]
    press_type: Literal["ANY", "ALL"]


MOVEMENT_SWITCH = "KEY_SPACE"
MODE_SWITCH = "KEY_RIGHTALT"

TELEOP_CONFIG: Dict[str, COMMAND_CONFIG] = {
    "ACTION": COMMAND_CONFIG(
        key_list=(
            "KEY_W",
            "KEY_A",
            "KEY_S",
            "KEY_D",
            "KEY_UP",
            "KEY_DOWN",
            "KEY_RIGHT",
            "KEY_LEFT",
        ),
        activation_switch=True,
        activation_switch_key=MOVEMENT_SWITCH,
        press_type="ANY",
    ),
    "LAND": COMMAND_CONFIG(
        key_list=("KEY_L",),
        activation_switch=True,
        activation_switch_key=MODE_SWITCH,
        press_type="ALL",
    ),
    "RTL": COMMAND_CONFIG(
        key_list=("KEY_R",),
        activation_switch=True,
        activation_switch_key=MODE_SWITCH,
        press_type="ALL",
    ),
    "LOITER": COMMAND_CONFIG(
        key_list=("KEY_H",),
        activation_switch=True,
        activation_switch_key=MODE_SWITCH,
        press_type="ALL",
    ),
    "TAKEOFF": COMMAND_CONFIG(
        key_list=("KEY_T",),
        activation_switch=True,
        activation_switch_key=MODE_SWITCH,
        press_type="ALL",
    ),
    "SPEED_UP": COMMAND_CONFIG(
        key_list=("KEY_EQUAL",),
        activation_switch=False,
        activation_switch_key=None,
        press_type="ALL",
    ),
    "SPEED_DOWN": COMMAND_CONFIG(
        key_list=("KEY_MINUS",),
        activation_switch=False,
        activation_switch_key=None,
        press_type="ALL",
    ),
    "KILL_SWITCH": COMMAND_CONFIG(
        key_list=("KEY_K", "KEY_I", "KEY_L"),
        activation_switch=True,
        activation_switch_key=MODE_SWITCH,
        press_type="ALL",
    ),
    "KILL_CONFIRM": COMMAND_CONFIG(
        key_list=("KEY_Y",),
        activation_switch=True,
        activation_switch_key=MODE_SWITCH,
        press_type="ALL",
    ),
    "ARM": COMMAND_CONFIG(
        key_list=("KEY_A",),
        activation_switch=True,
        activation_switch_key=MODE_SWITCH,
        press_type="ALL",
    ),
    "DISARM": COMMAND_CONFIG(
        key_list=("KEY_D",),
        activation_switch=True,
        activation_switch_key=MODE_SWITCH,
        press_type="ALL",
    ),
    "CONSOLE_TOGGLE": COMMAND_CONFIG(
        key_list=("KEY_ESC",),
        activation_switch=True,
        activation_switch_key=MODE_SWITCH,
        press_type="ALL",
    ),
    "CONTROL_TOGGLE": COMMAND_CONFIG(
        key_list=("KEY_C",),
        activation_switch=True,
        activation_switch_key=MODE_SWITCH,
        press_type="ALL",
    ),
}

class ClampedIndex:
    """
    Clamped index object, allows safe incrementing and decrementing index values within the valid index range of a fixed-length list
    """
    def __init__(self, start: int, size: int):
        if size <= 0:
            raise ValueError("size must be >= 1")
        self._max_len = size
        self._idx = max(0, min(start, size - 1))
        self._lock = threading.Lock()

    def increase(self):
        with self._lock:
            self._idx = min(self._idx + 1, self._max_len - 1)

    def decrease(self):
        with self._lock:
            self._idx = max(self._idx - 1, 0)

    def get(self):
        with self._lock:
            return self._idx

    def set_min(self):
        with self._lock:
            self._idx = 0

    def set_max(self):
        with self._lock:
            self._idx = self._max_len - 1


class NoEchoTerminal:
    """
    Put the controlling terminal into cbreak mode and disable ECHO,
    then restore on exit.
    """
    def __init__(self, fd):
        self.fd = fd
        self._old = None

    def __enter__(self):
        if not sys.stdin.isatty():
            return self

        self._old = termios.tcgetattr(self.fd)
        new = termios.tcgetattr(self.fd)

        # Disable echo
        new[3] &= ~termios.ECHO

        termios.tcsetattr(self.fd, termios.TCSADRAIN, new)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self._old)


class KeyState:
    """Thread-safe key pressed state store."""
    def __init__(self):
        self._lock = threading.Lock()
        self._pressed: Set[str] = set()
        self._last_event_t = time.monotonic()

    def add_key(self, key_name: str) -> None:
        with self._lock:
            self._last_event_t = time.monotonic()
            self._pressed.add(key_name)

    def remove_key(self, key_name: str) -> None:
        with self._lock:
            self._last_event_t = time.monotonic()
            self._pressed.discard(key_name)  # no KeyError if missing

    def snapshot(self) -> Tuple[FrozenSet[str], float]:
        with self._lock:
            # Return an immutable view so callers can’t mutate internal state
            return frozenset(self._pressed), self._last_event_t


