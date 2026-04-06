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


ACTION_SWITCH = "KEY_SPACE"
COMMAND_SWITCH = "KEY_RIGHTALT"

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
        activation_switch_key=ACTION_SWITCH,
        press_type="ANY",
    ),
    "LAND": COMMAND_CONFIG(
        key_list=("KEY_L",),
        activation_switch=True,
        activation_switch_key=COMMAND_SWITCH,
        press_type="ALL",
    ),
    "RTL": COMMAND_CONFIG(
        key_list=("KEY_R",),
        activation_switch=True,
        activation_switch_key=COMMAND_SWITCH,
        press_type="ALL",
    ),
    "TAKEOFF": COMMAND_CONFIG(
        key_list=("KEY_T",),
        activation_switch=True,
        activation_switch_key=COMMAND_SWITCH,
        press_type="ALL",
    ),
    "GUIDED": COMMAND_CONFIG(
        key_list=("KEY_G",),
        activation_switch=True,
        activation_switch_key=COMMAND_SWITCH,
        press_type="ALL",
    ),
    "SPEED_UP_HORIZONTAL": COMMAND_CONFIG(
        key_list=("KEY_EQUAL", "KEY_F6"),
        activation_switch=True,
        activation_switch_key=COMMAND_SWITCH,
        press_type="ALL",
    ),
    "SPEED_DOWN_HORIZONTAL": COMMAND_CONFIG(
        key_list=("KEY_MINUS", "KEY_F6"),
        activation_switch=True,
        activation_switch_key=COMMAND_SWITCH,
        press_type="ALL",
    ),
    "SPEED_UP_VERTICAL": COMMAND_CONFIG(
        key_list=("KEY_EQUAL", "KEY_F7"),
        activation_switch=True,
        activation_switch_key=COMMAND_SWITCH,
        press_type="ALL",
    ),
    "SPEED_DOWN_VERTICAL": COMMAND_CONFIG(
        key_list=("KEY_MINUS", "KEY_F7"),
        activation_switch=True,
        activation_switch_key=COMMAND_SWITCH,
        press_type="ALL",
    ),
    "SPEED_UP_YAW": COMMAND_CONFIG(
        key_list=("KEY_EQUAL", "KEY_F8"),
        activation_switch=True,
        activation_switch_key=COMMAND_SWITCH,
        press_type="ALL",
    ),
    "SPEED_DOWN_YAW": COMMAND_CONFIG(
        key_list=("KEY_MINUS", "KEY_F8"),
        activation_switch=True,
        activation_switch_key=COMMAND_SWITCH,
        press_type="ALL",
    ),
    "KILL_SWITCH": COMMAND_CONFIG(
        key_list=("KEY_K", "KEY_I", "KEY_L"),
        activation_switch=True,
        activation_switch_key=COMMAND_SWITCH,
        press_type="ALL",
    ),
    "KILL_CONFIRM": COMMAND_CONFIG(
        key_list=("KEY_Y",),
        activation_switch=True,
        activation_switch_key=COMMAND_SWITCH,
        press_type="ALL",
    ),
    "ARM": COMMAND_CONFIG(
        key_list=("KEY_A",),
        activation_switch=True,
        activation_switch_key=COMMAND_SWITCH,
        press_type="ALL",
    ),
    "DISARM": COMMAND_CONFIG(
        key_list=("KEY_D",),
        activation_switch=True,
        activation_switch_key=COMMAND_SWITCH,
        press_type="ALL",
    ),
    "KEYBOARD_TOGGLE": COMMAND_CONFIG(
        key_list=("KEY_ESC",),
        activation_switch=True,
        activation_switch_key=COMMAND_SWITCH,
        press_type="ALL",
    ),
    "CONTROL_TOGGLE": COMMAND_CONFIG(
        key_list=("KEY_C",),
        activation_switch=True,
        activation_switch_key=COMMAND_SWITCH,
        press_type="ALL",
    ),
    "PRESS_SAFETY_SWITCH": COMMAND_CONFIG(
        key_list=("KEY_S",),
        activation_switch=True,
        activation_switch_key=COMMAND_SWITCH,
        press_type="ALL",
    ),
    "RECORD_VIDEO_TOGGLE": COMMAND_CONFIG(
        key_list=("KEY_F2",),
        activation_switch=True,
        activation_switch_key=COMMAND_SWITCH,
        press_type="ALL",
    ),
    "STREAM_TOGGLE": COMMAND_CONFIG(
        key_list=("KEY_F3",),
        activation_switch=True,
        activation_switch_key=COMMAND_SWITCH,
        press_type="ALL",
    ),
    "ALT_SUPPORT_TOGGLE": COMMAND_CONFIG(
        key_list=("KEY_H",),
        activation_switch=True,
        activation_switch_key=COMMAND_SWITCH,
        press_type="ALL",
    ),
}


class ClampedFloat:
    """
    Thread-safe float value clamped between [min_val, max_val],
    adjustable by a fixed increment.
    """
    def __init__(self, default: float, min_val: float, max_val: float, increment: float):
        if min_val > max_val:
            raise ValueError("min_val must be <= max_val")
        self._min = min_val
        self._max = max_val
        self._increment = increment
        self._value = max(min_val, min(default, max_val))
        self._lock = threading.Lock()

    def increase(self) -> float:
        with self._lock:
            self._value = min(self._value + self._increment, self._max)
            return self._value

    def decrease(self) -> float:
        with self._lock:
            self._value = max(self._value - self._increment, self._min)
            return self._value

    def get(self) -> float:
        with self._lock:
            return self._value


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