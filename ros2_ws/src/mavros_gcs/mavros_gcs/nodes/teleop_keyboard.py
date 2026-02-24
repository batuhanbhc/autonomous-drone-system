#!/usr/bin/env python3
import sys
import time
import signal
import threading
from typing import Dict

import termios

from evdev import InputDevice, ecodes


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
            return self  # nothing to do

        self._old = termios.tcgetattr(self.fd)

        new = termios.tcgetattr(self.fd)

        # Disable echo
        new[3] &= ~termios.ECHO

        # Disable canonical mode (so characters don't get line-buffered)
        new[3] &= ~termios.ICANON

        # Make reads return quickly if anything tries to read stdin
        new[6][termios.VMIN] = 0
        new[6][termios.VTIME] = 0

        termios.tcsetattr(self.fd, termios.TCSADRAIN, new)
        return self

    def __exit__(self, exc_type, exc, tb):
        if self._old is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self._old)


class KeyState:
    """
    Class for storing and adjusting the state of keyboard keys
    """
    def __init__(self):
        self._lock = threading.Lock()
        self._state: Dict[str, bool] = {}

    def set_key(self, key_name: str, pressed: bool):
        with self._lock:
            if pressed:
                self._state[key_name] = True
            else:
                self._state.pop(key_name, None)

    def snapshot(self):
        with self._lock:
            return dict(self._state)


def callback_handler(state: Dict[str, bool]):
    pressed = sorted([k for k, v in state.items() if v])
    print(pressed)


def start_timer_callback(get_state, handler, hz, stop_event,):
    """
    get_state:  Callback function to obtain state of the keys
    handler:  Handler function to be called when timer ticks
    hz: Frequency of timer 
    stop_event: Threading Event object to stop timer
    """ 
    if hz <= 0:
        raise ValueError("hz must be > 0")

    period = 1.0 / hz

    def loop():
        next_t = time.perf_counter()
        while not stop_event.is_set():
            next_t += period

            try:
                handler(get_state())
            except Exception as e:
                print(f"\nCallback error: {e}", file=sys.stderr)

            remaining = next_t - time.perf_counter()
            if remaining > 0:
                stop_event.wait(remaining)

    t = threading.Thread(target=loop, name="key-state-callback", daemon=True)
    t.start()
    return t


def main():
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} /dev/input/eventX [hz]")
        sys.exit(1)

    path = sys.argv[1]
    hz = float(sys.argv[2]) if len(sys.argv) >= 3 else 4.0

    dev = InputDevice(path)
    key_state = KeyState()
    stop_event = threading.Event()

    # Disable terminal echo so typed characters won't appear.
    with NoEchoTerminal(sys.stdin.fileno()):
        start_timer_callback(
            get_state=key_state.snapshot,
            handler=callback_handler,
            hz=hz,
            stop_event=stop_event,
        )

        try:
            for event in dev.read_loop():
                if event.type != ecodes.EV_KEY:
                    continue
                key_name = ecodes.KEY.get(event.code, f"UNKNOWN_{event.code}")
                if event.value == 1:
                    key_state.set_key(key_name, True)
                elif event.value == 0:
                    key_state.set_key(key_name, False)
        except KeyboardInterrupt:
            pass
        finally:
            stop_event.set()
            print()


if __name__ == "__main__":
    main()