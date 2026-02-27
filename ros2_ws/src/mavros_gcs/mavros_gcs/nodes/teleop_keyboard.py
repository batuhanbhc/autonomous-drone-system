#!/usr/bin/env python3
"""
Example:
  python3 teleop_keyboard.py /dev/input/eventX 10
"""

import sys
import time
import threading
from typing import List

from evdev import InputDevice, ecodes

from mavros_gcs.teleop_utils.definitions import NoEchoTerminal, KeyState, TELEOP_CONFIG
from mavros_gcs.teleop_utils.commands import (
    Command, KillConfirm, KillSwitch, Arm, Disarm,
    ConsoleToggle, ModeLand, ModeLoiter, ModeRTL, Takeoff, 
    SpeedDown, SpeedUp, VelocityYaw
)
from mavros_gcs.teleop_utils.command_helpers import assign_priorities_from_list_order
from mavros_gcs.teleop_utils.params import load_teleop_yaml
from mavros_gcs.teleop_utils.command_manager import CommandManager

# ---------------------------
# Timer thread

def start_timer_callback(get_state, handler, hz: float, stop_event: threading.Event):
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

# ---------------------------
# Main

def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} /dev/input/eventX /path/to/teleops_param_file")
        sys.exit(1)

    keyboard_path = sys.argv[1]
    
    dev = InputDevice(keyboard_path)
    key_state = KeyState()
    stop_event = threading.Event()

    # ---- Load param file ----
    teleop_cfg = load_teleop_yaml(sys.argv[2])

    hz = float(teleop_cfg.get("hz", 4.0))

    vel_categories = teleop_cfg["velocity"]["categories"]
    vel_start_index = int(teleop_cfg["velocity"]["categories"].get("default_start_index", 0))
    yaw_rate = float(teleop_cfg["velocity"].get("yaw_rate", 1.0))
    cmd_params = teleop_cfg["commands"]["params"]

    # -------------------------
    # command manager will be set properly after commands are created
    manager = None

    # IMPORTANT: Priorities are determined from list order. Lower index = higher priority
    commands: List[Command] = [
        KillConfirm(
            config=TELEOP_CONFIG["KILL_CONFIRM"],
            latch=cmd_params["KILL_CONFIRM"]["latch"],
            activation_time_s=cmd_params["KILL_CONFIRM"]["activation_time_s"],
            is_window_open_fn=lambda: manager.kill_window_open(),
            clear_window_fn=lambda: manager.clear_kill_window(),
        ),
        KillSwitch(
            config=TELEOP_CONFIG["KILL_SWITCH"],
            hook_fn=lambda seconds=2.0: manager.arm_kill_window(seconds),  # <-- accept arg + call
            latch=cmd_params["KILL_SWITCH"]["latch"],
            activation_time_s=cmd_params["KILL_SWITCH"]["activation_time_s"],
        ),
        Arm(
            config=TELEOP_CONFIG["ARM"],
            hook_fn=None,
            latch=cmd_params["ARM"]["latch"],
            activation_time_s=cmd_params["ARM"]["activation_time_s"],
        ),
        Disarm(
            config=TELEOP_CONFIG["DISARM"],
            hook_fn=None,
            latch=cmd_params["DISARM"]["latch"],
            activation_time_s=cmd_params["DISARM"]["activation_time_s"],
        ),
        ConsoleToggle(
            config=TELEOP_CONFIG["CONSOLE_TOGGLE"],
            hook_fn=lambda: manager.toggle_console(),
            latch=cmd_params["CONSOLE_TOGGLE"]["latch"],
            activation_time_s=cmd_params["CONSOLE_TOGGLE"]["activation_time_s"],
        ),
        ModeLand(
            config=TELEOP_CONFIG["LAND"],
            hook_fn=None,
            latch=cmd_params["MODE_LAND"]["latch"],
            activation_time_s=cmd_params["MODE_LAND"]["activation_time_s"],
        ),
        ModeRTL(
            config=TELEOP_CONFIG["RTL"],
            hook_fn=None,
            latch=cmd_params["MODE_RTL"]["latch"],
            activation_time_s=cmd_params["MODE_RTL"]["activation_time_s"],
        ),
        ModeLoiter(
            config=TELEOP_CONFIG["LOITER"],
            hook_fn=None,
            latch=cmd_params["MODE_LOITER"]["latch"],
            activation_time_s=cmd_params["MODE_LOITER"]["activation_time_s"],
        ),
        Takeoff(
            config=TELEOP_CONFIG["TAKEOFF"],
            hook_fn=None,
            latch=cmd_params["TAKEOFF"]["latch"],
            activation_time_s=cmd_params["TAKEOFF"]["activation_time_s"],
        ),
        SpeedUp(
            config=TELEOP_CONFIG["SPEED_UP"],
            hook_fn=lambda: manager.increment_velocity(),
            latch=cmd_params["SPEED_UP"]["latch"],
            activation_time_s=cmd_params["SPEED_UP"]["activation_time_s"],
        ),
        SpeedDown(
            config=TELEOP_CONFIG["SPEED_DOWN"],
            hook_fn=lambda: manager.decrement_velocity(),
            latch=cmd_params["SPEED_DOWN"]["latch"],
            activation_time_s=cmd_params["SPEED_DOWN"]["activation_time_s"],
        ),
        VelocityYaw(
            config=TELEOP_CONFIG["ACTION"],
            hook_fn=lambda: manager.get_velocity(),
            latch=cmd_params["VEL_YAW"]["latch"],
            activation_time_s=cmd_params["VEL_YAW"]["activation_time_s"],
            yaw_rate=yaw_rate,
        ),
    ]

    # set priority fields of commands from list order
    assign_priorities_from_list_order(commands)

    # set command manager properly
    manager = CommandManager(
        commands=commands,
        hz=hz,
        vel_categories=vel_categories,
        vel_start_index=vel_start_index,
    )

    def timer_handler(state: tuple):
        manager.tick(state)

    with NoEchoTerminal(sys.stdin.fileno()):
        start_timer_callback(
            get_state=key_state.snapshot,
            handler=timer_handler,
            hz=hz,
            stop_event=stop_event,
        )

        try:
            for event in dev.read_loop():
                if event.type != ecodes.EV_KEY:
                    continue
                
                key_name = ecodes.KEY.get(event.code, f"UNKNOWN_{event.code}")

                # event.value: 1=press, 0=release, 2=autorepeat
                if event.value == 1:
                    key_state.add_key(key_name)     # key pressed
                elif event.value == 0:
                    key_state.remove_key(key_name)  # key is let go
                else:
                    # Ignore autorepeat at the evdev level; timer tick handles behavior
                    pass

        except KeyboardInterrupt:
            pass
        finally:
            stop_event.set()
            print()


if __name__ == "__main__":
    main()