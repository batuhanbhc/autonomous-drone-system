from typing import Dict, Optional
import time
import math
from dataclasses import dataclass

from mavros_gcs.teleop_utils.commands import Command, VelocityYaw
from mavros_gcs.teleop_utils.definitions import TELEOP_CONFIG, ClampedIndex
from mavros_gcs.teleop_utils.teleop_io import log_info, log_warn, log_error

@dataclass
class HoldSlot:
    cmd: "Command"
    ticks_left: int
    complete: bool = False


class CommandManager:
    """
    Enforces:
      - only one command executes per tick
      - priority arbitration
      - command_on_hold slot for hold-to-activate commands
      - latching behavior: once executed, won't re-execute until released
    """
    def __init__(self, commands, hz: float, *, vel_categories: dict, vel_start_index: int):
        one_tick_s = 1 / float(hz)

        # commands must be a sorted list in terms of priorities, starting with higher priority commands
        self.commands = list(commands)     

        # calculate required ticks for execution for each command once
        for cmd in self.commands:
            act_t = cmd.get_activation_time()
            if act_t <= 0:
                required_ticks = 0
            else:
                required_ticks = int(math.ceil(act_t/ one_tick_s))

            cmd.set_required_ticks(required_ticks)

        # command on hold slot
        self._hold: Optional[HoldSlot] = None

        # flag for accepting command inputs
        self._console_active = True

        # hover command to keep Hz > 2 requirement for ardupilot guided mode
        self._hover_command= VelocityYaw(
            config=TELEOP_CONFIG["ACTION"],
            hook_fn=None,
            latch=False,
            activation_time_s=0.0,
            hover=True,
        )

        self.vel_categories = vel_categories
        size = len(self.vel_categories["horizontal"])
        self.vel_idx = ClampedIndex(start=vel_start_index, size=size)

        # Time variable that stores when kill-switch window will close
        self._kill_pending_until: float | None = None

    def arm_kill_window(self, seconds: float = 2.0) -> None:
        self._kill_pending_until = time.monotonic() + float(seconds)

    def kill_window_open(self) -> bool:
        return self._kill_pending_until is not None and time.monotonic() <= self._kill_pending_until

    def clear_kill_window(self) -> None:
        self._kill_pending_until = None

    def toggle_console(self) -> bool:
        """
        Toggles teleop input on/off
        """
        self._console_active = not self._console_active
        return self._console_active
    
    def get_velocity(self):
        """
        Returns in order horizontal velocity level, vertical velocity level, and velocity index
        """
        idx = self.vel_idx.get()
        return (
            float(self.vel_categories["horizontal"][idx]),
            float(self.vel_categories["vertical"][idx]),
            idx,
        )
    
    def increment_velocity(self):
        """
        Increases velocity index within allowed range.
        """
        self.vel_idx.increase()
        return self.get_velocity()
    
    def decrement_velocity(self):
        """
        Decreases velocity index within allowed range.
        """
        self.vel_idx.decrease()
        return self.get_velocity()
    
    def execute_hover(self):
        self._hover_command.execute(state=dict())

    def _is_allowed_now(self, cmd: Command):
        """
        Returns true if (teleop input is on) OR (command is allowed while teleop input is off) 
        """
        return self._console_active or cmd.allow_when_console_inactive

    def _select_candidate(self, state: Dict[str, bool]) -> Command:
        """
        Iterate priority list; return first triggered command.
        Issues dummy velocity command in case no command is selected
        """
        for cmd in self.commands:
            if not cmd.is_triggered(state):
                continue
            
            if self._is_allowed_now(cmd):
                return cmd
                    
        # No command input was given
        return self._hover_command

    def tick(self, state_tuple: tuple):
        """
        Callback function for timer.
        Selects which command to issue, updates command on hold structure, executes the command
        """
        state, _ = state_tuple

        # Select which command to run
        candidate = self._select_candidate(state)

        # Decide whether to switch/update hold slot
        if self._hold is None:
            self._hold = HoldSlot(
                cmd=candidate,
                ticks_left=candidate.get_required_ticks(),
                complete=False,
            )
        else:
            hold_cmd = self._hold.cmd

            # If current hold command is no longer triggered, drop it
            # (this is also the "release resets latch" behavior).
            if not hold_cmd.is_triggered(state):
                self._hold = HoldSlot(
                    cmd=candidate,
                    ticks_left=candidate.get_required_ticks(),
                    complete=False,
                )
            else:
                # Arbitration: if candidate has higher priority than current hold, preempt
                if candidate.priority > hold_cmd.priority:
                    self._hold = HoldSlot(
                        cmd=candidate,
                        ticks_left=candidate.get_required_ticks(),
                        complete=False,
                    )
                # else keep current hold

        # Execute the command
        self._update_selected(state_tuple)


    def _update_selected(self, state_tuple: tuple) -> None:
        state, last_event_t = state_tuple

        slot = self._hold
        cmd = slot.cmd

        # If command set to be complete, it means latched command is being sent
        if slot.complete:
            # Send hover command instead
            self.execute_hover()
            return
        
        # Countdown logic
        if slot.ticks_left > 0:
            slot.ticks_left -= 1
            cmd.on_hold_hook()

            # Send hover command while waiting
            self.execute_hover()
        
        else:
            if not slot.complete:
                # ticks left = 0 and command not complete, execute it
                cmd.execute(state)

                if cmd.latch:
                    # command has latch behavior, do not clear slot, but mark it complete
                    slot.complete = True
                else:
                    # command does not have latch behavior, clear slot
                    self._hold = None
            else:
                # Send hover command instead
                self.execute_hover()
