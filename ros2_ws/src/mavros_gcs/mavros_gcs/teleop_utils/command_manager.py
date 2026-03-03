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
        self._keyboard_active = True

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
        
        # Track last *executed* non-hover command
        self._last_executed_cmd: Optional[Command] = None
        self._last_executed_time_s: Optional[float] = None
    
    def toggle_keyboard(self) -> bool:
        """
        Toggles teleop input on/off
        """
        self._keyboard_active = not self._keyboard_active
        return self._keyboard_active
    
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

    def _is_non_hover_velocity_yaw(self, cmd: Optional[Command]) -> bool:
        if cmd is None:
            return False
        if not isinstance(cmd, VelocityYaw):
            return False
        return not getattr(cmd, "hover", False)

    def _maybe_send_hover_keepalive(self, now_s: float) -> None:
        # Only send hover if last executed cmd was non-hover VelocityYaw and it was < 1s ago
        if (
            self._last_executed_time_s is not None
            and self._is_non_hover_velocity_yaw(self._last_executed_cmd)
            and (now_s - self._last_executed_time_s) < 1.0
        ):
            self.execute_hover()
        # else: send nothing

    def _is_allowed_now(self, cmd: Command):
        """
        Returns true if (teleop input is on) OR (command is allowed while teleop input is off) 
        """
        return self._keyboard_active or cmd.allow_when_console_inactive

    def _select_candidate(self, state: Dict[str, bool]) -> Command:
        """
        Iterate priority list; return first triggered command.
        Returns None if no command is selected (do not default to hover).
        """
        for cmd in self.commands:
            if not cmd.is_triggered(state):
                continue
            if self._is_allowed_now(cmd):
                return cmd

        return None

    
    def tick(self, state_tuple: tuple):
        state, _ = state_tuple
        now_s = time.time()

        candidate = self._select_candidate(state)

        # If nothing triggered and nothing on hold, do not send hover blindly.
        if candidate is None and self._hold is None:
            self._maybe_send_hover_keepalive(now_s)
            return

        # If nothing triggered but we have a hold, we may need to drop it if released.
        if candidate is None and self._hold is not None:
            hold_cmd = self._hold.cmd
            if not hold_cmd.is_triggered(state):
                self._hold = None
                self._maybe_send_hover_keepalive(now_s)
                return
            # else: keep holding existing command; fall through to update

        # Normal hold-slot setup/arbitration (only if candidate exists)
        if candidate is not None:
            if self._hold is None:
                self._hold = HoldSlot(
                    cmd=candidate,
                    ticks_left=candidate.get_required_ticks(),
                    complete=False,
                )
            else:
                hold_cmd = self._hold.cmd
                if not hold_cmd.is_triggered(state):
                    self._hold = HoldSlot(
                        cmd=candidate,
                        ticks_left=candidate.get_required_ticks(),
                        complete=False,
                    )
                else:
                    if candidate.priority > hold_cmd.priority:
                        self._hold = HoldSlot(
                            cmd=candidate,
                            ticks_left=candidate.get_required_ticks(),
                            complete=False,
                        )

        self._update_selected(state_tuple)

    def _update_selected(self, state_tuple: tuple) -> None:
        state, _ = state_tuple
        now_s = time.time()

        slot = self._hold
        if slot is None:
            self._maybe_send_hover_keepalive(now_s)
            return

        cmd = slot.cmd

        # If command set to be complete, it means latched command is being sent
        if slot.complete:
            self._maybe_send_hover_keepalive(now_s)
            return

        # Countdown logic
        if slot.ticks_left > 0:
            slot.ticks_left -= 1
            cmd.on_hold_hook()

            # Only send hover if your rule allows it
            self._maybe_send_hover_keepalive(now_s)
            return

        # ticks_left == 0, execute it
        cmd.execute(state)

        # Remember last EXECUTED command (except hover)
        if cmd is not self._hover_command and not getattr(cmd, "hover", False):
            self._last_executed_cmd = cmd
            self._last_executed_time_s = now_s

        if cmd.latch:
            slot.complete = True
        else:
            self._hold = None