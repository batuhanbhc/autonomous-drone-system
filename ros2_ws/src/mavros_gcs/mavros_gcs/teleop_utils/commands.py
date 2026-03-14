from typing import Dict
import math

from mavros_gcs.teleop_utils.command_helpers import command_triggered
from mavros_gcs.teleop_utils.teleop_io import log_info, log_warn, log_error
from mavros_gcs.teleop_utils.teleop_io import publish_command, publish_action

class Command:
    """
    Base class for all teleop actions.

    A Command represents a high-level intent (e.g., LAND, RTL, TAKEOFF, velocity/yaw control)
    that can be triggered by the current input state and, if selected by CommandManager, executed.

    Key concepts:
    - Triggering:
        Subclasses implement `is_triggered(state) -> bool` to declare whether the command can 
        be active given the current key/button state.
    - Priority arbitration:
        `priority` is used by CommandManager to decide which commands to consider for execution first.
        Also handles cases when multiple commands are requested at the same time (higher priority wins).
    - Hold-to-activate (activation time):
        `activation_time_s` specifies how long the command must remain continuously triggered
        before execution. CommandManager converts this to a tick countdown once at startup via
        `_set_required_ticks(hz)` and then decrements the countdown every tick while the command
        remains triggered.
    - On-hold hook:
        `on_hold_hook()` is an optional per-tick callback while the command is being held
        (useful for warnings/feedback such as “arming in 2…1…”).
    - Execution:
        Subclasses implement `execute(state)` to perform the command’s effect.
        CommandManager calls `execute` when ticks required for execution reaches 0.
    - Latching:
        If `latch` is True, once executed the command will not execute again until it is released
        (i.e., it stops being triggered). This prevents repeated firing while a key is held.
        If `latch` is False, the command may execute repeatedly as long as it remains selected.


    Notes:
    - `_required_ticks` is computed once by the manager from `activation_time_s` and the manager’s
      tick rate (hz). `get_required_ticks()` will raise if the manager has not initialized it.
    """
        
    name: str = "Command"
    priority: int = 0
    activation_time_s: float = 0.0
    required_ticks: int = 0
    latch: bool = True

    def __init__(self, activation_time_s: float = 0.0):
        self.activation_time_s = float(activation_time_s)
        self.required_ticks: int | None = None  # computed once by manager

    def get_activation_time(self) -> float:
        return self.activation_time_s
     
    def set_required_ticks(self, ticks) -> None:
        self.required_ticks = ticks

    def get_required_ticks(self) -> int:
        if self.required_ticks is None:
            raise RuntimeError(
                f"{self} required ticks not initialized. "
                "Call set_required_ticks once during setup."
            )
        return self.required_ticks

    def is_triggered(self, state: Dict) -> bool:
        """
        Return True if command can be activated given current keys.
        """
        raise NotImplementedError

    def on_hold_hook(self):
        """
        Optional hook function to be run at every tick while the command is on hold
        """
        return
    
    def execute(self, state: Dict[str, bool]) -> None:
        """
        Implements command functionality.
        Called:
          - every tick command has activation time = 0.0 and latch = False
          - once when hold countdown finishes if latch = True
        """
        raise NotImplementedError

    def __repr__(self):
        return f"{self.name}(prio={self.priority}, act={self.activation_time_s})"

# ---------------------------

class KillSwitch(Command):
    """
    After sent, starts X second period that enables the activation of "KillConfirm" command.
    """
    name = "KILL_SWITCH"

    def __init__(self, config, hook_fn, latch, activation_time_s=2.0):
        self._keys = tuple(config.key_list)
        self._config = config
        self.activation_time_s = activation_time_s
        self.latch = latch

    def is_triggered(self, state):
        cfg = self._config
        return command_triggered(
            state, self._keys,
            cfg.press_type, cfg.activation_switch, cfg.activation_switch_key
        )

    def execute(self, state):
        publish_command(command_name=self.name)


class KillConfirm(Command):
    """
    Sends command request to open kill window for pre-determined seconds
    """
    name = "KILL_CONFIRM"
    priority = 9999          # ensure it's always selected when triggered

    def __init__(self, config, latch, hook_fn, activation_time_s):
        self._keys = tuple(config.key_list)  # confirm combo keys
        self._config = config
        self.latch = latch
        self.activation_time_s = activation_time_s


    def is_triggered(self, state):
        cfg = self._config
        return command_triggered(
            state, self._keys,
            cfg.press_type, cfg.activation_switch, cfg.activation_switch_key
        )

    def execute(self, state):
        publish_command(command_name=self.name)


class Arm(Command):
    """
    Sends command to arm the motors.
    """
    name = "ARM"

    def __init__(self, config, hook_fn, latch, activation_time_s=1.0):
        self._keys = tuple(config.key_list)
        self._config = config
        self.activation_time_s = activation_time_s
        self.latch = latch

    def is_triggered(self, state):
        cfg = self._config
        return command_triggered(
            state, self._keys,
            cfg.press_type, cfg.activation_switch, cfg.activation_switch_key
        )

    def execute(self, state):
        publish_command(command_name=self.name)


class Disarm(Command):
    """
    Sends command to disarm the motors.
    """
    name = "DISARM"

    def __init__(self, config, hook_fn, latch, activation_time_s=1.0):
        self._keys = tuple(config.key_list)
        self._config = config
        self.activation_time_s = activation_time_s
        self.latch = latch

    def is_triggered(self, state):
        cfg = self._config
        return command_triggered(
            state, self._keys,
            cfg.press_type, cfg.activation_switch, cfg.activation_switch_key
        )

    def execute(self, state):
        publish_command(command_name=self.name)


class ControlToggle(Command):
    """
    Toggle command.
    Toggles the control state of "command_gate" node between AUTO / MANUAL.
    """
    name = "CONTROL_TOGGLE"

    def __init__(self, config, hook_fn, latch, activation_time_s=1.0):
        self._keys = tuple(config.key_list)
        self._config = config
        self.activation_time_s = activation_time_s
        self.latch = latch

    def is_triggered(self, state):
        cfg = self._config
        return command_triggered(
            state, self._keys,
            cfg.press_type, cfg.activation_switch, cfg.activation_switch_key
        )

    def execute(self, state):
        publish_command(command_name=self.name)

class KeyboardToggle(Command):
    """
    Toggle command for enabling keyboard commands.
    """
    name = "KEYBOARD_TOGGLE"

    def __init__(self, config, hook_fn, latch, activation_time_s=1.0):
        self._keys = tuple(config.key_list)
        self._config = config
        self.activation_time_s = activation_time_s
        self.latch = latch

    def is_triggered(self, state):
        cfg = self._config
        return command_triggered(
            state, self._keys,
            cfg.press_type, cfg.activation_switch, cfg.activation_switch_key
        )

    def execute(self, state):
        publish_command(command_name=self.name)


class Land(Command):
    """
    Switches copter mode to land.
    """
    name = "LAND"

    def __init__(self, config, hook_fn, latch, activation_time_s=0.5):
        self._keys = tuple(config.key_list)
        self._config = config
        self.activation_time_s = activation_time_s
        self.latch = latch

    def is_triggered(self, state):
        cfg = self._config
        return command_triggered(
            state, self._keys,
            cfg.press_type, cfg.activation_switch, cfg.activation_switch_key
        )

    def execute(self, state):
        publish_command(self.name)


class RTL(Command):
    """
    Switches copter mode to RTL (Return-To-Launch).
    """
    name = "RTL"

    def __init__(self, config, hook_fn, latch, activation_time_s=1.0):
        self._keys = tuple(config.key_list)
        self._config = config
        self.activation_time_s = activation_time_s
        self.latch = latch 

    def is_triggered(self, state):
        cfg = self._config
        return command_triggered(
            state, self._keys,
            cfg.press_type, cfg.activation_switch, cfg.activation_switch_key
        )

    def execute(self, state):
        publish_command(self.name)


class Takeoff(Command):
    """
    Sends take-off task to copter
    """
    name = "TAKEOFF"

    def __init__(self, config, hook_fn, latch, activation_time_s=1.0):
        self._keys = tuple(config.key_list)
        self._config = config
        self.activation_time_s = activation_time_s
        self.latch = latch 

    def is_triggered(self, state):
        cfg = self._config
        return command_triggered(
            state, self._keys,
            cfg.press_type, cfg.activation_switch, cfg.activation_switch_key
        )

    def execute(self, state):
        publish_command(self.name)

class Guided(Command):
    """
    Sends take-off task to copter
    """
    name = "GUIDED"

    def __init__(self, config, hook_fn, latch, activation_time_s=1.0):
        self._keys = tuple(config.key_list)
        self._config = config
        self.activation_time_s = activation_time_s
        self.latch = latch 

    def is_triggered(self, state):
        cfg = self._config
        return command_triggered(
            state, self._keys,
            cfg.press_type, cfg.activation_switch, cfg.activation_switch_key
        )

    def execute(self, state):
        publish_command(self.name)

class SpeedUp(Command):
    """
    Sends command to switch to higher velocity level for action commands.
    """
    name = "SPEED_UP"

    def __init__(self, config, hook_fn, latch, activation_time_s=0.1):
        self._keys = tuple(config.key_list)
        self._config = config
        self.activation_time_s = activation_time_s
        self._hook_fn = hook_fn
        self.latch = latch

    def is_triggered(self, state):
        cfg = self._config
        return command_triggered(
            state, self._keys,
            cfg.press_type, cfg.activation_switch, cfg.activation_switch_key
        )

    def execute(self, state):
        hv, vv, _ = self._hook_fn()
        publish_command(self.name, float_1=hv, float_2=vv)


class SpeedDown(Command):
    """
    Sends command to switch to lower velocity level for action commands.
    """
    name = "SPEED_DOWN"

    def __init__(self, config, hook_fn, latch, activation_time_s=0.1):
        self._keys = tuple(config.key_list)
        self._config = config
        self.activation_time_s = activation_time_s
        self._hook_fn = hook_fn
        self.latch = latch

    def is_triggered(self, state):
        cfg = self._config
        return command_triggered(
            state, self._keys,
            cfg.press_type, cfg.activation_switch, cfg.activation_switch_key
        )

    def execute(self, state):
        hv, vv, _ = self._hook_fn()
        publish_command(self.name, float_1=hv, float_2=vv)

class PressSafetySwitch(Command):
    """
    Sends command to press hardware safety switch from software.
    """
    name = "PRESS_SAFETY_SWITCH"

    def __init__(self, config, hook_fn, latch, activation_time_s=0.1):
        self._keys = tuple(config.key_list)
        self._config = config
        self.activation_time_s = activation_time_s
        self.latch = latch

    def is_triggered(self, state):
        cfg = self._config
        return command_triggered(
            state, self._keys,
            cfg.press_type, cfg.activation_switch, cfg.activation_switch_key
        )

    def execute(self, state):
        publish_command(self.name)


class VelocityYaw(Command):
    """
    Sends desired Vx, Vy, Vz, Yaw Rate values to FCU.
    Mavros direction assumption is FLU (x+: forward, y+: left, z+:up), positive yaw_rate: ccw
    """
    name = "VEL_YAW"

    def __init__(
        self,
        config,
        hook_fn,
        latch,
        activation_time_s=0.0,
        hover=False,
        yaw_rate: float = 1.0,  # rad/s
    ):
        self.activation_time_s = activation_time_s
        self._keys = tuple(config.key_list)
        self._config = config
        self._hook_fn = hook_fn
        self.latch = latch
        self._hover = hover

        if hover:
            self.priority = -9999

        self._yaw_rate = yaw_rate

    def is_triggered(self, state):
        cfg = self._config
        return command_triggered(
            state, self._keys,
            cfg.press_type, cfg.activation_switch, cfg.activation_switch_key
        )

    def execute(self, state):
        if self._hover:
            publish_action("HOVER", 0.0, 0.0, 0.0, 0.0)
            return

        hv, vv, _ = self._hook_fn()  # hv: desired horizontal speed, vv: desired vertical speed

        # 1) Horizontal velocity
        # calculate unit direction vectors
        x_dir = (1.0 if "KEY_W" in state else 0.0) + (-1.0 if "KEY_S" in state else 0.0)
        y_dir = (1.0 if "KEY_A" in state else 0.0) + (-1.0 if "KEY_D" in state else 0.0)

        # normalize so combined velocity vector in x-y axis has length 1
        mag = math.hypot(x_dir, y_dir)
        if mag > 0.0:
            vx = (x_dir / mag) * hv
            vy = (y_dir / mag) * hv
        else:
            vx = 0.0
            vy = 0.0

        # 2) Vertical velocity
        z_dir= (1.0 if "KEY_UP" in state else 0.0) + (-1.0 if "KEY_DOWN" in state else 0.0)
        vz = vv * z_dir

        # 3) Yaw rate
        yaw_dir= (1.0 if "KEY_LEFT" in state else 0.0) + (-1.0 if "KEY_RIGHT" in state else 0.0)
        yaw_rate = yaw_dir * self._yaw_rate

        publish_action(self.name, vx, vy, vz, yaw_rate)

class RecordVideoToggle(Command):
    """
    Sends toggle message to camera_output node via dedicated topic.
    """
    name = "RECORD_VIDEO_TOGGLE"

    def __init__(self, config, hook_fn, latch, activation_time_s=0.1):
        self._keys = tuple(config.key_list)
        self._config = config
        self.activation_time_s = activation_time_s
        self.latch = latch
        self._hook_fn = hook_fn  

    def is_triggered(self, state):
        cfg = self._config
        return command_triggered(
            state, self._keys,
            cfg.press_type, cfg.activation_switch, cfg.activation_switch_key
        )

    def execute(self, state):
        if self._hook_fn is not None:
            self._hook_fn()

class StreamToggle(Command):
    """
    Sends toggle message to camera_output node via dedicated topic.
    """
    name = "STREAM_TOGGLE"

    def __init__(self, config, hook_fn, latch, activation_time_s=0.1):
        self._keys = tuple(config.key_list)
        self._config = config
        self.activation_time_s = activation_time_s
        self.latch = latch
        self._hook_fn = hook_fn   

    def is_triggered(self, state):
        cfg = self._config
        return command_triggered(
            state, self._keys,
            cfg.press_type, cfg.activation_switch, cfg.activation_switch_key
        )

    def execute(self, state):
        if self._hook_fn is not None:
            self._hook_fn()
        