from typing import Dict
import math

from mavros_gcs.teleop_utils.command_helpers import command_triggered

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
    - Console gating:
        If teleop input is disabled, commands are normally blocked unless
        `allow_when_console_inactive` is True (e.g., an emergency kill switch).

    Notes:
    - `_required_ticks` is computed once by the manager from `activation_time_s` and the manager’s
      tick rate (hz). `get_required_ticks()` will raise if the manager has not initialized it.
    """
        
    name: str = "Command"
    priority: int = 0
    activation_time_s: float = 0.0
    required_ticks: int = 0
    latch: bool = True
    allow_when_console_inactive: bool = False

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
class KillConfirm(Command):
    name = "KILL_CONFIRM"
    allow_when_console_inactive = True
    priority = 9999          # ensure it's always selected when triggered

    def __init__(self, config, latch, activation_time_s, is_window_open_fn, clear_window_fn):
        self._keys = tuple(config.key_list)  # confirm combo keys
        self._config = config
        self.latch = latch
        self.activation_time_s = activation_time_s
        self._is_window_open = is_window_open_fn
        self._clear_window = clear_window_fn

    def is_triggered(self, state):
        if not self._is_window_open():
            return False
        cfg = self._config
        return command_triggered(
            state, self._keys,
            cfg.press_type, cfg.activation_switch, cfg.activation_switch_key
        )

    def execute(self, state):
        self._clear_window()
        print("!!! KILL SWITCH ACTIVATED !!!")


class KillSwitch(Command):
    name = "KILL_SWITCH"
    allow_when_console_inactive = True

    def __init__(self, config, hook_fn, latch, activation_time_s=2.0):
        self._keys = tuple(config.key_list)
        self._config = config
        self.activation_time_s = activation_time_s
        self._hook_fn = hook_fn
        self.latch = latch

    def on_hold_hook(self):
        print("WARNING: Kill pending... press confirm combo!")

    def is_triggered(self, state):
        cfg = self._config
        return command_triggered(
            state, self._keys,
            cfg.press_type, cfg.activation_switch, cfg.activation_switch_key
        )

    def execute(self, state):
        self._hook_fn(seconds=2.0)
        print("Kill window ARMED for 2 seconds")

class Arm(Command):
    name = "Arm"

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
        print("Sending ARM")


class Disarm(Command):
    name = "Disarm"

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
    
    def on_hold_hook(self):
        print("WARNING: Disarming vehicle")

    def execute(self, state):
        print("Sending DISARM")


class ConsoleToggle(Command):
    name = "CONSOLE_TOGGLE"
    allow_when_console_inactive = True

    def __init__(self, config, hook_fn, latch, activation_time_s=1.0):
        self._keys = tuple(config.key_list)
        self._config = config
        self._hook_fn = hook_fn
        self.activation_time_s = activation_time_s
        self.latch = latch

    def is_triggered(self, state):
        cfg = self._config
        return command_triggered(
            state, self._keys,
            cfg.press_type, cfg.activation_switch, cfg.activation_switch_key
        )

    def execute(self, state):
        active = self._hook_fn()
        print(f"Console active -> {active}")


class ModeLand(Command):
    name = "MODE_LAND"

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
        print("Sending LAND")


class ModeRTL(Command):
    name = "MODE_RTL"

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
        print("Sending RTL")


class ModeLoiter(Command):
    name = "MODE_LOITER"

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

    def execute(self, state: Dict[str, bool]) -> None:
        print("Sending LOITER")


class Takeoff(Command):
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
        print("Sending TAKEOFF")


class SpeedUp(Command):
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
        hv, vv, vel_idx = self._hook_fn()
        print(f"Speed index -> {vel_idx} (HV={hv} m/s, VV={vv} m/s)")


class SpeedDown(Command):
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
        hv, vv, vel_idx = self._hook_fn()
        print(f"Speed index -> {vel_idx} (HV={hv} m/s, VV={vv} m/s)")


class VelocityYaw(Command):
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
            print(f"{self.name} CMD: HOVER command")
            return

        hv, vv, _ = self._hook_fn()  # hv: desired horizontal speed, vv: desired vertical speed

        # 1) Horizontal velocity
        # calculate unit direction vectors
        x_dir = (1.0 if "KEY_W" in state else 0.0) + (-1.0 if "KEY_S" in state else 0.0)
        y_dir = (1.0 if "KEY_D" in state else 0.0) + (-1.0 if "KEY_A" in state else 0.0)

        # normalize so combined velocity vector in x-y axis has length 1
        mag = math.hypot(x_dir, y_dir)
        if mag > 0.0:
            vx = (x_dir / mag) * hv
            vy = (y_dir / mag) * hv
        else:
            vx = 0.0
            vy = 0.0

        # 2) Vertical velocity (NED: up is negative Vz)
        z_dir= (-1.0 if "KEY_UP" in state else 0.0) + (1.0 if "KEY_DOWN" in state else 0.0)
        vz = vv * z_dir  # already NED-signed

        # 3) Yaw rate
        yaw_dir= (1.0 if "KEY_LEFT" in state else 0.0) + (-1.0 if "KEY_RIGHT" in state else 0.0)
        yaw_rate = yaw_dir * self._yaw_rate

        print(f"VEL cmd: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}, yaw_rate={yaw_rate:.2f}")