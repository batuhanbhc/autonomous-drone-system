# mavros_gcs/teleop_utils/teleop_io.py
from typing import Callable, Optional

_logger = None
_publish_command_fn: Optional[Callable[[str, float, float], None]] = None
_publish_action_fn: Optional[Callable[[float, float, float, float], None]] = None
_now_fn = None
_source_name: str = "teleop"

def init_teleop_io(logger, publish_command_fn, publish_action_fn, now_fn, source_name: str):
    global _logger, _publish_command_fn, _publish_action_fn, _now_fn, _source_name
    _logger = logger
    _publish_command_fn = publish_command_fn
    _publish_action_fn = publish_action_fn
    _now_fn = now_fn
    _source_name = source_name

def publish_command(command_name: str, float_1: float = 0.0, float_2: float = 0.0, bool_1: bool = True) -> None:
    if _publish_command_fn is None:
        raise RuntimeError("teleop_io not initialized: publish_command_fn is None")
    _publish_command_fn(command_name, float_1, float_2, bool_1)

def publish_action(vx: float, vy: float, vz: float, yaw_rate: float) -> None:
    if _publish_action_fn is None:
        raise RuntimeError("teleop_io not initialized: publish_action_fn is None")
    _publish_action_fn(vx, vy, vz, yaw_rate)

def log_info(msg: str) -> None:
    if _logger is not None:
        _logger.info(msg)
    else:
        print(msg)

def log_warn(msg: str) -> None:
    if _logger is not None:
        _logger.warn(msg)
    else:
        print(msg)

def log_error(msg: str) -> None:
    if _logger is not None:
        _logger.error(msg)
    else:
        print(msg)