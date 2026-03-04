from mavros_gcs.teleop_utils.definitions import TELEOP_CONFIG

_KEY_PRETTY = {
    "UP": "↑",
    "DOWN": "↓",
    "LEFT": "←",
    "RIGHT": "→",
    "RIGHTALT": "RightAlt",
    "LEFTALT": "LeftAlt",
    "SPACE": "Space",
    "ESC": "Esc",
    "EQUAL": "=",
    "MINUS": "-",
}

def _pretty_key_name(key_code: str) -> str:
    """
    Convert evdev key names like 'KEY_W' into a compact, user-facing label.
    """
    if not key_code:
        return ""
    name = key_code
    if name.startswith("KEY_"):
        name = name[4:]
    return _KEY_PRETTY.get(name, name)

def _format_key_combo(keys) -> str:
    return " + ".join(_pretty_key_name(k) for k in keys if k)

def teleop_manual_text() -> str:
    """
    Returns a human-readable manual for the current TELEOP_CONFIG.
    Activation keys are mentioned in section titles (not repeated per-line).
    """
    # Derive activation switches from config so the manual stays in sync.
    action_switch = getattr(TELEOP_CONFIG.get("ACTION", None), "activation_switch_key", None)
    command_switch = getattr(TELEOP_CONFIG.get("RTL", None), "activation_switch_key", None)

    lines = []
    lines.append("=== Teleop Keyboard Manual ===")
    lines.append("")

    # --- Mode changes (RTL/Takeoff/Guided/Land)
    mode_title = "Mode changes"
    if command_switch:
        mode_title += f" (hold {_pretty_key_name(command_switch)})"
    lines.append(mode_title)
    for name in ("RTL", "TAKEOFF", "GUIDED", "LAND"):
        cfg = TELEOP_CONFIG.get(name)
        if cfg is None:
            continue
        lines.append(f"  {name}: {_format_key_combo(getattr(cfg, 'key_list', ())) }")
    lines.append("")

    # --- Actions (movement)
    action_title = "Actions"
    if action_switch:
        action_title += f" (hold {_pretty_key_name(action_switch)})"
    lines.append(action_title)

    # Explicit human names for ACTION keys (more useful than 'KEY_W', etc.)
    action_map = [
        ("Forward", "KEY_W"),
        ("Back", "KEY_S"),
        ("Left", "KEY_A"),
        ("Right", "KEY_D"),
        ("Up", "KEY_UP"),
        ("Down", "KEY_DOWN"),
        ("Yaw left", "KEY_LEFT"),
        ("Yaw right", "KEY_RIGHT"),
    ]
    for label, key in action_map:
        lines.append(f"  {label}: {_pretty_key_name(key)}")
    lines.append("")

    # --- Other commands
    other_title = "Other commands"
    if command_switch:
        other_title += f" (hold {_pretty_key_name(command_switch)})"
    lines.append(other_title)

    # Order here is what shows up in the manual.
    other_cmds = (
        "ARM",
        "DISARM",
        "SPEED_UP",
        "SPEED_DOWN",
        "KILL_SWITCH",
        "KILL_CONFIRM",
        "KEYBOARD_TOGGLE",
        "CONTROL_TOGGLE",
        "PRESS_SAFETY_SWITCH",
    )
    for name in other_cmds:
        cfg = TELEOP_CONFIG.get(name)
        if cfg is None:
            continue
        keys = getattr(cfg, "key_list", ())
        lines.append(f"  {name}: {_format_key_combo(keys)}")

    lines.append("\nFor initial takeoff:")
    lines.append("1) Press safety switch")
    lines.append("2) Switch to Guided")
    lines.append("3) Arm the vehicle")
    lines.append("4) Send Takeoff")
    return "\n".join(lines)