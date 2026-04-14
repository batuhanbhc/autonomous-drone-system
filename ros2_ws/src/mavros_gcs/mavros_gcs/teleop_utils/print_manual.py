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
    if not key_code:
        return ""
    name = key_code
    if name.startswith("KEY_"):
        name = name[4:]
    return _KEY_PRETTY.get(name, name)

def _format_key_combo(keys) -> str:
    return " + ".join(_pretty_key_name(k) for k in keys if k)

def _keys_for(name: str) -> str:
    cfg = TELEOP_CONFIG.get(name)
    if cfg is None:
        return "(not configured)"
    return _format_key_combo(getattr(cfg, "key_list", ()))

def teleop_manual_text() -> str:
    action_switch  = getattr(TELEOP_CONFIG.get("ACTION"),  "activation_switch_key", None)
    command_switch = getattr(TELEOP_CONFIG.get("RTL"),     "activation_switch_key", None)

    cs = f" (hold {_pretty_key_name(command_switch)})" if command_switch else ""
    as_ = f" (hold {_pretty_key_name(action_switch)})" if action_switch else ""

    lines = []
    lines.append("=== Teleop Keyboard Manual ===")

    # ── Flight modes ──────────────────────────────────────────────────────────
    lines.append(f"\nFlight modes{cs}")
    for name in ("GUIDED", "TAKEOFF", "LAND", "RTL"):
        lines.append(f"  {name:<22} {_keys_for(name)}")

    # ── Movement ──────────────────────────────────────────────────────────────
    lines.append(f"\nMovement{as_}")
    movement = [
        ("Forward",   "KEY_W"),
        ("Back",      "KEY_S"),
        ("Left",      "KEY_A"),
        ("Right",     "KEY_D"),
        ("Up",        "KEY_UP"),
        ("Down",      "KEY_DOWN"),
        ("Yaw left",  "KEY_LEFT"),
        ("Yaw right", "KEY_RIGHT"),
    ]
    for label, key in movement:
        lines.append(f"  {label:<22} {_pretty_key_name(key)}")

    # ── Speed adjustment ──────────────────────────────────────────────────────
    lines.append(f"\nSpeed adjustment{cs}")
    speed_cmds = [
        ("SPEED_UP_HORIZONTAL",   "Horizontal +"),
        ("SPEED_DOWN_HORIZONTAL", "Horizontal -"),
        ("SPEED_UP_VERTICAL",     "Vertical +"),
        ("SPEED_DOWN_VERTICAL",   "Vertical -"),
        ("SPEED_UP_YAW",          "Yaw rate +"),
        ("SPEED_DOWN_YAW",        "Yaw rate -"),
    ]
    for name, label in speed_cmds:
        lines.append(f"  {label:<22} {_keys_for(name)}")

    # ── Vehicle control ───────────────────────────────────────────────────────
    lines.append(f"\nVehicle control{cs}")
    ctrl_cmds = [
        ("ARM",                "Arm"),
        ("DISARM",             "Disarm"),
        ("PRESS_SAFETY_SWITCH","Safety switch"),
        ("CONTROL_TOGGLE",     "Auto/Manual toggle"),
        ("KEYBOARD_TOGGLE",    "Keyboard on/off"),
        ("ALT_SUPPORT_TOGGLE", "AltHold on/off"),
    ]
    for name, label in ctrl_cmds:
        lines.append(f"  {label:<22} {_keys_for(name)}")

    # ── Kill switch ───────────────────────────────────────────────────────────
    lines.append(f"\nKill switch{cs}  !! motors stop immediately !!")
    lines.append(f"  {'Open window':<22} {_keys_for('KILL_SWITCH')}")
    lines.append(f"  {'Confirm kill':<22} {_keys_for('KILL_CONFIRM')}")

    # ── Camera ────────────────────────────────────────────────────────────────
    lines.append(f"\nCamera{cs}")
    lines.append(f"  {'Record toggle':<22} {_keys_for('RECORD_VIDEO_TOGGLE')}")
    lines.append(f"  {'Stream toggle':<22} {_keys_for('STREAM_TOGGLE')}")

    # ── Quick-start ───────────────────────────────────────────────────────────
    lines.append("\nQuick-start sequence:")
    lines.append(f"  1) PRESS_SAFETY_SWITCH  ({_keys_for('PRESS_SAFETY_SWITCH')})")
    lines.append(f"  2) GUIDED               ({_keys_for('GUIDED')})")
    lines.append(f"  3) ARM                  ({_keys_for('ARM')})")
    lines.append(f"  4) TAKEOFF              ({_keys_for('TAKEOFF')})")

    return "\n".join(lines)