from typing import Dict, Sequence, Literal



def assign_priorities_from_list_order(commands):
    """
    Smaller index in the list => higher priority number.
    """
    for i, cmd in enumerate(commands):
        cmd.priority = len(commands) - 1 - i


def all_keys_pressed(state: Dict[str, bool], required: Sequence[str]) -> bool:
    """
    Returns true if all keys in "required" exist in "state"
    """
    return all(k in state for k in required)

def any_keys_pressed(state: Dict[str, bool], required: Sequence[str]) -> bool:
    """
    Returns true if at least one key in "required" exists in "state"
    """
    return any(k in state for k in required)

def command_triggered(
        state: Dict[str, bool],
        required: Sequence[str],
        press_type: Literal["ANY", "ALL"],
        activation_switch: bool,
        activation_switch_key: str
    ):
    """
    Returns true if all keys required to trigger/execute a command are pressed
    """

    pressed = False
    if press_type == "ANY":
        pressed = any_keys_pressed(state, required)
    elif press_type == "ALL":
        pressed = all_keys_pressed(state, required)
    else:
        raise Exception("Unknown key-combo press type.")
    
    # early exit
    if pressed == False:
        return False
    else:
        # All keys required are pressed. Now we check whether activation key is required/pressed as well.
        pass
    
    if activation_switch:
        if activation_switch_key in state:
            return True
        else:
            return False
    
    # No activation key was required.
    return True

