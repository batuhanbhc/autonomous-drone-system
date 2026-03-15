# DroneState.msg
# Node self-state summary (subset of internal fields)

# Control mode enum
uint8 CONTROL_MODE_AUTO=0
uint8 CONTROL_MODE_MANUAL=1

uint8 control_mode

float64 velocity_h
float64 velocity_v
float32 velocity_yaw

bool keyboard_on
bool safety_switch_on
bool system_killed

# Computed at publish time: now - last_action_t
builtin_interfaces/Duration time_since_action