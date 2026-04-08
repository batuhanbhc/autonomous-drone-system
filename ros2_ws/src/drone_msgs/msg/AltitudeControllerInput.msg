# AltitudeControllerInput.msg
#
# Field semantics
# ───────────────
#  active         – true  → controller should run and publish output
#                   false → controller must stop, reset integral, and ignore MCU data
#
#  target_agl_m   – desired altitude above ground [m]
#                   Meaningful only when active == true.
#                   Ignored (but may be pre-loaded) when active == false.
#
#  reset_integral – when true the PID integral accumulator is zeroed before the
#                   next compute cycle (used when a new target altitude is set
#                   mid-flight to prevent windup carry-over).
#                   The flag is edge-triggered: the controller clears it internally
#                   after acting on it; the sender does not need to follow up with
#                   a false message.

bool   active
float32 target_agl_m
bool   reset_integral