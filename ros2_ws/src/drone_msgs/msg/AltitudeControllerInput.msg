# drone_msgs/msg/AltitudeControllerInput.msg
#
# Published by control_gate → subscribed by altitude_controller.
#
# active=true  : controller should run; first reception initialises PID state.
# active=false : controller must stop and reset all internal state.
#
bool    active

# Desired altitude above ground level [m].
# Set by:
#   • mode entry (enterGuidedTimeout / enterAltSupport) — snapshotted from
#     current agl at the moment the mode activates, unless a higher-priority
#     SetTargetHeight service call has already set a value.
#   • /drone_N/control_gate/set_target_height service call (on-air tuning).
float32 target_agl_m

# Live measurement from the MCU (via mcu_bridge → onVerticalEstimate).
# The PID node computes:  error = target_agl_m - current_agl_m
float32 current_agl_m

# Live vertical velocity from the MCU [m/s], positive = upward.
# Used as derivative-on-measurement:  D_term = -kd * current_vz_mps
float32 current_vz_mps


bool reset_integral     # When set to true, resets integral windup