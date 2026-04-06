# Published by control_gate to altitude_controller/input
# bool=true  → controller should run / keep running (send output)
# bool=false → controller should stop outputs and reset internal PID state

bool   active
float32 agl_m       # fixed target AGL set at moment of activation (m)
float32 vz_mps      # estimated global frame vertical velocity