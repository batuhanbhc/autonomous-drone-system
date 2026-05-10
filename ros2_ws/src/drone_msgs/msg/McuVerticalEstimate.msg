std_msgs/Header header

float32 z_world_m
float32 vz_world_mps
float32 agl_m

bool ekf_initialized
bool lidar_accepted
bool lidar_rejected
bool baro_rejected

# Latest raw lidar distance decoded from the sensor, in meters.
# NaN means no lidar frame has ever been received since boot.
float32 latest_lidar_m

# Age of the latest decoded lidar frame, in milliseconds.
# UINT32_MAX means no lidar frame has ever been received since boot.
uint32 lidar_age_ms

# Latest filtered absolute barometer pressure from the MCU, in Pascals.
float32 baro_pressure_pa
