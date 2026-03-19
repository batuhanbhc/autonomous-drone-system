# drone_msgs/msg/FrameData.msg

# Raw JPEG payload + ROS header (stamp, frame_id)
sensor_msgs/CompressedImage image

# Odometry snapshot — packed at capture time by CameraCapture
# Dummy values (all zero) when no odom has arrived yet
float64 pos_x
float64 pos_y
float64 pos_z
float64 quat_x
float64 quat_y
float64 quat_z
float64 quat_w
float64 vel_x
float64 vel_y
float64 vel_z
bool    odom_valid     # false → staleness timer fired or no data yet

# GPS snapshot
int32   lat            # degrees × 1e7
int32   lon            # degrees × 1e7
bool    gps_valid      # false → staleness timer fired or no data yet