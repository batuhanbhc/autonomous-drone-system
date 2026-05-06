builtin_interfaces/Time stamp
bool    odom_valid
float64 drone_x      # world frame, meters (ENU)
float64 drone_y      # world frame, meters (ENU)
float64 drone_z      # world frame, meters (ENU)
float64 drone_yaw    # radians, [-pi, pi], 0=east, CCW positive
float64 principal_x  # camera principal point ground intersection, world frame (ENU)
float64 principal_y  # camera principal point ground intersection, world frame (ENU)
drone_msgs/TrackedObject[] tracks
