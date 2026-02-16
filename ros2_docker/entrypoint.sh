#!/bin/bash
set +e

source /opt/ros/humble/setup.bash

if [ -f /home/batuhan/ros2_ws/install/setup.bash ]; then
    source /home/batuhan/ros2_ws/install/setup.bash
fi

# Set ROS2 related environment variables
ROS_DOMAIN_BASE=${ROS_DOMAIN_ID}
DRONE_ID=$(yq '.drone.id' /config/system.yaml)
ROS_DOMAIN_ID=$((ROS_DOMAIN_BASE + DRONE_ID))

export ROS_DOMAIN_ID
echo "ROS_DOMAIN_ID updated to $ROS_DOMAIN_ID"

# Set hailotools environment variables
export HAILO_TAPPAS=/opt/hailo/tappas
export GST_PLUGIN_PATH=/usr/lib/aarch64-linux-gnu/gstreamer-1.0:/opt/hailo/tappas/lib/aarch64-linux-gnu/gstreamer-1.0${GST_PLUGIN_PATH:+:$GST_PLUGIN_PATH}
export LD_LIBRARY_PATH=/opt/hailo/tappas/lib/aarch64-linux-gnu:/opt/hailo/tappas/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}