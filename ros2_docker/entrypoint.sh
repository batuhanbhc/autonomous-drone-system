#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
if [ -f /home/batuhan/ros2_ws/install/setup.bash ]; then
    source /home/batuhan/ros2_ws/install/setup.bash
fi

ROS_DOMAIN_BASE=${ROS_DOMAIN_ID}
DRONE_ID=$(yq '.drone.id' /config/system.yaml)
ROS_DOMAIN_ID=$((ROS_DOMAIN_BASE + DRONE_ID))

export ROS_DOMAIN_ID
echo "ROS_DOMAIN_ID updated to $ROS_DOMAIN_ID"