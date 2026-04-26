#!/bin/bash
set -e
source /entrypoint.sh

export ROS_LOCALHOST_ONLY=1

DRONE_ID=$(yq '.drone_id' /home/batuhan/ros2_ws/src/mavros_config/config/control_params.yaml)
LOG_DIR="/home/batuhan/shared/logs"
mkdir -p "$LOG_DIR"

if [ -n "${GCS_HOST:-}" ]; then
  BRIDGE_HOST="$GCS_HOST"
else
  BRIDGE_HOST="$(ip route show default 0.0.0.0/0 | awk '/default/ {print $3; exit}')"
fi

if [ -z "$BRIDGE_HOST" ]; then
  echo "[link] Could not determine GCS host. Set GCS_HOST explicitly."
  exit 1
fi

echo "DRONE_ID: $DRONE_ID"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
echo "GCS_HOST: $BRIDGE_HOST"

echo "[link] Starting pi_link_bridge..."
exec ros2 run drone_link pi_link_bridge \
  --ros-args \
  -p drone_id:=$DRONE_ID \
  -p gcs_host:=$BRIDGE_HOST \
  >> "$LOG_DIR/pi_link_bridge.log" 2>&1
