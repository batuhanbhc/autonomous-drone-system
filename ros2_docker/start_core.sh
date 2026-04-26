#!/bin/bash
set -e
source /entrypoint.sh

export ROS_LOCALHOST_ONLY=1

DRONE_ID=$(yq '.drone_id' /home/batuhan/ros2_ws/src/mavros_config/config/control_params.yaml)
LOG_DIR="/home/batuhan/shared/logs"
mkdir -p "$LOG_DIR"

echo "DRONE_ID: $DRONE_ID"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"

echo "[core] Checking for FCU on UART /dev/ttyAMA0..."
if [ ! -c /dev/ttyAMA0 ]; then
  echo "[core] /dev/ttyAMA0 not found. Is Pixhawk connected? Exiting."
  exit 1
fi
echo "[core] /dev/ttyAMA0 found."

echo "[core] Starting mavros..."
ros2 run mavros mavros_node \
  --ros-args \
  -r __ns:=/drone_0/mavros \
  -r /tf:=tf -r /tf_static:=tf_static \
  -r /uas1/mavlink_source:=mavlink_source \
  -r /uas1/mavlink_sink:=mavlink_sink \
  -p fcu_url:=/dev/ttyAMA0:230400 \
  --param system_id:=255 \
  >> "$LOG_DIR/mavros.log" 2>&1 &
MAVROS_PID=$!
echo "[core] mavros started with PID $MAVROS_PID."

echo "[core] Waiting for FCU heartbeat..."
until ros2 topic echo --once /drone_${DRONE_ID}/mavros/state 2>/dev/null | grep -q "connected: true"; do
  sleep 2
done
echo "[core] FCU heartbeat received."

echo "[core] Starting mavros_gate..."
ros2 launch mavros_gate mavros_gate_compositor.launch.py drone_id:=$DRONE_ID \
  >> "$LOG_DIR/mavros_gate.log" 2>&1 &
MAVROS_GATE_PID=$!
echo "[core] mavros_gate started with PID $MAVROS_GATE_PID."

echo "[core] Starting drone_pipeline..."
ros2 launch drone_pipeline drone_pipeline.launch.py \
  >> "$LOG_DIR/drone_pipeline.log" 2>&1 &
DRONE_PIPELINE_PID=$!
echo "[core] drone_pipeline started with PID $DRONE_PIPELINE_PID."

wait -n $MAVROS_PID $MAVROS_GATE_PID $DRONE_PIPELINE_PID
echo "[core] A critical node exited. Shutting down container."
exit 1
