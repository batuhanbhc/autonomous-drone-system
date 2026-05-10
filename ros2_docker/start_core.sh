#!/bin/bash
set -e
source /entrypoint.sh

export ROS_LOCALHOST_ONLY=1

CONFIG_PATH=/home/batuhan/ros2_ws/src/mavros_config/config/control_params.yaml
DRONE_ID=$(yq '.drone_id' "$CONFIG_PATH")
RECORDINGS_DIR=$(yq -r '.flight_params.logs_path' "$CONFIG_PATH")
LOG_DIR="/home/batuhan/shared/logs"
mkdir -p "$LOG_DIR"
mkdir -p "$RECORDINGS_DIR"
DIR_COUNT=$(find "$RECORDINGS_DIR" -mindepth 1 -maxdepth 1 -type d | wc -l)
printf -v SESSION_DIR "%s/%04d" "$RECORDINGS_DIR" "$((DIR_COUNT + 1))"
mkdir "$SESSION_DIR"

echo "DRONE_ID: $DRONE_ID"
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "ROS_LOCALHOST_ONLY: $ROS_LOCALHOST_ONLY"
echo "SESSION_DIR: $SESSION_DIR"

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
  session_dir:=$SESSION_DIR \
  >> "$LOG_DIR/mavros_gate.log" 2>&1 &
MAVROS_GATE_PID=$!
echo "[core] mavros_gate started with PID $MAVROS_GATE_PID."

echo "[core] Starting drone_pipeline..."
ros2 launch drone_pipeline drone_pipeline.launch.py \
  session_dir:=$SESSION_DIR \
  >> "$LOG_DIR/drone_pipeline.log" 2>&1 &
DRONE_PIPELINE_PID=$!
echo "[core] drone_pipeline started with PID $DRONE_PIPELINE_PID."

wait -n $MAVROS_PID $MAVROS_GATE_PID $DRONE_PIPELINE_PID
echo "[core] A critical node exited. Shutting down container."
exit 1
