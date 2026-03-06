#!/bin/bash
set -e
source /entrypoint.sh

DRONE_ID=$(yq '.drone.id' /config/system.yaml)
LOG_DIR="/home/batuhan/shared/logs"
mkdir -p "$LOG_DIR"

echo "[startup] Starting mavros..."
ros2 run mavros mavros_node \
  --ros-args \
  -r __ns:=/drone_0/mavros \
  -r /tf:=tf -r /tf_static:=tf_static \
  -r /uas1/mavlink_source:=mavlink_source \
  -r /uas1/mavlink_sink:=mavlink_sink \
  -p fcu_url:=/dev/ttyAMA0:115200 \
  --param system_id:=255 \
  >> "$LOG_DIR/mavros.log" 2>&1 &
MAVROS_PID=$!


# Wait until mavros arming service is available
echo "[startup] Waiting for mavros arming service..."
until ros2 service list 2>/dev/null | grep -q "/drone_${DRONE_ID}/mavros/cmd/arming"; do
  # Check mavros hasn't died while we wait
  if ! kill -0 $MAVROS_PID 2>/dev/null; then
    echo "[startup] mavros died during startup. Exiting."
    exit 1
  fi
  sleep 1
done
echo "[startup] mavros is ready."

echo "[startup] Starting control_gate..."
ros2 run mavros_gate control_gate \
  --ros-args \
  -p drone_id:=$DRONE_ID \
  >> "$LOG_DIR/control_gate.log" 2>&1 &

# Keep script alive — if any child dies, the container exits (triggering systemd restart)
wait -n
echo "[startup] A node exited. Shutting down container."
exit 1
