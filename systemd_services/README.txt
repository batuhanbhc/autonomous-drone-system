# Put .service files under /etc/systemd/system/
# Then call sudo systemctl enable xxx.service

# Put .rules files under /etc/udev/rules.d/

# Production split:
#   - drone-core.service  : onboard-local ROS graph only
#   - drone-link.service  : Pi <-> GCS bridge only
#
# Legacy fallback:
#   - drone.service       : old single-stack startup
#
# camera-settings.service works with either startup model.

# After all, call:
# sudo udevadm control --reload-rules
# sudo systemctl daemon-reload
