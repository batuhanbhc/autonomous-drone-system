# Put .service files under /etc/systemd/system/
# Then call sudo systemctl enable xxx.service

# Put .rules files under /etc/udev/rules.d/


# After all, call:
# sudo udevadm control --reload-rules
# sudo systemctl daemon-reload