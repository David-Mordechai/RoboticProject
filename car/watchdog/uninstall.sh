#!/bin/bash
# Remove the drive command watchdog (back to stock behaviour).
sudo -n systemctl disable --now cmd-watchdog.service
sudo -n rm -f /etc/systemd/system/cmd-watchdog.service
sudo -n systemctl daemon-reload
for p in $(pgrep -f '[c]md_watchdog.py'); do sudo -n kill $p; done
echo "watchdog removed"
