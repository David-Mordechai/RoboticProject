#!/bin/bash
# Install wifi_return.sh as a 60 s systemd timer on the car (run on the car with sudo). Remove: sudo bash this --remove
set -e
if [ "$1" = "--remove" ]; then systemctl disable --now wifi-return.timer; rm -f /etc/systemd/system/wifi-return.{service,timer} /usr/local/bin/wifi_return.sh; systemctl daemon-reload; echo removed; exit 0; fi
install -m 755 "$(dirname "$0")/wifi_return.sh" /usr/local/bin/wifi_return.sh
cat > /etc/systemd/system/wifi-return.service <<S
[Unit]
Description=Return car Wi-Fi from own AP to the phone hotspot
[Service]
Type=oneshot
ExecStart=/usr/local/bin/wifi_return.sh
S
cat > /etc/systemd/system/wifi-return.timer <<S
[Unit]
Description=Every 60 s: wifi_return.sh
[Timer]
OnBootSec=90
OnUnitActiveSec=60
[Install]
WantedBy=timers.target
S
systemctl daemon-reload; systemctl enable --now wifi-return.timer; systemctl list-timers wifi-return.timer --no-pager | head -3
