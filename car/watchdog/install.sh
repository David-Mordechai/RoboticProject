#!/bin/bash
# Install the drive command watchdog on a MentorPi car (run on the car host as pi, from the folder with these files).
# It runs as its own systemd service on the host, which execs the node inside the MentorPi container, so the stock
# launch files stay untouched. Revert: bash uninstall.sh
set -e
cp cmd_watchdog.py /home/pi/docker/tmp/cmd_watchdog.py            # = /home/ubuntu/shared inside the container
sudo -n tee /etc/systemd/system/cmd-watchdog.service >/dev/null <<'EOF'
[Unit]
Description=MentorPi drive command watchdog (stops the motors if drive commands stop while moving)
After=docker.service start_node.service
Requires=docker.service

[Service]
User=pi
# docker exec leaves the inner process running when the client is killed: clean up before start and after stop.
ExecStartPre=-/bin/sh -c 'for p in $$(pgrep -f "[c]md_watchdog.py"); do sudo -n kill $$p; done'
ExecStopPost=-/bin/sh -c 'for p in $$(pgrep -f "[c]md_watchdog.py"); do sudo -n kill $$p; done'
ExecStart=/usr/bin/sudo -n /usr/bin/docker exec -u ubuntu MentorPi zsh -c 'source ~/.zshrc >/dev/null 2>&1; exec python3 /home/ubuntu/shared/cmd_watchdog.py'
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF
sudo -n systemctl daemon-reload
sudo -n systemctl enable --now cmd-watchdog.service
sleep 8
systemctl is-active cmd-watchdog.service
journalctl -u cmd-watchdog.service -n 5 --no-pager
