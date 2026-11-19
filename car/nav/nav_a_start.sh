#!/bin/bash
# Phase A: start Nav2 (nav_a.launch.py) detached in the MentorPi container, on top of the stock autostart.
# No goal is sent: the car does not move. Log: ~/docker/tmp/nav/nav_a.log. Stop: car/nav/nav_a_stop.sh
# 2026-11-11: a session that ended without nav_a_stop.sh left an old depth_scan.py running; the next start added a
# second one (2 publishers on /depth_scan, ~17 % of a core each). Refuse to start while any old instance runs.
if pgrep -f '[n]av_a.launch.py|[n]av2_container|[d]epth_scan.py|[n]av_guard.py' > /dev/null; then
  echo "Nav2/depth_scan already running (pgrep below). Run ./nav_a_stop.sh first."; pgrep -af '[n]av_a.launch.py|[n]av2_container|[d]epth_scan.py|[n]av_guard.py'; exit 1
fi
sudo -n docker exec -d -u ubuntu MentorPi zsh -c "source ~/.zshrc >/dev/null 2>&1; cd /home/ubuntu/shared/nav && MAP=/home/ubuntu/shared/maps/goto6.yaml ros2 launch nav_a.launch.py > nav_a.log 2>&1"
echo started
