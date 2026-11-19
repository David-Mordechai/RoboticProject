#!/bin/bash
# Lower a MentorPi car's idle CPU load (and battery drain). Run on the car HOST as pi, next to lowload_patch.py.
# Measured on car #2, 2026-11-02, sitting in the air: load ~3.4-5 on 4 cores, battery 8.25 -> 7.38 V in 42 min.
# 1. Autostart without Hiwonder's start_app (lidar_app, line_following, object_tracking, hand_gesture): we do not use
#    them; their phone-app features stop working until reverted.
# 2. Depth camera at 15 fps instead of 30 (the camera fps fix raised it; 15 still gives readers ~15 fps).
# 'ros2 launch bringup' reads install/bringup/share/...; on car #2 that is a copy, not a link to src (need_compile=False
# only makes the files it includes come from src), so both copies are patched.
# Backups + revert: ~/docker/tmp/lowload_backup/revert.sh
set -e
H=/home/pi/docker/tmp/lowload_backup
mkdir -p $H
cp lowload_patch.py /home/pi/docker/tmp/lowload_patch.py
cat > $H/revert.sh <<'EOF'
sudo -n docker exec -u ubuntu MentorPi python3 /home/ubuntu/shared/lowload_patch.py revert
sudo -n systemctl restart start_node.service
echo "reverted; stack restarting (~40 s)"
EOF
sudo -n docker exec -u ubuntu MentorPi python3 /home/ubuntu/shared/lowload_patch.py apply
sudo -n systemctl restart start_node.service
echo "low-load settings applied; stack restarting (~40 s). Revert: bash $H/revert.sh"
