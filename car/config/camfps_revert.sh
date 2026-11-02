#!/bin/bash
# Revert the 2026-11-02 camera-fps changes on a MentorPi car. Run on the car HOST as pi:  bash ~/docker/tmp/camfps_backup/revert.sh
set -e
B=/home/ubuntu/shared/camfps_backup
sudo docker exec -u ubuntu MentorPi bash -c "
  cp -a $B/robotrc ~/ros2_ws/.robotrc
  cp -a $B/joystick_control.py ~/ros2_ws/src/peripherals/peripherals/joystick_control.py
  cp -a $B/ascamera.launch.py ~/ros2_ws/src/peripherals/launch/include/ascamera.launch.py
  rm -f ~/ros2_ws/fastdds_shm.xml"
sudo systemctl restart start_node.service
echo "reverted; stack restarting (allow ~40 s)"
