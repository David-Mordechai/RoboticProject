#!/bin/bash
# Apply the camera frame-rate fix (car #1, 2026-11-02: readers get ~25 fps instead of 5-6) to a MentorPi car.
# Run on the car HOST as pi, from a folder holding fastdds_shm.xml, patch_ascamera.py and camfps_revert.sh.
# 1. Fast DDS shared-memory transport with an 8 MB segment (the default 512 KB is too small for 0.9 MB frames, so they
#    went over fragmented UDP and were dropped).  2. joystick_control: 10 ms wait in its busy loop (100% of a core).
# 3. ascamera: fps 30 and no colour point cloud.  Backups + revert in ~/docker/tmp/camfps_backup.
set -e
H=/home/pi/docker/tmp; C=/home/ubuntu/shared
mkdir -p $H/camfps_backup
cp fastdds_shm.xml patch_ascamera.py $H/
cp camfps_revert.sh $H/camfps_backup/revert.sh
sudo -n docker exec -u ubuntu MentorPi bash -c "
  set -e
  B=$C/camfps_backup
  [ -f \$B/robotrc ] || cp -a ~/ros2_ws/.robotrc \$B/robotrc
  [ -f \$B/joystick_control.py ] || cp -a ~/ros2_ws/src/peripherals/peripherals/joystick_control.py \$B/joystick_control.py
  [ -f \$B/ascamera.launch.py ] || cp -a ~/ros2_ws/src/peripherals/launch/include/ascamera.launch.py \$B/ascamera.launch.py
  cp $C/fastdds_shm.xml ~/ros2_ws/fastdds_shm.xml
  grep -q FASTRTPS_DEFAULT_PROFILES_FILE ~/ros2_ws/.robotrc || \
    echo 'export FASTRTPS_DEFAULT_PROFILES_FILE=/home/ubuntu/ros2_ws/fastdds_shm.xml  # camera fps fix 2026-11-02' >> ~/ros2_ws/.robotrc
  python3 - <<'PY'
p = '/home/ubuntu/ros2_ws/src/peripherals/peripherals/joystick_control.py'
s = open(p).read()
old = '            while True:\n                for event in pg.event.get():'
new = ('            while True:\n                pg.time.wait(10)  # was a busy loop at 100% CPU (camera fps fix 2026-11-02)\n'
       '                for event in pg.event.get():')
if 'pg.time.wait(10)' not in s:
    assert s.count(old) == 1, 'joystick loop not found'
    open(p, 'w').write(s.replace(old, new))
print('joystick patched')
PY
  grep -q 'camera fps fix' ~/ros2_ws/src/peripherals/launch/include/ascamera.launch.py || python3 $C/patch_ascamera.py 30
"
sudo -n systemctl restart start_node.service
echo "camera fps fix applied; stack restarting (~40 s). Revert: bash ~/docker/tmp/camfps_backup/revert.sh"
