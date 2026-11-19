#!/bin/bash
# Phase A start of a floor session, car #1 AT THE CHARGER (2026-11-09 night). Restarts Nav2 (stock autostart stays up),
# finds the start pose by matching the LiDAR scan to the saved map around the charger (0, 0, -8 deg = how it stood
# today), refuses if the fit is poor (people / chairs in front: a_c1_2 fitted at 35 %), then sets and VERIFIES the
# AMCL pose (set_pose.py exits 1 if AMCL drops it, as on 2026-11-09). Exit 0 = ready to drive.
#   ./nav_a_ready.sh [MIN_FIT_PERCENT=80]
MIN=${1:-80}
X() { sudo -n docker exec -u ubuntu MentorPi bash -c "source /opt/ros/humble/setup.bash; cd /home/ubuntu/shared/nav; $1"; }
~/nav_a_stop.sh; mv ~/docker/tmp/nav/nav_a.log ~/docker/tmp/nav/nav_a_prev.log 2>/dev/null
~/nav_a_start.sh; sleep 4; bash ~/nav_a_wait.sh | grep -E 'active|ERROR|died'
[ $(grep -c 'Managed nodes are active' ~/docker/tmp/nav/nav_a.log) -ge 2 ] || { echo "NOT READY: Nav2 did not come up"; exit 1; }
B=$(X "python3 scan_map_match.py /home/ubuntu/shared/maps/goto6.yaml 0 0 -8" | tee /dev/stderr | grep '^BEST')
read _ BX BY BYAW BFIT <<< "$B"
[ -n "$BFIT" ] && [ "$BFIT" -ge "$MIN" ] || { echo "NOT READY: scan fits the map only $BFIT % (< $MIN): clear the view in front of the car or move it onto the charger"; exit 1; }
X "python3 set_pose.py $BX $BY $BYAW" || { echo "NOT READY: AMCL did not take the pose"; exit 1; }
echo "READY at ($BX, $BY, $BYAW deg), fit $BFIT %"
