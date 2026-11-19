#!/bin/bash
# Phase A trial C1: undock 0.35 m straight by odometry (nav/undock.py), then Nav2 goal = office door
# (2.95, -2.3, -90 deg; map frame of goto6, plan_check_a.py). Hard limit 120 s. Recorded with rec_run.sh.
#   ./nav_a_c1.sh NAME [noundock]   (Nav2 running and AMCL set + verified: set_pose.py; noundock = car already out)
N=${1:-a_c1}
if [ "$2" = "noundock" ]; then
  ~/rec_run.sh $N "timeout 120 python3 nav/nav_trip.py 2.95 -2.3 -90"
else
  ~/rec_run.sh $N "timeout 120 bash -c 'python3 nav/undock.py 0.35 && python3 nav/nav_trip.py 2.95 -2.3 -90'"
fi
