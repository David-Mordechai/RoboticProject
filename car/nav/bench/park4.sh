#!/bin/bash
# 2026-11-11: leg 1 (waypoint) with LOOSE tolerances (yaw 0.8 rad, xy 0.3) = only a pass-through point; leg 2 from skewed starts.
cd ~/phase_a/bench
while pgrep -f park3.sh > /dev/null; do sleep 10; done
L="--set controller_server.general_goal_checker.yaw_goal_tolerance=0.8 --set controller_server.general_goal_checker.xy_goal_tolerance=0.3"
G="--set controller_server.general_goal_checker.yaw_goal_tolerance=0.5 --set controller_server.general_goal_checker.xy_goal_tolerance=0.15"
STEERS="24 26 28" REPS=4 COURSES="H9" K0=100 ./batch.sh P4L navigate_ackermann_b.xml "$L" > batch_P4L.txt 2>&1
STEERS="28" REPS=6 COURSES="H9" K0=120 ./batch.sh P4LA navigate_ackermann_b.xml "$L --steer-max-right 24" > batch_P4LA.txt 2>&1
STEERS="24 28" REPS=3 COURSES="H11 H12" K0=140 ./batch.sh P4S navigate_ackermann_b.xml "$G" > batch_P4S.txt 2>&1
STEERS="28" REPS=3 COURSES="H11 H12" K0=160 ./batch.sh P4SA navigate_ackermann_b.xml "$G --steer-max-right 24" > batch_P4SA.txt 2>&1
