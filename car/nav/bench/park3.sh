#!/bin/bash
# 2026-11-11: two-leg park, waypoint (2.0, 0.2) facing out. Symmetric 24/26/28 x 3, then car #1 asymmetric 28 L / 24 R x 6.
cd ~/phase_a/bench
G="--set controller_server.general_goal_checker.yaw_goal_tolerance=0.5 --set controller_server.general_goal_checker.xy_goal_tolerance=0.15"
STEERS="24 26 28" REPS=3 COURSES="H9 H10" K0=140 ./batch.sh P3 navigate_ackermann_b.xml "$G" > batch_P3.txt 2>&1
STEERS="28" REPS=6 COURSES="H9 H10" K0=170 ./batch.sh P3A navigate_ackermann_b.xml "$G --steer-max-right 24" > batch_P3A.txt 2>&1
