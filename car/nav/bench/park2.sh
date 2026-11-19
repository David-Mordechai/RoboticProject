#!/bin/bash
# 2026-11-11: two-leg park at car #1 steering 24/26/28 (measured): H7 hall -> open waypoint facing out, H8 waypoint -> pre-dock.
cd ~/phase_a/bench
G="--set controller_server.general_goal_checker.yaw_goal_tolerance=0.5 --set controller_server.general_goal_checker.xy_goal_tolerance=0.15"
STEERS="24 26 28" REPS=5 COURSES="H7 H8" K0=120 ./batch.sh P2 navigate_ackermann_b.xml "$G" > batch_P2.txt 2>&1
