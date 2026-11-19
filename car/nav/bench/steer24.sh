#!/bin/bash
# 2026-11-11: car #1 steering MEASURED 28 deg left / 24 deg right (steer_direct.py + arc_icp.py). Re-test the final config
# (as N5) at 24/26/28, 5 repeats, courses C1-C3 + park H.
cd ~/phase_a/bench
STEERS="24 26 28" REPS=5 K0=100 ./batch.sh S24 navigate_ackermann_b.xml "" > batch_S24.txt 2>&1
STEERS="24 26 28" REPS=5 COURSES=H K0=180 ./batch.sh S24H navigate_ackermann_b.xml "--set controller_server.general_goal_checker.yaw_goal_tolerance=0.5 --set controller_server.general_goal_checker.xy_goal_tolerance=0.15" > batch_S24H.txt 2>&1
