#!/bin/bash
# Overnight 2026-11-09: bigger sample of the FINAL generated config (5 repeats): C1 C2 C3 + park H x 29/33/40.
REPS=5 K0=100 ./batch.sh N5 navigate_ackermann_b.xml "" > batch_N5.txt 2>&1
REPS=5 COURSES=H K0=180 ./batch.sh N5H navigate_ackermann_b.xml "--set controller_server.general_goal_checker.yaw_goal_tolerance=0.5 --set controller_server.general_goal_checker.xy_goal_tolerance=0.15" > batch_N5H.txt 2>&1
