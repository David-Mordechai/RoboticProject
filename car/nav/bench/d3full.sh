#!/bin/bash
# D3 (planner minimum_turning_radius 0.40 -> 0.50) full check: C1 C2 C3 x 29/33/40 x 3 + park H (with home tolerances).
R="--set planner_server.GridBased.minimum_turning_radius=0.5"
K0=100 ./batch.sh D3F navigate_ackermann_b.xml "$R" > batch_D3F.txt 2>&1
COURSES=H K0=140 ./batch.sh D3H navigate_ackermann_b.xml "$R --set controller_server.general_goal_checker.yaw_goal_tolerance=0.5 --set controller_server.general_goal_checker.xy_goal_tolerance=0.15" > batch_D3H.txt 2>&1
