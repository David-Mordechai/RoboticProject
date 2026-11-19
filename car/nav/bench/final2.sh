#!/bin/bash
# FINAL 2: the generated nav2_params_a.yaml (V4 + 0.5 m/s + scale_velocities + approach margin 3 cm), no --set:
# C1 C2 C3 x 29/33/40 x 3, and park course H with nav_a_home.sh's tolerances.
K0=100 ./batch.sh FIN2 navigate_ackermann_b.xml "" > batch_FIN2.txt 2>&1
COURSES=H K0=140 ./batch.sh FIN2H navigate_ackermann_b.xml "--set controller_server.general_goal_checker.yaw_goal_tolerance=0.5 --set controller_server.general_goal_checker.xy_goal_tolerance=0.15" > batch_FIN2H.txt 2>&1
