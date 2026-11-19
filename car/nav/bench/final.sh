#!/bin/bash
# FINAL check of the generated car/nav/nav2_params_a.yaml itself (no --set), tree B: C1 C2 C3 + H (home/park, with
# nav_a_home.sh's tolerances) x 29/33/40 x 3.
K0=100 ./batch.sh FIN navigate_ackermann_b.xml "" > batch_FIN.txt 2>&1
COURSES=H K0=140 ./batch.sh FINH navigate_ackermann_b.xml "--set controller_server.general_goal_checker.yaw_goal_tolerance=0.5 --set controller_server.general_goal_checker.xy_goal_tolerance=0.15" > batch_FINH.txt 2>&1
