#!/bin/bash
# The doorway on the way back at steering 29 deg (FIN2 C3 s29: 1 of 3 stuck, full lock 71 %). C3 x 29 deg x 6 each.
export COURSES=C3 STEERS=29 REPS=6
K0=100 ./batch.sh D0 navigate_ackermann_b.xml "" > batch_D0.txt 2>&1 &
K0=110 ./batch.sh D1 navigate_ackermann_b.xml "--set collision_monitor.FootprintApproach.points=[0.139,0.109,0.139,-0.109,-0.134,-0.109,-0.134,0.109]" > batch_D1.txt 2>&1
wait
K0=120 ./batch.sh D2 navigate_ackermann_b.xml "--set controller_server.FollowPath.regulated_linear_scaling_min_radius=1.5 --set controller_server.FollowPath.regulated_linear_scaling_min_speed=0.1" > batch_D2.txt 2>&1 &
K0=130 ./batch.sh D3 navigate_ackermann_b.xml "--set planner_server.GridBased.minimum_turning_radius=0.5" > batch_D3.txt 2>&1
wait
