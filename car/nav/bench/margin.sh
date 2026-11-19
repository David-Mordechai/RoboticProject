#!/bin/bash
# Collision Monitor "approach" polygon = footprint + 3 cm (was + 0): FINH s40_r1 touched the charger-desk legs 5 x.
# Undock/dock run outside Nav2 (undock.py / dock.py publish /controller/cmd_vel directly), so the charger behind the
# docked car no longer constrains this margin. Also: pre-dock 0.6 m out (H6) vs 0.35 (H).
M="--set collision_monitor.FootprintApproach.points=[0.149,0.119,0.149,-0.119,-0.144,-0.119,-0.144,0.119]"
T="--set controller_server.general_goal_checker.yaw_goal_tolerance=0.5 --set controller_server.general_goal_checker.xy_goal_tolerance=0.15"
K0=100 ./batch.sh M3C navigate_ackermann_b.xml "$M" > batch_M3C.txt 2>&1
COURSES="H H6" K0=140 ./batch.sh M3H navigate_ackermann_b.xml "$M $T" > batch_M3H.txt 2>&1
