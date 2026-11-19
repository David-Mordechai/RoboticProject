#!/bin/bash
# Full validation (C1 C2 C3 x steer 29/33/40 x 3) of the best C3 fix V4, then V6 = V4 + smoother scale_velocities
# (keeps v and w in proportion: the driver steers atan(0.145 w / v)), then V7 = V6 + top speed 0.5 m/s.
V4="--set controller_server.FollowPath.min_lookahead_dist=0.25 --set controller_server.FollowPath.lookahead_time=1.0 --set controller_server.FollowPath.use_collision_detection=false"
V6="$V4 --set velocity_smoother.scale_velocities=true"
V7="$V6 --set controller_server.FollowPath.desired_linear_vel=0.5 --set velocity_smoother.max_velocity=[0.5,0.0,1.5]"
# F4 done (27/27, 2026-11-09 night): K0=200 ./batch.sh F4 navigate_ackermann_b.xml "$V4" > batch_F4.txt 2>&1
K0=10 ./batch.sh F6 navigate_ackermann_b.xml "$V6" > batch_F6.txt 2>&1
K0=40 ./batch.sh F7 navigate_ackermann_b.xml "$V7" > batch_F7.txt 2>&1
