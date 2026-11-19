#!/bin/bash
# Isolate V7's gain: V8 = V4 + top speed 0.5 m/s WITHOUT scale_velocities (C1 C2 C3 x 29/33/40 x 3)
V4="--set controller_server.FollowPath.min_lookahead_dist=0.25 --set controller_server.FollowPath.lookahead_time=1.0 --set controller_server.FollowPath.use_collision_detection=false"
V8="$V4 --set controller_server.FollowPath.desired_linear_vel=0.5 --set velocity_smoother.max_velocity=[0.5,0.0,1.5]"
K0=70 ./batch.sh F8 navigate_ackermann_b.xml "$V8" > batch_F8.txt 2>&1
