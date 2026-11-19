#!/bin/bash
# On top of V1 (lookahead 0.25 / 1.0 s): RPP's own costmap collision check is the remaining false "collision ahead"
# near the charger desk legs. V4: off (Collision Monitor "approach" = the safety layer), V5: horizon 0.5 s.
export COURSES=C3 STEERS="29 33"
V1="--set controller_server.FollowPath.min_lookahead_dist=0.25 --set controller_server.FollowPath.lookahead_time=1.0"
K0=160 ./batch.sh V4 navigate_ackermann_b.xml "$V1 --set controller_server.FollowPath.use_collision_detection=false" > batch_V4.txt 2>&1
K0=180 ./batch.sh V5 navigate_ackermann_b.xml "$V1 --set controller_server.FollowPath.max_allowed_time_to_collision_up_to_carrot=0.5" > batch_V5.txt 2>&1
