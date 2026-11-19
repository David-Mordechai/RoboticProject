#!/bin/bash
# C3 near-desk "collision ahead" fixes (2026-11-09 night), each: C3 x steer 29/33 x 3
export COURSES=C3 STEERS="29 33"
K0=100 ./batch.sh V1 navigate_ackermann_b.xml "--set controller_server.FollowPath.min_lookahead_dist=0.25 --set controller_server.FollowPath.lookahead_time=1.0" > batch_V1.txt 2>&1
K0=120 ./batch.sh V2 navigate_ackermann_b.xml "--set global_costmap.inflation_layer.cost_scaling_factor=2.5" > batch_V2.txt 2>&1
K0=140 ./batch.sh V3 navigate_ackermann_b.xml "--set controller_server.FollowPath.min_lookahead_dist=0.25 --set controller_server.FollowPath.lookahead_time=1.0 --set global_costmap.inflation_layer.cost_scaling_factor=2.5" > batch_V3.txt 2>&1
