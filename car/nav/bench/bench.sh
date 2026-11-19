#!/bin/bash
# bench.sh NAME DOMAIN BT SX SY SYAW GX GY GYAW [extra bench_run args...]  -> results/NAME.json (ASUS, ~/phase_a/bench)
N=$1; D=$2; BT=$3; shift 3; S="$1 $2 $3 $4 $5 $6"; shift 6
mkdir -p results
timeout 300 docker run --rm -e ROS_DOMAIN_ID=$D -v ~/phase_a/bench:/w -w /w robotlab-nav2:humble bash -c \
  "source /opt/ros/humble/setup.bash; python3 bench_run.py goto6_map_after.npz \${PARAMS:-nav2_params_a.yaml} $BT $S --out results/$N.json $* > results/$N.log 2>&1; chmod -R a+rwX bench_logs results"
tail -1 results/$N.log
