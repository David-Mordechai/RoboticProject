#!/bin/bash
# Gate A rounds (docs/open-issues.md sec. 0): C1 door (noundock, from 0.3 m in front of the charger), C2 hall end, C3 back
# to the charger. Needs Nav2 up + AMCL set (nav_a_ready.sh or set_pose.py). Usage: ./gate_a_loop.sh FIRST_ROUND LAST_ROUND
# STOPS at the first trial that is not clean: nav_trip RESULT not ok (pose-checked, 2026-11-16), any recovery, or any
# nav_guard block. 2026-11-16 round 2 hit a table leg while the old loop trusted Nav2's "reached".
# Logs battery (mV) and the 1-min load average before each trial, and the trial's peak load. Log: ~/gate_a_loop.log
# Stop it from ssh by PID (pkill -f gate_a_loop.sh would match the ssh shell's own command line):
#   for p in $(pgrep -f '[g]ate_a_loop.sh'); do kill $p; done; ~/stop_trial.sh
A=${1:-1}; B=${2:-5}; L=~/gate_a_loop.log; echo "start rounds $A-$B $(date)" >> $L
BAT() { timeout 8 sudo -n docker exec -u ubuntu MentorPi zsh -c "source ~/.zshrc >/dev/null 2>&1; ros2 topic echo --once /ros_robot_controller/battery" 2>/dev/null | awk '/data/{print $2}'; }
LOAD() { cut -d' ' -f1 /proc/loadavg; }
for R in $(seq $A $B); do
  for T in c1 c2 c3; do
    N=g${R}_$T; echo "$N battery $(BAT) mV, load $(LOAD)" >> $L
    ( peak=0; while sleep 2; do l=$(LOAD); awk -v a=$l -v b=$peak 'BEGIN{exit !(a>b)}' && peak=$l; echo $peak > /tmp/peak_$N; done ) & PK=$!
    case $T in c1) ~/nav_a_c1.sh $N noundock ;; c2) ~/nav_a_trial.sh $N 2.95 -7.0 -90 ;; c3) ~/nav_a_trial.sh $N 0.3 0 180 ;; esac > /dev/null 2>&1
    kill $PK 2>/dev/null
    RES=$(grep '^RESULT' ~/docker/tmp/runs/$N/run.log | tail -1)
    echo "$N: ${RES:-RESULT missing} | peak load $(cat /tmp/peak_$N 2>/dev/null)" >> $L
    read _ OK _ _ REC _ BLK <<< "$RES"
    if [ "$OK" != ok ] || [ "${REC:-1}" != 0 ] || [ "${BLK:-1}" != 0 ]; then echo "STOP after $N (not clean)" >> $L; ~/stop_trial.sh > /dev/null 2>&1; exit 1; fi
  done
done
echo "ALL DONE $(date)" >> $L
