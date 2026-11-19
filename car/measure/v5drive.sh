#!/bin/bash
# Usage: v5drive.sh <name> [seconds] [model] [turn model]. Runs explore (v7g 2026-11-04) with the turnaround skill, records camera frames
# (2 per second, file name = seconds since start, same clock as the policy log's t) into ~/docker/tmp/policy/<name>_cam/.
N=$1; SEC=${2:-60}; M=${3:-explore_v7g_3500.onnx}; TM=${4:-turnaround_v2c_1149.onnx}
D=~/docker/tmp/policy/${N}_cam; mkdir -p $D
sudo -n docker exec -d -u ubuntu MentorPi zsh -c "source ~/.zshrc >/dev/null 2>&1; cd /home/ubuntu/shared/policy && timeout $((SEC+15)) python3 explore_policy.py $M --turn-model $TM --seconds $SEC --log $N.npz > $N.log 2>&1"
T0=$(date +%s.%N)
END=$(echo "$T0 + $SEC + 3" | bc)
while (( $(echo "$(date +%s.%N) < $END" | bc) )); do
  t=$(echo "$(date +%s.%N) - $T0" | bc); curl -s -m 1 -o $D/$(printf '%06.1f' $t).jpg "http://localhost:8080/snapshot?topic=/ascamera/camera_publisher/rgb0/image"; sleep 0.4
done
echo "frames: $(ls $D | wc -l)"
