#!/bin/bash
# Record everything around one run, for analysis later. Usage: rec_run.sh NAME 'command run in the container (cwd shared)'
# Saves to ~/docker/tmp/runs/NAME/: bag (ROS topics below), cam/ (2 frames/s, name = s since start), run.log (command
# output), map_before.npz / map_after.npz (SLAM map + pose), t0 (start time, epoch s).
N=$1; CMD=$2; R=~/docker/tmp/runs/$N; CR=/home/ubuntu/shared/runs/$N
mkdir -p $R/cam; date +%s.%N > $R/t0
X() { sudo -n docker exec -u ubuntu MentorPi zsh -c "source ~/.zshrc >/dev/null 2>&1; cd /home/ubuntu/shared && $1"; }
X "python3 map_dump.py >/dev/null 2>&1; mv map_now.npz $CR/map_before.npz"
# Nav2 (Phase A): /plan /local_plan /amcl_pose /cmd_vel_nav /cmd_vel_smoothed /behavior_tree_log /collision_monitor_state
TOPICS="/scan_raw /odom /odom_raw /odom_rf2o /imu /tf /tf_static /controller/cmd_vel /cmd_vel /map /depth_sectors /depth_scan /drive_to/path /ros_robot_controller/battery /pose /rosout /plan /local_plan /amcl_pose /cmd_vel_nav /cmd_vel_smoothed /behavior_tree_log /collision_monitor_state"
sudo -n docker exec -d -u ubuntu MentorPi zsh -c "source ~/.zshrc >/dev/null 2>&1; ros2 bag record -o $CR/bag $TOPICS > $CR/bag.log 2>&1"
sudo -n docker exec -d -u ubuntu MentorPi zsh -c "source ~/.zshrc >/dev/null 2>&1; python3 /home/ubuntu/shared/cam_saver.py $CR/cam $(cat $R/t0) $CR/done > $CR/cam.log 2>&1"
sleep 2
X "$CMD" > $R/run.log 2>&1
touch $R/done; sleep 1
X "python3 map_dump.py >/dev/null 2>&1; mv map_now.npz $CR/map_after.npz"
sudo -n docker exec MentorPi pkill -INT -f '[r]os2 bag record'; sleep 3
echo "run $N: $(ls $R/cam | wc -l) frames, bag $(du -sh $R/bag 2>/dev/null | cut -f1), log $(wc -l < $R/run.log) lines"
