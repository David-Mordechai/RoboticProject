#!/bin/bash
# Stop a Phase A trial NOW: kill the recorder wrapper first (else it starts the drive command after we kill the trial
# script: trial a_c1_2, 2026-11-09), then the drive scripts, then zero the motors. Nav2 itself keeps running (no goal).
for p in $(pgrep -f '[n]av_a_c1.sh|[r]ec_run.sh|[n]av_trip.py|[u]ndock.py|[g]o_dist.py'); do sudo -n kill -9 $p; done
sudo -n docker exec MentorPi bash -c 'source /opt/ros/humble/setup.bash; python3 -c "
import rclpy,time; from geometry_msgs.msg import Twist; rclpy.init(); n=rclpy.create_node(\"z\"); p=n.create_publisher(Twist,\"/controller/cmd_vel\",1)
for _ in range(10): p.publish(Twist()); time.sleep(0.05)
"'
sudo -n docker exec MentorPi pkill -INT -f '[r]os2 bag record'
echo stopped
