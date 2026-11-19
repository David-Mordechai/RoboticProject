#!/bin/bash
# Phase A: stop Nav2 + depth_scan started by nav_a_start.sh (kill by PID from the host: pkill inside zsh -c kills its own shell).
for p in $(pgrep -f '[n]av_a.launch.py|[n]av2_container|[d]epth_scan.py|[n]av_guard.py'); do sudo -n kill -INT $p; done; sleep 3
for p in $(pgrep -f '[n]av_a.launch.py|[n]av2_container|[d]epth_scan.py|[n]av_guard.py'); do sudo -n kill -9 $p; done
sudo -n docker exec MentorPi bash -c 'source /opt/ros/humble/setup.bash; python3 - <<PY
import rclpy; from geometry_msgs.msg import Twist; rclpy.init(); n=rclpy.create_node("z"); p=n.create_publisher(Twist,"/controller/cmd_vel",1)
import time
for _ in range(5): p.publish(Twist()); time.sleep(0.05)
PY' 2>/dev/null
echo stopped
