for p in $(pgrep -f '[j]oystick_control|[s]afe_drive|[g]o_dist|[d]rive_to.py|[e]xplore_policy.py|[g]oal.py'); do sudo -n kill -9 $p; done
for p in $(pgrep -f '[j]oystick_control|[s]afe_drive|[g]o_dist'); do sudo -n kill -9 $p; done
sudo -n docker exec -u ubuntu MentorPi zsh -c 'source ~/.zshrc >/dev/null 2>&1
timeout 3 ros2 topic pub -r 20 /controller/cmd_vel geometry_msgs/msg/Twist "{}" >/dev/null 2>&1 &
timeout 3 ros2 topic pub -r 20 /ros_robot_controller/set_motor ros_robot_controller_msgs/msg/MotorsState "{data: [{id: 1, rps: 0.0}, {id: 2, rps: 0.0}, {id: 3, rps: 0.0}, {id: 4, rps: 0.0}]}" >/dev/null 2>&1 &
wait' 2>&1 | grep -v -i pygame
echo "sources left: $(pgrep -fc '[j]oystick_control|[s]afe_drive|[g]o_dist')"
