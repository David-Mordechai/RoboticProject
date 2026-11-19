#!/bin/bash
# Run a ROS 2 tool on the PC (WSL distro Ubuntu-22.04, ROS 2 Humble, set up by setup_ros2_humble.sh) with the settings
# the cars need. Usage inside Ubuntu-22.04:   bash /mnt/d/PlayGround/RoboticProject/car/pc/ros.sh [command ...]
#   no command -> an interactive shell with ROS loaded.  Examples: ros.sh rviz2 | ros.sh rqt_graph | ros.sh rqt
# From Windows, the .cmd files next to this script call it.
source /opt/ros/humble/setup.bash
[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash   # Hiwonder message types, if built
export ROS_DOMAIN_ID=0                       # same as the cars (stock: ROS_DOMAIN_ID unset = 0)
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp   # Humble default, same as the cars
export LIBGL_ALWAYS_SOFTWARE=1               # rviz2 segfaults on the WSLg D3D12 GPU driver; software OpenGL works
if [ $# -eq 0 ]; then
  exec bash --rcfile <(echo 'PS1="(ros2) \w\$ "')
fi
exec "$@"
