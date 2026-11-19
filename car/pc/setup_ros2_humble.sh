#!/bin/bash
# ROS 2 Humble (the cars' release) inside the WSL distro Ubuntu-22.04 on the PC. Run inside that distro:
#   bash /mnt/d/PlayGround/RoboticProject/car/pc/setup_ros2_humble.sh
# Source: docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html (Humble supports Ubuntu 22.04 "jammy" only).
set -e
. /etc/os-release
[ "$VERSION_CODENAME" = jammy ] || { echo "needs Ubuntu 22.04 (jammy), this is $VERSION_CODENAME"; exit 1; }

sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $VERSION_CODENAME main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt update
sudo apt install -y ros-humble-desktop ros-dev-tools ros-humble-nav2-map-server ros-humble-teleop-twist-keyboard

# Same DDS settings as the cars: domain 0 (stock, .robotrc leaves ROS_DOMAIN_ID unset), Fast DDS (Humble default).
grep -q "ros/humble/setup.bash" ~/.bashrc || cat >> ~/.bashrc <<'EOF'
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
[ -f ~/ros2_ws/install/setup.bash ] && source ~/ros2_ws/install/setup.bash
EOF
echo "done. Next: copy the car's message packages (README step 5), then: ros2 topic list"
