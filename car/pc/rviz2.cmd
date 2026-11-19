@echo off
rem Start rviz2 on this PC (ROS 2 Humble in WSL Ubuntu-22.04), set up for the cars: map, LiDAR, depth scan, TF, paths.
wsl -d Ubuntu-22.04 -- bash /mnt/d/PlayGround/RoboticProject/car/pc/ros.sh rviz2 -d /mnt/d/PlayGround/RoboticProject/car/pc/car.rviz
