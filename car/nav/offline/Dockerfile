# Nav2 Humble for offline planner checks on saved maps (2026-11-09, Phase A). Same ROS distro as the cars.
FROM ros:humble-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends ros-humble-navigation2 ros-humble-nav2-bringup \
    python3-numpy python3-yaml && rm -rf /var/lib/apt/lists/*
