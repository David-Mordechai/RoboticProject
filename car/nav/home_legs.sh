#!/bin/bash
# Two-leg park (runs INSIDE the MentorPi container, from /home/ubuntu/shared; called by nav_a_home.sh via rec_run.sh).
# 2026-11-11: car #1 steers only ~28 L / ~24 R (measured, arc_icp.py). At 24-28 deg the one-goal park failed 4/14 on
# the bench (shuffling next to the dock wall). Instead:
#   leg 1: U-turn in the middle of the open room to a waypoint facing out (2.0, 0.2, -8 deg), LOOSE goal (yaw 0.8 rad,
#          xy 0.3 m): it is only a pass-through point; the tight version failed 1/9 by heading dither at the waypoint;
#   leg 2: reverse ~1.65 m straight to the pre-dock pose (0.35, -0.03, -8), goal yaw 0.5 rad / xy 0.15 m (bench 9/9);
#   then dock.py reverses in (rear LiDAR stop).
# Tolerances go back to make_params_a.py values (yaw 3.14, xy 0.3) on exit, also on failure.
G() { ros2 param set /controller_server general_goal_checker.yaw_goal_tolerance $1 >/dev/null
      ros2 param set /controller_server general_goal_checker.xy_goal_tolerance $2 >/dev/null; }
trap 'G 3.14 0.3' EXIT
G 0.8 0.3  && python3 nav/nav_trip.py 2.0 0.2 -8 || exit 1
G 0.5 0.15 && python3 nav/nav_trip.py 0.35 -0.03 -8 || exit 2
python3 nav/dock.py 0.4
