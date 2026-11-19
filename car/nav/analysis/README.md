# Phase A floor-run analysis (2026-11-09)
Read ROS 2 bags recorded by `car/measure/rec_run.sh` (Humble, sqlite3). Run where rosbag2_py is available
(the car's MentorPi container, or `robotlab-nav2:humble` on the ASUS): `python3 SCRIPT.py RUN/bag`.
- `bagtime.py`: per-second timeline: odom, controller wish (/cmd_vel_nav), motor command, nearest LiDAR / depth ahead.
  Found the depth "floor" phantom 0.3 m ahead in a_c2_1.
- `steer.py`: steering the driver really sets (atan(0.145 w / v), +-40 deg) at each command stage; flips, full lock.
- `flipcause.py`: what preceded each left/right flip (replan shift, AMCL jump, reversing).
- `revsegs.py`: reverse segments driven and reverse cusps in every /plan (the 1 Hz replan cusps).
- `bagsum.py`: message counts + sampled commands (Collision Monitor zeroing the commands in a_c1_1).
Live (car standing, no bag): `zone_pts.py` (LiDAR/depth points inside the Collision Monitor zones),
`dsnow.py` (/depth_scan nearest hits). Raw runs: `car/maps/phase_a_runs/phase_a_runs.tgz`.
