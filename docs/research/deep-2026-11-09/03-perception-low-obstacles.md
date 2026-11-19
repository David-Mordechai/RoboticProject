# Deep research 2026-11-09, track 3: low and thin obstacles with 2D LiDAR + depth camera

Source: research agent report (web), saved verbatim in substance. [I] = inference, not sourced.

## Key facts
- Nav2 tuning guide (https://docs.nav2.org/rolling/configuration_and_development/tuning_guide/):
  - ObstacleLayer suits a planar LiDAR.
  - VoxelLayer or STVL suit depth cameras. STVL decays obstacles over time.
  - Smac Hybrid-A* with the real footprint is recommended for Ackermann robots, with a large inflation radius and cost scale.
- STVL (https://github.com/SteveMacenski/spatio_temporal_voxel_layer). The Nav2 STVL tutorial uses:
  - voxel_decay 15
  - voxel_size 0.05
  - decay_acceleration 15
  - min_z 0.1
  - FOV params, clear_after_reading true, combination_method 1
- depthimage_to_laserscan uses one image row and has range_min 0.45 by default, so it is unsuitable for low obstacles.
- No Pi 4/5 benchmark exists for STVL or VoxelLayer. One Pi 4 user reports the RGB-D driver alone took about 90% of one core.
- Thin obstacles:
  - Below about 3 cm diameter, LiDAR returns are sparse. Office chair bases are invisible to a LiDAR above them.
  - Structured light loses thin, dark or shiny parts.
  - Camera costmap fusion fixed chair-leg misses in a published study (https://arxiv.org/pdf/2212.13218).
  - Kobuki used bumper hits as a costmap source.
- HP60C: binocular structured light, 0.2-4 m, spec FOV 73.8 x 58.8 deg; measured 56.6 x 43.9 deg, mounted at 0.121 m. [I] Anything inside about 0.2-0.3 m is invisible to it.
- Hallway localisation:
  - Error is weakest along the corridor.
  - Fuse IMU yaw rate in a robot_localization EKF in two_d_mode.
  - Subtract the gyro bias. The measured 0.0011 rad/s is about 3.8 deg/min.
- Reference config, TurtleBot 4:
  - voxel_layer on the scan only
  - inflation 0.45, scale 4.0
  - resolution 0.06

## Recommended perception setup (from the report)
1. **Local costmap:** obstacle_layer for the LiDAR, plus STVL for the depth points.
   - voxel 0.05
   - Height band 0.02 m to the car height plus a margin.
   - min_z 0.2, max_z 3.0
   - Measured FOV 0.988 x 0.766 rad
   - Decay 15 s, decay_acceleration 5-15
2. **Inflation:** polygon footprint. The inflation radius must cover a chair's star-base radius around the post the LiDAR sees. Cost scale 3-5.
3. **Global costmap:** static + scan + inflation. Keep-out zones for cabinet bodies and the space under desks.
4. **Collision Monitor before the motors:** stop and slow-down polygons from the scan and the depth cloud, min_points 4. A bumper or low ToF sensor covers the 0-0.2 m blind zone.
5. **Localisation:** EKF on commanded vx + IMU vyaw with the bias removed. slam_toolbox localisation with HuberLoss.
6. **CPU:** profile the Pi 5 with the camera at 15 fps first.
