# Car #2: back to factory state (plan for 2026-11-05)

The user wants car #2 exactly as it was delivered, to find out why the stock software does not drive the car (car #1
stays the training car). Below: every change we made on car #2 (from our notes, `robot2-access` memory and the scripts in
`car/`), and how to undo it. Two ways:

- **A. Re-flash the SD card** with Hiwonder's image: guaranteed factory state; needs the user to swap the card, the
  image file and a card writer. Afterwards only re-do the network step (otherwise the car is unreachable on the hotspot).
- **B. Undo our changes one by one** with the revert scripts we left on the car (newest first). Keeps the Wi-Fi.

**Decide with the user tomorrow.** B is faster if every revert works; A is the only proof of "as delivered".

## Our changes on car #2, newest first

| # | When | Change | Undo |
|---|---|---|---|
| 1 | 2026-11-04 | Explore v5 stack files in `~/docker/tmp/` (nav/, perception/depth_scan.py, policy/, `~/start_mapstack.sh`, `yawsrc.sh`, `estop.sh`, `dry_run.sh`) | Delete them; the stale originals are in `~/docker/tmp/stale_backup_2026-11-04` |
| 2 | 2026-11-04 | ONNX Runtime 1.19.2 installed (pip, in the container) | `pip uninstall onnxruntime` in the container |
| 3 | 2026-11-04 | `ROS_DOMAIN_ID` 2 (both cars on one hotspot saw each other) | `~/docker/tmp/domain_backup/revert.sh`. Keep 2 while both cars are on the same hotspot? Ask |
| 4 | 2026-11-04 | Wi-Fi STA on the phone hotspot (factory: own access point HW-5400820C) | `~/hiwonder-toolbox/wifi_conf_ap_backup.py` → `wifi_conf.py`. **Last step**, or the car drops off the hotspot |
| 5 | 2026-11-02 | Motor direction fix: sign flip at `ackermann.py:50` (both rear wheels now drive forward) | `~/docker/tmp/motorfix_backup/revert.sh` |
| 6 | 2026-11-02 | Low-load: no start_app nodes, camera 15 fps (both copies of `bringup.launch.py`) | `~/docker/tmp/lowload_backup/revert.sh` |
| 7 | 2026-11-02 | Drive command watchdog (systemd service on the host) | `~/watchdog/uninstall.sh` |
| 8 | 2026-11-02 | Camera fps fix: Fast DDS shared memory, joystick busy-loop sleep, ascamera 30 fps / no colour point cloud | `~/docker/tmp/camfps_backup/revert.sh` |
| 9 | 2026-11-01 | `MACHINE_TYPE` set to `MentorPi_Acker` (it was a Mecanum value when delivered) | `~/.typerc.bak-mecanum`. **Check the docs first**: if Hiwonder's version-configuration step says to set Acker, that is setup, not our change |
| 10 | 2026-11-01 | Our SSH key in `~/.ssh/authorized_keys` | Harmless; keep (we need it to work on the car) |

## Hiwonder's documented procedure (read 2026-11-04 from `docs/vendor/hiwonder/manuals`)

- **Chassis type ("8.2 Robot Version Configuration Instruction").** Over VNC, run the desktop "robot system
  configuration tool": Machine Type `MentorPi_Acker`, Camera Type `ascamera`, then Save → Apply → Quit ("This sequence
  must be followed") and wait for the beep. On car #2 we set MACHINE_TYPE by hand instead.
  - The URDF (`mentorpi.xacro`) and the odometry node depend on this type.
  - The odometry node also reads `calibrate_params.yaml` (`linear_correction_factor`, `angular_correction_factor`).
- **Mapping (web-7, §6.5).**
  - Network: LAN mode and ROS_DOMAIN_ID 0 on both the car and the RViz PC/VM.
  - On the car: `~/.stop_ros.sh`, then `ros2 launch slam slam.launch.py`. On the VM: `ros2 launch slam rviz_slam.launch.py`.
  - Drive with `ros2 launch peripherals teleop_key_control.launch.py`, keyboard W/S/A/D, slowly.
  - Map quality (§6.4.3–6.4.4): start facing a straight wall, close a loop, leave no grey area where the car will drive.
  - Save: `cd ~/ros2_ws/src/slam/maps && ros2 run nav2_map_server map_saver_cli -f "map_01" --ros-args -p map_subscribe_transient_local:=true`
- **Navigation (web-8, §7.4).**
  - On the car: `~/.stop_ros.sh`, then `ros2 launch navigation navigation.launch.py map:=map_01`.
  - On the VM: `ros2 launch navigation rviz_navigation.launch.py`.
  - Set "2D Pose Estimate" first (AMCL otherwise assumes the map origin), then "2D Nav Goal", or waypoints with
    "Start Nav Through Poses".
  - Afterwards: `sudo systemctl restart start_node.service` on the host.
- **2025 hardware ("MentorPi Ackermann 2025 Version Upgrade Overview").**
  - The steering goes to 40° (2024: 29°) for a smaller turning radius, and the PDF says to "refer to the 2025 version documentation", which we do not have.
  - Unverified: are our cars 2025 units, and does the image match the hardware? Our code and sim assume 29–37° (CLAUDE.md).
  - The speed lesson (9.6) gives 29° in one place and 36° in another, and linear speed ±0.6 m/s.
- **Not in our folder:** §3.3 "IMU, Linear Velocity and Angular Velocity Calibration", the 2025-version docs.
  `8.5 MentorPi Servo Deviation Adjustment.pdf` is in the folder but not yet read.

**Differences between what we did on 2026-11-04 and the documented procedure:**
- Map built by our own drives, not by teleop with a closed loop.
- Chassis type not set through the official tool.
- The autostart stopped with `systemctl`, not `~/.stop_ros.sh`.
- Goals sent by script, not from RViz.
- No odometry / IMU calibration after our motor-direction and trim changes.

## Questions the factory test must answer (why stock does not drive)

These come from what we found on both cars, not yet checked against Hiwonder's manuals (`docs/vendor/hiwonder/manuals`):

1. **Chassis type.** The delivered `MACHINE_TYPE` was Mecanum on an Ackermann car. Hiwonder's "8.2 Robot Version
   Configuration Instruction" is probably the documented step that sets it. With the wrong type, odometry and
   kinematics are wrong, so Nav2 cannot drive.
2. **Motor direction.** Without our sign flip, the rear wheels drove in opposite directions on both cars. Is that a
   wiring / version mismatch (see "MentorPi Ackermann 2025 Version Upgrade Overview")?
3. **Odometry.** The stock odom integrates the unclamped yaw rate when the steering clamp saturates (`research/02`
   §2). Measure: drive 1 m straight and a 90° turn, and compare `/odom` with tape-measured truth.
4. **Stock Nav2 config for a car.** Holonomic NavFn, TEB `min_turning_radius` 0.785 m with `max_vel_theta` 3.0,
   5 cm round footprint, Spin recovery (`research/02` §2). On car #1, 2026-11-04: "trajectory is not feasible", then
   Spin. Does Hiwonder ship a different config or launch argument for the Acker version?
5. **Procedure.** Map with `slam.launch.py` on a factory car, save it, `navigation.launch.py map:=...` with the
   autostart stopped, goal from RViz as the tutorial shows. Record every run (camera, bag) on the PC.
