# 03: Sensors, camera, mapping and multi-robot localization

*Research date: 2026-10-31. This builds on [01-mentorpi-a1-hardware.md](01-mentorpi-a1-hardware.md) (hardware) and [02-mentorpi-a1-software.md](02-mentorpi-a1-software.md) (stock software); it doesn't repeat them. It answers open issues **#7** (leader pose on the real cars) and **#8** (LiDAR fidelity), and the plan's goal of shared maps (plan §1, §9.3). Cars haven't arrived, so nothing here is measured yet.*

**Key sources** (the short names from 01/02 still apply: [HW-A1], [DOCS], [GH], [GD])

| Short name | What |
|---|---|
| [GH] | Hiwonder/MentorPi branch `MentorPi-A1`, cloned to the git-ignored `docs/vendor/hiwonder/code/MentorPi-A1/` on 2026-10-31. Paths below are relative to it |
| [HW-A1] | Product page, Shopify JSON re-read 2026-10-31 (updated 2026-11-01 +08:00) |
| [DOCS-4] … [DOCS-12] | MentorPi docs chapters 4 (LiDAR), 5 (depth camera), 6 (mapping), 7 (navigation), 12 (master-slave). Local copies are in `vendor/hiwonder/manuals/web-*.html` |
| [M10.1] | `vendor/hiwonder/manuals/10.1 Introduction to Lidar.pdf` (the MS200 spec) |
| [LD19-DS] | `vendor/hiwonder/datasheets/LDROBOT_LD19_Datasheet_v2.5.pdf` (new) |
| [LD19-DM] | `vendor/hiwonder/datasheets/LDROBOT_LD19_Development_Manual_EN_v2.4.pdf` |
| [STL19P-DS] | `vendor/hiwonder/datasheets/LDROBOT_STL-19P_Datasheet_EN_v1.0.pdf` (new) |
| [QMI-DS] | `vendor/hiwonder/datasheets/QST_QMI8658A_Datasheet_RevA.pdf` (new; QST datasheet, mirrored by Waveshare) |
| [RRC-Lx] | `vendor/hiwonder/rrc-lite/Lesson x …pdf` |
| [ST] | slam_toolbox README and `docs/decentralized_multi_robot_slam.md`, branch `ros2`, https://github.com/SteveMacenski/slam_toolbox (read 2026-10-31) |
| [MM] | m-explore-ros2 README, https://github.com/robo-friends/m-explore-ros2 (read 2026-10-31) |
| [SWARM] | Swarm-SLAM, Lajoie & Beltrame, IEEE RA-L 9(1) 2024, https://github.com/MISTLab/Swarm-SLAM |
| [ZENOH] | rmw_zenoh ROSCon 2024 talk, https://roscon.ros.org/2024/talks/RMW_Zenoh-_An_alternative_middleware_for_ROS_2.pdf |

---

## 0. TL;DR

- **The sensors that matter for driving:** the 2D LiDAR, the HP60C depth camera (RGB + depth), and the gyro in the RRC Lite IMU. **Wheel encoders aren't reachable from the Pi.** Stock `/odom` = *commanded* speed + gyro heading (§1.4). The WonderEcho Pro is irrelevant for driving and costs a USB port.
- **The LiDAR model is still unresolved.** The product page lists *both* "Oradar MS200" and "COIN-D6" in its spec tables (re-read 2026-10-31). The stock code drives only MS200 or LD19. The MS200 launch spins it at **15 Hz**; the LD19 defaults to **10 Hz**.
- **Hiwonder's multi-robot feature is group joystick control**, not shared mapping. Its formation nodes rely on start-aligned odometry and send a lateral velocity that an Ackermann car can't execute (§3.1).
- **Recommended path for the leader pose (#7):**
  1. Build **one map** and load it on both cars.
  2. Each car localizes in that common `map` frame.
  3. Each car broadcasts a small pose/speed/steer message at 20 Hz.
  4. The follower computes the relative pose and corrects it for message age.

  An **AprilTag on each car's rear**, seen by the follower's HP60C, is the independent cross-check and fallback. Live map sharing (slam_toolbox decentralized multi-robot, Swarm-SLAM) is a later step (plan §9.3).
- **For the policy:**
  - Keep the 6-value leader block, but train it with the *real* error structure: map-frame bias, relocalization jumps, latency and dropouts.
  - Add a **pose-age** input.
  - Optionally add ~12 **depth-camera obstacle rays** for things below or above the LiDAR plane.
  - Don't feed absolute map coordinates. Global planning (Nav2) stays outside the policy.

## 1. Sensor inventory (re-checked)

Status: **C** = confirmed by two sources or by vendor code plus docs; **X** = sources conflict; **U** = unknown or unpublished.

| Sensor | Model | Key specs | Interface / ROS 2 (stock) | Status |
|---|---|---|---|---|
| 2D LiDAR | MS200 **or** COIN-D6 **or** LD19/STL-19P (§1.1) | 360°, ~12 m, 4500–5000 samples/s, 0.72–0.9° | UART→USB `/dev/ldlidar`, 230400 baud; `/scan_raw` (LaserScan) | **X** model |
| Depth camera | Angstrong Nuwa-HP60C, structured light | Depth 640×480 @ ≤20 fps, RGB 1920×1080 @ ≤20 fps, 0.2–4 m, <2 mm @ 1 m, depth FOV H73.8°×V58.8°, RGB FOV H80.9°×V51.7° | USB 2.0 Type-C; `ascamera` driver; stock config **640×480 @ 15 fps** for both streams (§1.2) | **C** specs, **U** noise model |
| IMU | QST QMI8658 (A or C variant **U**) on the RRC Lite | 6-axis; gyro noise 13 mdps/√Hz (A) | I2C to STM32 → serial → `/ros_robot_controller/imu_raw` → calib → complementary filter → `/imu` | **C** chip, **U** variant/config |
| Wheel encoders | JGA27-310 motor with AB Hall encoder | CPR unpublished for the A1 | Read by the STM32 only; **no upload to the Pi** | **C** (not reachable) |
| Battery voltage | STM32 ADC | mV | `/ros_robot_controller/battery` (UInt16, mV) | **C** |
| Steering servo | LD-1501MG, PWM | No position feedback (PWM servo) | `~/pwm_servo/set_state`; the SDK can read back a PWM-servo "position" (`pwm_servo_read_position`); a PWM hobby servo has no feedback wire, so this is most likely the last commanded pulse (*unverified*) | **C**; readback **U** |
| Mic/speaker | WonderEcho Pro (CI1302) | 3 W speaker, wake word | USB audio + CH340 serial | **C**; not useful for driving |
| Gamepad | 2.4 GHz USB dongle | – | `/joy` | **C** |

### 1.1 LiDAR: which model, and what it means for the sim

| Model | Range | Accuracy / noise | Angular res. | Samples/s | Scan rate | Source |
|---|---|---|---|---|---|---|
| **Oradar MS200** | 0.1–12 m (13.4 m black/white) | ±20 mm at 3–12 m; <3 m not given | 0.8° | 4500 | 7–15 Hz | [M10.1] §3.3, [DOCS-4] |
| **LDROBOT LD19** | 0.02–12 m (70 % reflectivity) | mean ±45 mm, **std 10 mm** (0.3–12 m); "some data" below 0.3 m; angle error ≤2° | 0.8° @ 10 Hz | 4500 | 5–13 Hz, default **10 ± 0.1 Hz** | [LD19-DS], [LD19-DM] §1 |
| **LDROBOT STL-19P** (launch-era "LD19") | 0.03–12 m | ±10 mm/std 2 mm (0.03–0.5 m), ±20/4 mm (0.5–2 m), ±30/15 mm (2–12 m), white target | 0.72° @ 10 Hz | 5000 | 6–13 Hz, default 10 Hz | [STL19P-DS] §3, §7.2 |
| **COIN-D6** (Guoke Optics) | 0.05–12 m | ±10 mm (0.05–1 m), ±40 mm (1–12 m) | 0.9° | 4000 | 10 Hz | **Reseller page only** (rcdrone.top "witmotion-dtof-lidar"); no vendor datasheet found |

- **Product page conflict.** Today's [HW-A1] lists "Lidar: Oradar MS200" in one spec table and "Lidar: COIN-D6 LiDAR" in another, plus a "COIN-D6 LiDAR" feature block. [DOCS-4] describes only the MS200.
- **Stock driver support.** `peripherals/launch/lidar.launch.py` supports only `LIDAR_TYPE` = `MS200` or `LD19`; anything else falls back to the MS200 driver [GH].
  - **MS200:** `oradar_scan` runs with `motor_speed: 15`, `range_min 0.05`, `range_max 12.0` (`include/ms200_scan.launch.py`). Stock MS200 cars therefore scan at **15 Hz**, so each scan has about 4500/15 = 300 points at about 1.2° spacing *(derived)*.
  - **LD19:** `ldlidar_stl_ros2` runs at its 10 Hz default (`include/ldlidar_LD19.launch.py`).
- **Correction to 02 §2.** The `laser_filters` node is created but **left out of the returned LaunchDescription**. Everything in the stock stack consumes the unfiltered **`/scan_raw`**: slam_toolbox, Nav2 AMCL and costmaps, RTAB-Map, rf2o and the apps (`slam/config/slam.yaml`, `navigation/config/nav2_params.yaml`) [GH].
- **Scan-plane height.** The URDF puts `lidar_frame` 0.0925 m above `base_link`, which is 0.07 m above `base_footprint`. That gives **≈0.16 m** (`simulations/mentorpi_description/urdf/ack.xacro`).
  - The same URDF's wheel offsets don't match the 65 mm wheel, so treat this as approximate.
  - The sim uses 0.14 m (*estimate*). **Measure it.**
  - The plane height matters: anything lower than this is invisible to the LiDAR (§2.2).
- **Sim implications (#8):**
  - Randomize the scan rate over **10–15 Hz**. That means a refresh every 2 policy steps at 10 Hz, or every 1.33 at 15 Hz *(derived)*.
  - Randomize per-ray noise over std **≈4–20 mm** plus a per-car bias up to **±45 mm**. These are the envelope of the vendor figures above.
  - Use a minimum range of **0.02–0.1 m**.
  - **Downsample the same way in sim and on the car.** The real 300–450 points must be reduced to our 36 sectors. Use the **minimum range per 10° sector** on the car, and give the sim enough sub-rays per sector to compute the same minimum. Taking the single nearest beam can miss thin obstacles (chair legs) that a min-pool would catch.
  - Dropouts on glass, black or shiny surfaces: no vendor number. Keep it as an estimate (open-issues #1 already does this).

### 1.2 Depth camera (HP60C)

- **Specs [HW-A1] (re-read 2026-10-31):**
  - Operating range 0.2–4 m.
  - Accuracy "<2 mm @ 1000 mm".
  - Depth FOV H73.8°×V58.8°×D86.4°; colour FOV H80.9°×V51.7°×D88.9°.
  - Depth 640×480 @ 20 fps max; RGB 1920×1080 @ 20 fps max.
  - Indoor only; USB 2.0 Type-C; <2 W; 89.9×19.0×25.0 mm.
  - The Yahboom HP60C page agrees on 73.8° and a 0.2 m blind zone.
- **No Angstrong datasheet is public.** The accuracy figure is given only at 1 m. Structured-light depth error usually grows with distance squared, so model σ(z) ≈ σ₁ₘ·(z / 1 m)² as an *estimate* until measured.
- **Stock driver configuration.** `ascamera_node` runs at `depth 640×480`, `rgb 640×480`, **`fps: 15`**, with `color_pcl: True` (`peripherals/launch/include/ascamera.launch.py`).
  - Topics: `/ascamera/camera_publisher/rgb0/image`, `.../rgb0/camera_info`, `.../depth0/image_raw`, `.../depth0/points` [GH], [DOCS-5 §5.5.3].
  - The `hp60c.launch.py` variant remaps these to `/depth_cam/...`, and the AR app subscribes to `/depth_cam/rgb/camera_info`. **Check which topic names the image really uses.**
- **Mount.** The URDF puts `depth_cam` at x = +0.064 m and z ≈ 0.12 m above the ground, with no tilt (`ack.xacro` `cam_Joint`).
  - If that's right, the lower FOV edge (29.4° below horizontal) meets the floor ≈0.12 / tan 29.4° ≈ **0.21 m** ahead *(derived)*. That is about the 0.2 m blind zone anyway.
  - **Measure the height and tilt.**

### 1.3 IMU (QMI8658 on the RRC Lite)

- **Chip.** The RRC Lite uses a QMI8658 on I2C (PB10/PB11, interrupt PB12) [RRC-L5]. The lesson text still says "MPU6050" (copied from the older board; see 01).
- **Variant unknown.** The lessons never say whether it's the A or C variant.
- **Datasheet values** [QMI-DS] §3.4, tables 7–8 (QMI8658A):
  - Gyro noise **13 mdps/√Hz**; ranges ±16…±2048 dps.
  - Accelerometer noise 150 µg/√Hz; ranges ±2…±16 g.
  - ODR up to 8 kHz (gyro) and 1 kHz (accelerometer).
  - Gyro initial offset tolerance ±10 dps.
  - Which range and ODR the RRC firmware sets is **unknown** (firmware closed, 02 §3).
- **Path to ROS:**
  - The firmware sends 6 floats (IMU packet, func 7).
  - The Pi node polls its queue in a loop with a **20 ms sleep**, so it publishes at most about **50 Hz** *(derived from `ros_robot_controller_node.py`; the firmware send rate is unknown)*.
  - The node scales the accelerometer reading by g.
  - `imu_calib/apply_calib` applies the offline calibration, then **`imu_complementary_filter`** runs with `do_bias_estimation: true` (`peripherals/launch/imu_filter.launch.py`).
  - *Correction to 02 §2:* it's a complementary filter, not Madgwick.
- **What it gives us.** The gyro's **yaw rate is the only stock motion measurement that isn't derived from a command**. At 13 mdps/√Hz over a ~25 Hz bandwidth, white noise is ≈0.07 °/s RMS *(derived)*, which is negligible. Bias is the real error, and the filter estimates it.

### 1.4 Encoders and odometry (refines F1)

- [RRC-L11] confirms the drive motor family is the **JGA27-310** with an AB Hall encoder. The gear ratio and CPR for the A1 are still unpublished.
- Encoder counts close the STM32 speed PID only. No packet carries them to the Pi (02 §3).
- **Stock EKF** (`driver/controller/config/ekf.yaml`, 30 Hz, `two_d_mode`):
  - `odom0 = odom_raw`: vx, vy and vyaw, all from the *commanded* Twist.
  - `imu0 = imu`: yaw and yaw rate.
  - `odom1 = odom_rf2o`: x, y, yaw.
- **rf2o is configured but never launched.** `rf2o_laser_odometry.launch.py` (10 Hz, `/scan_raw`) isn't included by any launch file [GH]. So stock `/odom` = commanded speed integrated along gyro heading. Heading is good. **Distance travelled is open-loop**: wrong by any wheel slip, battery sag or a speed-loop error.

### 1.5 Battery, WonderEcho, others

- **Battery:** `/ros_robot_controller/battery`, UInt16 mV, polled in the same ≤50 Hz loop [GH]. The ADC divider and accuracy aren't in [RRC-L4]. Useful for logging, and for checking speed vs voltage (F11); not a policy input.
- **WonderEcho Pro:**
  - Mic, 3 W speaker and an offline wake-word chip (`manuals/01 WonderEchoPro Introduction.pdf`).
  - No driving use beyond spoken alerts and voice commands.
  - It occupies one of the 4 USB ports (F8). **Unplug it on the driving cars** to free a port for a mesh adapter.
- **Nothing else senses.** There's no bumper, cliff, ultrasonic or line sensor. The RRC Lite I2C port is free for additions (01 §7).

## 2. Camera: what exists and what's useful to us

### 2.1 What Hiwonder's software does with the camera [GH], [DOCS 5, 6, 8–11]

| Feature | Uses | Notes |
|---|---|---|
| RTAB-Map 3D SLAM and navigation | RGB + depth + `/scan_raw` + `/odom` | `Reg/Strategy 1` (ICP on scan), `Reg/Force3DoF true`, `Grid/RangeMin 0.2` (`slam/launch/include/rtabmap.launch.py`). [DOCS-6 §6.6] warns about weak-texture areas and slow feature computation |
| AprilTag AR demo | RGB, `tag36h11`, pose from `camera_info` | `app/app/ar_app.py`, Python `apriltag`: **tag pose estimation already exists on the image** |
| Line following, colour tracking, QR | RGB, OpenCV | Line following also reads `/scan_raw` to stop for obstacles |
| Lane, sign and traffic-light "self driving" | RGB, YOLOv5 | Needs the printed map |
| MediaPipe hands and body | RGB | – |
| VLM tracking (`vllm_track`) | RGB + depth | Target distance from depth |
| Nav2 obstacle avoidance | **LiDAR only** | Voxel and obstacle layers read `/scan_raw` only (`navigation/config/nav2_params.yaml`). **Stock never uses depth for obstacles** |

**CPU cost on the Pi 5.** No Hiwonder or vendor source publishes CPU or FPS numbers for any of these on a Pi 5 (DOCS-6 gives no figures). The only external data point: one AprilTag 36h11 pipeline reports ~60 fps (~3.5 ms per detection) on a Pi 5, at higher resolution with a mono global-shutter camera (github.com/ejmccalla/AprilAssay). That's a ceiling, not our number. **Profile on the car** (plan §9.3 already requires it).

### 2.2 Camera uses that help our policy

1. **Depth obstacle rays (most useful).**
   - **The gap:** the LiDAR sees a single plane at ≈0.14–0.16 m (§1.1). An office has things it misses:
     - low: cables, low boxes, a fallen bag, a chair base
     - high: table tops, desk edges, open drawers above the plane
     - glass, which is an issue for both sensors
   - **The method:** per image column band, take the **minimum depth over a height band** from just above the floor to just above the car's 157 mm height. This is the `depthimage_to_laserscan` idea, but height-filtered via the point cloud.
   - **The output:** for example 12 forward rays over 73.8° (≈6° each) *(proposal)*.
   - **The cost:** cheap at 160×120 subsampling *(to be profiled)*.
   - **The sim** needs the same 2-D ray pattern, a **0.2 m blind zone** (report "unknown/near", not "free"), a 4 m maximum (clip to 3 m like the LiDAR), ≈15 fps, and distance-growing noise (§1.2).
2. **AprilTag relative pose (car-to-car).**
   - **The method:** mount a tag on each car's rear. The follower's RGB camera gets the leader's relative x, y and **yaw** directly, with **no network and no map**. Hiwonder already ships the detector code path.
   - **Limits:**
     - Forward cone only (RGB H80.9°), so it works for column and diagonal slots, not side-by-side.
     - Range depends on tag size: about 0.126°/pixel at 640 px *(derived)*.
     - Accuracy is unpublished for this camera.
   - **Uses:**
     - Ground truth for validating the shared-map pose (#7).
     - A fallback when Wi-Fi drops.
     - The same tags on the overhead camera serve as ground truth (00 "What to buy").
3. **Visual/RGB-D odometry or RTAB-Map localization.**
   - A second, LiDAR-independent pose source, which helps in long featureless corridors where 2D scan matching slides.
   - Heavier than 2D. Treat it as an option after the 2D pipeline is measured.
4. **Not recommended now:** feeding raw images into the policy. The plan's policy is LiDAR/pose-based (plan §3: camera-based policy inference isn't needed), and image sim-to-real would add a rendering domain gap.

## 3. Mapping, map sharing and localization

### 3.1 What Hiwonder ships

- **2D mapping:**
  - **slam_toolbox, synchronous node** (`sync_slam_toolbox_node`), 5 cm resolution, `max_laser_range 12`.
  - It processes a scan only after **0.1 m or 0.5 rad of travel** and at most **every 0.5 s** (`minimum_time_interval 0.5`).
  - Map updates every 0.2 s; TF every 0.02 s.
  - Loop closing is on (`slam/config/slam.yaml`).
  - Maps are saved as PGM via `map_saver_cli` [DOCS-6 §6.5.4].
- **3D mapping:** RTAB-Map (§2.1); the map saves on Ctrl+C [DOCS-6 §6.6.3].
- **Localization: Nav2 AMCL** (`navigation/config/nav2_params.yaml`):
  - `DifferentialMotionModel`, 500–3000 particles, `likelihood_field`, 60 beams, `laser_max_range 10`.
  - **Updates only after 0.25 m or 0.2 rad of motion** (`update_min_d`, `update_min_a`).
  - `transform_tolerance 1.0`.
  - Between updates, the map pose moves only with `/odom`.
- **Multi-robot:**
  - Namespaces `robot_N` set via `HOST`/`MASTER`; slaves join the master's hotspot; time sync by manual `date -s` [DOCS-12 §12.1–12.2] (02 §5).
  - The only documented function is **group joystick control** [DOCS-12 §12.2].
  - **Notably, the SLAM launch already contains shared-frame plumbing.** In `slam/launch/include/robot.launch.py`, a non-master robot's EKF `map_frame` is set to `<MASTER>/map`. And `navigation.launch.py` runs a namespaced Nav2 per robot with a map file argument. So "one map, every car localizes in it" fits the stock layout.
  - **The formation nodes don't suit the A1.** `multi/tf_follower.py` and `slave_tf_listener.py` place slots relative to the leader in a shared **odom** frame. That frame needs aligned start poses (`set_odom.py`) and drifts. The controller is a P-controller on x, y and yaw that outputs `linear.y` (clamped ±0.25 m/s), which an Ackermann car ignores (02 §2, `ackermann.py` uses v and ω only).
- **Accuracy and latency:** no Hiwonder page gives localization accuracy, latency or CPU load [DOCS-6], [DOCS-7].

### 3.2 Options for a common frame and the teammate's pose

| # | Approach | Common frame from | Teammate pose | Needs | Maturity / notes |
|---|---|---|---|---|---|
| **A** | **One static map, both localize** | The same map file loaded on both cars | Each car broadcasts `{stamp, x, y, yaw, v, δ, cov}` in `map`; the receiver transforms it into its own frame | Map built once; AMCL or slam_toolbox localization per car; **chrony** time sync; namespaces | Stock pieces (Nav2 AMCL, slam_toolbox `mode: localization` [ST "Localization"]). **Recommended first** |
| B | Direct sensing: AprilTag (§2.2) and/or LiDAR cluster tracking | None needed (relative) | Measured directly by the follower | Tags on the cars; detector at ≥10 Hz | Validation and fallback for A; forward cone only |
| C | **slam_toolbox decentralized multi-robot** | `global_odom`, from **known start poses** (static TFs) | Each node publishes the peer's base in its own map from shared `LocalizedLaserScan` | Same DDS domain; start poses known "at least roughly aligned" [ST docs §5.1] | The node is on the `ros2` (Rolling) and `lyrical` branches only, **not `jazzy` or `humble`** (GitHub API check, 2026-10-31). Doc added 2025-10-22. One LiDAR per instance |
| D | `multirobot_map_merge` (m-explore-ros2) | Merged occupancy grid, known or **unknown** start poses | Not provided (map only) | Tested with slam_gmapping; slam_toolbox only on an **experimental branch** [MM] | Map-only merge; pose comes from localizing in the merged map |
| E | **Swarm-SLAM** (plan §9.3) | Inter-robot loop closures, no known start | Relative pose from the shared graph | LiDAR, stereo or RGB-D front end; ROS 2 | Research code; evaluated on datasets plus a 3-robot real run [SWARM]. The building-scale target |

- **Recommendation.** Start with **A**, since it satisfies #7 with stock pieces. Use **B** as ground truth and fallback.
- For the plan's "share the map when one car finds a new room" (plan §1): start with **C** if we're willing to build slam_toolbox from source on Jazzy (backport risk, **unverified**), else **E** as already planned.
- Do the whole progression in sim first, with the same message interface.

### 3.3 Communication and bandwidth

- **DDS over Wi-Fi.**
  - The stock setup uses one `ROS_DOMAIN_ID` (0) with namespaces (02 §5; F6).
  - DDS multicast discovery is a known weak point on wireless. **rmw_zenoh** (router-based discovery, TCP) is the main alternative [ZENOH]. Caveat: all hosts must run the same ROS distro.
  - With only 2 cars and a handful of topics, plain DDS with namespaces should do *(unverified; measure in Phase 2)*.
  - For the mesh (plan §9.1), only bridge the few inter-car topics.
- **What crosses the link, per car** *(derived message sizes, not measured)*:

  | Data | Size and rate | Bandwidth | Verdict |
  |---|---|---|---|
  | Pose/speed/steer/cov message | ≈100–400 B at 20 Hz | **<10 kB/s** | Fine |
  | Raw LaserScan | 4500 samples/s × 8 B (range + intensity) | ≈36 kB/s | Fine, but unnecessary |
  | slam_toolbox `LocalizedLaserScan` | One per processed scan (≤2 Hz with Hiwonder's 0.5 s interval) | Less than raw | Fine |
  | Full OccupancyGrid | 1 B/cell at 5 cm; 160 kB for a 20×20 m area | Send on change only (plan §9.1 reliable multicast) | Fine on change |
  | Raw RGB 640×480 @ 15 fps | 13.8 MB/s | **Never send raw** | Pi 5 has no H.264 encoder (F12) |

- **Latency.** No source gives the DDS-over-Wi-Fi latency for these cars. Plan Phase 2 measures inter-car latency and pose noise. Until then the sim's 0–100 ms is an *estimate*.
- **Clock sync.** Relative-pose correction uses message stamps, so **chrony** between the cars is required, not optional. Hiwonder's manual `date -s` with "add the elapsed seconds" [DOCS-12 §12.2.1] is not adequate.

### 3.4 Expected accuracy and latency

- **No source gives localization accuracy for this car**, for AMCL, slam_toolbox or RTAB-Map. slam_toolbox's localization mode "requires quite a bit of tuning … and needs high quality odometry", and its author recommends AMCL for most beginners [ST "Localization"]. Our odometry is the weak part (§1.4).
- **What the configs do imply about error structure** *(derived from the configs above)*:
  - Map-frame pose = `map→odom` (AMCL or SLAM, updated only every ≥0.25 m, 0.2 rad, or 0.5 s) × `odom→base` (EKF at 30 Hz, open-loop distance).
  - So the error **grows smoothly with distance** between corrections, then **jumps** at each correction. That is the "SLAM pose noise and jumps" the plan already lists (plan §7).
- **Measure in Phase 2.** With the overhead camera and tags, log each car's map pose against ground truth while driving the office loop. Fit four things: a per-run bias, the jitter std, the jump size and frequency, and the latency.

## 4. How this feeds the RL policy

Current observation (54 values): trail error 4 · trail preview 4 · car motion 4 · leader 6 (relative x, y, sin Δψ, cos Δψ, v, δ/δmax) · LiDAR 36 (3 m clip). This is the layout in open-issues #2.

### 4.1 Where each input comes from on the real car

| Block | Real source | Error to train against |
|---|---|---|
| Trail error / preview (8) | The leader's broadcast pose history (breadcrumb trail) or a Nav2 global path, both expressed in the shared `map` frame, then transformed into the car frame **outside** the policy | Own-pose error (§3.4) + leader-pose error |
| Car motion (4) | Speed = commanded (open-loop, §1.4) unless rf2o or encoders are added; yaw rate = gyro | Speed: a scale error (wheel diameter 65/67 mm, battery sag); yaw rate: small bias |
| Leader (6) | Option A message: pose → relative pose, extrapolated by message age using the leader's broadcast v and δ. **v and δ are the leader's own commands**, so they're exact but delayed | Map bias of *both* cars, jumps, latency, dropouts. Cross-check against the AprilTag |
| LiDAR (36) | `/scan_raw`, min-pooled to 10° sectors | §1.1 |

### 4.2 Proposed observation changes (54 → 55, or 67 with the depth rays)

1. **Add `leader_age`** (1 value). Seconds since the last leader message, clipped at 1 s and normalized.
   - Lets the policy learn to slow down or hold when data is stale.
   - The sim must produce it from the same dropout and latency model.
2. **Optionally add `depth_rays`** (12 values). Forward 73.8° fan from the depth camera (§2.2).
   - Do this only together with sim obstacles the LiDAR can't see (open-issues #11), or the policy will learn to ignore them.
3. **Don't add absolute own pose** (x, y in the map). It ties the policy to one map and isn't needed; everything the policy needs is already relative.
4. **Don't add pose covariance yet.** AMCL and slam_toolbox covariances aren't calibrated for these cars (`position_covariance_scale` exists for tuning [ST]). Add it only if Phase 2 shows that reported covariance tracks the real error.
5. **Later, for MAPPO (#5):** each car gets the other car's block through the same message. The critic may see ground truth (CTDE, plan §7).

### 4.3 Randomization to add or adjust in sim (feeds open-issues #1, #7, #8)

- **Leader pose:**
  - A per-episode **bias**: sum of both cars' map errors.
  - Per-step jitter.
  - **Jumps:** a random offset applied at intervals of 0.25 m of travel or 0.5 s, which is the stock AMCL/SLAM update cadence.
  - **Latency** 0–100 ms (*estimate*).
  - **Burst dropouts:** hold the last value and increase `leader_age`.
  - All magnitudes are *estimates* until the Phase 2 measurement.
- **Own speed:** scale error ±3 % (wheel 65 vs 67 mm, 01 §3) *(derived)*, plus a delay.
- **LiDAR:**
  - Scan rate 10–15 Hz.
  - Min-pooled sectors.
  - Std 4–20 mm; bias ±45 mm.
  - Min range 0.02–0.1 m.
  - Plane height 0.14–0.16 m.
  - Values from §1.1.
- **Depth rays (if added):** 15 fps, 0.2 m blind zone, distance-squared noise, dropouts on dark or shiny surfaces.

### 4.4 What stays out of the policy

- SLAM, map merging and localization: classical nodes on the car.
- Global planning: Nav2 with Smac Hybrid-A* or a lattice planner for a car (02 §8) produces the leader's route or the trail.
- Formation slot choice: MCP/agent.
- Safety stop and watchdog: our safety node (F4, F5).

The policy stays **local control**: follow the trail or slot, avoid what the LiDAR and depth camera see, and handle stale or noisy teammate data.

---

## Findings that affect the plan

| # | Finding | Affects | What it means |
|---|---|---|---|
| G1 | **Stock `/odom` = commanded speed + gyro heading.** rf2o is configured but never launched; the EKF has no measured distance input | F1, plan §5, open-issues #7 | Launch rf2o (or KISS-ICP) into the EKF before trusting AMCL or relative poses. Encoders still need firmware |
| G2 | **The LiDAR model is still contradictory.** Product page lists MS200 *and* COIN-D6; code supports MS200/LD19 only; the MS200 runs at 15 Hz in stock | 01 §5, open-issues #3, #8 | Read the label. A COIN-D6 needs its own driver (none found). Sim: scan rate 10–15 Hz, noise envelope §1.1 |
| G3 | **The stack uses unfiltered `/scan_raw`.** The `laser_filters` node isn't launched (02 §2 was wrong) | 02 §2 | Self-hits from the car's own body or bracket aren't filtered; check on the car and add a box filter if needed |
| G4 | **Hiwonder multi-robot = group joystick.** Its formation code uses aligned odom and lateral velocity, which doesn't suit the A1 | Plan §5, open-issues #5–#7 | Don't build on `multi/`. Reuse only the namespace and `<MASTER>/map` layout |
| G5 | **Leader pose via one shared map (A) + AprilTag check (B)** is the shortest path to #7 | Open-issues #7, plan §5 | Needs: map once, both localize, a 20 Hz pose message, chrony, the overhead-camera error measurement |
| G6 | **slam_toolbox decentralized multi-robot exists, but not in the Jazzy release branch,** and it needs known start poses | Plan §9.3 | Live map sharing on Jazzy means a source build or backport (unverified), or Swarm-SLAM as planned |
| G7 | **The LiDAR plane is blind below and above ≈0.14–0.16 m.** The depth camera covers 0.2–4 m in a 73.8° cone, but stock never uses it for obstacles | Plan §5, open-issues #11 | Optional 12 depth rays in the obs; add low and high obstacles to the office sim |
| G8 | **The WonderEcho isn't needed for driving** | F8 | Unplug it to free a USB port for mesh radios |
| G9 | **No source gives Pi 5 CPU, localization accuracy or Wi-Fi latency** | Plan §9.3, Phase 2 | Profile slam_toolbox/AMCL, the AprilTag detector and depth rays on the Pi 5. Measure pose error and latency against overhead-camera ground truth |

## Open items

- **Per car:**
  - LiDAR label and `LIDAR_TYPE`.
  - Actual scan rate (`ros2 topic hz /scan_raw`) and points per scan.
  - LiDAR plane height.
  - Depth camera height and tilt.
- **On the image:**
  - Which depth topics exist (`/ascamera/...` vs `/depth_cam/...`).
  - IMU publish rate (`ros2 topic hz /ros_robot_controller/imu_raw`).
  - Whether the firmware's IMU range/ODR can be read.
- **Measure these:**
  - Gyro bias drift (car standing still, 10 min).
  - Speed scale vs command (tape measure plus overhead camera).
  - Pose error (bias, jitter, jumps) for AMCL and for slam_toolbox localization, with rf2o off and on.
  - Inter-car message latency and dropouts over the planned network.
- **Profile on the Pi 5:** slam_toolbox localization, AMCL, rf2o, the AprilTag detector at 640×480, depth-ray extraction. All while the policy runs at 20 Hz.
- **Test whether `decentralized_multirobot_slam_toolbox_node` builds on Jazzy.** Compare it with Swarm-SLAM for plan §9.3.
- **Ask Hiwonder:**
  - Which LiDAR ships now.
  - A COIN-D6 driver.
  - The RRC Lite firmware source (encoders, IMU config).
  - The HP60C datasheet with its depth-noise curve.
