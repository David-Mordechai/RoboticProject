# 00: Research overview and decisions

*2026-10-27. Synthesis of the three research reports. Read this first, then dive in:*

| Report | What it covers |
|---|---|
| [01-adeept-mecanum-kit.md](01-adeept-mecanum-kit.md) | Kit ADR036: Robot HAT V3.2 chips and pinout, every sensor, Adeept code, Pi 5 compatibility, upgrades, first-week checklist |
| [02-raspberry-pi5-robotics.md](02-raspberry-pi5-robotics.md) | Pi 5 for robots: power, GPIO, OS, ROS 2, camera, on-device AI (Hailo, LLMs), networking, accessories, gotchas |
| [03-autonomy-architecture-and-roadmap.md](03-autonomy-architecture-and-roadmap.md) | Layered architecture (UGV to UAV), ROS 2 stack, LLM agents, fleet coordination, self-learning, PX4, .NET's role, 10-phase roadmap + KPIs |

---

## What the robot actually is

The kit is **Adeept ADR036**, a 4WD mecanum car. Code: https://github.com/adeept/Adeept_4WD_Smart_Car_for_RPi. Clone it from GitHub, not the zip on Adeept's site: GitHub has a Pi 5 fix dated Sept 2026. The repo has no license, so we write our own drivers.

| Subsystem | Hardware | What it means for autonomy |
|---|---|---|
| Brain | Pi 5 8GB + 128GB A2 SD | Plenty for ROS 2 + SLAM + Nav2 + small CV models |
| Motor/servo PWM | PCA9685 @ I2C 0x5F (servos ch0–7, motors ch8–15, **50 Hz**) | Crude motor control; fine to start |
| Motors | 4x TT gearmotors via DRV8833, **no encoders** | **No wheel odometry**: the biggest gap |
| ADC | ADS7830 @ 0x48 (battery, light sensor, 6 free channels) | Battery telemetry for free |
| Range | 1x HC-SR04 ultrasonic on the pan/tilt head (2 cm–2 m, wide cone) | Obstacle stop and "radar sweep", not mapping |
| Vision | CSI camera (probably OV5647, 62°) on pan/tilt servos | Object detection, AprilTags, VLM snapshots |
| Ground | 3-ch IR line tracker, 2-LDR light tracker | Line following, light seeking (good first exercises) |
| Display/IO | SSD1306 OLED, HT16K33 8x16 matrix, WS2812 LEDs, buzzer, IR receiver | Status display per robot (handy in a fleet) |
| **IMU** | **Empty MPU6050 socket on the HAT** | A $3–5 MPU6050 gives you a gyro right away |
| Power | 2x 18650 (2S, 8.4 V), USB-C charging on the HAT, about 3.75 A peak | **Pi 5 brownouts are the #1 risk** |

## Top risks, in priority order

1. **Power.** The HAT feeds the Pi 5 through the GPIO 5V pins from an undocumented regulator, probably shared with the servos. A sibling Adeept kit with a Pi 5 rebooted every time a servo moved. On GPIO power the Pi 5 also limits USB to 600 mA. *Mitigation:* on day 1, measure with `vcgencmd get_throttled` while servos and motors move. If it's not `0x0`, add a dedicated 5V/5A buck for the Pi (Pololu D36V50F5).
2. **No odometry.** Open-loop TT motors plus mecanum slip means you can't tell where you are. *Mitigation:* a LiDAR for scan-matching odometry, plus an IMU. Encoder motors and a Pico come later.
3. **Fleet defaults are insecure and identical.** Same hotspot SSID and password, the same `admin:123456` login, and everything runs as root. Change these before powering up three robots.
4. **The camera cable:** the Pi 5 needs a 22→15-pin cable. The box probably includes one, but buy spares.

## Decisions (proposed; revisit as we learn)

| # | Decision | Why |
|---|---|---|
| D1 | **Host OS: Raspberry Pi OS (Trixie, 64-bit). ROS 2 Jazzy in Docker** (`--net=host`, devices passed through) | Adeept code, Picamera2/libcamera and Hailo all work best on Pi OS; Docker gives us standard ROS 2 Jazzy images. *(Report 03 assumed Ubuntu 24.04 + native Jazzy. That stays the fallback if Docker device access becomes painful.)* |
| D2 | **Phase 0 uses the stock Adeept code** only to validate hardware; then we write our own HAL as a ROS 2 node | License, plus we need continuous `(vx, vy, ω)` control, not 10 discrete moves |
| D3 | **rmw_zenoh** as the ROS 2 middleware once there's more than one robot | DDS multicast discovery is unreliable over Wi-Fi |
| D4 | **Narrow waist = Skill API** (`NavigateTo`, `Explore`, `Search`, `ReturnHome`, `Capture`, …) as ROS 2 actions | Everything above it (mission, fleet, LLM, learning) transfers unchanged to UAVs |
| D5 | **LLM proposes, deterministic layer verifies and executes.** Claude produces a schema-validated mission plan; a validator plus sim dry-run run before a behavior tree executes it. The LLM never drives motors | Safety; the same pattern is mandatory on drones |
| D6 | **Centralized planning, decentralized execution.** Contract Net first, CBBA later; leases + comm-loss policy from day one | Debuggable first; decentralization is earned later |
| D7 | **.NET for the ground station** (fleet manager, event store, LLM orchestration, UI, via rclnet + the Anthropic C# SDK). **Python/C++ on the robots** | Plays to your strengths without fighting the ROS ecosystem |
| D8 | **Self-improvement = fixed benchmark + automated tuning (Optuna) + gated LLM reflection proposals.** RL is a side quest | Realistic on hobby hardware; measurable |
| D9 | **PX4** for the UAV phase: SITL, then Crazyflie/Crazyswarm2, then a real quad | Native ROS 2 bridge; custom modes map onto the Skill API |

## Shopping list

**Before the kits arrive (essentials):**
- 8–12 quality high-drain 18650s (Samsung 30Q/25R, Molicel P28A, Sony VTC6) bought as **matched pairs**; check the holder for button-top vs flat-top. 12 lets you hot-swap packs.
- External 18650 charger (Nitecore/XTAR, 4-bay).
- 3x official **Pi 5 Active Cooler**.
- 3–6x **Pi 5 camera cables** (22→15 pin, "camera" type).
- 3x **MPU6050** modules for the HAT socket (about $3–5): instant gyro.
- Multimeter and an inline USB-C power meter (if you don't have them).

**Phase 1–2 (once one robot runs):**
- 2D LiDAR per robot: **RPLIDAR C1** (about $80) or **LDROBOT LD19** (about $70–100). For the C1, mount it centred on top plate A07 on **4x M2.5 x 15 mm brass standoffs** with **M2.5 x 4 mm screws** (max 4 mm into the LiDAR, per Slamtec). That puts the laser plane about 13 mm above the tilted camera head; see `docs/images/cad/lidar-height-options.png`. Buy an M2.5 standoff assortment (10/15/20 mm) so you can adjust after measuring the real head height.
- **Pololu D36V50F5** 5V/5.5A buck per robot, *if* the power test fails.
- Better IMU (BNO085) if the MPU6050 drift hurts.
- Overhead USB webcam + printed **AprilTags** as a ground-truth rig for measuring accuracy.

**Later:** encoder TT motors (DFRobot FIT0450) + Raspberry Pi Pico per robot, VL53L1X ToF sensors, Camera Module 3 Wide, AI HAT+ (Hailo-8) if CPU vision is too slow.

## Roadmap (condensed from report 03 §8)

| Phase | Milestone | Exit KPI |
|---|---|---|
| 0 Bring-up | One robot assembled; wheels turn from a ROS 2 `cmd_vel` | Power test passes; drift baseline over 2 m |
| 1 ROS 2 foundations | Teleop + live sensors in Foxglove; bag replay | — |
| 2 State estimation | LiDAR/IMU/EKF odometry | Drift < 2% vs AprilTag ground truth |
| 3 SLAM + Nav2 | 20 random goals in the arena | ≥95% success, ≤5 cm error, 0 collisions |
| 4 Digital twin + CI | Same launch files in Gazebo; benchmark v1 | Sim-vs-real gap recorded |
| 5 Skill API | "Explore and find the red ball" (hand-written mission) | Coverage m²/min, time-to-detect |
| 6 Multi-robot | 3 robots explore cooperatively (Contract Net → CBBA) | ≥2× speedup, <25% overlap |
| 7 LLM missions | Natural-language missions via Claude planner + validator | ≥80% success, **0 unsafe executions** |
| 8 Self-improvement | Benchmarks improve across 3 auto-promoted config versions | Δ KPIs, no safety regression |
| 9 UAVs | Same mission layer drives PX4 SITL quads, then Crazyflies | Zero changes above the Skill API |

## Open questions to answer on the bench

The 5V regulator rating and whether the servos share it · camera sensor model · which FFC is the Pi 5 cable · TT motor RPM and wheel diameter · robot dimensions and mass · does the HAT charger balance cells · active-cooler clearance under the 21 mm standoffs. See report 01 §9.
