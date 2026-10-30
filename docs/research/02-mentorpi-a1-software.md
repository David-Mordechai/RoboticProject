# 04: Hiwonder MentorPi A1: stock software

*Research date: 2026-10-30. Hardware is covered in [01-mentorpi-a1-hardware.md](01-mentorpi-a1-hardware.md). I read the full English manual and the source zips it links, plus the GitHub repo ([Hiwonder/MentorPi](https://github.com/Hiwonder/MentorPi/tree/MentorPi-A1), branch `MentorPi-A1`, last push 2026-01-16; the other branches are `MentorPi-M1` (the default) and `MentorPi-T1`). We haven't seen a live system image yet: Hiwonder hands it out only by email request. Items marked **[unverified]** come from secondary sources only.*

Manual: https://docs.hiwonder.com/projects/MentorPi/en/latest/ (the same content is on wiki.hiwonder.com).

---

## 0. TL;DR

- **Layout.** The host runs Raspberry Pi OS (desktop, VNC). All robot code runs in a **Docker container with Ubuntu 22.04 + ROS 2 Humble**, and a systemd service auto-launches the whole stack at boot.
- **Driver and stack.** It's a small Python driver for the RRC Lite (serial at 1 Mbaud) plus a full stack:
  - EKF over odometry, IMU and rf2o
  - slam_toolbox for 2D mapping, RTAB-Map for 3D
  - Nav2 (AMCL, TEB)
  - OpenCV, MediaPipe and YOLOv5 demos, a lane/sign "autonomous driving" demo
  - LLM/VLM agent demos (OpenAI or OpenRouter, plus WonderEcho voice)
  - master/slave multi-robot
- **Weaknesses for our purposes:**
  - Open-loop odometry.
  - A Nav2 config that isn't right for a car: holonomic planner, tiny round footprint, spin recovery.
  - Humble reaches end of life in May 2027.
  - **No license file**: treat the code as all rights reserved.
  - Closed `.so` binaries for speech.
  - Insecure, identical defaults on both robots.
- **Plan.** Use the stock image to validate the hardware. Reuse the RRC protocol, launch files and configs as *reference*. Build our own HAL and stack on **native Jazzy**; community ports already exist. Keep one SD card with the stock image as a known-good baseline.

## 1. OS image and access defaults

| Item | Value | Source |
|---|---|---|
| Structure | Pi OS host; all robot code is inside the container. The manual says: "you must enter the container" | [Ch.2](https://docs.hiwonder.com/projects/MentorPi/en/latest/docs/3.set_development_environment.html) |
| Enter the container | Run `docker ps -a`, then `docker exec -it -u ubuntu -w /home/ubuntu <id> /bin/bash`. The shell is zsh; the workspace is `/home/ubuntu/ros2_ws`; environment variables live in `/home/ubuntu/ros2_ws/.typerc` | Ch.2 §2.3–2.4 |
| Shared folder | Host `~/docker/tmp` ↔ container `/home/ubuntu/shared` | Ch.6 §6.5 |
| Host home directory | `MentorPi/` (debug tools), `hiwonder-toolbox/` (Wi-Fi manager), `docker/` | Ch.2 §2.3.2 |
| Image download | Email support@hiwonder.com with the order number. A Google Drive folder has tools, a PC VM image and the firmware flasher | [Appendix](https://docs.hiwonder.com/projects/MentorPi/en/latest/docs/resources_download.html) |
| Hotspot (default AP mode) | SSID starts with `HW`; password **`hiwonder`**; robot IP **`192.168.149.1`** | Ch.2 §2.1.2, [Ch.13](https://docs.hiwonder.com/projects/MentorPi/en/latest/docs/14.Robot%20Network%20Configuration.html) |
| VNC / SSH | `pi` / **`raspberrypi`** | Ch.2, Ch.6 |
| Wi-Fi config | `~/hiwonder-toolbox/wifi_conf.py`: `HW_WIFI_MODE` 1 = AP, 2 = STA, 3 = direct; plus the `HW_WIFI_AP_*` and `HW_WIFI_STA_*` keys. Apply with `sudo systemctl restart wifi.service`. If the robot fails to join an STA network 3 times, it falls back to AP. **NetworkManager is not used** | Ch.13 |
| Autostart | systemd `start_app_node.service` runs `ros2 launch bringup bringup.launch.py`: controller, camera, LiDAR, **rosbridge (:9090)**, **web_video_server (:8080)**, apps, joystick. Stop it with `~/.stop_ros.sh` before running your own code | [GH bringup](https://github.com/Hiwonder/MentorPi/tree/MentorPi-A1/bringup), Ch.5 |
| Container sudo password | Not documented | – |

### Switching to Ackermann (known gotcha)
- On the VNC desktop, open the **system configuration tool**:
  - Set **Machine Type = `MentorPi_Acker`**.
  - Set **Camera Type = `ascamera`** (the depth camera).
  - Click **Save → Apply → Quit**, then wait for the beep.
- The tool writes environment variables into `.typerc`: `MACHINE_TYPE`, `DEPTH_CAMERA_TYPE`, `LIDAR_TYPE` (`MS200`/`LD19`/…), `HOST`, `MASTER`, `need_compile`.
- The URDF, odom node, launches and apps all read `MACHINE_TYPE`. **If it's wrong, the steering servo is driven as if the car were a mecanum base.** One review says the image ships set to mecanum **[unverified]**.
- In the WonderPi app, choose **MentorPi-A1**.
- To calibrate steering trim, run `./Servo.sh` (Ch.2 §2.5).
- The A1 doesn't support the "LiDAR guarding" demo.

## 2. ROS 2 workspace

**Packages** (`~/ros2_ws/src`):

| Group | Packages |
|---|---|
| Drivers | `driver/ros_robot_controller` (+`_msgs`), `controller`, `sdk` |
| Robot setup | `peripherals`, `bringup`, `calibration`, `simulations/mentorpi_description` |
| SLAM and navigation | `slam`, `navigation` |
| Apps and interfaces | `app`, `example`, `interfaces` |
| AI | `large_models` (+`_msgs`), `yolov5_ros2` |
| Multi-robot | `multi` |

**Key launch files**

| Purpose | Command |
|---|---|
| Board driver | `ros2 launch ros_robot_controller ros_robot_controller.launch.py` |
| IMU filter + odometry + EKF | `ros2 launch controller controller.launch.py` |
| Peripherals | `ros2 launch peripherals lidar.launch.py` · `depth_camera.launch.py` · `teleop_key_control.launch.py` |
| 2D / 3D SLAM | `ros2 launch slam slam.launch.py` · `rtabmap_slam.launch.py` |
| Navigation | `ros2 launch navigation navigation.launch.py map:=map_01` · `rtabmap_navigation.launch.py` |
| Calibration | `ros2 launch calibration linear_calib.launch.py` / `angular_calib.launch.py`; `ros2 run imu_calib do_calib` |

**Topics, services and frames** (from source):

| Area | Interface | Notes |
|---|---|---|
| Board → ROS | `/ros_robot_controller/imu_raw`, `/joy`, `/sbus`, `/button`, `/battery` | `/battery` is UInt16 in mV |
| ROS → board | `~/set_motor` (MotorsState, rev/s per motor), `~/pwm_servo/set_state`, `~/bus_servo/*`, `~/set_led`, `~/set_buzzer`, `~/set_oled`, `~/set_rgb` | – |
| Velocity input | **`/controller/cmd_vel`** (Twist, uncapped; used by Nav2 and the apps) | – |
| Velocity input | `/cmd_vel` (app input) | Clamped to 0.2 m/s, 0.5 rad/s |
| Pose reset | `set_odom` (Pose2D) | – |
| Odometry | `/odom_raw` → EKF → **`/odom`** + TF `odom→base_footprint` | – |
| Odometry | `/odom_rf2o` from rf2o laser odometry | Where it gets launched is unclear |
| IMU | `/imu` | Madgwick filter |
| LiDAR | `/scan_raw` → `laser_filters` → `/scan` | – |
| Depth camera | `/ascamera/camera_publisher/rgb0/image`, `.../depth0/points` | Launched by `hp60c.launch.py` |
| TF tree | `map → odom → base_footprint → base_link → {imu_link, lidar_frame, camera, wheels}` | Multi-robot prefixes frames with `robot_N/` |

**Ackermann kinematics** ([ackermann.py](https://github.com/Hiwonder/MentorPi/blob/MentorPi-A1/driver/controller/controller/ackermann.py)):
- Geometry: L = 0.145 m, track = 0.133 m, wheel = 0.067 m.
- Steering δ = atan(L·ω / v), clamped to **±45° in code**; the manual says 29°.
- Servo pulse = 1500 + 2000·deg/180, sent to PWM servo id 3.
- Rear wheels get v ± ω·track/2, sent as rev/s to motors 2 and 4.
- **Bug:** when the steering clamp saturates, the odom node still integrates the un-clamped ω.

**URDF.** `ack.xacro` makes all four wheels plain `continuous` joints, with **no steering joint**. The community repo [ViktorNfa/mentorpia1_simulator](https://github.com/ViktorNfa/mentorpia1_simulator) (MIT) has a corrected Ackermann Gazebo model.

**SLAM and navigation.**
- **slam_toolbox** for 2D (5 cm resolution). Maps are saved to `~/ros2_ws/src/slam/maps`.
- **RTAB-Map** for 3D with the depth camera.
- **Nav2**: AMCL + NavFn + SimpleSmoother + **TEB** (a DWB config also exists).
- That Nav2 config is **wrong for a car**:
  - 5 cm round footprint
  - `max_vel_theta` 3.0 alongside `min_turning_radius` 0.785
  - holonomic NavFn planner
  - spin recovery enabled
  - copy-pasted from Hiwonder's JetRover

## 3. RRC Lite firmware and protocol

- **Python SDK:** `ros_robot_controller_sdk.py`, `Board(device="/dev/rrc", baudrate=1000000)`.
- **Frame format:** `AA 55 | func | len | payload | CRC8 (Maxim)`.
- **Function codes:**

  | Code | Function | Notes |
  |---|---|---|
  | 0 | SYS | Battery sub-code 0x04 = uint16 mV |
  | 1 | LED | – |
  | 2 | BUZZER | – |
  | 3 | MOTOR | Sub 0x01: count, then [id−1 u8, rps float] … |
  | 4 | PWM_SERVO | – |
  | 5 | BUS_SERVO | – |
  | 6 | KEY | – |
  | 7 | IMU | 6 floats |
  | 8 | GAMEPAD | – |
  | 9 | SBUS | – |
  | 10 | OLED | – |
  | 11 | RGB | – |

  Worked byte examples are in the [RRC protocol docs](https://wiki.hiwonder.com/projects/ROS-Robot-Control-Board/en/latest/docs/3_RosRobot_Controller_Program_Analysis.html) and in `vendor/hiwonder/rrc-lite/Lesson 13 …pdf`.
- **No encoder or wheel-speed upload exists.** See hardware report §4.
- **Firmware:** preinstalled. Flash over USB serial (UART1) or SWD; the Drive folder has a "Firmware Flashing Tool". STM32 example projects exist for the bigger RRC board. **RRC Lite / MentorPi firmware source is not on GitHub.** Ask Hiwonder support; it's the key to real wheel odometry.
- C# note: this is a plain serial protocol, so a .NET client (`System.IO.Ports`) is easy if we want one for testing.

## 4. AI features

- **Large-model course** ([Ch.14](https://docs.hiwonder.com/projects/MentorPi/en/latest/docs/15.Large_AI_Model_Course.html)):
  - The English docs use **OpenAI**: `gpt-4o-mini` for the LLM, Whisper / `gpt-4o-transcribe` for ASR, `tts-1` / `gpt-4o-mini-tts` for TTS. **OpenRouter** serves the VLM.
  - GitHub's `config.py` is the China variant (Qwen via DashScope, StepFun).
  - Keys sit in plaintext in `large_models/config.py`.
  - The robot needs internet access (STA mode).
- **Architecture:**
  - Nodes: `vocal_detect` (wake word), `agent_process` (LLM/VLM), `tts_node`, `navigation_controller`.
  - The LLM returns JSON `{"action":[...],"response":...}`, which maps to **whitelisted functions** such as `move('kitchen')`, `vision(...)`, `line_following('black')` and `object_tracking('red')`.
  - The `SetModel` service sends `api_key` and `base_url` **in the request**, so the key is visible on DDS.
  - The speech SDK is a closed `speech.so` binary.
- **Demos:** `llm_control_move`, `llm_visual_patrol`, `llm_color_track`, `vllm_with_camera`, `vllm_navigation` (a "smart home" waypoint agent), `vllm_track`.
- **Vision:**
  - OpenCV: colour, QR codes, line following, tracking.
  - MediaPipe: hands, pose, fall detection.
  - YOLOv5 (`yolov5_ros2`, traffic-sign models; training on the Pi).
  - The sources are inconsistent: one zip is Jetson TensorRT code, and the README says YOLOv11.
- **Autonomous driving** ([Ch.11](https://docs.hiwonder.com/projects/MentorPi/en/latest/docs/12.autonomous_driving_lesson.html)): lane keeping, signs, traffic lights, turns, parking, and a full demo on the printed map (`self_driving.launch.py`). *The printed map ships only with some bundles; check whether we have one.*

## 5. Apps, PC tools, multi-robot

- **Apps and tools:**
  - **WonderPi** mobile app ([Android](https://play.google.com/store/apps/details?id=com.Wonder.Pi) / [iOS](https://apps.apple.com/cn/app/wonderpi/id1477946178)), connecting through rosbridge and web_video_server.
  - Wireless gamepad.
  - On-robot colour-threshold tool.
  - PC VM image for RViz.
- **Multi-robot** ([Ch.12](https://docs.hiwonder.com/projects/MentorPi/en/latest/docs/13.MasterSlaveAndGroupControl.html)):
  - **Namespaces, not domain IDs.** `.typerc` sets `MASTER=robot_1`; each robot sets `HOST=robot_N`; launches use `PushRosNamespace`. **`ROS_DOMAIN_ID` is 0 everywhere.**
  - Slaves join the master's hotspot.
  - Time sync is a manual `sudo date -s` (no chrony).
  - Formation demos: line, column, triangle via `tf_follower`. Group gamepad control.
  - Gamepads **cross-pair** between nearby robots.

## 6. Licensing, quality, community

- **License:**
  - The repo has **no LICENSE file**; `package.xml` says "TODO: License declaration".
  - The manual says copyright Shenzhen Hiwonder, "no reproduction without authorization".
  - The README still says "fully open source".
  - Treat the code as **all rights reserved**: read it and run it on our robots, but don't copy it into our repo. Clones belong in the git-ignored `docs/vendor/hiwonder/code/`.
- **Quality:**
  - Python only, no tests.
  - Missing env vars crash nodes.
  - Hard-coded `/home/ubuntu` paths.
  - `__pycache__` and `build/` are committed.
  - Code is copy-pasted from other Hiwonder robots.
  - `TimerAction` delays are used instead of lifecycle nodes.
  - Docs and code disagree (29°/45°, LiDAR model, YOLO version).
- **Community ports** (better starting points than the stock image):

  | Repo | License | What it is |
  |---|---|---|
  | [Matzefritz/HiWonder_MentorPi](https://github.com/Matzefritz/HiWonder_MentorPi) | MPL-2.0 | Native **Ubuntu 24.04 + Jazzy**, no Docker. M1-focused; same RRC/controller packages |
  | [LinusGro/MentorPi_ris](https://github.com/LinusGro/MentorPi_ris) | MIT | TU Darmstadt fork of the above |
  | [Frussen-Robotics/mentorpi](https://github.com/Frussen-Robotics/mentorpi) | – | Capability API for LLM agents over rosbridge; close to our Skill API idea |

- **Reviews:** Amazon reviews (mirrored) complain about the Docker setup not exposing network ports **[unverified]**.

## 7. Official tutorial chapters

| # | Chapter | # | Chapter |
|---|---|---|---|
| 1 | Getting ready (unboxing, charging, assembly, startup, app, gamepad) | 8 | ROS + OpenCV |
| 2 | Development environment (VNC, version config, Docker, servo trim) | 9 | MediaPipe HRI |
| 3 | Motion control (kinematics, IMU/linear/angular calibration, odometry) | 10 | Machine learning (YOLOv5) |
| 4 | LiDAR (avoidance, following) | 11 | Autonomous driving |
| 5 | Depth camera | 12 | Master-slave and group control |
| 6 | Mapping (slam_toolbox, RTAB-Map) | 13 | Network configuration |
| 7 | Navigation (AMCL, DWA/TEB, multi-point, 3D) | 14 | Large AI models (WonderEcho, LLM/ASR/TTS/VLM, embodied AI) |

## 8. What this means for us

**Reuse as reference**
- The RRC protocol and driver node.
- slam_toolbox and RTAB-Map configs.
- Camera and LiDAR launch files.
- udev rules.
- The calibration procedure.
- The LLM "JSON action whitelist" pattern, though our framework owns it.

**Replace**
1. **Odometry:** add real wheel feedback (firmware), or fuse rf2o / KISS-ICP / visual odometry with the IMU, before trusting Nav2 or multi-robot localisation.
2. **Nav2 for a car:**
   - Smac Hybrid-A* or State Lattice planner.
   - Real footprint polygon, about 0.21 × 0.16 m (*estimate, measure*).
   - Regulated Pure Pursuit or MPPI with the Ackermann motion model.
   - No spin recovery.
   - URDF with steering joints.
3. **ROS distro:** Humble in Docker → native **Jazzy** on Ubuntu 24.04, following the community ports. Keep one stock SD card as the reference.
4. **AI layer:** our own agent framework talks to the robot through a narrow capability/Skill API. API keys stay on the ground station, never on the robot.

**Security checklist before the two identical robots share a network**
1. Change `pi` / `raspberrypi`. Use SSH keys and turn off password login.
2. Give each robot a unique AP SSID and password, or use STA mode on a dedicated router or VLAN.
3. Give each robot a unique `HOST` namespace (and/or `ROS_DOMAIN_ID`). Otherwise `/controller/cmd_vel` crosses between robots.
4. Disable rosbridge (:9090) and web_video_server (:8080), or bind or firewall them; they have no authentication.
5. Don't leave LLM keys in `config.py` on the robots.
6. Replace the manual `date -s` with chrony.
7. Re-pair each gamepad to its own robot.
8. Remember the `docker` group is effectively root.

**Check on the real robots**
- `.typerc` values: `LIDAR_TYPE`, default `MACHINE_TYPE`, `HOST`.
- `docker inspect` for network mode and privileges.
- Whether rf2o is running.
- The full AP SSID.
- The container password.
- The RRC Lite firmware version.
