# 00: Research overview

*Updated 2026-10-30. **The project plan is [`docs/project-plan.md`](../project-plan.md)**: two-car formation driving trained with RL in a GPU simulator on the GB10, an MCP server for agent control, and building-scale mesh networking. This page summarises the hardware research and lists the findings that affect that plan.*

| Doc | What it covers |
|---|---|
| [../project-plan.md](../project-plan.md) | **The plan**: goals, architecture, MCP server, simulator and training, phases, comms, open decisions |
| [01-mentorpi-a1-hardware.md](01-mentorpi-a1-hardware.md) | Kit contents, RRC Lite controller, chassis geometry, motors and servo, LiDAR, depth camera, WonderEcho, power, expansion |
| [02-mentorpi-a1-software.md](02-mentorpi-a1-software.md) | Stock image (Pi OS + Humble Docker), ROS 2 packages and topics, RRC serial protocol, SLAM/Nav2, AI demos, multi-robot, security checklist |

Vendor manuals, datasheets and images are in `docs/vendor/hiwonder/`.

---

## The cars (×2): Hiwonder MentorPi A1 Advanced, Pi 5 8GB

| Subsystem | Hardware |
|---|---|
| Brain | Pi 5 8GB, 64 GB microSD; Raspberry Pi OS host, **ROS 2 Humble in Docker** |
| Motion controller | **RRC Lite** (STM32F407) over USB serial at 1 Mbaud (`/dev/rrc`): motor speed PID, steering servo, IMU, battery, buzzer and LEDs |
| Chassis | Aluminium, **Ackermann-style** (one servo steers both front wheels in parallel). Regular wheels, so **no strafing or in-place turns**. 213×159×157 mm, ~1.2 kg |
| Geometry (code values; *measure*) | Wheelbase 0.145 m, track 0.133 m, wheel Ø 65–67 mm; max steer **29° or 40°** depending on build year; min turn radius ≈0.17–0.26 m (*estimate*) |
| Drive | 2 rear 310 gear motors with quadrature encoders + LD-1501MG steering servo |
| LiDAR | 360° 2D TOF, 12 m. The model varies by batch (LD19, MS200 or COIN-D6): **check each car** |
| Depth camera | Angstrong HP60C RGB-D, 0.2–4 m, USB |
| IMU | QMI8658 6-axis on the RRC Lite |
| Voice | WonderEcho Pro (USB mic and speaker, offline wake word) |
| Power | 7.4 V 2200 mAh LiPo (~16 Wh); RRC Lite feeds the Pi 5V/5A via USB-C PD; runtime ~1–1.5 h (*estimate*), ~1 h to charge |

## Research findings that affect the plan

| # | Finding | Plan section | What it means |
|---|---|---|---|
| F1 | **Stock odometry is open-loop.** `/odom` integrates the *commanded* velocity. The encoders close only the STM32's internal speed PID, and the serial protocol has no encoder upload. | §5 localisation, §7 sim parameters | Don't treat `/odom` as ground truth. Measure real speed, acceleration and delay with the overhead camera and LiDAR in Phase 2. For better odometry, get the RRC Lite firmware source (ask Hiwonder) and add an encoder packet. |
| F2 | **Speed and steering don't map cleanly to stock `cmd_vel`.** The stock node turns a Twist (v, ω) into a steering angle δ = atan(L·ω/v), clamped. At v = 0 it can't set a steering angle, and odometry integrates the un-clamped ω. `/cmd_vel` is clamped to 0.2 m/s; `/controller/cmd_vel` is not clamped. | §5.3 low-level control, §6 `drive()` | For a policy that outputs speed + steering, use our own thin node that sends **motor rev/s + servo pulse** straight to the RRC. This gives exact steering-angle control and the same interface as the sim's bicycle model. |
| F3 | **Steering differs by build year.** It's 29° (2024) or 40° (2025), with the code clamped at 45°. Both wheels turn by the same angle. | §7 sim parameters | The bicycle model still fits. Measure max angle and turning radius **per car** and give each car its own parameters. Trim each car with `Servo.sh`. |
| F4 | **The kill switch isn't independent by default.** The kit gamepad goes through the Pi and the ROS stack, and gamepads **cross-pair between nearby cars**. | §2, §6 safety | Fine as a *software* stop. A truly independent kill switch needs a separate path: an RF relay on the battery or motor line, or the planned LoRa e-stop (§11). Re-pair each gamepad to its own car. |
| F5 | **Watchdog behaviour is unknown.** It's not documented whether the stock controller or the RRC firmware stops on a command timeout. | §6 watchdog | Our safety node must enforce the 0.5 s stop itself. Test on the bench (wheels off the ground) whether the RRC keeps spinning motors after the Pi stops sending. |
| F6 | **ROS networking defaults clash.** Both cars use `ROS_DOMAIN_ID` 0 and the same `HOST` namespace, so topics like `/controller/cmd_vel` would cross-command between cars. | §12 step 2 | Shared domain is fine, but give each car a **unique namespace** (`HOST=car_1` / `car_2`, Hiwonder's `robot_N` scheme). Sync clocks with chrony; Hiwonder's multi-robot docs use a manual `date -s`. |
| F7 | **Wi-Fi is managed by Hiwonder's `wifi.service`.** It boots in AP mode at `192.168.149.1`, uses no NetworkManager, and falls back to AP if it can't join a network. | §9.1 mesh, §10.3 IPs | Disable or replace `hiwonder-toolbox` Wi-Fi management before setting up 802.11s / BATMAN-adv; otherwise it will fight the mesh config. |
| F8 | **USB ports are already full.** The RRC, LiDAR, depth camera, gamepad dongle and WonderEcho are **5 devices for 4 USB-A ports**. | §10.1 two USB Wi-Fi adapters per car | Two mesh adapters per car need a **powered USB hub**, or dropping the WonderEcho or gamepad dongle. Also check total USB current: the Pi gives 1.6 A to USB only with the 5 A PD supply, which the RRC Lite claims to provide. |
| F9 | **Default credentials and open services.** Both cars ship with `pi` / `raspberrypi`, hotspot password `hiwonder`, rosbridge on :9090 and web video on :8080, all **unauthenticated** and auto-started. | §4 local only, §10.3 credentials | Change passwords and use SSH keys. Stop or firewall rosbridge and web video unless the MCP server uses them. Checklist: report 02 §8. |
| F10 | **Humble support ends May 2027.** Hiwonder's code has no license; the speech module is a closed binary. | §3 Humble in Docker on GB10 | Humble-in-Docker everywhere is consistent for now. Plan a Jazzy move later; community ports exist (report 02 §6). Don't copy Hiwonder code into this repo. |
| F11 | **Battery is small.** 2200 mAh gives ~1–1.5 h per charge (*estimate*). The STM32 speed loop should hide most voltage sag. | §1 time gates, §7 battery randomisation | Spare packs keep data collection going. Measure speed vs battery voltage to see whether battery randomisation in the sim is even needed. |
| F12 | **Pi 5 has no hardware H.264 encoder** (the plan already notes this). The depth camera's RGB stream is up to 1080p at 20 fps. | §9.3 compute budget | Stream at 640×480 or lower and profile CPU early, as the plan says. |

## Open items: check on the cars at Phase 0

- `.typerc` values: `MACHINE_TYPE` must be `MentorPi_Acker`, plus `LIDAR_TYPE` and `HOST`.
- Docker setup: run `docker inspect` to see network mode and privileges.
- Steering revision (29° / 40°) and the measured max wheel angle.
- Measured wheelbase, track and wheel diameter.
- Motor gear ratio and encoder CPR.
- Battery connector and real runtime.
- USB topology (which devices sit on which ports).
- RRC Lite firmware version, and whether it stops motors on serial timeout.
- Each car's IP and credentials. Defaults: AP `192.168.149.1`, `pi` / `raspberrypi`.

## What to buy

The kits are complete for Phases 0–5. Small items:
- **Overhead camera** (USB webcam or phone mount) over the test area. Printed **ArUco/AprilTag** markers on the cars give ground-truth poses.
- **Spare 2S LiPo packs** (7.4 V ~2200 mAh; *check the connector on the real pack first*) and a LiPo-safe charging bag.
- **Spare microSD cards**, so each car keeps its stock image untouched.
- **Independent kill switch** (F4): an RF relay module on the motor or battery line, or the LoRa e-stop from plan §11.
- *Pending decision §10.1:* USB Wi-Fi adapters (mt76) with external antennas, plus a **powered USB hub** per car (F8).
