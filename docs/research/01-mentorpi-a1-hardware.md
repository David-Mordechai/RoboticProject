# 01: Hiwonder MentorPi A1 (Ackermann), Advanced kit: hardware

*Research date: 2026-10-30. The robot we own (**2 units**): **Hiwonder MentorPi A1 Advanced with Raspberry Pi 5 8GB** (Amazon [B0FLJRW5XN](https://www.amazon.com/dp/B0FLJRW5XN), listed as "MentorPi A1 Advanced P58"). Software is covered in [02-mentorpi-a1-software.md](02-mentorpi-a1-software.md).*

**Key sources**

| Short name | URL |
|---|---|
| [HW-A1] product page (Shopify JSON read 2026-10-30) | https://www.hiwonder.com/products/mentorpi-a1 |
| [DOCS] current MentorPi docs | https://docs.hiwonder.com/projects/MentorPi/en/latest/ (mirror: wiki.hiwonder.com) |
| [DOCS-2024] 2024 docs | https://docs.hiwonder.com/projects/MentorPi/en/2024-versions/ |
| [GH] Hiwonder GitHub, branch `MentorPi-A1` | https://github.com/Hiwonder/MentorPi/tree/MentorPi-A1 |
| [RRC] RRC Lite controller page | https://www.hiwonder.com/products/rrc-lite |
| [GD] Hiwonder's public Google Drive (linked from the docs) | https://drive.google.com/drive/folders/1P00n6IemouAnga8EjpO7suF3Mou8zv4K |
| [CNX] launch article | https://cnx-software.com/2024/10/23/mentorpi-is-a-ros2-compatible-raspberry-pi-5-based-robot-car-with-mecanum-or-ackermann-chassis |

Local copies of manuals, datasheets and product images are in [`docs/vendor/hiwonder/`](../vendor/hiwonder/). The dimension drawing is [`product-page-images/a1-dimensional-diagram.jpg`](../vendor/hiwonder/product-page-images/a1-dimensional-diagram.jpg).

---

## 0. TL;DR

- **Two computers.** The Pi 5 runs ROS 2 and AI. An **RRC Lite** board (STM32F407) runs the motors, steering servo, IMU, buzzer, LEDs and battery monitor. It talks to the Pi over **USB serial at 1 Mbaud** (`/dev/rrc`).
- **Car-like chassis.** Two rear drive motors with encoders, and one LD-1501MG servo that steers both front wheels in parallel. It **can't turn in place or strafe**. Navigation needs a car-like planner.
- **Real sensors in the box.** A 360° 2D LiDAR, an **Angstrong HP60C** depth camera (RGB-D), the IMU on the RRC Lite, and wheel encoders. The **WonderEcho Pro** voice box (mic and speaker) is the only Advanced-kit extra.
- **The Pi is powered properly.** The RRC Lite supplies **5V/5A over USB-C with PD**, so the Pi 5 gets full USB current.
- **Biggest gap: odometry.** The encoders close the speed loop *inside* the STM32. The stock host protocol **never sends encoder counts to the Pi**, and stock `/odom` is dead-reckoned from the *commanded* velocity. See §4.
- **Check on each car.** Which LiDAR is fitted (the model has changed three times), which steering revision we have (29° or 40°), and the real motor gear ratio.

## 1. Kit variants and box contents

| Variant | Camera | Voice box | Price with Pi 5 8GB [HW-A1] |
|---|---|---|---|
| A1 Starter | Monocular USB cam (GC0308, 640×480, 170°) on a 2-DOF pan-tilt | – | $589.99 |
| A1 Standard | Angstrong HP60C 3D depth camera | – | $729.99 |
| **A1 Advanced (ours)** | HP60C depth camera | **WonderEcho Pro** | $759.99 |

Hiwonder also sells each variant without a Pi or with a 2/4/8/16 GB Pi 5. The M1 is the mecanum sibling and the T1 the tank sibling; they share the same software repo.

**Advanced packing list** [HW-A1], [DOCS 1.getting_ready]:
- Assembled A1 chassis with battery
- Bracket set; controller top, front and rear covers
- Raspberry Pi 5 8GB, 64 GB microSD and card reader
- Hiwonder heatsink and fan for the Pi 5 (not the official Active Cooler)
- Right-angle USB-C to USB-C Pi power cable
- RRC Lite controller and its data cable
- Battery cable
- LiDAR with 4-pin wire and USB adapter cable
- 8.4V 2A charger (DC 5.5×2.5)
- HP60C depth camera and cable
- Wireless gamepad with USB receiver
- 40 mm EVA ball
- 3-pin wire (100 mm)
- WonderEcho Pro and Type-C cable
- Screw bag (M2/M2.5/M3), nylon standoffs, screwdriver
- Manual

*(One packing image labels the bracket "M1 (Mecanum) Chassis". This is probably an image error.)*

## 2. Compute and controller

### Raspberry Pi 5 8GB
- Runs Raspberry Pi OS. All ROS code runs inside a **Ubuntu 22.04 + ROS 2 Humble Docker container** [HW-A1].
- Powered from the RRC Lite's **5V/5A USB-C output**. The RRC Lite's PD chip "supports the Raspberry Pi 5 power supply protocol… up to 5V/5A" [RRC]. The buck converter is an **RT8289GSP** [GD RRC Lite Lesson 2]; the datasheet is in `vendor/hiwonder/datasheets/`.
- That USB-C port carries **power only, no data** [GD Lesson 1].
- The **40-pin GPIO header is free**, because the RRC Lite connects over USB, not over GPIO.

### RRC Lite controller board

| Item | Value | Source |
|---|---|---|
| MCU | STM32F407VET6 (Cortex-M4F, 168 MHz, 512 KB flash, 192 KB RAM) | [RRC], [GD Lesson 1/2] |
| IMU | **QMI8658** 6-axis. Text copied from the older full-size RRC board still says MPU6050 | [GD Lesson 1, Lesson 5] |
| Link to the Pi | USB-C via a **CH9102F** USB-UART (VID:PID `1a86:55d4`) to MCU UART1. udev symlink **`/dev/rrc`**, **1,000,000 baud** | [GD Lesson 2]; [GH] `ros_robot_controller_sdk.py`, `99-ttyACM0.rules` |
| Protocol | `AA 55 · func · len · payload · CRC8 (Maxim)`, little-endian. SYS 0, LED 1, BUZZER 2, MOTOR 3 (rev/s as float), PWM_SERVO 4, BUS_SERVO 5, KEY 6, IMU 7, GAMEPAD 8, SBUS 9, OLED 10, RGB 11 | [GH] SDK; [GD Lesson 13] |
| Input voltage | "6–14 V" [RRC] vs "DC 5–12.6 V" [GD Lesson 1] | Conflict; harmless at 7.4 V |
| Motor driver | 4 encoder-motor ports (PH2.0 6-pin). Chip: SA8870C [RRC] vs YX-4055AM [GD Lesson 2] | Conflict |
| Servos | 4 PWM ports (5–8.4 V; one jumper per pair selects 5 V or VIN); 2 serial bus-servo ports (6–12 V) | [RRC], [GD Lesson 1] |
| I/O | Buzzer, 3 LEDs, 2 WS2812 RGB, 2 user keys, reset, power slide switch, I2C port, GPIO/SWD header | [RRC], [GD Lesson 1/2] |
| Battery monitor | MCU ADC to ROS topic `/ros_robot_controller/battery` (UInt16, mV) | [GH] |
| Size | 85×56×17 mm, 32 g; holes **57.5×48.5 mm**, almost the Pi's 58×49 pattern, so it stacks over the Pi | [RRC] |

**Boot indicators** [DOCS]:
- LED1 turns blue and the buzzer beeps once: the hardware self-check passed.
- LED2 blinking at 1 Hz: Wi-Fi AP mode.
- Long-pressing KEY1: forces AP mode.
- MODE+START on the gamepad: selects Ackermann control.

## 3. Chassis and geometry

| Item | Value | Source |
|---|---|---|
| Overall size (depth-camera version) | **213 L × 159 W × 157 H mm**; top deck at 122 mm | [HW-A1] drawing |
| Weight | ~1.2 kg | [HW-A1] |
| Material | Anodised aluminium alloy | [HW-A1] |
| Wheels | Ø65 × 25 mm (drawing); the code uses **0.067 m** | [HW-A1]; [GH] `ackermann.py` |
| Wheelbase / track (code) | **0.145 m / 0.133 m** | [GH] `ackermann.py`, `odom_publisher_node.py` |
| Wheelbase / track (URDF) | ~0.137 m / ~0.139 m | [GH] `ack.xacro` |
| Steering | One servo and a linkage. **Both front wheels turn by the same angle**, so this is not true Ackermann geometry | [DOCS 4.motion_control §3.2.1] |
| Max steering angle | **29°** on the 2024 build; **40°** on the 2025 build; the GitHub code clamps at 45° | [GD "Ackermann 2025 Version Upgrade Overview"]; [GH] |
| Min turning radius (rear-axle centre, R = L / tan δ) | **≈0.26 m at 29°, ≈0.17 m at 40°** | *Estimate: calculated, not measured* |
| Ground clearance, top speed | **Not published** | – |
| Stock software speed limits | App: 0.2 m/s and 0.5 rad/s. Joystick: 0.7 m/s and 3 rad/s | [GH] |

All dimensions marked *estimate* or "not published" must be measured on a real car before we rely on them (see the `CLAUDE.md` rule).

## 4. Drive and odometry

- **Drive motors.** **2 rear** "310" metal-gear brushed motors with AB-phase Hall quadrature encoders, on RRC ports **M2 and M4**. The code sends `[0, vl, 0, -vr]` [GH], and the wiring image `docs-images/assembly-step10.jpg` shows the same. *(OpenELAB says "four motors"; that is wrong for the A1.)*
- **Probable motor part** (not confirmed for this robot): Hiwonder's standalone **JGA27-310R20** [hiwonder.com/products/310-dc-gear-encoder-motor]:
  - 7.4 V, 1:20 gearbox, 450 rpm no-load, 0.65 A rated, ≤1.4 A stall
  - 13-line magnetic encoder, giving **13×4×20 = 1040 counts per wheel turn** *(calculated)*
  - About **1.5 m/s** no-load top speed on a 67 mm wheel *(calculated)*
- **Steering servo: LD-1501MG** [HW-A1], [GD manual]:

  | Spec | Value |
  |---|---|
  | Signal | PWM, 500–2500 µs = 0–180°, 20 ms period |
  | Supply | 6–8.4 V |
  | Torque | 13/15/17 kg·cm at 6/6.5/7.4 V |
  | Speed | 0.16 s/60° |
  | Current | 2.4–3 A stall |
  | Size, weight | 54.4×20×45.5 mm, 61 g |
  | Wiring | RRC port J5. Software uses PWM servo id 3, 1500 µs centre, ~11.1 µs/° |

  STEP and DXF drawings are in `vendor/hiwonder/cad/`. Per-unit trim is set with the servo-deviation tool (`./Servo.sh`).
- **⚠ Stock odometry is not real odometry.** `odom_publisher_node.py` integrates the **commanded** v/ω at 50 Hz. The RRC host protocol has **no encoder-count or wheel-speed upload**; it uploads only battery, keys, IMU, gamepad, SBUS and servo readback. The encoders close only the STM32's internal speed PID. The EKF partly rescues this by fusing the IMU and rf2o laser odometry.

  Our options, in order of effort:
  1. Better fusion: LiDAR scan-matching plus IMU, or visual odometry from the depth camera.
  2. Custom RRC firmware that reports encoder ticks, after confirming the firmware source is available.
  3. A Pico reading the encoders in parallel.

## 5. Sensors

### LiDAR: check which one each car has
| Period | Model | Source |
|---|---|---|
| Launch, Oct 2024 | **LDROBOT STL-19P / LD19** (12 m, 230400 baud, 4500 Hz, 5–13 Hz scan) | Product page (Wayback 2024-10), [CNX], [DOCS-2024] |
| 2025 docs and product spec tab | **Oradar MS200** (TOF, 0.1–12 m, 0.8°, 4500 Hz, 7–15 Hz scan, UART, 5 V, ~0.3 A, 40 g) | [DOCS] LiDAR lesson, [HW-A1] spec tab |
| Product page text, Oct 2026 | **"COIN-D6"** (reseller specs: 0.05–12 m, 10 Hz, ~4000 samples/s, 230400 baud) | [HW-A1] |

- Stock code (`peripherals/launch/lidar.launch.py`) supports only `LIDAR_TYPE` = `MS200` or `LD19`. **No COIN-D6 driver was found.**
- All three LiDARs connect through a 4-pin UART cable and a USB adapter (udev `/dev/ldlidar` / `/dev/lidar`), on a plate at the rear of the top deck.
- **Action:** read the label on each of our LiDARs and the `LIDAR_TYPE` in `.typerc`.
- The LD19 manual is in `vendor/hiwonder/datasheets/`. No public datasheet for the MS200 or COIN-D6 was found.

### Depth camera: Angstrong Nuwa-HP60C (binocular structured light)
| Spec | Value |
|---|---|
| Depth | 0.2–4 m (best below 2 m), <2 mm error at 1 m; 640×480 @ ≤20 fps; FOV 73.8°×58.8° |
| RGB | 1920×1080 @ ≤20 fps (640×480 recommended); FOV 80.9°×51.7° |
| Interface / power | USB 2.0 Type-C, 5 V, <2 W |
| Size | 89.8×19×25 mm |
| Notes | Class 1 laser, indoor only |
| ROS driver | `ascamera` (config value `ascamera`) |

Sources: [HW-A1], [hiwonder.com/products/ros-real-sense-depth-camera](https://www.hiwonder.com/products/ros-real-sense-depth-camera). No public Angstrong datasheet was found.

### WonderEcho Pro (Advanced kit only)
- USB 2.0, driver-free audio: noise-cancelling mic and a 3 W speaker.
- A **CI1302** offline wake-word chip also appears as a CH340 serial port (`/dev/ttyUSB0`, 115200 baud).
- The wake word ("hello hiwonder", "mentorpi") can be reflashed with ChipIntelli tools [DOCS 15], [GD].

### Other
- 2.4 GHz gamepad (ROS `/joy`).
- No ultrasonic sensor, OLED or line sensor ships, although the RRC Lite has an I2C port and an OLED protocol.

## 6. Power

- **Battery:** 7.4 V (2S) **2200 mAh LiPo**, about 16 Wh, with a built-in protection board, >300 cycles. Rated **10C** [HW-A1]; [CNX] says 20C.
- **Charger:** 8.4 V 2 A via a DC 5.5×2.5 plug. The LED is red while charging and green when full; a charge takes about 1 h [DOCS].
- **Runtime:** not published. *Estimate:* **~1–1.5 h**, assuming Pi 5 + LiDAR + depth camera ≈ 8–12 W plus the motors. Measure it.
- **Low voltage:** the docs only say "charge below 6.4 V". No auto-cutoff or alarm is documented; the protection board is the hard cutoff. Watch `/ros_robot_controller/battery`.
- **Fleet note:** one charger per car. For back-to-back sessions, spare 2S 2200 mAh packs with the same connector help. Check the connector type on the real pack first.

## 7. Expansion

| Where | What's free |
|---|---|
| RRC Lite | Motor ports M1/M3; 3 PWM servo ports; 2 bus-servo ports; I2C; GPIO/SWD |
| Pi 5 | Full 40-pin GPIO; the PCIe FFC (AI HAT+ or NVMe, if it fits) |
| USB | The RRC, LiDAR, depth camera, gamepad dongle and WonderEcho add up to **5 devices against 4 USB-A ports**. Check how Hiwonder wires this; one may go through the RRC or a hub |
| Top deck | Slot and hole patterns. **No hole drawing is published; measure it** |

## 8. Conflicts and open questions (check on the bench)

1. **LiDAR model:** LD19/STL-19P → MS200 → COIN-D6 (§5).
2. **Steering angle:** 29° (2024), 40° (2025), or the 45° code clamp. Is "servo angle" the same as wheel angle?
3. **Geometry:** wheel 65 mm (drawing) vs 67 mm (code); wheelbase/track 0.145/0.133 m (code) vs ~0.137/0.139 m (URDF).
4. **Motor:** gear ratio and encoder CPR are not published for the A1.
5. **Battery C rating:** 10C vs 20C. Connector type?
6. **RRC Lite:** input range, motor-driver chip, IMU (QMI8658 vs MPU6050 in copied text).
7. **Not published:** ground clearance, top speed, runtime, low-voltage cutoff, top-deck hole pattern, USB topology.
