# Shopping list: 3 robots

*Compiled 2026-10-27. Prices are approximate single-unit USD and change often; re-check before ordering. Links go to the product, or to a search where many equivalent sellers exist.*

## A. Before the kits arrive (essentials)

| # | Item | Qty | ~Price | Link | Notes |
|---|---|---|---|---|---|
| 1 | **18650 Li-ion cells, high-drain**: Molicel P28A (2800 mAh, 35 A) | **12** (2 per robot + 2 spare packs) | $6 each | [18650batterystore: Molicel P28A](https://www.18650batterystore.com/products/molicel-p28a) | Buy from an authorised seller; fakes are common on Amazon/AliExpress. **Flat-top** fits most holders; check yours. Samsung 30Q / Sony VTC6 are fine alternatives. Charge and use them as **matched pairs**. |
| 2 | **4-bay smart charger**: XTAR VC4SL | 1 | $30 | [Battery Junction: XTAR VC4SL](https://www.batteryjunction.com/xtar-vc4sl) | Shows the voltage of each cell, USB-C. Needs a 5V/3A USB-C supply (not included). Nitecore D4 is an alternative. |
| 3 | **Raspberry Pi 5 Active Cooler** (official) | 3 | $5 | [raspberrypi.com: Active Cooler](https://www.raspberrypi.com/products/active-cooler/) (lists approved resellers) · [SparkFun](https://www.sparkfun.com/raspberry-pi-active-cooler.html) · [CanaKit](https://www.canakit.com/raspberry-pi-5-active-cooler.html) | Fits under the Robot HAT (checked in CAD). Don't remove it once fitted. |
| 4 | **Pi 5 camera cable, 22-pin to 15-pin, 200 mm** | 3–6 | $2–3 | [Adafruit 5818 (200 mm)](https://www.adafruit.com/product/5818) · [Adafruit 5819 (300 mm)](https://www.adafruit.com/product/5819) | The kit probably includes one; spares are cheap. It must be the **camera** cable, not the display cable. |
| 5 | **MPU6050 IMU module (GY-521)** for the HAT's empty IMU socket | 3 (+1 spare) | $2–4 | [Amazon search: GY-521 MPU6050](https://www.amazon.com/s?k=GY-521+MPU6050) | Gives a gyro right away (heading for the EKF). **Compare the socket pin order on the HAT with the module before plugging in.** |
| 6 | **M2.5 brass standoff assortment** (male-female, 6–20 mm, with screws and nuts) | 1 kit | $10 | [Amazon search: M2.5 brass standoff kit](https://www.amazon.com/s?k=M2.5+brass+standoff+assortment+kit) | For the LiDAR mount: 4x **15 mm**, plus 10/20 mm to fine-tune. |
| 7 | **M2.5 x 4 mm screws** | 1 bag | $5 | [Amazon search: M2.5x4 screws](https://www.amazon.com/s?k=M2.5x4mm+screws) | Screws into the LiDAR base must be **≤ 4 mm** (Slamtec warning). |
| 8 | **Digital multimeter**, auto-ranging | 1 | $20–40 | [Amazon search: auto-ranging multimeter](https://www.amazon.com/s?k=auto+ranging+digital+multimeter) | For the power test, battery voltages and continuity. |
| 9 | **USB-C power meter**: FNIRSI FNB58 | 1 | $25–35 | [Core Electronics: FNB58](https://core-electronics.com.au/fnirsi-fnb58-usb-fast-charge-tester.html) · [Elektor (Bluetooth)](https://www.elektor.com/products/fnirsi-fnb58-usb-tester-with-bluetooth) | Measures what the Pi 5 really draws on the bench. |
| 10 | **USB-C PD power supply 5V/5A** (official Raspberry Pi 27W) | 1–2 | $12 | [raspberrypi.com: 27W USB-C PSU](https://www.raspberrypi.com/products/27w-power-supply/) | For bench setup without batteries. You already have one from the Jetson; 5A is still better for the Pi 5. |

**Section A total: about $230–290**

## B. Phase 1–2: once one robot runs

| # | Item | Qty | ~Price | Link | Notes |
|---|---|---|---|---|---|
| 11 | **2D LiDAR: SLAMTEC RPLIDAR C1** (recommended) | 3 (start with 1) | $80 | [Waveshare: RPLIDAR C1](https://www.waveshare.com/rplidar-c1.htm) · [Slamtec specs](https://www.slamtec.com/en/C1) | 12 m, IP54, 110 g; our CAD model and mount (15 mm standoffs) are based on this one. Comes with a USB adapter. |
| 11b | *Alternative:* LDROBOT LD19 (D300 kit) | 3 | $99 | [RobotShop: Hiwonder LD19 D300 kit](https://www.robotshop.com/products/hiwonder-ld19-d300-lidar-developer-kit-360-dtof-laser-scanner-supports-ros1-2-raspberry-pi-jetson-nano) | Waveshare discontinued its LD19 kit and recommends the D500 instead. If you choose this one, the mount height must be recalculated. |
| 12 | **5V/5.5A buck regulator: Pololu D36V50F5** | 3 | $35 | [Pololu 4091](https://www.pololu.com/product/4091) | **Only if the power test fails** (Pi resets, `get_throttled` ≠ 0). Gives the Pi its own power supply. |
| 13 | **Better IMU: Adafruit BNO085** (9-DoF with on-chip fusion) | 3 | $25 | [Adafruit 4754](https://www.adafruit.com/product/4754) | Upgrade if the MPU6050's heading drift is a problem. |
| 14 | **USB webcam (1080p) + printed AprilTags** | 1 | $25–40 | [Amazon search: 1080p USB webcam](https://www.amazon.com/s?k=1080p+usb+webcam) | Mounted over the test area as a "ground truth" camera, to measure how accurate the robots really are. |

## C. Later upgrades

| # | Item | Qty | ~Price | Link | Notes |
|---|---|---|---|---|---|
| 15 | **TT motor with encoder**: DFRobot FIT0450 | 12 (4 per robot) | £7 | [The Pi Hut: FIT0450](https://thepihut.com/products/tt-motor-with-encoder-6v-160rpm-120-1) · [DFRobot wiki](https://wiki.dfrobot.com/fit0450/) | Wheel odometry. Same shaft as the kit motors. |
| 16 | **Raspberry Pi Pico 2** (reads the encoders and runs the motor control loop) | 3 | $5 | [raspberrypi.com: products (Pico 2)](https://www.raspberrypi.com/products/) | Talks to the Pi 5 over UART or micro-ROS. |
| 17 | **VL53L1X ToF distance sensors** | 3–6 | $12–15 | [Amazon search: VL53L1X breakout](https://www.amazon.com/s?k=VL53L1X+breakout) | Close-range obstacles and glass the LiDAR misses. |
| 18 | **Raspberry Pi AI HAT+ (Hailo-8, 26 TOPS)** | 1 to try | $110 | [raspberrypi.com: AI HAT+](https://www.raspberrypi.com/products/ai-hat/) | Only if CPU vision (~15 FPS YOLO) is too slow. Needs a stacking header and taller standoffs under the Robot HAT. |

## Not needed
- **Brass/nylon standoffs for the robot itself**: all included in the kit (M2.5 x 6 and x 21, M3 x 8, x 12 and x 30, M2 x 11, nylon M3 x 50). See the Adeept packing list.
- **SD cards and Pi 5 boards**: already ordered.

## Safety notes for the 18650s
- Buy from authorised sellers only. Never mix old and new cells, or different brands, in one robot.
- Charge in the external charger, never unattended, on a non-flammable surface.
- Take the cells out of the robot for storage. Recycle any cell that drops below 2.5 V.
