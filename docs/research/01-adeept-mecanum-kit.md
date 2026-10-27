# 01: Adeept 4WD Mecanum Wheel Smart Car Kit (ADR036) on Raspberry Pi 5

*Research date: 2026-10-27. Scope: everything practically useful about the kit the project is using (3 units, each on a Raspberry Pi 5 8 GB, no LiDAR yet). Primary sources: Adeept's official download package `ADR036-Adeept_4WD_Smart_Car_Kit_for_RPi-20260317.zip` (code, 30+ tutorial PDFs, bundled datasheets), the official GitHub repo at commit `11a4ac7` (2026-09-22), retailer listings, chip datasheets, Raspberry Pi and gpiozero docs, and GitHub issues and community repos for sibling Adeept kits that use the same Robot HAT.*

> **How to read this document**
> - **[VERIFIED]**: confirmed from a primary source (Adeept code or tutorial, a datasheet, or official docs). The source is linked or named.
> - **[INFERRED]**: my deduction from verified facts, such as reading the code or applying general electronics knowledge. It is probably right, but nobody states it outright.
> - **[UNVERIFIED]**: retailer or third-party claims I could not cross-check, or open questions to settle on the bench.
> - "Tutorial Lesson N" means the PDF of that name inside the official zip (see §11). Code paths are relative to the repo root of `adeept/Adeept_4WD_Smart_Car_for_RPi`.

---

## 0. TL;DR: the 12 things that matter most

1. **The kit is Adeept item code ADR036**, mecanum version. The official repo is **https://github.com/adeept/Adeept_4WD_Smart_Car_for_RPi**. It has **no open-source license**, and Adeept has declined to add one on sibling repos (§5.9). **[VERIFIED]**
2. **The GitHub repo is newer than the zip on Adeept's site.** The zip is dated 2026-03-17. GitHub got a Pi 5 / new-kernel GPIO fix on 2026-09-22. **Clone from GitHub.** **[VERIFIED]**
3. **The software stack is already Pi 5-friendly in design.** It uses gpiozero (lgpio backend), Adafruit Blinka/CircuitPython for I2C, `spidev` for WS2812, and **Picamera2**. No code imports `RPi.GPIO`, `pigpio` or `rpi_ws281x`. **[VERIFIED by grep]** Adeept's own Lesson 1, though, still says "We recommend using Raspberry Pi 4, 3B+, 3B". **[VERIFIED]**
4. **Robot HAT V3.2 chips:** **PCA9685** at I2C **0x5F** (16 PWM channels: servos on 0–7, motors on 8–15), **2x DRV8833** class dual H-bridges fed by PCA9685 PWM (inferred from the bundled datasheet and code), **ADS7830** 8-bit ADC at **0x48** (A0 = battery, A1 = light sensor, A2–A7 free), **SSD1306 OLED at 0x3C**, **HT16K33 16x8 matrix at 0x70**, and an optional **MPU6050 at 0x68** (port present, module **not included**). **[VERIFIED, except the DRV8833 count]**
5. **There are no wheel encoders and no IMU.** Motors are open-loop 2-wire TT motors. Wheel odometry is impossible without hardware changes. **[VERIFIED]**
6. **Motor PWM runs at 50 Hz**, because motors share the PCA9685 with the servos. That is crude for DC motors: poor low-speed control and audible whine. **[VERIFIED in code; the impact is INFERRED]**
7. **Mecanum control in the stock code is 10 discrete moves**, not continuous `(vx, vy, ω)` mixing. The wheel sign pattern matches the standard X-configuration inverse kinematics, so a proper mixer is easy to add (§4). **[VERIFIED]**
8. **Pi 5 power is the main hardware risk.** The HAT powers the Pi through the GPIO 5V pins from 2x 18650 cells (2S, 8.4 V max). Adeept measured a **3.75 A peak at 7.4 V for the whole robot**. The 5V regulator's current rating is **not published**. A retailer names a GS2678 (5 A-class buck), which is unverified for V3.2. When powered over GPIO, the Pi 5 does no USB-PD negotiation, so it assumes a 3 A supply and **limits USB peripherals to 600 mA** unless configured otherwise (§2.5). **[mixed; see §2]**
9. **The camera interface is CSI via Picamera2** at 640x480. A Pi 5 needs a **22-pin to 15-pin cable**. The tutorial implies a Pi 5 cable is supplied, but the packing list is ambiguous. **Buy spares (about $2–3 each).** The sensor model is not stated; it is probably an OV5647 5 MP. **[mixed]**
10. **The ultrasonic sensor and camera sit together on the 2-servo pan/tilt head.** "Radar scan" sweeps the pan servo ±90° and returns points closer than 50 cm. That is a useful poor-man's scanner, but **it would occlude a top-mounted LiDAR** unless the LiDAR is raised (§7). **[VERIFIED from code and assembly lesson]**
11. **Fleet gotchas for 3 robots:** every robot uses the same fallback hotspot SSID `Adeept_Robot` / password `12345678`, the same hard-coded WebSocket credentials `admin:123456`, and runs everything as **root** via a systemd autostart. Change all of these (§8). **[VERIFIED]**
12. **I2C address quirk:** the PCA9685's power-on "LED All Call" address is **0x70**, the same as the HT16K33 LED matrix. The NXP datasheet says this address "must not be used as a regular I2C-bus slave address". The Adafruit library disables All Call when it initialises the PCA9685, so **initialise the PCA9685 before talking to the matrix** in your own code (§2.3). **[VERIFIED datasheet; practical impact INFERRED]**

---

## 1. Exact identification

| Item | Value | Status / source |
|---|---|---|
| Product name (official) | "Adeept 4WD Smart Car Kit for Raspberry Pi", sold in two versions: **Mecanum wheel** and Ordinary wheel | [VERIFIED] Tutorial "Introduction to 4WD Smart Car.pdf"; [repo README](https://github.com/adeept/Adeept_4WD_Smart_Car_for_RPi) |
| Item code | **ADR036** | [VERIFIED] tutorial intro and repo README ("[Item Code]: ADR036") |
| Retail SKU (mecanum) | `ADR036-4WD-Mecanum Wheel` | [UNVERIFIED, one retailer] [OzRobotics listing](https://ozrobotics.com/shop/smart-robot-car-kit-with-4wd-mecanum-wheel-for-raspberry-pi-5-4b-3b-3b-board-not-included/) |
| Retail listings | [RobotShop UK](https://uk.robotshop.com/products/adeept-4wd-mecanum-wheel-smart-car-kit-raspberry-pi-5-4b-3b-w-o-rpi), [Tindie (Adeept store), $74.99](https://www.tindie.com/products/adeept/adeept-4wd-mecanum-wheel-smart-car-kit-2/) | [VERIFIED] (RobotShop blocks automated fetch; its content was seen through search snippets) |
| Official downloads | [adeept.com/learn/detail-97.html](https://www.adeept.com/learn/detail-97.html) gives a ZIP: [ADR036-Adeept_4WD_Smart_Car_Kit_for_RPi-20260317.zip](https://www.adeept.com/video/static1/itemsfile/ADR036-Adeept_4WD_Smart_Car_Kit_for_RPi-20260317.zip) (about 66 MB: code, Android APK, tutorial PDFs, datasheets) | [VERIFIED] downloaded and inspected |
| Official code | https://github.com/adeept/Adeept_4WD_Smart_Car_for_RPi (created 2025-07-24, last push 2026-09-22, 3 stars, no license) | [VERIFIED] GitHub API |
| Kit versions | Mecanum and Ordinary-wheel versions share the code base. Both use separate `setup_MecanumWheels.py` / `setup_OrdinaryWheels.py` scripts and `Server_MecanumWheels/` / `Server_OrdinaryWheels/` folders. The HAT revision is **V3.2**. Older Adeept kits used HAT V3.0 (WS2812 via `rpi_ws281x`, not Pi 5-capable) and V3.1 (moved WS2812 to SPI). | [VERIFIED] code; Adeept reply on [rasptank2#3](https://github.com/adeept/adeept_rasptank2/issues/3) |
| **Don't confuse with** | "Adeept 4WD **RC** Mecanum Robotic Car Kit for Raspberry Pi **Pico**" (MicroPython, ESP8266, LCD1602). That is a different product. | [VERIFIED] [RobotShop](https://uk.robotshop.com/products/adeept-4wd-rc-mecanum-robotic-car-kit-raspberry-pi-pico-w-lcd1602) |
| Dimensions / weight | **Not published anywhere I could find.** Measure on arrival (you need wheelbase and track width for kinematics, §4). | [UNVERIFIED] |
| Wheel diameter, TT gear ratio | Not published. | [UNVERIFIED] |
| Tutorials | Retailers say "370 pages, 30 lessons", no paper copy, and the download link is on a card in the box. | [VERIFIED] Tindie; zip contents |

### 1.1 What's in the box (official packing list, mecanum version)

From `Tutorial/Packing List for Mecanum Wheels.pdf` **[VERIFIED]**:

- **Electronics:** Adeept Robot HAT V3.2; OLED screen; camera module; ultrasonic module; 3-CH line tracking module; light tracking module; I2C matrix display module (8x16); WS2812 RGB LED module; 18650 battery holder (2-cell).
- **Drive:** 4x DC (TT) motor + 4x motor mounting bracket; 2x servo (AD002 per retailers); 2x Mecanum Wheel-L + 2x Mecanum Wheel-R; 4x coupling.
- **Cables:** 5-pin line tracking (18 cm); 4-pin OLED (15 cm); 4-pin ultrasonic (15 cm); 4-pin I2C matrix (18 cm); 3-pin light tracking (19 cm); 3-pin WS2812 (20 cm); one FFC labelled "AWM 20624 80C 60V VW-1"; one "HBV-Raspberry-150FPC" camera FFC.
- **Hardware:** M1.6/M2/M2.5/M3 screws and nuts, copper standoffs (including **4x M2.5x21** for the HAT, 8x M3x30, and others), 4x M3x50 nylon standoffs, acrylic plates A01–A07.
- **Tools:** cross socket wrench, 2 Phillips screwdrivers, USB-C cable, ribbon strip for battery removal, cable wrap.
- **Not included:** Raspberry Pi, **18650 cells**, IR remote (the HAT has an IR receiver but the kit lists no remote), MPU6050 module.

**Camera cable for Pi 5 [UNVERIFIED].** Tutorial Lesson 3 says: *"The picture below uses a Raspberry Pi 5, so you need to use the corresponding Raspberry Pi 5 camera cable. If you are using a Raspberry Pi 3B, 3B+, 4B. You will need to use the black camera cable. Black ribbon cable is included in the kit."* The packing list shows two FFCs, so **one of them is probably the Pi 5 22-pin to 15-pin cable**, but nothing says so explicitly. A Pi 5 RaspTank2 buyer reported "the raspberry pie 5 camera cables are supplied" with a recent Adeept kit ([rasptank2#3](https://github.com/adeept/adeept_rasptank2/issues/3)). Either way, order spares. They are $1.89–$3.16 ([Adafruit 5818 200 mm](https://www.adafruit.com/product/5818), [Adafruit 5819 300 mm](https://www.adafruit.com/product/5819), [Waveshare](https://waveshare.com/pi5-camera-cable.htm?sku=25945)).

### 1.2 Mechanical layout (from Tutorial Lesson 6)

**[VERIFIED from the assembly lesson]**
- **A01 (base plate):** 18650 holder (cable through a square slot), OLED on standoffs, 8x M3x30 standoffs, 2x M3x12, and 4 nylon M3x50 standoffs (these appear to support the top plate A07).
- **A02 (motor deck):** 4 TT motors on brackets. *"The colors of the motor wires on the same side should be consistent, while those on the opposite side should be opposite."* The WS2812 module, line tracker, light tracker and LED matrix (on plate A06) also mount here.
- **Head (A03–A05):** a pan servo and a tilt servo carry **both the camera and the ultrasonic module** on plate A05.
- **The HAT sits on the Pi on M2.5x21 standoffs.** That tall spacing probably leaves room for a Pi 5 Active Cooler, but this is **[INFERRED]**; check clearance.
- **Wheels:** **O-rectangle mounting.** Rollers form an "X" seen from above and an "O" seen from below. Wheel-L and Wheel-R must go in the right corners.
- **Payload room [INFERRED]:** the top plate A07 is the only flat mounting area. A 2D LiDAR (for example the RPLIDAR C1, 110 g) would need a raised mast or a 3D-printed bracket so it scans above the pan/tilt head. Weigh the robot and check the motors' headroom after adding the Pi 5, cooler and LiDAR.

---

## 2. Adeept Robot HAT V3.2

### 2.1 Chips and functions

| Function | Part | I2C addr / pins | Status / source |
|---|---|---|---|
| 16-ch PWM (servos + motor H-bridge inputs) | **NXP PCA9685** | **0x5F** (not the 0x40 default) | [VERIFIED] code (`PCA9685(i2c, address=0x5f)`), Lesson 9 ("The I2C address of Adeept Robot HAT V3.2 is 0x5f"), PCA9685 datasheet in zip |
| DC motor drivers (M1–M4) | **TI DRV8833** dual H-bridge, so **2 chips** for 4 motors | driven by PCA9685 ch 8–15 | [INFERRED] the zip bundles `DRV8833.PDF`, and the code drives each motor with 2 PCA9685 channels (IN1/IN2) using `adafruit_motor.DCMotor` slow-decay mode, which fits DRV8833 IN/IN control. Silkscreen not checked. |
| DRV8833 limits | 2.7–10.8 V motor supply; 1.5 A RMS / 2 A peak per H-bridge (HTSSOP/WSON). The TSSOP "PW" variant is only 0.5 A RMS. | | [VERIFIED] [TI DRV8833 datasheet](https://www.ti.com/lit/ds/symlink/drv8833.pdf) (bundled) |
| 8-ch 8-bit ADC | **TI ADS7830** | **0x48**. A0 = battery divider, A1 = light-tracking module, **A2–A7 on a free header** | [VERIFIED] Lessons 14 and 19, code |
| Op-amp | **LM324** datasheet bundled; its location is unclear (possibly comparators on the line-tracking module or HAT) | | [UNVERIFIED] |
| 5V DC-DC (powers the Pi via GPIO 5V) | **Not documented by Adeept.** One retailer listing for an Adeept Robot HAT says the buck is a **GS2678** (datasheet: 5 A, 350 kHz, 3.6–25 V input). Board-level continuous capability is unknown (inductor, thermals). | [UNVERIFIED] [search result citing Newegg](https://www.neweggbusiness.com/p/9B-2RU-03NE-000G5); [GS2678 datasheet](https://static.chipdip.ru/lib/273/DOC029273893.pdf). CNX notes Adeept "lacks all the specifications for the various ICs" ([CNX](https://www.cnx-software.com/2024/07/29/adeept-robot-hat-for-raspberry-pi-is-designed-for-diy-projects-and-educational-needs/)) |
| 2S Li-ion charger (USB-C) | Unnamed "charging control chip". Charge current is deliberately limited, so charging takes **about 2–3 h**. Red LED = charging, green = full or no battery. | [VERIFIED] Lessons 30/31 |
| Battery gauge LEDs | 4 LEDs at about 25% each; the last one is red | [VERIFIED] Lesson 2 |
| Onboard extras | Passive buzzer, IR receiver, 2 WS2812 LEDs plus an extension port, 3 "switch" LEDs (LED1–3), power switch, power LED | [VERIFIED] Lesson 2 |
| Headers | 16 servo headers (0–15), M1–M4 motor terminals, X1/X2 I2C, X3 light tracking, X4/X5 RGB (common-anode LEDs), X6/X7 UART, X8 line tracking, X9 ultrasonic, MPU6050 socket, 40-pin pass-through | [VERIFIED] Lesson 2 |

### 2.1a Physical layout: every part on the top, the underside and the edges

Sources:
- Adeept's labelled product photo of both sides, `docs/vendor/adeept/photos/robot-hat-v3.3-top-and-bottom-labeled.jpg`, from [adeept.com p0429](http://www.adeept.com/adeept-robot-hat-for-raspberry-picompatible-with-pi-5-4-smart-robot-car-driver_p0429.html). The photo shows **V3.3**.
- Adeept's V3.2 back-side drawing (Lesson 6 p.32). It shows the same layout, but some parts differ in detail. Confirm on the real board.

Pi-local coordinates as used in `hardware/cad/robot_model.py`: u runs along the 65 mm length (u = 0 is the GPIO pin-1 end), and v runs across the board.

| Side | Part | Where | Height (EST) | Matters for |
|---|---|---|---|---|
| Underside | Tall 2x20 GPIO socket | Along the GPIO long edge | Reaches down to the Pi pins | Connects to the Pi; doesn't overlap the cooler |
| Underside | **4x white motor connectors M1–M4** (side-entry) | Short edge at the pin-1 end (u ≈ 0–9) | ~6 mm (5–7.5) | **Tightest spot over a cooler push-pin: ~3.3 mm** |
| Underside | **DC-DC power block**: 2x XL4015 buck regulators (TO-263), 2x "470" power inductors, 2x diodes | Short edge at the USB-C end (u ≈ 44–65), directly above the cooler's **blower/fan** | Regulators ~4.5, inductors ~5.5, diodes ~2.4 | ~5 mm above the fan. The fan draws air from just under the warm regulators |
| Underside | PCA9685, 2x DRV8833, ADS7830, small SOIC chips, resistors | Middle | ~1–1.5 mm | Within the 2 mm allowance |
| Through board | Long narrow slot (camera ribbon pass-through) | Middle, near the servo headers | — | Route the camera FFC through it |
| Top | 40-pin pass-through header, X1–X9 JST sockets, 16 servo headers, buzzer, IR receiver, 2 WS2812 LEDs, 2x 470 µF caps, 4R7 inductor, battery-level LEDs, MPU6050 socket | — | Tallest are the 470 µF caps, ~12.5 mm | Clears top plate A07 (HAT top parts ≈ 42.7 mm vs A07 at 50 mm) |
| Edge | USB-C (charge), VIN battery JST, **ON/OFF slide switch lever sticking out past the board edge** | USB-C end | — | Keep the switch reachable; don't block it with a mast |

*Version note:* V3.3 uses two XL4015 (5 A-class) buck regulators. One retailer lists a GS2678 for "V3.x". Check which regulator your V3.2 boards actually have; it matters for the Pi 5 power question in §2.5.

### 2.2 GPIO map (BCM numbering)

All entries are **[VERIFIED from code]** unless marked otherwise.

| GPIO | Use | Code |
|---|---|---|
| 2 / 3 | I2C1 SDA/SCL (PCA9685, ADS7830, OLED, matrix, optional MPU6050) | `board.SCL/SDA`, `smbus.SMBus(1)` |
| 10 | **WS2812 data via SPI0 MOSI** (`/dev/spidev0.0`, GRB, bit patterns encoded into SPI bytes) | `RobotLight.py` |
| 9, 11 | LED1 and LED3 "switch" outputs. **These collide with SPI0 MISO and SCLK.** See §5.4. | `Switch.py` |
| 8 (7) | SPI0 CE0 (CE1), claimed by the SPI driver when SPI is enabled [INFERRED] | |
| 25 | LED2 switch output | `Switch.py` |
| 12 | Onboard IR receiver (input) | HAT example `10_IR_Receiver.py` ([repo](https://github.com/adeept/Adeept_Robot_HAT-V3)) |
| 18 | Passive buzzer (`TonalBuzzer`) | `Buzzer.py` |
| 23 / 24 | Ultrasonic Trig / Echo | `Ultra.py` |
| 17 / 27 / 22 | Line tracker S1 (right) / S2 (middle) / S3 (left) | `Functions.py`, Lesson 13 |
| 13, 19, 0 / 1, 5, 6 | RGB ports X4/X5 (left R,G,B / right R,G,B). **Not used by this kit**, which uses WS2812 instead. GPIO 0/1 are the HAT-EEPROM ID pins. | HAT example `11_RGB.py` |
| 14 / 15 | UART on X6/X7 | Lesson 2 |
| **Probably free** | **4, 16, 20, 21, 26**, plus 5, 6, 13, 19 (and 0/1) if X4/X5 are unused | [INFERRED] Check with `pinctrl` / `gpioinfo` on the real board before wiring encoders. |

**PCA9685 channel map [VERIFIED]:** servo 0 = **pan** (`lookleft/lookright`, radar scan); servo 1 = **tilt** (`up/down`); channels 2–7 are free for more servos. Motors: **M1 = ch15/14, M2 = ch12/13, M3 = ch11/10, M4 = ch8/9**. Lesson 9: *"pins 0–7 are solely for servo use; pins 8–15 can be used for both the motor and the servo."*

### 2.3 I2C address table and the 0x70 quirk

`i2cdetect -y 1` should show **0x3C** (OLED), **0x48** (ADS7830), **0x5F** (PCA9685), **0x70** (HT16K33 matrix), and **0x68** only if you add an MPU6050.

**[VERIFIED]** The [PCA9685 datasheet](https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf) §7.1.2 says: *"At power-up, LED All Call I2C-bus address is enabled... default... E0h (1110 000X)"*, which is 7-bit **0x70**, and *"must not be used as a regular I2C-bus slave address."* So until the PCA9685 is initialised, a write meant for the matrix at 0x70 is also accepted by the PCA9685, which interprets the bytes as register writes. **[INFERRED]** The Adafruit `PCA9685` driver writes `MODE1=0x00` in `reset()`, which disables All Call. Adeept's code works in practice, but **any custom driver should initialise the PCA9685 (or clear MODE1 bit 0) before touching 0x70**. Expect `i2cdetect` at power-on to show 0x70 even without the matrix connected (compare the i2cdetect dump in [RaspClaws#4](https://github.com/adeept/Adeept_RaspClaws/issues/4), which shows 0x40 and 0x70 for a PCA9685-only bus).

### 2.4 Battery monitoring

**[VERIFIED]** ADS7830 channel 0 reads the pack through a **3 kΩ / 1 kΩ divider (÷4)**. Code: `V = adc/255 × Vref / 0.25`. `Voltage.py` uses Vref = **5.2 V**, while the example script uses **5.0 V**: about a 4% inconsistency. The command byte `0x84` selects single-ended mode with the internal reference **off**, so the reference is the board's supply rail [INFERRED from the ADS7830 datasheet]. Readings therefore track 5V-rail accuracy. Resolution is about **78 mV per LSB at the pack**. The server warns below **6.0 V** (3.0 V/cell) with a buzzer beep and flashing LEDs, and displays 0–100% linearly between 6.0 and 8.4 V. That is a crude gauge: Li-ion discharge is not linear.

### 2.5 Power path, and is it enough for a Pi 5?

What is **[VERIFIED]** (Lessons 2, 3, 5, 30, 31):
- Power comes from **2x 18650 in series (2S, 7.4 V nominal, 8.4 V full)** into Vin. *"Robot HAT can be supplied by a power source under 15V."*
- *"We've tested when powering with 7.4V, the peak current of the robot would be around 3.75A, which means you need to connect batteries with a 4A output."*
- The HAT's **USB-C** charges the pack and can also power the Pi through GPIO. *"When USB port on the Robot HAT is used as power supply, the switch of Robot HAT does not control whether to power on or off."* *"Do not use both the USB port and Vin ... at the same time."* Lesson 31: *"When charging, ensure the power switch of the expansion board is turned off."*
- Lesson 3: *"If you are using a Raspberry Pi 5 or have connected many sensors to the expansion board, powering via a Type-C cable may cause insufficient power supply. Please use two 3.7V 18650 batteries."*
- Brownout symptom documented by Adeept: the robot reboots at boot or when it starts moving, because servos homing at startup pull the rail down.
- Real-world report: a Pi 5 on the sibling RaspTank2 (HAT V3.1) **rebooted every time a servo moved** ([rasptank2#3](https://github.com/adeept/adeept_rasptank2/issues/3)). That user was still investigating.

What is **[INFERRED] / risk**:
- **The 15 V claim conflicts with the DRV8833's 10.8 V absolute operating limit** if VM comes straight from Vin. Do not exceed 2S (8.4 V). Never use 3S.
- TT motors are typically rated 3–6 V. At 100% duty on 8.4 V they are over-volted. The stock code's 30–60% speeds keep them in range. Cap the duty in your own driver, for example at about 70% (roughly 6 V average).
- **Pi 5 power specifics.** Raspberry Pi recommends **5 V / 5 A** for the Pi 5. When it is powered through the GPIO 5V pins there is no USB-PD negotiation, so the firmware assumes a **3 A-class supply and limits the USB ports to 600 mA total**. You can override this with `usb_max_current_enable=1` in `/boot/firmware/config.txt` or `PSU_MAX_CURRENT=5000` in the bootloader EEPROM, but **only if the 5V rail can really supply it** ([RPi docs, PSU_MAX_CURRENT and USB boot](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html); [bret.dk Pi 5 power guide](https://bret.dk/?p=7113)). A USB LiDAR (about 0.3–0.5 A) plus a USB Wi-Fi dongle or webcam can hit that 600 mA ceiling.
- **The servos almost certainly share the same 5V rail as the Pi** (the usual Adeept design, and consistent with the brownout description). Servo stall or startup current is the classic cause of Pi 5 resets.
- **Recommendation:** give the Pi 5 its own **dedicated 5 V ≥5 A buck** from the 2S pack, for example the [Pololu D36V50F5 (5 V 5.5 A, $34.95)](https://www.pololu.com/product/4091/specs), feeding the Pi's USB-C or GPIO 5V. Leave the HAT's regulator for servos and logic. If you power the Pi separately, **cut or remove the HAT-to-Pi 5V link**, or don't back-feed two regulators onto one 5V net. How the HAT's 5V reaches the header is undocumented: confirm with a multimeter and continuity checks before modifying anything. Monitor with `vcgencmd get_throttled` and `vcgencmd pmic_read_adc`.

---

## 3. Sensors and actuators

| Part | Model / physics | Interface | Limits | How Adeept code uses it |
|---|---|---|---|---|
| **Ultrasonic** | HC-SR04-type: 40 kHz burst, echo time of flight, `s = 340·t/2`. Lesson 12 names "HC-SR04", "2cm-400cm, accuracy 3mm" (the datasheet-style claim; real-world accuracy is about 1 cm, with a wide ~15–30° cone) [VERIFIED claim; real-world figures INFERRED] | Trig GPIO23, Echo GPIO24 | `gpiozero.DistanceSensor(max_distance=2)`, so readings cap at **2 m**. Soft, angled or narrow targets give no echo. **Do not plug it into the I2C header**: the pinout differs and it will burn the module (Lesson 12). Whether the HAT level-shifts the 5 V echo is not documented [UNVERIFIED]. | `Ultra.checkdist()` returns cm. `Functions.distRedress()` retries while the reading is >900 cm. Used by obstacle avoidance, keep-distance and radar scan. |
| **Line tracker** | 3-channel IR reflective (IR LED + phototransistor per channel) with digital outputs (onboard comparators) | GPIO17 / 27 / 22 (right / middle / left) | **1 = black line or nothing; 0 = white**. Binary only, no analog position. Sensitive to mounting height and ambient IR. Calibrate any onboard pots [INFERRED]. | `trackLineProcessing()` uses a 3-bit state and rotates left or right at speed 30. It does not strafe. |
| **Light tracker** | **Two LDRs in a voltage divider**: SIG = 2.5 V when both see equal light | ADS7830 **A1** (8-bit) | A single differential value (0–255; 127 = balanced). Gives a direction, not intensity. Resistor tolerance offsets the zero. | Threshold 127 ± 15: rotate toward the brighter side. |
| **Camera** | Model **not stated** in the ADR036 docs. Adeept's standalone camera ([ADR028-class OV5647 5 MP listing](https://www.robotshop.com/products/adeept-5mp-ov5647-camera-raspberry-pi-w-adjustable-holder-ribbon-cable)) is OV5647, 2592x1944, fixed focus, **~62° FOV**, 1080p30. This kit's camera is probably the same [UNVERIFIED]. Run `rpicam-hello --list-cameras` to confirm. Lessons 17/18 say "USB Camera Module" but the code is CSI-only (Picamera2), so treat that as a doc error. | **CSI** via Picamera2. Pi 5 needs the 22-pin to 15-pin cable. | The stock pipeline is 640x480 RGB888, JPEG-encoded per frame for MJPEG; expect ~15–30 fps without CV. hflip/vflip settings are in `camera_opencv.py`. | Flask MJPEG at `:5000/video_feed`. OpenCV modes: `findColor` (HSV mask with PID + 1D Kalman pan/tilt tracking), `watchDog` (frame-difference motion detection), `findlineCV` (threshold + two scan lines). Examples include gesture (skin) detection. |
| **Servos (2x)** | **Adeept AD002**, sold as an MG90S-compatible metal-gear micro servo: **2.0 kg·cm @ 4.8 V, 0.11 s/60°, 13 g, 180°** [UNVERIFIED retailer specs] | PCA9685 ch0 (pan), ch1 (tilt), 50 Hz | Code maps **500–2400 µs to 0–180°**. Hobby-grade, with backlash of a few degrees. | `RPIservo.ServoCtrl` thread with smoothing. `set_angle()` **re-creates the I2C bus and PCA9685 object on every call** (inefficient; it also rewrites the frequency, briefly sleeping the chip). |
| **TT motors (4x)** | Generic yellow TT gearmotors. **Ratio, RPM and voltage are not published by Adeept** [UNVERIFIED]. Typical TT motors are 1:48, 3–6 V, about 200 rpm no-load at 6 V. | 2-wire, M1–M4 via DRV8833 | **No encoders [VERIFIED]**. Unit-to-unit speed mismatch is common, so open-loop mecanum drive will drift and yaw. | `adafruit_motor.DCMotor`, `SLOW_DECAY`, throttle −1..1, **PWM at 50 Hz**. All four `M*_Direction = -1`. |
| **OLED** | **SSD1306**, 128x64 presumed (6 text lines at y = 0–50) | I2C **0x3C** | | `luma.oled`. Shows IP, network mode (Station / AP), current function, voltage. |
| **LED matrix 8x16** | **HT16K33**-driven (the code uses `adafruit_ht16k33.matrix.Matrix16x8`) | I2C **0x70** | Monochrome | Shows a smiley at server start. See the 0x70 quirk in §2.3. |
| **WS2812** | 2 on the HAT + external module (the server drives 5 pixels; other scripts assume 8 or 16) | SPI0 MOSI GPIO10 | Needs `dtparam=spi=on` | Breathing, flowing and police patterns. Pi 5-compatible because it uses `spidev`, not PWM/DMA. |
| **Buzzer** | Passive | GPIO18 | lgpio software PWM on the Pi 5, so tones may jitter slightly [INFERRED] | Low-battery alarm, songs |
| **IR receiver** | Onboard, model unknown | GPIO12 | No NEC decoder in the kit code (the HAT example only detects edges) | Unused by the car code |
| **MPU6050 socket** | 6-axis IMU, **not included** | I2C 0x68 | No magnetometer; gyro drifts | `setup` installs `mpu6050-raspberrypi`, but the car server never reads it |

---

## 4. Mecanum kinematics in the Adeept code

**[VERIFIED]** `Server/Server_MecanumWheels/Move.py`:

```
Motor layout (top view, front up):    M1 ___ M4      M1 = front-left   (PCA ch15/14)
                                         |   |        M2 = rear-left    (PCA ch12/13)
                                      M2 |___| M3     M3 = rear-right   (PCA ch11/10)
                                                      M4 = front-right  (PCA ch8/9)
```

`move(speed, direction, turn)` supports only these combinations. "+" means the wheel drives the robot forward; "0" means stopped.

| Command | M1 FL | M2 RL | M3 RR | M4 FR |
|---|---|---|---|---|
| forward (`direction=1, 'mid'`) | + | + | + | + |
| backward | − | − | − | − |
| strafe `left` | − | + | − | + |
| strafe `right` | + | − | + | − |
| `forward-left` (diagonal) | 0 | + | 0 | + |
| `forward-right` | + | 0 | + | 0 |
| `backward-left` | − | 0 | − | 0 |
| `backward-right` | 0 | − | 0 | − |
| `rotate-left` (CCW) | − | − | + | + |
| `rotate-right` (CW) | + | + | − | − |

This is exactly the standard inverse kinematics for an X-roller (seen from above) mecanum base, with vx forward, vy left and ω counter-clockwise:

```
FL = vx − vy − (lx+ly)·ω      FR = vx + vy + (lx+ly)·ω
RL = vx + vy − (lx+ly)·ω      RR = vx − vy + (lx+ly)·ω     (wheel surface speeds; divide by r for rad/s)
```

Here `lx` is half the wheelbase and `ly` is half the track: **measure them**. There is **no speed scaling between modes, no ramping, and no combined motion**. The `radius` argument is accepted but ignored, and `speed` is a 0–100 duty percentage, not a velocity. A continuous mixer that reuses Adeept's polarity convention:

```python
# throttle sign: Adeept's Motor() negates speed when direction == -1, and all M*_Direction = -1,
# so "wheel forward" == negative adafruit throttle on every channel.
M_DIR = {1: -1, 2: -1, 3: -1, 4: -1}          # flip per motor if a wheel spins the wrong way
def drive(vx, vy, wz, k=1.0, max_duty=0.7):   # vx, vy, wz in [-1, 1] (normalised)
    fl, fr = vx - vy - k*wz, vx + vy + k*wz
    rl, rr = vx + vy - k*wz, vx - vy + k*wz
    s = max(1.0, abs(fl), abs(fr), abs(rl), abs(rr))           # keep the ratios when saturating
    for ch, w in ((1, fl), (2, rl), (3, rr), (4, fr)):
        getattr(move, f"motor{ch}").throttle = M_DIR[ch] * max_duty * w / s   # needs move.setup() first
```

Without encoders, open-loop mecanum drive will **drift during strafing**. Roller slip and floor friction dominate, and 50 Hz PWM makes low-speed behaviour lumpy. Closed-loop wheel speed (§7) or external odometry (LiDAR scan-matching, optical flow) is needed for anything like `cmd_vel` accuracy.

---

## 5. Software

### 5.1 Repo layout [VERIFIED]

```
Adeept_4WD_Smart_Car_for_RPi/
├── setup_MecanumWheels.py / setup_OrdinaryWheels.py   # installer (run with sudo)
├── initPosServos.py                                    # sets all 16 PCA9685 channels to 90° (assembly aid)
├── wifi_hotspot_manager.sh                             # Wi-Fi client with fallback to AP mode
├── Examples/01_LED … 14_Remote_Control/                # one script per tutorial lesson
├── Server/Server_MecanumWheels/
│   ├── WebServer.py      # default autostart: WebSocket :8888 + Flask :5000 (web UI + MJPEG)
│   ├── APPServer.py      # for the Android "Adeept" app (also WebSocket :8888)
│   ├── GUIServer.py      # TCP :10223 for the desktop Tkinter client; FPV.py pushes video via ZeroMQ to client:5555
│   ├── Move.py RPIservo.py Functions.py Ultra.py Switch.py RobotLight.py Voltage.py OLED.py Matrix.py Buzzer.py
│   ├── camera_opencv.py base_camera.py app.py FPV.py PID.py Kalman_Filter.py Info.py
│   └── dist/             # prebuilt Vue/Vuetify web UI
└── Client/Client_MecanumWheels/GUI.py                  # Tkinter desktop client (Windows/Linux/macOS)
```

- **Language:** Python 3 (the bytecode in the repo is CPython 3.11, so it was developed on Bookworm). **The OS target is Raspberry Pi OS** (Imager; *"both 64-bit and 32-bit versions are compatible"*). The hotspot script mentions **Bookworm and Trixie**, and Adeept states its V3 code supports both ([DarkPaw-V3#1](https://github.com/adeept/Adeept_DarkPaw-V3/issues/1)). **[VERIFIED]**
- **Libraries [VERIFIED from `setup_MecanumWheels.py`]:**
  - apt: `i2c-tools python3-smbus python3-gpiozero python3-pigpio python3-pyqt5 python3-opengl python3-picamera2 python3-opencv opencv-data python3-pyaudio`
  - pip (as root, `--break-system-packages` on Debian ≥12): `adafruit-circuitpython-motor`, `-pca9685`, `-busdevice`, `-ssd1306`, `-ads7830`, `-ht16k33`, `luma.oled`, `mpu6050-raspberrypi`, `spidev`, `pillow`, `flask`, `flask_cors`, `numpy`, `pyzmq`, `imutils`, `zmq`, `pybase64`, `psutil`, `websockets==13.0`
- **Not used:** `RPi.GPIO`, `pigpio` (installed but not imported), `rpi_ws281x`, legacy `picamera`. **[VERIFIED by grep]**
- **Installer side effects [VERIFIED]:** it purges `wolfram-engine` and `libreoffice*`; installs **two root systemd units**, `wifi-hotspot-manager.service` and `Adeept_Robot.service` (the latter runs `~/startup.sh`, which runs `sudo python3 .../WebServer.py`); then **reboots automatically**. You must enable I2C and SPI in `raspi-config` first (Lesson 3).

### 5.2 Control interfaces [VERIFIED]

| Interface | How | Notes |
|---|---|---|
| **Web UI** (recommended by Adeept) | `http://<ip>:5000` in Chrome; the page talks to `ws://<ip>:8888` | First WebSocket message must be `admin:123456`. Commands are plain strings (`forward`, `left`, `rotate-left`, `DS`/`TS` stop, `scan`, `findColor`, `automatic`, `trackLine`, `CVFL`, `wsB <speed>`...). Replies are JSON. |
| Android app | `APP/Adeept.apk` in the zip, or "Adeept" in the app store; requires switching autostart to `APPServer.py` | Same port 8888 |
| Desktop GUI | `Client/.../GUI.py` (Tkinter + OpenCV + ZeroMQ) to robot TCP 10223 | The robot **connects back** to the client on port 5555 for video |
| Wi-Fi | `wifi_hotspot_manager.sh`: try the preconfigured NetworkManager Wi-Fi; on failure, start AP **SSID `Adeept_Robot`, PSK `12345678`, 192.168.4.1** | All three robots will advertise the same SSID |

**Autonomous behaviours in `Functions.py` [VERIFIED]:**
- **Obstacle avoidance:** if the front distance is ≥40 cm, drive forward at 60. If 20–40 cm, stop, look ±45° with the pan servo, and rotate toward the side with more space. If <20 cm, reverse.
- **Keep distance:** hold 25–35 cm.
- **Line and light tracking:** see §3.
- **Radar scan:** pan sweeps +90° to −90° in 2° steps and returns `[dist, θ]` for hits under 50 cm, which the web UI plots.

### 5.3 Pi 5 compatibility matrix

| Area | Status on Pi 5 (Bookworm/Trixie) | Evidence |
|---|---|---|
| GPIO via gpiozero | **Works.** gpiozero uses the lgpio factory. *"Only lgpio works on the Pi 5"*; the RPi.GPIO, pigpio and native factories are "not Pi 5". gpiozero 2.x also fixed lgpio picking the wrong gpiochip after the kernel renumbered gpiochip4 to gpiochip0, so keep `python3-gpiozero` updated. | [VERIFIED] [gpiozero pin docs](https://gpiozero.readthedocs.io/en/stable/api_pins.html), [changelog](https://gpiozero.readthedocs.io/en/rtd/_sources/changelog.rst.txt) |
| I2C via Blinka (`board`, `busio`) + smbus | **Should work.** Blinka supports the Pi 5. If `import board` complains about `RPi`/GPIO, follow Adafruit's Pi 5 route: `pip install rpi-lgpio` and **uninstall `RPi.GPIO`**. One user needed extra Blinka/RPi packages when using the HAT examples in a venv. | [VERIFIED] [Adafruit Blinka install](https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/installing-circuitpython-on-raspberry-pi); [philipmcgaw.com](https://philipmcgaw.com/adeept-robot-hat-for-raspberry-pi/) |
| WS2812 | **Works** (SPI MOSI bit-encoding avoids the `rpi_ws281x` PWM/DMA path that breaks on the Pi 5). HAT V3.0 kits used `rpi_ws281x`; V3.1+ moved to SPI. | [VERIFIED] code; [rasptank2#3](https://github.com/adeept/adeept_rasptank2/issues/3) |
| Status LEDs on GPIO 9/11 | **Conflict** with SPI0 MISO/SCLK. GitHub commit `9805983` (2026-09-22) now runs `gpioinfo` and **disables LED1/LED3 when the kernel or SPI owns the pin**. The March zip lacks this fix. Expect the "warning light" lesson and the low-battery LED flash to only partly work. | [VERIFIED] git history |
| Camera | **Works** with Picamera2 / libcamera, given the correct 22-pin cable. Error text still mentions `raspistill` (legacy, gone), which is cosmetic. | [VERIFIED] code |
| Ultrasonic timing | Works with lgpio. pigpio (historically the most accurate for `DistanceSensor`) is unavailable on the Pi 5, so expect a few cm of jitter; median-filter it. | [INFERRED]; [gpiozero 1.5 notes](https://www.raspberrypi.com/news/gpio-zero-v1-5/) |
| `Info.get_gpu_tempfunc()` | Broken (`/opt/vc/bin/vcgencmd`, and `os` is not imported in `Info.py`), but never called | [VERIFIED] |
| Python packaging | `sudo pip3 install --break-system-packages` writes to the system site-packages as root. This risks apt/pip conflicts, for example a pip `numpy` shadowing the numpy that apt's `python3-opencv` was built against. | [VERIFIED] behaviour; conflict risk [INFERRED] |
| asyncio / websockets | Pinned `websockets==13.0`, with the legacy `handler(websocket, path)` signature and `asyncio.get_event_loop().run_until_complete()`. These are deprecated patterns on Python 3.12+/3.13 (Trixie). A sibling repo has an open "no running event loop" issue. | [VERIFIED] code; [RaspTank#40](https://github.com/adeept/Adeept_RaspTank/issues/40) |
| Power | See §2.5. This is the dominant Pi 5 risk. | |

### 5.4 Known issues across Adeept repos (same HAT family)

- **Wrong PCA9685 address in old branches** (0x40 vs 0x5F), fixed later ([RaspClaws#4](https://github.com/adeept/Adeept_RaspClaws/issues/4)). Lesson: **run `i2cdetect -y 1`** before trusting any script.
- **`/etc/rc.local` autostart removed in newer Pi OS.** This kit already uses systemd ([rasptank2#3](https://github.com/adeept/adeept_rasptank2/issues/3), [picar-b2 PR#2](https://github.com/adeept/adeept_picar-b2/pull/2)).
- **Camera backend chosen by OS version string, not capability** ([DarkPaw-V3#1](https://github.com/adeept/Adeept_DarkPaw-V3/issues/1)). This kit hard-codes Picamera2, so it is not affected.
- **Motor direction inverted** (Adeept AWR): fix with direction constants or by swapping leads ([AWR#7](https://github.com/adeept/Adeept_AWR/issues/7)). This kit has `M1..M4_Direction` constants for the same purpose.
- **Tutorial copy-paste slips:** Lessons 13 and 19 `cd` into `Adeept_PiCar-Pro/...`; Lesson 13's listing swaps left and right pins; Lesson 14 says "HAT V3.1"; Lesson 3 says "HAT V3.3". Trust the code over the PDFs.
- **Licensing:** the repo has **no LICENSE**. On sibling repos Adeept replied that "we cannot fully open up the permissions" ([picar-b2#6](https://github.com/adeept/adeept_picar-b2/issues/6)). **[INFERRED]** For your own project, treat Adeept's code as reference only: rewrite the drivers (they are tiny) and keep only facts such as pin numbers and addresses.

---

## 6. Community projects and mods (same HAT family / siblings)

There are **no public projects specific to the ADR036 mecanum kit**. Searches across GitHub, Reddit, Hackster and YouTube found only retailer pages and Adeept repos (as of Oct 2026). Sibling kits (AWR-V3, PiCar-Pro/B2, RaspTank) use the same Robot HAT V3.x, PCA9685 at 0x5F, ADS7830 and pinout, so their ports transfer almost directly.

| Project | What | Relevance |
|---|---|---|
| [pi-drg/picar_adeept_fastapi](https://github.com/pi-drg/picar_adeept_fastapi) | Clean-room FastAPI driver for the **AWR-V3 (Robot HAT V3.2)** using only `smbus2`, `gpiozero`, `spidev`, `picamera2`; no Adeept code and no Blinka. Its constants table matches this kit (0x5F, motors on ch 8–15 slow-decay, line 22/27/17, ADS7830 ch0 ×4, buzzer 18). It notes the constants are not yet hardware-checked. | **Best template for a lean, license-clean driver** |
| [danielzurawski/zig-awr-v3](https://github.com/danielzurawski/zig-awr-v3) (+ [adeept-dashboard](https://github.com/danielzurawski/adeept-dashboard)) | Zig rewrite for AWR-V3 / HAT V3.2 (Pi 3–5): register-level PCA9685, SPI WS2812, HC-SR04, ADS7830, PID/Kalman, and a toy 80x80 occupancy grid from the ultrasonic sensor | Good reference for register-level drivers |
| [xonder05/ROS2_For_Adeept_AWR](https://github.com/xonder05/ROS2_For_Adeept_AWR) | Bachelor's thesis: **ROS 2 (Iron) + added IMU + LiDAR, slam_toolbox + Nav2** on an Adeept AWR | Closest thing to a ROS 2 navigation port on this HAT |
| [alexandergmzx/rasptank-ros2](https://github.com/alexandergmzx/rasptank-ros2) / [robot-fleet](https://github.com/alexandergmzx/robot-fleet) | ROS 2 **Jazzy**, part of a multi-robot fleet: RaspTank Pro with RPLIDAR C1 + MPU6050, **no encoders, so odometry comes from laser scan matching**, a governor and deadman on the robot, and "no safety function may depend on Wi-Fi". Pre-hardware. | Architecture notes match this project's goals well |
| [giannisp09/physical-ai-picar](https://github.com/giannisp09/physical-ai-picar) | **Claude vision + tool-use agent** driving an Adeept PiCar-B2 (perceive frame + ultrasonic, reason, call one drive tool), behind a `RobotBackend` abstraction. VLA / world-model plans. MIT license. | Direct reference for the LLM-agent layer |
| [khalidbourr/Adeept-RaspberryYOLO-Follower](https://github.com/khalidbourr/Adeept-RaspberryYOLO-Follower) | YOLOv5 person-following on AWR (ROS Noetic, Pi 4) | Vision-servoing example |
| [TheFujirose/adeept-mars-rover-v5-ros](https://github.com/TheFujirose/adeept-mars-rover-v5-ros), [tbiocanin/rl-mobile-robotics](https://github.com/tbiocanin/rl-mobile-robotics), [SaraPettinari/adeeptcar](https://github.com/SaraPettinari/adeeptcar), [stephdenman/rasptank-ros](https://github.com/stephdenman/rasptank-ros) | Assorted ROS 1/2 ports | Older; patterns only |
| [RockingMan3010/RockingMan-PiCar](https://github.com/RockingMan3010/RockingMan-PiCar) | PiCar-Pro V2 on a **Pi 5** (OLED, battery, Xbox pad, UDP) | Evidence that the HAT runs on a Pi 5 in practice |
| [cuinixam/pi-robot-hat](https://github.com/cuinixam/pi-robot-hat) | Python modules for the Adeept Robot HAT | Older HAT revision |

**No public examples found** of encoder upgrades, LiDAR mounts or swarm work on *this* chassis. You would be first. Swarm and LLM work exists only on siblings (above).

---

## 7. Limitations and recommended upgrades for serious autonomy

### 7.1 Limitations

1. **No odometry.** There are no encoders, and mecanum slip makes even encoder odometry mediocre for strafing.
2. **No heading sensor.** The MPU6050 socket is empty, and even an MPU6050 has no magnetometer.
3. **Perception:** one forward ultrasonic sensor (2 m cap, wide cone) and a monocular 640x480 camera at about 62° FOV. That is not enough for SLAM or Nav2 costmaps.
4. **Motor control:** 50 Hz PWM shared with the servos, slow-decay, open-loop TT motors with large unit-to-unit variance.
5. **Power:** the Pi 5 shares an undocumented 5V regulator with the servos. 2x 18650 at about 3.75 A peak gives modest runtime; with a Pi 5 under load plus a LiDAR, expect roughly 1 hour or less [INFERRED; measure it].
6. **Software:** a monolithic root Python process with global state, plaintext credentials, no watchdog or e-stop independent of Wi-Fi, and no ROS.

### 7.2 Upgrade path (priority order; prices are single-unit, Oct 2026, and must be re-checked)

| # | Upgrade | Concrete parts | Approx. price | Why / notes |
|---|---|---|---|---|
| 1 | **Pi 5 power isolation** | Pololu **D36V50F5** 5 V 5.5 A buck ([link](https://www.pololu.com/product/4091/specs)) from the 2S pack to the Pi; keep the HAT 5V for servos | $35–40 | Removes brownout resets. Add a pack fuse (about 5 A) and a real power switch. |
| 2 | **2D LiDAR** | **SLAMTEC RPLIDAR C1** (DTOF, 12 m, IP54, 110 g; [Waveshare](https://www.waveshare.com/product/rplidar-c1.htm)) or LDROBOT **LD19/LD06** class | C1 ≈ $80; LD19 ≈ $70–100 [UNVERIFIED] | Enables slam_toolbox, Nav2 and **scan-matching odometry** (rf2o, KISS-ICP), which sidesteps mecanum wheel slip. Mount it on a raised mast above the pan/tilt head. Prefer UART on X6/X7 or a USB adapter (mind the 600 mA USB budget). |
| 3 | **IMU** | **Adafruit BNO085** 9-DoF with onboard fusion ([Adafruit 4754](https://adafruit.com/product/4754)), or ICM-20948 / MPU6050 for the existing socket | BNO085 ≈ $25–30; MPU6050 ≈ $3–5 | Yaw rate and heading for an EKF (`robot_localization`). The BNO085 is happier on a microcontroller (I2C clock stretching) or in UART-RVC mode [INFERRED]. |
| 4 | **Encoders + real-time motor control** | **DFRobot FIT0450** TT motor with encoder (120:1, about 160 rpm @ 6 V, 0.8 kgf·cm stall, Hall encoder; [The Pi Hut £7.20](https://thepihut.com/products/tt-motor-with-encoder-6v-160rpm-120-1), [wiki](https://wiki.dfrobot.com/fit0450/)) ×4, plus a **Raspberry Pi Pico / Pico 2** (PIO quadrature decode + PID; **micro-ROS** or a simple serial protocol over the HAT UART) | ≈ £30 motors + ≈ $5 Pico | The TT shaft is the same, so the stock couplings probably fit [INFERRED]. Sources disagree on encoder PPR (8 vs 16), so measure it. Python on the Pi 5 can't reliably count 4 quadrature channels; offload to the Pico. A mid-term option is to have the Pico drive the H-bridges directly at about 20 kHz PWM, bypassing the 50 Hz PCA9685 limit (needs rewiring). |
| 5 | **Close-range ToF** | ST **VL53L1X** (single zone, 4 m) or **VL53L5CX** (8x8 multizone) breakouts on I2C (Pololu, SparkFun, Pimoroni) | ≈ $12–25 [UNVERIFIED] | Covers the LiDAR's blind band and glass or low obstacles better than ultrasound. Use an XSHUT pin per sensor to re-address several. |
| 6 | **Optical-flow odometry** (optional) | PMW3901 breakout (Pimoroni / Bitcraze-style) | ≈ $20–30 [UNVERIFIED] | Slip-independent translational odometry for mecanum; fuse in an EKF |
| 7 | **Better camera** | Raspberry Pi **Camera Module 3 (Wide)**, autofocus, 120° | ≈ $35 | Wider FOV for AprilTags and VLM snapshots. Same 22-pin cable on the Pi 5. |
| 8 | **Batteries and charging** | Reputable high-drain 18650s (e.g. Samsung 25R/30Q, Molicel P26A/P28A, Sony VTC6), bought as **matched pairs**, plus an **external smart charger** | ≈ $5–8 per cell | The HAT charger's balancing is undocumented [UNVERIFIED], so check per-cell voltages. Swappable packs let you keep testing. |
| 9 | **Pi 5 Active Cooler** | Official | ≈ $5 | Sustained CV/SLAM loads throttle without it. Check clearance under the HAT (21 mm standoffs). |

**Strategic note [Opinion].** For the long-term goal (autonomy stack, then UAV fleets), the Adeept chassis is best treated as a **cheap, crashable learning platform**: drivers, ROS 2 bring-up, LiDAR SLAM, multi-robot comms, and an LLM supervisory layer. If wheel-odometry-quality control becomes a bottleneck, it is often cheaper in time to move to a mecanum chassis that ships with encoder motors and an MCU driver board than to retrofit four TT motors. Keep the Pi 5 + LiDAR + IMU stack portable across chassis by building to a `cmd_vel` / `odom` / `scan` / `imu` interface from day one.

---

## 8. "First week with the hardware" checklist

**Before the boxes arrive**
- [ ] Order: 6–8 quality 18650 cells (matched pairs, protected or unprotected according to the holder's length; button-top vs flat-top, measure the holder), an external Li-ion charger, 3–6 spare Pi 5 camera cables, 3x Pi 5 Active Coolers, microSD cards (or NVMe later), and optionally the 5 V/5 A bucks.
- [ ] Flash **Raspberry Pi OS (64-bit, Bookworm or Trixie)** with Imager. Set a **unique hostname per robot** (`adeept-1..3`), SSH keys, and Wi-Fi.

**Day 1: bench bring-up (no motors, no batteries)**
- [ ] Inventory each kit against the packing list. Identify which FFC is the 22-pin (Pi 5) cable.
- [ ] Peel the protective film off the acrylic and clean residue from the holes (packing list note).
- [ ] `sudo raspi-config` → enable **I2C** and **SPI**.
- [ ] `git clone https://github.com/adeept/Adeept_4WD_Smart_Car_for_RPi.git` (**GitHub, not the March zip**). Read `setup_MecanumWheels.py` before running it: it purges packages, installs root services and **reboots**. Consider running it on one robot only, then reproducing it cleanly (venv with `--system-site-packages` for picamera2/opencv) on the others.
- [ ] Before running setup, edit `wifi_hotspot_manager.sh` so each robot gets a unique SSID and a strong PSK. Change `admin:123456` in `WebServer.py`/`APPServer.py` (and the web UI if it hard-codes it).
- [ ] Mount the HAT on the Pi (M2.5x21), **power from the HAT's USB-C with the HAT switch OFF and no batteries**, then run `i2cdetect -y 1` and expect `3c 48 5f 70`.
- [ ] Camera: `rpicam-hello --list-cameras`. Record the sensor model; it answers an open question.

**Day 2: servos and assembly**
- [ ] Centre the servos **before** fitting horns: `sudo python3 initPosServos.py` (all channels to 90°). Fit the pan horn so the head faces forward and the tilt horn level. Lesson 6 gives the reference angles (180° and 90° horn positions).
- [ ] Motors: keep wire colours consistent on each side and mirrored across sides (Lesson 6). **Wheel-L and Wheel-R in the right corners**, giving the X pattern seen from above.
- [ ] Don't plug the ultrasonic sensor into an I2C header (the pinout differs; Lesson 12).
- [ ] Measure and record wheelbase, track, wheel diameter, and the mass of each robot.

**Day 3: battery and power (18650 safety)**
- [ ] Use only genuine, matched cells; no "9900 mAh" fakes. Check polarity twice; the holder is 2S, so reversed cells are a short-circuit risk.
- [ ] **Charge with the HAT switch OFF** (Lesson 31). Expect 2–3 h. Never charge unattended or on flammable surfaces. Stop if the board gets hot.
- [ ] Don't feed USB-C and battery at the same time *with the switch ON* (Lesson 5).
- [ ] After a session, switch off and **remove the cells for storage**. The software alarm (6.0 V) is not a cutoff; an over-discharged cell (<2.5 V) should be recycled, not recharged.
- [ ] Check `vcgencmd get_throttled` (expect `0x0`) while servos sweep and motors run. Non-zero means undervoltage: do the §2.5 power isolation before anything else.

**Day 4: calibration**
- [ ] **Wheels on blocks first.** Run `Examples/04_Motor/Motor_Mecanum.py`, then forward, strafe and rotate at speed 30. Fix any wrong-direction wheel with `M*_Direction` in `Move.py` (or by swapping leads).
- [ ] Per-motor speed trim: on the floor, drive 2 m forward and measure lateral drift and yaw. Record per-motor gain factors (this is crude until you have encoders).
- [ ] Line tracker: check the 0/1 response at your floor height over black tape and white; adjust onboard pots if present.
- [ ] Light tracker: log the A1 value with uniform lighting and set the centre offset (stock assumes 127).
- [ ] Ultrasonic: compare against a tape measure at 10/30/60/100/150 cm; note jitter on the Pi 5.
- [ ] Battery ADC: compare the reported pack voltage against a multimeter; fix `ADCVref` (5.0 vs 5.2) per board.
- [ ] Camera: set `hflip`/`vflip`; check the field of view.

**Day 5: fleet hygiene**
- [ ] Unique hostname, SSID and credentials per robot. Keep all three on a dedicated router or VLAN; the stock services run as root with no auth beyond a fixed string.
- [ ] Decide whether to disable `Adeept_Robot.service` once you start writing your own stack. It grabs the camera, I2C devices and SPI at boot (`sudo killall python3` frees them, per Lesson 3).
- [ ] Put the measured constants (geometry, trims, ADC offsets) in a per-robot YAML file in this repo.

---

## 9. Open questions to settle on the bench

1. **5V regulator part and continuous current** on HAT V3.2. Does the servo rail share it with the Pi? Read the silkscreen and probe.
2. **Motor driver part number** (DRV8833 assumed) and **where VM comes from** (raw Vin or regulated).
3. **Camera sensor model**, and whether the second FFC in the box is the Pi 5 22-pin cable.
4. **TT motor gear ratio / rpm** and **wheel diameter** (measure no-load rpm with a phone strobe or tachometer).
5. Does the ultrasonic echo line get level-shifted to 3.3 V on the HAT?
6. Does the onboard charger **balance** the 2S pack? Is there a hardware low-voltage cutoff?
7. Robot mass with Pi 5 + cooler (and later LiDAR): is there TT motor headroom for strafing on carpet?

---

## 10. Sources

**Primary (Adeept)**
- Adeept download page: https://www.adeept.com/learn/detail-97.html. ZIP: https://www.adeept.com/video/static1/itemsfile/ADR036-Adeept_4WD_Smart_Car_Kit_for_RPi-20260317.zip. Inspected PDFs: Introduction to 4WD Smart Car; Packing List for Mecanum Wheels; Lessons 2 (Robot HAT V3.2), 3 (Getting Started), 5 (Preparations), 6 (Assembly, Mecanum), 9 (Servo), 10 (Motor, Mecanum), 12 (Ultrasonic), 13 (Line), 14 (Light), 15 (LED Matrix), 16 (OLED), 17/18 (Camera/OpenCV), 19 (Battery), 22/30 (Web UI / App), 30/31 (Charging). Bundled datasheets: PCA9685, DRV8833, ADS7830, LM324, picamera2 manual.
- Code: https://github.com/adeept/Adeept_4WD_Smart_Car_for_RPi (commits `934e5d0` 2026-03-17, `9805983` / `11a4ac7` 2026-09-22)
- Robot HAT V3 examples: https://github.com/adeept/Adeept_Robot_HAT-V3
- Adeept GitHub org (sibling repos): https://github.com/adeept

**Retail listings**
- Tindie (Adeept): https://www.tindie.com/products/adeept/adeept-4wd-mecanum-wheel-smart-car-kit-2/
- RobotShop UK: https://uk.robotshop.com/products/adeept-4wd-mecanum-wheel-smart-car-kit-raspberry-pi-5-4b-3b-w-o-rpi
- OzRobotics (SKU): https://ozrobotics.com/shop/smart-robot-car-kit-with-4wd-mecanum-wheel-for-raspberry-pi-5-4b-3b-3b-board-not-included/
- AD002 servo: https://uk.robotshop.com/products/adeept-ad002-servo-motor-rc-robot-car-8x
- Adeept OV5647 camera: https://www.robotshop.com/products/adeept-5mp-ov5647-camera-raspberry-pi-w-adjustable-holder-ribbon-cable
- Robot HAT V3.1 (Tindie): https://www.tindie.com/products/adeept/adeept-robot-hat-for-raspberry-pi/

**Datasheets and official docs**
- TI DRV8833: https://www.ti.com/lit/ds/symlink/drv8833.pdf
- TI ADS7830: https://www.ti.com/lit/ds/symlink/ads7830.pdf
- NXP PCA9685: https://www.nxp.com/docs/en/data-sheet/PCA9685.pdf
- GS2678 buck: https://static.chipdip.ru/lib/273/DOC029273893.pdf
- Raspberry Pi hardware docs (PSU_MAX_CURRENT, usb_max_current_enable): https://www.raspberrypi.com/documentation/computers/raspberry-pi.html
- Pi 5 power guide: https://bret.dk/?p=7113
- gpiozero pin factories: https://gpiozero.readthedocs.io/en/stable/api_pins.html
- Adafruit Blinka on Raspberry Pi (Pi 5 notes): https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/installing-circuitpython-on-raspberry-pi

**Reviews, articles, issues**
- CNX Software on Robot HAT V3.0: https://www.cnx-software.com/2024/07/29/adeept-robot-hat-for-raspberry-pi-is-designed-for-diy-projects-and-educational-needs/
- Philip McGaw on Robot HAT software: https://philipmcgaw.com/adeept-robot-hat-for-raspberry-pi/
- Issues: [rasptank2#3](https://github.com/adeept/adeept_rasptank2/issues/3), [RaspClaws#4](https://github.com/adeept/Adeept_RaspClaws/issues/4), [DarkPaw-V3#1](https://github.com/adeept/Adeept_DarkPaw-V3/issues/1), [picar-b2#6](https://github.com/adeept/adeept_picar-b2/issues/6), [AWR#7](https://github.com/adeept/Adeept_AWR/issues/7), [RaspTank#40](https://github.com/adeept/Adeept_RaspTank/issues/40)

**Upgrade parts**
- Adafruit Pi 5 camera cables: https://www.adafruit.com/product/5818, https://www.adafruit.com/product/5819
- RPLIDAR C1: https://www.waveshare.com/product/rplidar-c1.htm
- Pololu D36V50F5: https://www.pololu.com/product/4091/specs
- DFRobot FIT0450: https://wiki.dfrobot.com/fit0450/, https://thepihut.com/products/tt-motor-with-encoder-6v-160rpm-120-1
- Adafruit BNO085: https://adafruit.com/product/4754
