# 02 — Raspberry Pi 5 for Mobile Robotics: Practical Research Report

**Context:** 3x Raspberry Pi 5 8GB + SanDisk Extreme 128GB A2 microSD, each going onto an Adeept 4WD Mecanum Wheel Smart Car (Robot HAT V3.2 on the 40-pin header, CSI camera, ultrasonic, servos, TT motors, 2x18650 = 2S pack). No LiDAR yet. Long-term: an autonomous-agent framework, first on ground robots and later on UAVs. The developer is experienced in C#/.NET and simulators.

**Research date:** 2026-10-27.

**Labels used throughout**
- **VERIFIED**: I checked it against the linked source during this research.
- **INFERRED**: my reasoning, arithmetic, or background knowledge that I did not re-check against a primary source. Confirm these on the hardware before relying on them.

---

## 0. TL;DR: the top 12 things to know

1. **Power will cause most of your early problems.** The Pi 5 wants 5V/5A. Without USB-PD 5A negotiation it assumes 3A and caps total USB current at 600 mA. When you feed it from a battery buck converter, set `PSU_MAX_CURRENT=5000` in the EEPROM or `usb_max_current_enable=1` in config.txt. The Adeept HAT's DC-DC output current is not published, so measure it before trusting it. [VERIFIED](https://pip-assets.raspberrypi.com/categories/685-app-notes-guides-whitepapers/documents/RP-009856-WP-1-USB%20Power%20delivery%20on%20Raspberry%20Pi%205.pdf)
2. **Cool it actively.** A bare Pi 5 throttles within about a minute of heavy load. Fit the official Active Cooler (about $5) on every robot. [VERIFIED](https://core-electronics.com.au/guides/answering-your-pi-5-questions-cooling-overclocking-power-and-more/)
3. **The camera connector is different.** The Pi 5 has 22-pin, 0.5 mm "mini" CSI connectors. Older or kit cameras with 15-pin cables need a 22→15 adapter cable. Check whether the Adeept kit includes one. [VERIFIED](https://www.adafruit.com/product/5818)
4. **Old GPIO libraries are dead.** `RPi.GPIO` and `pigpio` don't work on the Pi 5 because GPIO now lives on the RP1 chip. Use `gpiozero`/`lgpio`, use `rpi-lgpio` as a drop-in for old kit code, and use `libgpiod` from C#. [VERIFIED](https://www.pi4j.com/documentation/providers/pigpio)
5. **.NET GPIO works on Pi 5, with one fix.** Upgrade to System.Device.Gpio ≥ 4.x and don't hard-code `gpiochip4`; the header chip is `gpiochip0` on current kernels. [VERIFIED](https://github.com/dotnet/iot/issues/2366)
6. **OS choice is a fork in the road.** Raspberry Pi OS (Trixie) gives the best camera, Hailo and Pi tooling, but ROS 2 is only Tier 3 there. Ubuntu 24.04 + Jazzy or Ubuntu 26.04 + Lyrical gives Tier-1 ROS 2 but more friction with libcamera and Hailo. My recommendation: **Raspberry Pi OS Trixie 64-bit on the host, with ROS 2 in Docker**, or Ubuntu 24.04 + Jazzy if you want native ROS. See §4–5.
7. **ROS 2 distro:** use **Jazzy** (LTS to May 2029, largest ecosystem) now. **Lyrical** (LTS to May 2031, Tier 1 on Ubuntu 26.04 arm64) is the forward-looking choice once your drivers support it. Skip **Kilted**, which is short-lived. [VERIFIED](https://discourse.openrobotics.org/t/ros-2-lyrical-luth-released/55021)
8. **Multi-robot over Wi-Fi: use rmw_zenoh.** It has been a Tier-1 RMW since Kilted, uses a router instead of multicast discovery, and is a good fit for Wi-Fi. It is not the default; Fast DDS still is. [VERIFIED](https://www.openrobotics.org/blog/2025/5/23/ros-2-kilted-kaiju-released)
9. **CPU-only AI is usable for small models.** A Pi 5 CPU runs YOLO26n (NCNN) at about 67 ms/inference, roughly 15 FPS before pre/post-processing. A 1.5B LLM runs at about 10 tok/s and a 3B at about 5 tok/s. [VERIFIED](https://docs.ultralytics.com/guides/raspberry-pi) [VERIFIED](https://stratospherelinuxips.readthedocs.io/en/develop/immune/research_rpi_llm_performance.html)
10. **The AI HAT+ 2 (Hailo-10H) is not an LLM accelerator in practice.** CNX measured LLM speed *slower* than the Pi 5 CPU, and its vision performance is about equal to the Hailo-8 AI HAT+. For vision on a robot, the **AI HAT+ 26 TOPS (Hailo-8)** or **13 TOPS (Hailo-8L)** is the better buy. [VERIFIED](https://www.cnx-software.com/2026/01/20/raspberry-pi-ai-hat-2-review-a-40-tops-ai-accelerator-tested-with-computer-vision-llm-and-vlm-workloads/)
11. **PCIe is single-lane and exclusive.** The AI HAT+ and an NVMe SSD both need the one PCIe FFC connector. Running both requires a PCIe-switch board. Stacking the AI HAT+ under the Adeept Robot HAT needs the included stacking header plus taller standoffs. [VERIFIED](https://www.cnx-software.com/2024/08/01/pineboards-introduces-ai-bundle-hailo-8l-raspberry-pi-hat-with-ai-accelerator-and-nvme-ssd-support/)
12. **Runtime on 2x18650 (about 22 Wh) is roughly 1–2 hours**, depending on CPU and motor load. Plan for battery swaps or a larger pack (2S2P or 3S). [INFERRED from arithmetic in §2.6]

---

## 1. Raspberry Pi 5: specs that matter for robotics

| Item | Pi 5 detail | Status |
|---|---|---|
| SoC | Broadcom BCM2712, 16 nm, quad-core Cortex-A76 @ 2.4 GHz, 512 KB L2 per core, 2 MB shared L3 | [VERIFIED](https://www.raspberrypi.com/news/introducing-raspberry-pi-5/) |
| GPU | VideoCore VII, OpenGL ES 3.1, Vulkan 1.2 (Mesa); dual 4Kp60 HDMI; 4Kp60 HEVC **decode** | [VERIFIED](https://www.raspberrypi.com/news/introducing-raspberry-pi-5/) |
| Video encode | **No hardware H.264 encoder.** Encoding is software on the CPU. Raspberry Pi published a white paper (May 2026) on software H.264 performance; a secondary source reports real-time 1080p30 in RPi's tested configs | [VERIFIED (whitepaper exists)](https://pip-assets.raspberrypi.com/categories/685-app-notes-guides-whitepapers/documents/RP-010033-WP-1-H.264%20encoding%20performance%20on%20Raspberry%20Pi%205_series%20computers.pdf); 1080p30 claim from a secondary source, not checked in the PDF |
| RAM | LPDDR4X-4267, 32-bit; 2/4/8/16 GB variants (1 GB later added) | [VERIFIED](https://www.raspberrypi.com/news/introducing-raspberry-pi-5/) |
| Performance | 2–3x Pi 4 CPU/GPU; about 2x memory and I/O bandwidth | [VERIFIED](https://www.raspberrypi.com/news/introducing-raspberry-pi-5/) |
| RP1 I/O chip | In-house southbridge (TSMC 40LP). Provides 2x USB 3.0 + 2x USB 2.0, GbE, 2x 4-lane MIPI, 3.3V GPIO, UART/SPI/I2C/I2S/PWM, plus **1 PIO block (4 state machines)**. Connected to BCM2712 over a PCIe 2.0 **x4** link (about 16 Gb/s) | [VERIFIED](https://www.raspberrypi.com/news/introducing-raspberry-pi-5/) |
| External PCIe | 1 lane PCIe 2.0 on a 16-pin 0.5 mm FPC. Gen 3 (8 GT/s) can be forced with `dtparam=pciex1_gen=3`, but is "not certified" and may be unstable | [VERIFIED](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html) |
| Camera/display | 2x 4-lane MIPI transceivers (CAM/DISP 0 and 1), each usable as CSI **or** DSI, on 22-pin 0.5 mm "mini" FPC | [VERIFIED](https://www.raspberrypi.com/news/introducing-raspberry-pi-5/), [VERIFIED (22-pin)](https://www.adafruit.com/product/5818) |
| Wireless | Dual-band 802.11ac Wi-Fi, Bluetooth 5.0/BLE (CYW43455, same chip as Pi 4 but on a faster SDIO interface) | [VERIFIED](https://www.tomshardware.com/news/raspberry-pi-5-wi-fi-faster) |
| Ethernet | GbE with PTP hardware timestamping (`/dev/ptp0`) | [VERIFIED](https://canonical-industrial-documentation.readthedocs-hosted.com/en/latest/iot-communication-protocols/how-to/rpi5-ptp-time-sync) |
| RTC | Built-in RTC. Optional rechargeable Li-Mn coin cell on J5 (BAT). Charging is off by default; enable with `dtparam=rtc_bbat_vchg=3000000`. Li-ion cells must not be used | [VERIFIED](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html) |
| Power button | Onboard. On Lite, one press = shutdown; hold = hard off; press when halted = boot. J2 pads take an external momentary switch, which is useful for a robot chassis button | [VERIFIED](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html) |
| Fan header | 4-pin JST-SH 1 mm (5V, PWM, GND, Tach). Shares the current budget with USB | [VERIFIED](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html) |
| Halt power | About 1–1.4 W when "off" by default. `POWER_OFF_ON_HALT=1` in EEPROM brings this to about 0.01 W, which matters for robots left with a battery attached | [VERIFIED](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html) |

### 1.1 Thermal behaviour
- **Throttle points.** The Arm cores throttle progressively between 80 °C and 85 °C. At 85 °C, both the cores and the GPU throttle. [VERIFIED](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html)
- **Default fan curve.** Off below 50 °C, 30% at 50 °C, 50% at 60 °C, 70% at 67.5 °C, 100% at 75 °C. Tune it with `dtparam=fan_temp0=...`. The official fan draws about 60 mA at 5V. [VERIFIED](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html)
- **Bare board.** Core Electronics saw throttling in under a minute of heavy load. With the Active Cooler's fan running, the board stayed clear of throttling over a 16-hour test. With the fan removed (heatsink only), it took about 10 minutes to throttle. [VERIFIED](https://core-electronics.com.au/guides/answering-your-pi-5-questions-cooling-overclocking-power-and-more/)
- **Real robot data point.** A Pi 5 8GB running Jazzy + Nav2 + LiDAR on Ubuntu 24.04 sat at 66.4 °C with `throttled=0x0` while its load average was above 4. [VERIFIED](https://discourse.openrobotics.org/t/turtlebot4-navigation-tuning-result/53609)
- **Robot-specific note** [INFERRED]: the Adeept Robot HAT sits directly over the SoC. The Active Cooler is 1-2 cm tall (I didn't verify the exact height), so you'll likely need taller standoffs or a stacking header to clear it. Check physical clearance before assembly. Airflow under a HAT is poor, so monitor with `vcgencmd measure_temp` and `vcgencmd get_throttled`.

### 1.2 Pricing (volatile in 2026 because of the DRAM shortage)
Raspberry Pi raised prices several times in 2025–2026 because LPDDR4 costs rose. The April 2026 round added **+$50 to the Pi 5 8GB**, **+$100 to the 16GB**, and **+$50 to the AI HAT+ 2**. [VERIFIED](https://www.gigazine.net/gsc_news/en/20260402-raspberry-pi-price-increases)

Sources disagree on the current 8GB street price. BGR reports $175 after April, while Tom's Hardware lists $130. [VERIFIED (conflict)](https://www.bgr.com/2150106/raspberry-pi-cost-ram-crisis/) The October 2026 round touched only 2GB models (Pi 5 2GB → $77.50). [VERIFIED](https://www.omgubuntu.co.uk/2026/10/raspberry-pi-2gb-costs-how-much-now/amp) **Check the official reseller before budgeting.**

---

## 2. Power: the #1 source of robot pain

### 2.1 What the Pi 5 expects (official)
- **Supply.** The recommended supply is 5V/5A (25W) via USB-PD. The Pi 5 only requests 5V PDOs and ignores higher voltages. It does **not** support PPS. [VERIFIED](https://pip-assets.raspberrypi.com/categories/685-app-notes-guides-whitepapers/documents/RP-009856-WP-1-USB%20Power%20delivery%20on%20Raspberry%20Pi%205.pdf)
- **Without a 5A PDO** (any non-PD source, including a buck converter on the GPIO pins):
  - the Pi assumes **5V/3A**;
  - total USB current is limited to **600 mA** instead of 1.6 A;
  - USB boot is disabled;
  - the desktop shows a "power limited" warning.

  It still "will function perfectly well with a 3A supply, provided that only lower-power peripherals are attached." [VERIFIED](https://pip-assets.raspberrypi.com/categories/685-app-notes-guides-whitepapers/documents/RP-009856-WP-1-USB%20Power%20delivery%20on%20Raspberry%20Pi%205.pdf)
- **Overrides** (from the same whitepaper):
  - EEPROM: `PSU_MAX_CURRENT=5000` (set with `sudo rpi-eeprom-config --edit`) skips PD negotiation and assumes a 5A supply.
  - config.txt: `usb_max_current_enable=1` raises the USB limit to 1.6 A.
  - Either setting re-enables USB boot.
  - Check the current state with `vcgencmd get_config usb_max_current_enable`. [VERIFIED](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html)
- **Budget figures from the RPi docs:**
  - bare-board active draw is about 800 mA;
  - a Camera Module draws about 250 mA;
  - GPIO total is 50 mA max;
  - the USB budget is shared with the fan header;
  - peak board draw is about 12 W (vs 8 W for Pi 4). [VERIFIED](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html), [VERIFIED](https://www.raspberrypi.com/news/introducing-raspberry-pi-5/)
- **Undervoltage.** The Pi detects supply below **4.63 V (±5%)** and logs a kernel warning. [VERIFIED](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html)

### 2.2 Brownout / undervoltage symptoms (what you will see)
- `vcgencmd get_throttled` returns non-zero. Bit 0 means under-voltage now; bit 16 means it has occurred since boot. [INFERRED: standard vcgencmd bit layout from background knowledge, not re-checked]
- `dmesg` shows "Undervoltage detected!". [VERIFIED that the kernel logs it](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html)
- Typical failure modes on a robot [INFERRED]:
  - random reboots when motors start or stall;
  - USB LiDAR or USB camera disconnecting and re-enumerating;
  - SD card I/O errors and file system corruption after hard resets;
  - Wi-Fi drops.

  Motor inrush pulling the battery voltage down is the classic cause.

### 2.3 USB-C vs GPIO 5V pins
- **The 5V header pins accept 5V input**, and the RPi docs say so explicitly, e.g. for PoE+ HATs. [VERIFIED](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html) However, power fed this way **bypasses the USB-C input protection**, so over-voltage on these pins can kill the board. [VERIFIED (secondary)](https://bret.dk/?p=7113)
- **USB-C from a buck converter** keeps the protection, but you'll still get the 3A assumption unless the converter speaks USB-PD 5V/5A, which few do. [INFERRED]
- **Practical recommendation** [INFERRED]: feed the 5V header from a quality 5V/5A+ regulator on short, thick wires, then set `PSU_MAX_CURRENT=5000`. One Pololu forum user saw 4.9–4.95 V at the Pi under load, attributed to wire drop. Measure **at the Pi**, not at the regulator. [VERIFIED](https://forum.pololu.com/t/low-voltage-on-pi-5-with-pololu-d36v50f5-step-down-voltage-regulator/29212)
- **Never feed 5V from two sources at once.** For example, don't connect the HAT's DC-DC and USB-C, or the HAT's DC-DC and a separate buck, unless you know there are ideal-diode/OR-ing circuits. [INFERRED]

### 2.4 The Adeept Robot HAT's own power path
- The Robot HAT V3.x takes an 8.4 V (2S) battery input. It has an onboard charger (USB-C, charge only), a battery-level ADC and an integrated DC-DC converter. **The vendor doesn't specify the DC-DC output current or ICs.** [VERIFIED](https://www.cnx-software.com/2024/07/29/adeept-robot-hat-for-raspberry-pi-is-designed-for-diy-projects-and-educational-needs/)
- A hands-on write-up of V3.0 says the USB-C powers the HAT and the Pi and charges a 2S1P 18650 pack. [VERIFIED](https://philipmcgaw.com/adeept-robot-hat-for-raspberry-pi/) The kit listing doesn't state V3.2 power specs. [VERIFIED (absence)](https://uk.robotshop.com/products/adeept-4wd-mecanum-wheel-smart-car-kit-raspberry-pi-5-4b-3b-w-o-rpi)
- **Action items** [INFERRED]:
  1. Before installing the Pi, measure the HAT's 5V rail under a dummy load of about 3–4 A, or check the regulator part number on the PCB.
  2. If it sags below about 5.0 V, power the Pi from a dedicated buck converter and isolate the HAT's 5V from the Pi.
  3. Leave motors and servos on the HAT's battery-side rail.
  4. Add a bulk electrolytic capacitor (a few hundred µF) near the motor driver.
  5. Set `PSU_MAX_CURRENT=5000` only if the rail really can deliver 5A.

### 2.5 Candidate regulators / UPS boards for a 2S 18650 robot

| Part | Key spec | Notes | Status |
|---|---|---|---|
| Pololu D36V50F5 | 5V, 5.5A typical max, 5.5–50 V in | Small, well documented; continuous current is thermally limited. Fixed 5.0 V (no headroom for wire drop) | [VERIFIED](https://www.pololu.com/product/4091/specs) |
| DFRobot DFR0753 | 6–14V in → 5V/8A | Plenty of headroom from 2S (6.0–8.4 V) | [VERIFIED (name/spec from forum title)](https://forum.core-electronics.com.au/t/dc-dc-buck-converter-6-14v-to-5v-8a-dfr0753/19160) |
| DFRobot FIT0992 Pi 5 18650 UPS HAT | 5.1V/5A out, 6–18V in | Marketed for "smart cars". Listings disagree on the number of 18650 holders (2 vs 4); adds height | [VERIFIED](https://core-electronics.com.au/raspberry-pi-5-18650-battery-ups-hat-51v-5a.html.plain.md) |
| Waveshare UPS HAT (B) | Up to 5A, pogo-pin, 2x18650 | Pogo pins under the Pi, so it doesn't consume the 40-pin header | [VERIFIED](https://www.waveshare.com/ups-hat-b.htm) |
| SupTronics/Geekworm X1202 | 4x18650, 5.1V/5A (25W) | A big, heavy UPS, probably too large for the Adeept chassis | [VERIFIED](https://cnx-software.com/?p=130922) |

### 2.6 Realistic power draw and runtime (2x18650 ≈ 3000 mAh each, 2S)

**Measured component figures**
- Pi 5 8GB (C1 stepping): **3.2 W idle**, **9.8 W under stress-ng**. The 2GB D0 stepping measures 2.4 W / 8.9 W. [VERIFIED](https://www.jeffgeerling.com/blog/2024/new-2gb-pi-5-has-33-smaller-die-30-idle-power-savings/)
- Camera module: about 250 mA at 5V (about 1.25 W). [VERIFIED](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html)
- RPLIDAR C1: 230 mA at 5V (1.15 W). LDRobot STL-19P/D500: 290 mA (1.45 W). [VERIFIED](https://www.waveshare.com/wiki/RPLIDAR_C1), [VERIFIED](https://www.waveshare.com/wiki/D500_LiDAR_Kit)
- Pi 5 + Hailo-8L/8 running YOLO (whole system, at USB-C input): 6.1–6.5 W average. Peaks reach 9.2–17.8 W depending on model and chip. [VERIFIED](https://sixfab.com/blog/raspberry-pi-5-ai-hat-benchmark-deepx-vs-hailo/)
- Pi 5 CPU running a 1.5B LLM (CM5): 10.2–10.6 W. [VERIFIED](https://www.cnx-software.com/2026/01/20/raspberry-pi-ai-hat-2-review-a-40-tops-ai-accelerator-tested-with-computer-vision-llm-and-vlm-workloads/)

**Runtime arithmetic** [INFERRED]
- Pack: 2S × 3.0 Ah × 3.7 V nominal = **22.2 Wh nominal**. Assume about 85% usable (cutoff, ageing) and 88% buck efficiency, giving **about 16.6 Wh delivered at 5V**. Many cheap "3000 mAh" cells deliver much less, so measure yours.
- TT motors and servos: I found no verified per-motor figures for the Adeept motors. Budget 3–8 W average while driving, with multi-amp stall spikes, and measure with an inline meter.

| Scenario | Avg load at 5V side | Est. runtime |
|---|---|---|
| Idle, Wi-Fi + SSH, cooler fan | about 4 W | about 4 h |
| ROS 2 + camera stream + Wi-Fi, stationary (CPU moderate) | about 7 W | about 2.3 h |
| Full autonomy stack (Nav2/SLAM + LiDAR + camera + CV), stationary | about 11–12 W | about 1.4 h |
| Full stack + driving (motors avg about 5 W from the battery side) | about 16–17 W equivalent | **about 1 h** |

**Implication:** for development, keep a bench supply / USB-C PD tether for "on the stand" work. For field runs, use 2S2P (6 Ah) or a 3S Li-ion pack with a 5V/5A buck. For a fleet, plan swappable packs and a charging station.

---

## 3. GPIO on the Pi 5 (RP1 changes)

### 3.1 What changed
- **GPIO moved to RP1.** The header GPIO, PWM, I2C, SPI and UART now live on RP1, behind PCIe. Libraries that poked BCM283x registers through `/dev/mem` break. That includes **RPi.GPIO** (unmaintained since 2022) and **pigpio** (no Pi 5 support and none planned; Pi4J says it would need a "large development effort"). [VERIFIED](https://raspberry.tips/en/raspberrypi-tutorials/raspberry-pi-5-gpio-rpigpio-not-working-alternatives), [VERIFIED](https://www.pi4j.com/documentation/providers/pigpio)
- **gpiochip numbering changed in 2024.**
  - Early Pi 5 kernels exposed the header as `gpiochip4`.
  - raspberrypi/linux PR #6144 (merged 5 Aug 2024) aliased the RP1 header chip to **`gpiochip0`** and moved SoC chips to gpiochip10+.
  - Raspberry Pi OS ships a udev symlink `gpiochip4 → gpiochip0` for compatibility.
  - **Best practice: look the chip up by label `pinctrl-rp1`, never by number.** [VERIFIED](https://github.com/raspberrypi/linux/pull/6144)
- **The PIO block is reachable from userspace via PIOLib.** Only the FIFOs are reachable (over DMA); the PIO registers are not. This is useful for WS2812 or custom protocols. [VERIFIED](https://www.raspberrypi.com/news/piolib-a-userspace-library-for-pio-control/)

### 3.2 Python options

| Library | Pi 5 status | Use for |
|---|---|---|
| `gpiozero` | Works. Uses the lgpio backend on Pi 5. | New Python code, quick tests |
| `lgpio` | Works (uses `/dev/gpiochip*`) | Low-level Python/C |
| `rpi-lgpio` | Drop-in replacement for the `RPi.GPIO` API. Uninstall `python3-rpi.gpio` first. Only helps if code uses the standard RPi.GPIO API. | **Running the Adeept sample code unchanged** |
| `libgpiod` (v1 on Bookworm, v2 on Trixie) | Works | Kernel-standard interface for C, C++, C#, Rust |
| `pigpio` | **Not supported on Pi 5** | Avoid |
| `rpi_ws281x` | Experimental Pi 5 branch with a kernel module, PWM-based; not all features work | WS2812 LEDs (or use SPI-based or PIOLib-based alternatives) |

Sources: [rpi-lgpio](https://rpi-lgpio.readthedocs.io/en/latest/api.html), [raspberry.tips comparison](https://raspberry.tips/en/raspberrypi-tutorials/raspberry-pi-gpio-python), [rpi_ws281x Pi 5 wiki](https://github.com/jgarff/rpi_ws281x/wiki/Raspberry-Pi-5-Support) (all VERIFIED).

**Adeept code note.** A V3.0 Robot HAT user needed `RPi.GPIO`, Adafruit Blinka (`board`), `adafruit-circuitpython-pca9685` and `adafruit-circuitpython-motor`, and had to enable I2C in raspi-config. [VERIFIED](https://philipmcgaw.com/adeept-robot-hat-for-raspberry-pi/) On a Pi 5, substitute `rpi-lgpio` for `RPi.GPIO` [INFERRED]. The HAT's servo driver is a **PCA9685** on I2C. [VERIFIED](https://www.cnx-software.com/2024/07/29/adeept-robot-hat-for-raspberry-pi-is-designed-for-diy-projects-and-educational-needs/) Motor PWM most likely comes from the PCA9685 too [INFERRED: verify against the Adeept V3.2 schematic]. That's good news: motor and servo control is I2C-only and therefore OS- and library-agnostic, including from C#.

### 3.3 Hardware PWM, I2C, SPI, UART
- **Hardware PWM.**
  - Enable with `dtoverlay=pwm-2chan`; the default pins are GPIO18/19. To move it to 12/13, use `dtoverlay=pwm-2chan,pin=12,func=4,pin2=13,func2=4`.
  - On Pi 5, channels 0/1 map to GPIO12/13 and channels 2/3 to GPIO18/19. The community library uses **`pwmchip2`**; verify with `ls /sys/class/pwm`. [VERIFIED (community)](https://pypi.org/project/rpi-hardware-pwm)
- **I2C.** Enable it in raspi-config; the bus is `/dev/i2c-1` on header pins 3/5. **Clock-stretching caution:** the BNO085/BNO086 rely on clock stretching. One project reports that RP1's DesignWare I²C controller "does not handle clock stretching cleanly during the address phase". The workarounds are UART-RVC/SPI mode or slowing the bus. [VERIFIED (project assessment, not official)](https://codeberg.org/buildacubesat-project/bac-hardware/issues/45), [VERIFIED (Adafruit workaround)](https://learn.adafruit.com/raspberry-pi-i2c-clock-stretching-fixes?view=all)
- **UART / SPI.** Standard overlays apply. RP1 provides more UARTs than earlier Pis [INFERRED: check `dtoverlay -h uart2` etc.].
- **5V sensors.** **GPIO is 3.3V only and not 5V tolerant.** [VERIFIED](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html) An HC-SR04 ultrasonic echo pin at 5V must be level-shifted or divided. Check whether the Adeept HAT already does this [INFERRED].

### 3.4 .NET: System.Device.Gpio / Iot.Device.Bindings on Pi 5
- **Releases.** dotnet/iot v4.0.0 added libgpiod **v2** support (experimental at the time) and made the V2 driver public. In **v4.2.0** the notes say "LibGpiodV2 is no longer experimental" and also fix pull-up/pull-down handling. v3.1 added Pi 5 board info. [VERIFIED](https://github.com/dotnet/iot/releases)
- **Known breakage.** Issue #2366: from RaspiOS 2024-10-28 onward, `new GpioController()` threw "Unable to find a chip" because the library hard-coded chip 4. The issue was closed by PR #2386 (v4.0.0 milestone). The thread's robust pattern is shown below. [VERIFIED](https://github.com/dotnet/iot/issues/2366)
  ```csharp
  #pragma warning disable SDGPIO0001
  var gpio = new GpioController(new LibGpiodDriver(0 /* pinctrl-rp1 */, LibGpiodDriverVersion.V1)); // or V2 on Trixie/Ubuntu 24.04+
  #pragma warning restore SDGPIO0001
  ```
  A maintainer's earlier advice for Pi 5 was also "manually select the driver (libgpiod or sysfs)". [VERIFIED](https://github.com/dotnet/iot/issues/2262) A separate report says `new GpioController()` hangs after installing cargo/llvm tools on a Pi 5. [VERIFIED](https://github.com/dotnet/iot/issues/2332)
- **libgpiod version must match the OS.**
  - Bookworm ships libgpiod **1.x**.
  - Trixie ships **2.x**. [VERIFIED (via a third-party README)](https://github.com/lumbermill/rgpio/wiki)
  - Ubuntu 24.04 is libgpiod 2.x [INFERRED].

  Pick `LibGpiodDriverVersion` accordingly, or let 4.2 auto-detect it and verify.
- **Recommendation** [INFERRED]: use System.Device.Gpio/Iot.Device.Bindings ≥ 4.2 on .NET 8 or 10, construct the driver explicitly, and look up the chip whose label is `pinctrl-rp1`.
  - Use `System.Device.I2c` + `Iot.Device.Pwm.Pca9685` for the Adeept servos and motors; that's pure I2C with no GPIO-chip issues.
  - Use `System.Device.Pwm` (sysfs) for hardware PWM.
  - Prefer libgpiod event APIs over polling for ultrasonic timing, but expect jitter in user space. Precise echo timing is a good job for a microcontroller (§5.5).
- **.NET on the Pi.** .NET 10 (LTS, released 11 Nov 2025) supports Debian 12/13 on Arm64 via Microsoft's apt feed. [VERIFIED](https://learn.microsoft.com/lt-lt/dotnet/core/install/linux-debian), [VERIFIED](https://www.petecodes.co.uk/install-and-use-microsoft-dot-net-10-with-the-raspberry-pi/) In practice, cross-publish from Windows (§8.3) and only install the runtime on the robots, or ship self-contained.

---

## 4. Operating system options

| Option | Pros | Cons | Status |
|---|---|---|---|
| **Raspberry Pi OS (Trixie, Debian 13) 64-bit**, Oct 2025 | First-class libcamera/rpicam/picamera2, Hailo packages (AI HAT+ 2 *requires* Trixie 64-bit), PIOLib, raspi-config (now with a persistent WLAN power-save toggle), fastest fixes for Pi hardware. Kernel 6.12 LTS | ROS 2 is only Tier 3 (no official apt). Options are Docker, source builds, or the community "rospian" Jazzy apt repo | [VERIFIED](https://www.cnx-software.com/2025/10/06/raspberry-pi-os-debian-13-trixie/), [VERIFIED](https://www.cnx-software.com/2026/01/20/raspberry-pi-ai-hat-2-review-a-40-tops-ai-accelerator-tested-with-computer-vision-llm-and-vlm-workloads/), [VERIFIED](https://discourse.openrobotics.org/t/native-ros-2-jazzy-debian-packages-for-raspberry-pi-os-debian-trixie-arm64/51965) |
| Raspberry Pi OS Bookworm (Debian 12) | Mature | Now legacy; libgpiod 1.x; no AI HAT+ 2 support | [INFERRED] |
| **Ubuntu 24.04 LTS** (Server) | Tier-1 for **ROS 2 Jazzy**; `apt install ros-jazzy-*`; the reference stack for TurtleBot-style robots | The Pi camera needs the raspberrypi libcamera fork built manually for camera_ros. Hailo needs the PCIe driver built from source and version-pinned HailoRT. Raspberry Pi utilities lag | [VERIFIED](https://docs.ros.org/en/jazzy/p/camera_ros), [VERIFIED](https://ubuntu.com/blog/hackers-guide-to-the-raspberry-pi-ai-kit-on-ubuntu) |
| **Ubuntu 26.04 LTS** (Apr 2026) | Pi 5 is "certified". Tier-1 for **ROS 2 Lyrical** (arm64). Longest runway | Needs a boot EEPROM ≥ 2025-02-11. Young ecosystem; check each driver for Lyrical. Same camera and Hailo friction as 24.04 | [VERIFIED](https://ubuntu.com/hardware/docs/boards/how-to/ubuntu_supported/raspberry-pi/), [VERIFIED](https://ubuntu.com/raspberry-pi) |

**Recommendation for this project** [INFERRED, reasoned]:
- **Option A: Raspberry Pi OS Trixie 64-bit host + ROS 2 in Docker** (`ros:jazzy-ros-base` or later lyrical images, with `--net=host --privileged` or explicit `/dev` mappings). This gives the best Pi hardware support: camera, Hailo, GPIO, power tooling. ROS stays isolated and reproducible across 3 robots, and you'll later move the same containers to UAV companion computers. The cost is that camera access from inside the container needs care. Either run camera_ros in the container with `/dev/media*`, `/dev/video*` and `/run/udev` mapped plus the RPi libcamera, or publish frames from a host-side picamera2/rpicam process.
- **Option B: Ubuntu 24.04 + native Jazzy.** This is the most tutorials-compatible path. You'll fight libcamera and Hailo once, then script it.
- Either way, **write a provisioning script (Ansible or cloud-init) and image all three robots identically.** Hostnames, `ROS_DOMAIN_ID`/Zenoh config, robot IDs and SSH keys should be templated.

### 4.1 SD vs NVMe
- **SD (your SanDisk Extreme A2).** The Pi 5 supports SDR104 and SD command queueing for A2 cards. RPi's own A2 cards claim 5,000 random-read IOPS on Pi 5. [VERIFIED](https://www.mouser.com/new/raspberry-pi/raspberry-pi-a2-class-micro-sd-cards/) Some SanDisk Extreme cards log `mmc0: running CQE recovery`; the fix users report is `dtparam=sd_cqe=0`. Others run fine. [VERIFIED (anecdotal)](https://discourse.zynthian.org/t/log-warnings-with-64gb-sandisk-extreme/11588)
- **NVMe** via M.2 HAT+ or Pimoroni NVMe Base gives much better I/O and endurance for logging, rosbag recording and Docker layers. Gen 3 is generally reported stable on the NVMe Base. [VERIFIED (secondary)](https://raspberry.tips/en/raspberrypi-tutorials/nvme-ssd-raspberry-pi-5-buying-guide) **But** it competes with the AI HAT+ for the single PCIe lane (§7.5) and adds height and power. The NVMe Base sits *under* the Pi, which may suit the chassis better [INFERRED].
- **Robot advice** [INFERRED]:
  - Start on SD.
  - Mount `/var/log` as tmpfs or use `log2ram`.
  - Write rosbags to a USB 3 SSD only when needed.
  - **Always shut down cleanly.** Wire a "low battery → `systemctl poweroff`" path from the HAT's battery ADC.
  - Keep a spare pre-imaged card per robot.

---

## 5. ROS 2 on the Pi 5

### 5.1 Which distro (as of Oct 2026)

| Distro | Released | EOL | Tier-1 arm64 | Notes |
|---|---|---|---|---|
| **Jazzy Jalisco** | May 2024 | May 2029 | Ubuntu 24.04 | Largest package ecosystem today; most LiDAR drivers target Humble/Jazzy |
| Kilted Kaiju | May 2025 | about Nov 2026 (non-LTS) | Ubuntu 24.04 | First release with **rmw_zenoh Tier 1**. Not worth adopting now. [VERIFIED](https://www.openrobotics.org/blog/2025/5/23/ros-2-kilted-kaiju-released) |
| **Lyrical Luth** | 22 May 2026 | May 2031 | **Ubuntu 26.04** (amd64+arm64); Debian Trixie is Tier 3 | [VERIFIED](https://discourse.openrobotics.org/t/ros-2-lyrical-luth-released/55021). Nav2 Lyrical binaries were announced Aug 2026 (ros-lyrical-nav2-* 1.5.1). [VERIFIED](https://discourse.openrobotics.org/t/nav2-lyrical-release-key-features/57666) |

The Kilted EOL date is [INFERRED] from the standard 18-month non-LTS cycle.

**Recommendation:**
- **Start with Jazzy** because of driver availability (LiDAR, camera_ros, slam_toolbox, Nav2 are all proven).
- **Plan to move to Lyrical in 2027** once your specific drivers have Lyrical binaries on arm64. Check `apt-cache policy ros-lyrical-<pkg>`; the August build notes I saw mentioned amd64 [INFERRED that arm64 follows].
- The default RMW in both Jazzy and Lyrical is still Fast DDS. [VERIFIED](https://discourse.openrobotics.org/t/upcomming-rmw-feature-freeze-april-6th-2026-ros-lyrical/53696.md)

### 5.2 Native vs Docker on Raspberry Pi OS
- **Official Docker images** exist (`ros:jazzy-ros-core`, `-ros-base`), and the ROS docs point Pi users to them. [VERIFIED (search summary of docs.ros.org Installing-on-Raspberry-Pi)](https://docs.ros.org/en/jazzy/How-To-Guides/Installing-on-Raspberry-Pi.html)
- **Community native apt for Pi OS Trixie arm64 ("rospian")**, announced Jan 2026, is unofficial. [VERIFIED](https://discourse.openrobotics.org/t/native-ros-2-jazzy-debian-packages-for-raspberry-pi-os-debian-trixie-arm64/51965)
- **Docker overhead** [INFERRED]:
  - CPU overhead with `--net=host` is negligible.
  - The main costs are SD-card I/O for image pulls (use a local registry or pre-load images) and device and permission plumbing (`/dev/i2c-1`, `/dev/gpiochip0`, `/dev/ttyUSB*`, `/dev/video*`, `/dev/media*`, `/dev/hailo0`).

### 5.3 Nav2 / slam_toolbox performance on a Pi 5
I found no controlled benchmark. These are the best data points:
- **Pi 5 8GB, Ubuntu 24.04, Jazzy, Nav2 + AMCL + 2D LiDAR.** Total CPU 38.5%, but the 1-min load was 4.37 (about 109% of 4 cores), at 66 °C with no throttling, using 1.4 GB RAM. The author used Fast DDS with a **Discovery Server**. Mid-mission failures were linked to Fast DDS CPU spikes when nodes start (rmw_fastrtps #741). A commenter's TurtleBot setup runs at about 40–50% CPU with full Nav2 + AMCL. [VERIFIED](https://discourse.openrobotics.org/t/turtlebot4-navigation-tuning-result/53609)
- **slam_toolbox async on a Pi 5 (Humble, replayed topics)** used about 1% of one core and about 500 MB. [VERIFIED (anecdotal)](https://forum.dexterindustries.com/t/raspberry-pi-5-continued-ros-2-humble-in-docker-on-pios-bookworm/9561)
- **Takeaway** [INFERRED]: SLAM + Nav2 fits on a Pi 5 8GB, but with little headroom for on-board CV and LLM work. Typical tuning:
  - lower costmap update rates;
  - use the MPPI controller with fewer samples, or Regulated Pure Pursuit;
  - use smaller local costmaps;
  - run perception on the Hailo NPU;
  - optionally offload global planning or LLM reasoning to a base-station PC (§8).

### 5.4 RMW: Zenoh vs DDS over Wi-Fi
- **rmw_zenoh** turns off multicast discovery and uses a **router** (`ros2 run rmw_zenoh_cpp rmw_zenohd`, TCP 7447 by default). It's explicitly recommended for Wi-Fi and other networks where multicast is unreliable. [VERIFIED](https://www.stereolabs.com/docs/integrations/ros-2/using-zenoh-as-middleware) It has been Tier 1 since Kilted, with a Jazzy binary also available [INFERRED: check `ros-jazzy-rmw-zenoh-cpp` exists on your platform].
- **DDS on Wi-Fi.** Discovery is peer-to-peer with O(N²) traffic and fragile UDP multicast. [VERIFIED](https://roscon.ros.org/2024/talks/RMW_Zenoh-_An_alternative_middleware_for_ROS_2.pdf) A 2026 arXiv paper models Fast DDS "discovery storms" on wireless. [VERIFIED (exists)](https://arxiv.org/pdf/2608.02242) The mitigations are a Fast DDS Discovery Server or a Cyclone DDS unicast peer list.
- **Fleet architecture suggestion** [INFERRED]:
  - Each robot runs a local Zenoh router: intra-robot traffic stays local, shared memory where possible.
  - The robot routers connect to a base-station router.
  - Namespace each robot (`/robot1/...`), or use separate domains with selective bridging.
  - This maps cleanly to the UAV phase, where links are lossy and intermittent.
  - Disable Wi-Fi power save (§8.1).

### 5.5 micro-ROS / MCU offload
- The robust pattern is for the Pi to run high-level ROS while an MCU (RP2040/RP2350 Pico, ESP32) does real-time motor PID, encoder counting, ultrasonic timing and IMU sampling over UART/USB, either with micro-ROS or a simple serial protocol. Commercial example: Yahboom's ESP32-S3 micro-ROS board with a 4-channel encoder motor driver + IMU for Pi 5. [VERIFIED](https://ca.robotshop.com/products/yahboom-micro-ros-control-board) A DigiKey tutorial (Dec 2025) shows a Pico as a "deterministic motor control layer". [VERIFIED](https://www.digikey.com/en/maker/tutorials/2025/intro-to-ros-part-12-talking-to-microcontrollers)
- **Why this matters for you** [INFERRED]: Linux user-space GPIO can't reliably count TT-motor encoder pulses or time HC-SR04 echoes at µs precision under ROS load. For accurate odometry on mecanum wheels, which already slip, put encoders and the IMU on an MCU. The same split (flight controller + companion computer) is exactly what you'll use on the UAVs (PX4/ArduPilot + Pi/Jetson).

---

## 6. Camera stack

- **The stack is libcamera.**
  - `rpicam-apps` (formerly libcamera-apps) are the CLI tools.
  - **Picamera2** is the official Python library.
  - OpenCV can't capture from libcamera directly. Feed frames via Picamera2 arrays, GStreamer `libcamerasrc`, or camera_ros. [VERIFIED (libcamera maintainer)](https://lists.libcamera.org/pipermail/libcamera-devel/2024-November/046411.html)
- **Pi 5 ISP.** The new PiSP ISP lives in the BCM2712; the CSI receivers are on RP1 [INFERRED: architectural background]. Use the RPi fork of libcamera for full Pi camera support.
- **ROS 2: `camera_ros`** (Christian Rauch, v0.7.0). It publishes `image_raw`, `image_raw/compressed` and `camera_info`. The ROS binary is built against *upstream* libcamera and may lack full Raspberry Pi camera support. On Ubuntu you must build the **raspberrypi libcamera fork** from source; the blocker is missing V4L2 "embedded data" support upstream. A community guide for Pi 5 + Ubuntu 24.04 + Jazzy exists (ARLunan/Raspberry-Pi-Camera-ROS). [VERIFIED](https://docs.ros.org/en/jazzy/p/camera_ros), [VERIFIED](https://discourse.openrobotics.org/t/installation-and-configuration-of-the-raspberry-pi-camera-on-a-ros-2-jazzy-raspberry-pi-5/45177) On Raspberry Pi OS the RPi libcamera is native, which is another argument for Option A [INFERRED].
- **Latency.** I found no verified glass-to-glass number for Picamera2 on Pi 5. One user's best Pi 5 + Camera Module 3 **network** streaming setup reached about 200 ms end-to-end, using WebRTC/RTSP comparisons. [VERIFIED (single blog)](https://gektor650.medium.com/comparing-video-stream-latencies-raspberry-pi-5-camera-v3-a8d5dad2f67b) [INFERRED] For control loops, process frames on-board, which avoids encode/network latency. Remember that H.264 encode is CPU-only on Pi 5, so stream MJPEG or low-res compressed topics, or use WebRTC sparingly.
- **USB (UVC) cameras** work via V4L2 (`v4l2_camera`, `usb_cam`). They count against the 600 mA/1.6 A USB budget and add USB bandwidth and CPU decode for MJPEG [INFERRED]. They're useful for a second wide-angle camera.
- **Cables.** Pi 5 needs a 22-pin (0.5 mm) to 15-pin (1 mm) **camera** cable for standard Pi cameras. Display cables are different and not interchangeable. Lengths are 200/300/500 mm. [VERIFIED](https://thepihut.com/products/camera-adapter-cable-for-raspberry-pi-5)

---

## 7. On-device AI

### 7.1 Options at a glance

| Device | Interface | Key numbers | Price | Fit for these robots |
|---|---|---|---|---|
| Pi 5 CPU only | n/a | YOLO26n NCNN 67 ms/inference; YOLO11n ONNX 6.8 FPS; LLM 1.5B about 10 tok/s | $0 | Baseline; fine for slow perception and small LLMs |
| **AI HAT+ 13 TOPS (Hailo-8L)** | PCIe FFC + header (stacking header) | YOLOv8l 15.0 FPS NPU-async, 8.7 FPS end-to-end sync; YOLO26n 78.5 FPS async; about 6.1 W system | about $70 at launch (check) | Good budget vision |
| **AI HAT+ 26 TOPS (Hailo-8)** | same | YOLOv8l 19.6 FPS async; YOLO26n 96.7 FPS async; DeepLabV3 121.9 FPS; about 6.5 W system | about $110 at launch (check) | **Best vision value** |
| AI HAT+ 2 (Hailo-10H, 40 TOPS INT4, 8 GB onboard) | same | CV about equal to Hailo-8. LLM tok/s *lower* than Pi CPU (e.g. Qwen2.5-1.5B: 6.74 vs 11.73) | $130 launch, +$50 in Apr 2026 | Not recommended for this project |
| Raspberry Pi AI Camera (Sony IMX500) | CSI | On-sensor inference (MobileNet etc.); 12 MP; 2028×1520 @ 30 fps; host CPU stays free | $70 | Interesting: zero PCIe/header use, but less capable than Hailo |
| Google Coral USB | USB | Official PyCoral wheels only for Python ≤ 3.9; community builds needed | n/a | Legacy; avoid for new work |

Sources: [Sixfab Hailo benchmark, Jul 2026](https://sixfab.com/blog/raspberry-pi-5-ai-hat-benchmark-deepx-vs-hailo/), [CNX AI HAT+ 2 review](https://www.cnx-software.com/2026/01/20/raspberry-pi-ai-hat-2-review-a-40-tops-ai-accelerator-tested-with-computer-vision-llm-and-vlm-workloads/), [Phoronix AI HAT+ 2 launch](https://phoronix.com/news/Raspberry-Pi-AI-HAT-2), [Ultralytics Pi guide](https://docs.ultralytics.com/guides/raspberry-pi), [AI Camera (CNX)](https://www.cnx-software.com/2024/10/01/raspberry-pi-ai-camera-with-sony-imx500-ai-sensor-and-rp2040-mcu/), [Coral on Pi 5](https://www.jeffgeerling.com/blog/2023/pcie-coral-tpu-finally-works-on-raspberry-pi-5/). All VERIFIED; launch prices for the 13/26 TOPS boards come from secondary sources.

### 7.2 Important nuance from the Sixfab benchmark (VERIFIED)
NPU-only throughput, async with 8 in flight, is high. **End-to-end single-stream sync throughput** is about 9–15 FPS for YOLO-class detectors on any of these accelerators, because it includes video decode and CPU pre/post-processing. To get the NPU numbers you need an async or pipelined design: GStreamer/TAPPAS or HailoRT async APIs. [VERIFIED](https://sixfab.com/blog/raspberry-pi-5-ai-hat-benchmark-deepx-vs-hailo/)

### 7.3 CPU-only benchmarks (Pi 5)
- **YOLO (Ultralytics, FP32, 640 px, Pi OS Bookworm):** YOLO26n inference is **67.0 ms** with NCNN, 91.9 ms with MNN, 104.6 ms with OpenVINO, 126.0 ms with ONNX and 299 ms with PyTorch. Times exclude pre/post-processing. ONNX FPS: YOLO11n 6.79, YOLO26n 7.79. **Use NCNN.** [VERIFIED](https://docs.ultralytics.com/guides/raspberry-pi)
- **LLMs (Ollama, Q4_K_M):** qwen2.5:1.5b runs at **9.97 tok/s** in about 1.8 GB RAM; qwen2.5:3b at **5.2 tok/s** in about 3.0 GB. [VERIFIED](https://stratospherelinuxips.readthedocs.io/en/develop/immune/research_rpi_llm_performance.html) CNX measured (Ollama, CM5 = Pi 5 silicon):

  | Model | tok/s |
  |---|---|
  | DeepSeek-R1 1.5B | 9.04 |
  | Qwen2 1.5B | 11.29 |
  | Llama 3.2 3B | 4.78 |
  | Qwen2.5-Instruct 1.5B | 11.73 |

  System power was 10.2–10.6 W. [VERIFIED](https://www.cnx-software.com/2026/01/20/raspberry-pi-ai-hat-2-review-a-40-tops-ai-accelerator-tested-with-computer-vision-llm-and-vlm-workloads/) The NYU edge leaderboard lists Qwen2.5-3B Q4_K_M on Pi 5 8GB (llama.cpp) at 4.81 tok/s decode and 10.64 tok/s prefill. [VERIFIED (search summary)](https://stratospherelinuxips.readthedocs.io/en/develop/immune/research_rpi_llm_performance.html)
- **Interpretation** [INFERRED]: about 10 tok/s at 1.5B is enough for slow, event-driven "agent reasoning" on-board, like mission-step selection or parsing a command, but not for tight control loops. Prompt processing (prefill) is the bottleneck for long context. Keep prompts short and structured: tool-calling JSON with a small schema. Route heavy reasoning to a base-station GPU or the cloud when the link is available, and keep the on-board model as a fallback for comms-denied operation. That fallback matters for UAVs.
- **Whisper:** I found no rigorous Pi 5 whisper.cpp benchmark. Secondary sources say `tiny` runs faster than real time and `base` about real time with 4 threads. [VERIFIED (weak source)](https://fast.io/resources/openclaw-raspberry-pi-speech-recognition-offline-voice-agent/) Run `whisper.cpp`'s `bench` yourself.
- **Depth estimation:** I found no verified Pi 5 FPS numbers for Depth Anything V2 or SCDepthV3 on Hailo. An AAU master's thesis (2026) benchmarks monocular depth on Pi 5 + AI HAT+. [VERIFIED (exists)](https://projekter.aau.dk/projekter/files/835813981/Master_Thesis_Simao.pdf) [INFERRED] Treat monocular depth as a relative cue, not a metric sensor; the thesis abstract says the same. Use ToF or LiDAR for metric ranges.

### 7.4 Hailo software and OS
- Raspberry Pi OS supports the AI HAT+ family natively (rpicam-apps / Picamera2 post-processing). The AI HAT+ 2 requires Trixie 64-bit. [VERIFIED](https://www.raspberrypi.com/documentation/accessories/ai-hat-plus.html), [VERIFIED](https://www.cnx-software.com/2026/01/20/raspberry-pi-ai-hat-2-review-a-40-tops-ai-accelerator-tested-with-computer-vision-llm-and-vlm-workloads/)
- On Ubuntu, build the Hailo PCIe driver from source and pin it to the HailoRT version (Canonical's guide). There is **no official Hailo ROS 2 package**; only community wrappers exist. [VERIFIED](https://ubuntu.com/blog/hackers-guide-to-the-raspberry-pi-ai-kit-on-ubuntu), [VERIFIED](https://community.hailo.ai/t/is-there-an-official-ros2-package-for-running-yolo-on-raspberry-pi-5-with-ai-hat/12539)

### 7.5 Physical and electrical conflict with the Adeept Robot HAT
- **Header stack.** The AI HAT+ installs on 4 threaded spacers with a **GPIO stacking header**, plus a PCIe ribbon from the Pi's FFC. An Active Cooler under it is recommended. [VERIFIED](https://www.raspberrypi.com/documentation/accessories/ai-hat-plus.html) The AI HAT+ 2 box includes a 40-pin extension header. If you stack another HAT on top, don't press the extension header fully down, or the pins won't reach. [VERIFIED](https://www.cnx-software.com/2026/01/20/raspberry-pi-ai-hat-2-review-a-40-tops-ai-accelerator-tested-with-computer-vision-llm-and-vlm-workloads/)
- **So the physical order would be: Pi 5 → Active Cooler → AI HAT+ (on stacking header) → Adeept Robot HAT (on a second extension header or longer standoffs).** [INFERRED] Risks:
  1. Header pin engagement through two stacking headers.
  2. Total height in the chassis and camera-cable routing.
  3. The Robot HAT's battery and motor wiring sits on top.
  4. Heat trapped under two boards.
  5. The Robot HAT's DC-DC must now also power the Hailo (about +1–3 W).
- **Electrical conflicts** [INFERRED]: the AI HAT+ uses the 40-pin header mostly for power, ID EEPROM and mechanics, and talks over PCIe. The Robot HAT mainly uses I2C (PCA9685, ADC), some GPIO, UART and WS2812. A pin conflict is unlikely, but **check the HAT+ ID EEPROM I2C pins (GPIO 0/1)** and any GPIO the Adeept HAT uses against the AI HAT+ pinout.
- **Alternatives:**
  - (a) Use the **Raspberry Pi AI Camera**: no header or PCIe use, and it fits the existing camera mount.
  - (b) Use a **bottom-mount PCIe board** (Pimoroni-style base) with an M.2 Hailo module, keeping the 40-pin header free. Check that a Hailo M.2 B+M-key module and a base board are compatible.
  - (c) If you want **NVMe + Hailo together**, use a PCIe-switch board: Pineboards AI Bundle (Hailo-8L + NVMe, ASM1182e, about €90 ex VAT) or Seeed's dual-M.2 PCIe 3.0 HAT (ASM2806; Seeed reported 416 MB/s SSD + Hailo-8 at 187 FPS concurrently). [VERIFIED](https://www.cnx-software.com/2024/08/01/pineboards-introduces-ai-bundle-hailo-8l-raspberry-pi-hat-with-ai-accelerator-and-nvme-ssd-support/), [VERIFIED](https://core-electronics.com.au/seeed-studio-pcie-3-0-to-dual-m-2-hat-for-raspberry-pi-5.html.plain.md)

**Recommendation** [INFERRED]: Don't buy accelerators until the base robots drive, map and stream reliably. When you do, start with **one** AI HAT+ 26 TOPS **or** one AI Camera on a single robot, and compare against CPU NCNN for your actual mission model.

---

## 8. Networking for a 3-robot fleet

### 8.1 Wi-Fi
- **Throughput.** In one home test, Pi 5 iperf3 on 5 GHz reached **about 217 Mbit/s** near the router, versus 64 Mbit/s for Pi 4. The advantage shrank with distance. [VERIFIED](https://www.tomshardware.com/news/raspberry-pi-5-wi-fi-faster)
- **Power save.** Turn off Wi-Fi power save: `iw dev wlan0 set power_save off`, or the raspi-config **A12 WLAN Power Save** option on Trixie, which persists. Measure latency before and after. [VERIFIED](https://raspberrytips.com/raspberrypi-wifi-fixes/)
- **Recommendation** [INFERRED]:
  - Give the fleet a **dedicated 5 GHz access point** (a travel router or a Wi-Fi 6 AP) on its own SSID and channel.
  - Use static DHCP leases and a base-station PC on Ethernet.
  - The onboard antenna is a PCB/chip antenna inside a metal-ish chassis, so place the Pi so the antenna area isn't shielded by batteries or the HAT.
  - A USB Wi-Fi dongle with an external antenna is an option, at the cost of USB power budget.

### 8.2 Mesh / ad-hoc
- **batman-adv over IBSS (ad-hoc)** is the commonly documented Pi path. Whether the onboard brcmfmac supports 802.11s mesh is **unclear**; one report says Pis didn't support 802.11s. [VERIFIED (weak)](https://www.redmine.org/boards/1/topics/68524)
- OpenMANET and similar projects use separate Wi-Fi HaLow HATs for 802.11s. [VERIFIED](https://github.com/OpenMANET/docs)
- **Advice** [INFERRED]: for 3 ground robots indoors, infrastructure Wi-Fi + Zenoh is far simpler. Revisit mesh (HaLow, or dedicated mesh radios) for the UAV phase.

### 8.3 Time sync
- **chrony.** Use chrony on all robots, pointing at the base station as the NTP server [INFERRED: standard practice]. Expect roughly ms-level sync over Wi-Fi [INFERRED]. That's adequate for ROS TF and message timestamps at robot speeds.
- **PTP.** The Pi 5's Ethernet supports PTP hardware timestamping (`ptp4l` + `phc2sys`). [VERIFIED](https://canonical-industrial-documentation.readthedocs-hosted.com/en/latest/iot-communication-protocols/how-to/rpi5-ptp-time-sync) This is only useful when wired, e.g. a docking station or bench calibration; Wi-Fi PTP is not hardware-timestamped [INFERRED].
- **RTC.** Fit an RTC battery (Li-Mn, J5) so logs have sane time before network sync. [VERIFIED](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html)

### 8.4 Remote dev workflow (C#/.NET-friendly) [INFERRED unless linked]
- **VS Code Remote-SSH** into each robot works on arm64 Linux. Alternatively, develop on Windows and deploy.
- **Cross-publish .NET:** `dotnet publish -c Release -r linux-arm64 --self-contained false` (needs the .NET runtime on the Pi), or `--self-contained true -p:PublishSingleFile=true`. Native AOT for linux-arm64 needs an arm64 Linux toolchain, so build it in an arm64 Docker container or on a Pi.
- **Docker multi-arch images:** `docker buildx build --platform linux/arm64` on Windows with QEMU, or a native arm64 builder (one of the Pis, or an Arm cloud VM). Push to a LAN registry and `docker compose pull` on each robot.
- **Remote debugging:** use `vsdbg` over SSH (VS Code `pipeTransport`) or Visual Studio's SSH attach [INFERRED].
- **ROS 2 + .NET** [INFERRED]: there's no Tier-1 C# client library. Options:
  - Write ROS nodes in Python or C++, and keep the .NET "agent brain" as a separate process talking over Zenoh directly. Zenoh has C, Python and Rust APIs, and a C# binding should be checked for maturity. Other transports are gRPC or a rosbridge/WebSocket.
  - Community `ros2_dotnet` / `ros2cs` exist; check their Jazzy/Lyrical status before depending on them.

---

## 9. Recommended accessories (prices vary; verify at purchase)

| Item | Why | Price seen | Status |
|---|---|---|---|
| **Raspberry Pi Active Cooler** (x3) | Mandatory for sustained load | $5 launch | [VERIFIED](https://www.raspberrypi.com/news/introducing-raspberry-pi-5/) |
| **Pi 5 22→15-pin camera cable** (if not in the kit) | Kit camera compatibility | $1 (200 mm) at launch; Adafruit #5818 | [VERIFIED](https://www.adafruit.com/product/5818) |
| **RTC battery** (Li-Mn) | Correct time before NTP | $5 launch | [VERIFIED](https://www.raspberrypi.com/news/introducing-raspberry-pi-5/) |
| **Official 27W USB-C PSU** (x3) | Bench/tethered development with full 5A | $12 launch | [VERIFIED](https://www.raspberrypi.com/news/introducing-raspberry-pi-5/) |
| **5V/5A+ buck** (Pololu D36V50F5 or DFRobot DFR0753) | Battery → Pi if the HAT's DC-DC is weak | about $20–30 | [VERIFIED (spec)](https://www.pololu.com/product/4091/specs) |
| **NVMe**: Pimoroni NVMe Base (about €33) / official M.2 HAT+ (about £10) / SSD kit 256 GB (about €55) | Logging/rosbag/Docker I/O; conflicts with the AI HAT+ unless using a switch | prices from July 2026 | [VERIFIED](https://raspberry.tips/en/raspberrypi-tutorials/nvme-ssd-raspberry-pi-5-buying-guide) |
| **LiDAR: RPLIDAR C1** | DTOF, 0.05–12 m (6 m on black), 5 kHz, 10 Hz, 0.72°, UART 460800, 1.15 W. Driver: Slamtec `sllidar_ros2` (C1 launch file `view_sllidar_c1_launch.py`); `rplidar_ros` on Jazzy is stale | about $99 (Yahboom); A$130–141 | [VERIFIED](https://www.waveshare.com/wiki/RPLIDAR_C1), [VERIFIED](https://github.com/Slamtec/sllidar_ros2), [VERIFIED](https://discourse.ros.org/t/rplidar-driver-incorrect-version-in-repos/43827) |
| **LiDAR: LDRobot LD19 / STL-19P (D500 kit)** | DTOF, 0.03–12 m, 5 kHz, 10 Hz, UART 230400, 1.45 W. Drivers: `ldlidar_stl_ros2` (upstream reportedly fails on new GCC; use a patched fork), `Myzhar/ldrobot-lidar-ros2` (Humble-targeted) | LD06 about $99; varies | [VERIFIED](https://www.waveshare.com/wiki/D500_LiDAR_Kit), [VERIFIED](https://github.com/kaiaai/awesome-2d-lidars) |
| LiDAR: YDLIDAR X2/X3/T-mini Plus | Cheap (X3 about $65, T-mini Plus about $71–79). X2/X3 are triangulation, 8 m. `ydlidar_ros2_driver` needs YDLidar-SDK; Jazzy is unverified | | [VERIFIED](https://github.com/kaiaai/awesome-2d-lidars) |
| **IMU: BNO085/BNO086** | On-chip fusion, which is easy. ROS 2 `bno08x_driver` (I2C). Watch out for clock stretching on RP1; prefer UART-RVC or SPI | breakout about $25–45 | [VERIFIED](https://docs.ros.org/en/humble/p/bno08x_driver/) |
| IMU: ICM-20948 | Raw 9-DoF, so you fuse with `imu_filter_madgwick` or `robot_localization`. No clock-stretch issue | generic module about $12 | [VERIFIED (price)](https://www.walmart.com/c/kp/9-axis-imu) |
| **ToF: VL53L1X** | 30–4000 mm, up to 50 Hz, I2C; cheap close-range obstacle sensing around the chassis | Adafruit #3967 $14.95; Pololu #3415 about $23 | [VERIFIED](https://www.adafruit.com/product/3967) |
| **Encoder TT motors** (1:48, Hall AB) | Real odometry for mecanum | about €11–12 each (Funduinoshop); SparkFun DG01D-E $12.50 | [VERIFIED](https://www.sparkfun.com/hobby-motor-with-encoder-metal-gear-dg01d-e.html) |
| **MCU for real-time I/O** (Pico 2 / ESP32-S3) | Encoders, PID, ultrasonic, IMU; micro-ROS | a few $; Yahboom micro-ROS board is a turnkey option | [VERIFIED](https://ca.robotshop.com/products/yahboom-micro-ros-control-board) |

**LiDAR pick** [INFERRED]: the **RPLIDAR C1** has the most mature vendor-maintained ROS 2 driver, low power and DTOF accuracy. The LD19/STL-19P is a close second. Mount it high and centred, above the Robot HAT, with the USB-UART adapter. Its about 1.2–1.5 W fits within the USB budget even at 600 mA.

---

## 10. Pi 5 gotchas specific to mobile robots

1. **The 3A assumption on non-PD power.** USB is limited to 600 mA, so a USB LiDAR + USB camera + dongle can brown out the USB ports. Set `PSU_MAX_CURRENT=5000` only if your rail really delivers it. [VERIFIED](https://pip-assets.raspberrypi.com/categories/685-app-notes-guides-whitepapers/documents/RP-009856-WP-1-USB%20Power%20delivery%20on%20Raspberry%20Pi%205.pdf)
2. **Motor noise and sag reaching the Pi's 5V.** Separate the regulators, add bulk capacitance, use a star ground, and twist motor leads [INFERRED]. Watch `vcgencmd get_throttled` in your telemetry. Make it a ROS diagnostic topic.
3. **The GPIO 5V input bypasses protection.** Over-voltage kills the Pi. [VERIFIED (secondary)](https://bret.dk/?p=7113)
4. **Heat under stacked HATs.** Active Cooler plus clearance is mandatory, especially with Hailo. [VERIFIED](https://core-electronics.com.au/guides/answering-your-pi-5-questions-cooling-overclocking-power-and-more/)
5. **The camera connector and cable type.** Use 22-pin mini to 15-pin, and the camera (not display) cable. [VERIFIED](https://thepihut.com/products/camera-adapter-cable-for-raspberry-pi-5)
6. **Legacy kit code.** RPi.GPIO, pigpio and rpi_ws281x need replacements on Pi 5. [VERIFIED](https://www.pi4j.com/documentation/providers/pigpio)
7. **gpiochip numbering changed (4 → 0).** Look up the chip by label. This bit .NET users in late 2024. [VERIFIED](https://github.com/dotnet/iot/issues/2366)
8. **I2C clock stretching** with the BNO08x on RP1. [VERIFIED (project report)](https://codeberg.org/buildacubesat-project/bac-hardware/issues/45)
9. **One PCIe lane.** NVMe *or* Hailo unless you add a switch. [VERIFIED](https://www.cnx-software.com/2026/01/20/raspberry-pi-ai-hat-2-review-a-40-tops-ai-accelerator-tested-with-computer-vision-llm-and-vlm-workloads/)
10. **No hardware H.264 encoder.** Video streaming costs CPU; budget for it or stream low-res MJPEG. [VERIFIED (secondary)](https://forum.jellyfin.org/t-high-cpu-load)
11. **SD corruption on battery death.** Add a battery-low shutdown, an overlay or read-only root (raspi-config overlay FS) for "production" runs, and spare cards. `sd_cqe=0` if you see CQE recovery messages. [VERIFIED (anecdotal)](https://discourse.zynthian.org/t/log-warnings-with-64gb-sandisk-extreme/11588)
12. **Halt still draws about 1–1.4 W**, which slowly drains and can over-discharge packs. Set `POWER_OFF_ON_HALT=1`, and use a physical battery switch. [VERIFIED](https://www.raspberrypi.com/documentation/computers/raspberry-pi.html)
13. **Wi-Fi power save and multicast discovery.** Disable power save and use Zenoh, or a DDS discovery server. [VERIFIED](https://www.stereolabs.com/docs/integrations/ros-2/using-zenoh-as-middleware)
14. **Fast DDS CPU spikes when nodes start mid-mission** were seen on a Pi 5 Nav2 robot. [VERIFIED](https://discourse.openrobotics.org/t/turtlebot4-navigation-tuning-result/53609)
15. **Ubuntu 26.04 needs a recent EEPROM** (≥ 2025-02-11). Update the bootloader first from Pi OS or 24.04. [VERIFIED](https://ubuntu.com/hardware/docs/boards/how-to/ubuntu_supported/raspberry-pi/)
16. **Board-revision and stepping differences.** The 2GB D0 stepping uses less power than the 8GB C1. Your 3 boards may differ, so log `cat /proc/cpuinfo` revision per robot. [VERIFIED](https://www.jeffgeerling.com/blog/2024/new-2gb-pi-5-has-33-smaller-die-30-idle-power-savings/)

---

## 11. Suggested first-week plan (per robot) [INFERRED]

1. On the bench, with the 27W PSU: flash **Raspberry Pi OS Trixie Lite 64-bit**. Update the EEPROM, fit the Active Cooler and RTC battery, enable I2C, set the hostname to `robotN`, and add SSH keys.
2. Run Adeept samples with `rpi-lgpio` (or port them to gpiozero) to validate motors, servos, ultrasonic and LEDs.
3. **Measure power:**
   - put an inline USB meter on the bench supply;
   - measure the HAT's 5V rail on battery while running `stress-ng --cpu 4` + camera + motors;
   - log `vcgencmd get_throttled` and decide whether you need a separate buck.
4. Install Docker and run `ros:jazzy-ros-base` with `--net=host`. Bring up `camera_ros` (or a host Picamera2 publisher), teleop, and `rmw_zenoh` with a router on your PC.
5. Write the first C# service: System.Device.I2c + Iot.Device.Pca9685 motor/servo control behind a small API, with an explicit LibGpiod driver for the ultrasonic trigger/echo or offloaded to a Pico.
6. Then add a LiDAR to one robot, then slam_toolbox, then Nav2. Measure CPU, temperature and battery per stage before replicating to the other two.

---

## Sources (primary links used)
- Raspberry Pi docs, Pi 5 hardware/config: https://www.raspberrypi.com/documentation/computers/raspberry-pi.html
- Raspberry Pi 5 announcement: https://www.raspberrypi.com/news/introducing-raspberry-pi-5/
- RPi whitepaper "USB Power Delivery on Raspberry Pi 5" (Mar 2026): https://pip-assets.raspberrypi.com/categories/685-app-notes-guides-whitepapers/documents/RP-009856-WP-1-USB%20Power%20delivery%20on%20Raspberry%20Pi%205.pdf
- Jeff Geerling Pi 5 power by model: https://www.jeffgeerling.com/blog/2024/new-2gb-pi-5-has-33-smaller-die-30-idle-power-savings/
- raspberrypi/linux PR #6144 (gpiochip renumbering): https://github.com/raspberrypi/linux/pull/6144
- dotnet/iot releases and issue #2366: https://github.com/dotnet/iot/releases, https://github.com/dotnet/iot/issues/2366
- PIOLib: https://www.raspberrypi.com/news/piolib-a-userspace-library-for-pio-control/
- ROS 2 Lyrical release: https://discourse.openrobotics.org/t/ros-2-lyrical-luth-released/55021 ; Kilted / Zenoh Tier 1: https://www.openrobotics.org/blog/2025/5/23/ros-2-kilted-kaiju-released
- Nav2 on Pi 5 field report: https://discourse.openrobotics.org/t/turtlebot4-navigation-tuning-result/53609
- camera_ros docs: https://docs.ros.org/en/jazzy/p/camera_ros
- Sixfab Hailo/DEEPX benchmark (Jul 2026): https://sixfab.com/blog/raspberry-pi-5-ai-hat-benchmark-deepx-vs-hailo/
- CNX AI HAT+ 2 review (Jan 2026): https://www.cnx-software.com/2026/01/20/raspberry-pi-ai-hat-2-review-a-40-tops-ai-accelerator-tested-with-computer-vision-llm-and-vlm-workloads/
- Ultralytics Raspberry Pi guide: https://docs.ultralytics.com/guides/raspberry-pi
- Pi 5 LLM study (Ollama): https://stratospherelinuxips.readthedocs.io/en/develop/immune/research_rpi_llm_performance.html
- Ubuntu on Raspberry Pi: https://ubuntu.com/hardware/docs/boards/how-to/ubuntu_supported/raspberry-pi/
- Raspberry Pi OS Trixie (CNX): https://www.cnx-software.com/2025/10/06/raspberry-pi-os-debian-13-trixie/
- Adeept Robot HAT (CNX): https://www.cnx-software.com/2024/07/29/adeept-robot-hat-for-raspberry-pi-is-designed-for-diy-projects-and-educational-needs/
- LiDAR list: https://github.com/kaiaai/awesome-2d-lidars ; RPLIDAR C1 wiki: https://www.waveshare.com/wiki/RPLIDAR_C1 ; D500/STL-19P wiki: https://www.waveshare.com/wiki/D500_LiDAR_Kit
- Pi 5 PTP (Canonical): https://canonical-industrial-documentation.readthedocs-hosted.com/en/latest/iot-communication-protocols/how-to/rpi5-ptp-time-sync
- Price changes 2026: https://www.gigazine.net/gsc_news/en/20260402-raspberry-pi-price-increases
