# ROS 2 on the PC (WSL2)

ROS 2 **Humble** in a separate WSL distro, **Ubuntu-22.04**, so the PC sees the cars' topics and runs RViz2.
- **Why Humble:** the cars run Humble. Humble only supports Ubuntu 22.04. The existing `Ubuntu` distro is 24.04, which would mean ROS 2 Jazzy, and Jazzy ↔ Humble isn't supported.
- **What stays untouched:** the existing distros (Ubuntu 24.04, the two NVIDIA JetPack ones, docker-desktop).

## 1. Install the distro (normal PowerShell)
```powershell
wsl --install -d Ubuntu-22.04      # asks for a Linux user name and password
```

## 2. Mirrored networking (affects ALL WSL distros and Docker Desktop)
ROS 2 finds other machines by UDP multicast. In WSL's default NAT mode the PC hides WSL behind a private address (172.23.x.x), so the cars can't see it. Mirrored mode gives WSL the PC's own LAN and Wi-Fi addresses.

Your `%USERPROFILE%\.wslconfig` already has a `[wsl2]` section with NVIDIA's JetPack kernel.
1. Back the file up first.
2. Add the `networkingMode=mirrored` line under that existing `[wsl2]` section; there must be only one `[wsl2]` header.
3. Restart WSL:
```powershell
Copy-Item $env:USERPROFILE\.wslconfig $env:USERPROFILE\.wslconfig.bak
# add under the existing [wsl2] section:
#   networkingMode=mirrored
wsl --shutdown
```
- **To undo:** restore the `.bak` and run `wsl --shutdown`.
- **Risk:**
  - Mirrored mode needs a recent WSL kernel. NVIDIA's custom kernel may not support it.
  - If `ip -4 addr` inside WSL then doesn't show the PC's 10.10.x / 192.168.149.x addresses, mirrored mode isn't active.
  - Check that your JetPack and Docker work still behaves.

## 3. Firewall (PowerShell as Administrator)
Mirrored mode puts WSL behind the Hyper-V firewall. Allow ROS 2 DDS traffic. For domain 0, discovery is UDP 7400–7401 plus the unicast ports just above them; 7400–7700 covers domains 0–1 with room to spare.
```powershell
New-NetFirewallHyperVRule -Name ROS2-DDS-in -DisplayName "ROS 2 DDS (WSL)" -Direction Inbound `
  -VMCreatorId '{40E0AC32-46A5-438A-A0B2-2B479E8F2E90}' -Protocol UDP -LocalPorts 7400-7700 -Action Allow
```
- **To undo:** `Remove-NetFirewallHyperVRule -Name ROS2-DDS-in`

## 4. ROS 2 Humble (inside Ubuntu-22.04)
```bash
bash /mnt/d/PlayGround/RoboticProject/car/pc/setup_ros2_humble.sh
```

## 5. The car's message types
Hiwonder's custom messages (`ros_robot_controller_msgs`, used for battery, motors and servos) aren't in ROS 2. Copy them from a car into a local workspace and build them. Hiwonder's code has no license, so it goes in `~/ros2_ws` in WSL, never into this repo.
```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
scp -r pi@10.10.77.35:/home/pi/docker/tmp/ros_robot_controller_msgs .   # after exporting it from the container, see below
cd ~/ros2_ws && colcon build && source install/setup.bash
```
To export it on the car host: `sudo docker cp MentorPi:/home/ubuntu/ros2_ws/src/driver/ros_robot_controller_msgs /home/pi/docker/tmp/`

## 6. Check
```bash
ros2 topic list                       # the car's topics: /scan_raw, /odom, /ros_robot_controller/battery, ...
ros2 topic echo --once /ros_robot_controller/battery
rviz2                                 # Fixed frame "map" (when mapping) or "odom"; add LaserScan /scan_raw, Map /map, Image
```

## Notes
- **Same network:** the PC has to be on the same network as the car: the office LAN, or the car's hotspot.
- **One shared ROS domain:** both cars currently use ROS domain 0 with the same topic names. With both on, the PC (and each car) sees the two mixed together, and a `cmd_vel` reaches both cars. Separate them before running two cars (open-issues #7).
- **Driving from the PC:** the car's command watchdog (`car/watchdog/`) stops the car if no drive command arrives for 0.5 s, so a sender has to repeat its command faster than that. Check how a teleop tool sends before relying on it; `teleop_twist_keyboard` only sends when a key event arrives.
