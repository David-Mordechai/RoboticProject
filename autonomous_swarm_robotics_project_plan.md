# Autonomous Swarm Robotics Project: Architecture & Hardware Plan

## 1. Project Overview
The objective is to build a decentralized, autonomous swarm of three intelligent robotic agents. Moving away from a centralized command architecture, each robot operates as an independent Edge computing node. They will map unknown indoor environments (like homes or offices), share data peer-to-peer (P2P), and negotiate tasks autonomously without relying on a central server.

## 2. Software & System Architecture (Decentralized Edge)
Each robot runs an identical, fully containerized software stack to ensure true autonomy and redundancy.

*   **Core Logic (Execution Agents):** Written in C# / .NET 8 or 9. These agents handle decision-making, path planning, and sensor data processing.
*   **Middleware (ROS 2):** Utilizing ROS 2 (DDS protocol) over Wi-Fi for P2P communication. The robots will broadcast their physical positions (Odometry) and local map data to a shared network topic.
*   **Distributed Knowledge Base:** 
    *   Each agent runs a local **PostgreSQL** database (accessed via EF Core) to store waypoints, tasks, and telemetry.
    *   **MinIO** runs locally on each Pi for object storage (saving heavy image files or large map data).
    *   **Event Sourcing:** Data synchronization across the swarm is handled via broadcasting events. If an agent goes offline, it syncs up once the connection is restored.
*   **Collaborative Mapping & SLAM:** 
    *   Agents use **Frontier-Based Exploration** to identify unmapped areas (e.g., open doorways).
    *   Maps are generated locally and shared over the network. Each agent runs a Map Merging service to stitch the shared data into a complete Global Map.
*   **Autonomous Decision Making (Contract Net Protocol):** When a new unmapped area is discovered, agents negotiate via a bidding system. The agent with the lowest "cost" (e.g., closest distance, highest battery) claims the task, preventing duplicated efforts.

## 3. Hardware Bill of Materials (BOM)

### The Robotic Platform
*   **3x Adeept 4WD Mecanum Wheel Smart Car Kits**
    *   *Features:* Omnidirectional movement, Robot HAT (motor driver/power management), front camera on a pan/tilt servo mount, ultrasonic sensor, and OLED/LED matrix displays.

### Computing & Storage (The Edge Brain)
*   **3x Raspberry Pi 5 (8GB RAM)**
    *   Chosen for its massive performance leap over the Pi 4, necessary for running .NET containers, local databases, and heavy SLAM algorithms.
*   **3x SanDisk Extreme 128GB MicroSD Cards (A2 / V30 / U3)**
    *   *Crucial Spec:* The **A2 rating** is mandatory for running a full Ubuntu OS, ROS 2, and databases without I/O bottlenecks. 128GB provides ample space for logging and local storage.

### Power & Thermal Management
*   **6x 18650 High-Drain Lithium-Ion Batteries**
    *   Recommended brands: LG (e.g., HG2), Samsung, or Sony. Minimum 2 per car.
*   **1x 4-Channel External 18650 Smart Charger** (e.g., Nitecore, XTAR)
    *   *Safety Note:* Required to safely charge batteries outside the robots and enable hot-swapping for continuous development. The onboard USB charging on the Adeept kit has known thermal issues.
*   **3x Official Raspberry Pi 5 Active Coolers**
    *   Mandatory to prevent thermal throttling. The cooler connects to the Pi's dedicated fan header, leaving the 40-pin GPIO free for the Adeept Robot HAT.
*   **1x USB-C PD Power Supply (Reused from Jetson Orin Nano)**
    *   Will be used on the desk to power the Pi 5s directly from the wall during the initial OS and software setup phase (provides stable 15W/3A at 5V).

### Advanced Sensing Upgrade (For Reliable Mapping)
*   **3x 2D LiDAR Sensors** (e.g., RPLidar A1 or C1)
    *   *Why:* Visual SLAM (camera only) fails on blank walls and requires perfect lighting. 2D LiDAR guarantees millimeter-accurate, 360-degree room mapping regardless of lighting conditions.
*   **Brass Standoffs (M2.5 or M3, ~3-5cm long)**
    *   *Mechanical Mod:* Required to mount the LiDAR on the top acrylic plate, elevating it high enough so the laser clears the front camera/ultrasonic mount, ensuring a 360° clear line of sight.