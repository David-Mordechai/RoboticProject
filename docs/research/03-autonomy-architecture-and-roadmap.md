# 03: Autonomy Architecture, AI Agents and Learning Roadmap

*Research date: 2026-10-27. Scope: the software and architecture side of a 3-robot ground fleet (Adeept 4WD mecanum kits, Raspberry Pi 5 8 GB, camera and ultrasonic sensors, no LiDAR or encoders yet), used to learn how to build an autonomous, AI-driven multi-UAV framework later.*

> **How to read this document**
> - **[Fact]** claims come with a link. Where the sources were secondary (aggregators, blogs) or disagreed, the text says so.
> - **[Opinion]** marks my recommendations. They are deliberately opinionated.
> - **[Speculation]** marks extrapolation that no source in this research verifies.
> - Versions and dates were checked in October 2026. ROS packages move quickly, so re-check `index.ros.org` before you pin anything.

---

## 0. Executive summary (TL;DR)

1. **Architecture.** Build a layered stack with one narrow waist: a **vehicle-agnostic Skill API**, made of typed, long-running ROS 2 actions such as `NavigateTo`, `Explore`, `Search`, `Hold`, `ReturnHome` and `Capture`. Everything below the waist (drivers, estimation, control, Nav2 or PX4) is vehicle-specific. Everything above it (mission executive, fleet coordination, LLM reasoning, event log, learning loop) is reusable and is the part that will carry over to UAVs. This mirrors how the UAV world already works: MAVLink mission items and MAVSDK plugins, PX4 ROS 2 *modes and executors*, and Aerostack2 *behaviors*.
2. **ROS 2 distro.** **[Opinion]** Use **ROS 2 Jazzy (LTS to 2029) on Ubuntu 24.04** on the robots now. Jazzy has the most mature package set (Nav2, slam_toolbox, m-explore-ros2, rmw_zenoh binaries). Plan a move to **Lyrical Luth** (the new LTS, released May 2026, Ubuntu 26.04) in 2027 once the long tail of packages catches up. Use **rmw_zenoh** rather than DDS on Wi-Fi from day one of the multi-robot phase.
3. **Hardware gap.** The current kits can't run a real navigation stack. TT motors have no encoders, and an ultrasonic sensor plus a monocular camera can't drive slam_toolbox. **[Opinion]** The highest-leverage purchase is a **2D LiDAR and an IMU per robot** (LD19/LD06 or RPLIDAR C1 class, plus BNO085/ICM-20948 class). Next come encoder gear motors driven by a small micro-ROS MCU. Add an **overhead camera with AprilTags** as cheap ground truth for measuring accuracy.
4. **AI layer.** Follow **"LLM proposes, deterministic layer verifies and executes."** A cloud LLM (Claude Opus 5.5 for mission planning, Haiku 5.5 for cheap high-frequency summarisation) turns natural language into a **schema-validated mission plan** over the Skill API. A deterministic validator checks the plan (geofence, battery, capabilities, preconditions), dry-runs it in simulation, and puts risky actions behind human approval. A behavior tree executes it. The LLM never publishes `cmd_vel`. VLMs answer occasional semantic questions, in the cloud or on a laptop. A Pi 5 CPU takes about 10 to 50 s per VLM query. VLAs (pi0.5, SmolVLA) target manipulation and are mostly irrelevant for a mecanum base right now.
5. **Coordination.** **[Opinion]** Start with *centralized planning and allocation on a ground station, with decentralized execution and autonomous comm-loss behavior on each robot*. Then implement **Contract Net or sequential auctions** with Nav2 path cost as the bid. After that, add **CBBA** as the decentralized experiment that will matter for UAVs. Event-source the mission domain (append-only events in a store) and keep raw telemetry in **rosbag2/MCAP**.
6. **Self-improvement.** The realistic options are: (a) **automated parameter tuning** in simulation and on the real robots (Bayesian optimisation with Optuna); (b) **LLM post-mission reflection** that writes *proposals* (parameter changes, behavior-tree variants, "lessons" memory), each gated by a fixed regression benchmark; and (c) **fleet-shared knowledge**: maps, hazards, tuned parameters. RL in Isaac Lab or MuJoCo is good for learning and for *narrow* skills. End-to-end learned navigation on this hardware is mostly hype.
7. **.NET fits** the **ground station / fleet manager / mission service / event store / LLM orchestration / UI**, using **rclnet** (a modern ROS 2 client for .NET 8 to 10 that supports Jazzy, Kilted and Lyrical), the **official `Anthropic` C# SDK** (which implements `IChatClient` from Microsoft.Extensions.AI), Microsoft Agent Framework if wanted, and **Asv.Mavlink** for MAVLink. **Python and C++** are the pragmatic choice *on the robots*: drivers, ros2_control, Nav2 plugins, perception, RL.
8. **UAV path.** **[Opinion]** Use **PX4**: native uXRCE-DDS ROS 2 bridge, rmw_zenoh-compatible Zenoh in v1.17, and the **PX4 ROS 2 Interface Library** for custom modes and executors, which map 1:1 onto the Skill API. Go to **Gazebo SITL multi-vehicle** first, then indoor **Crazyflie / Crazyswarm2** for safe physical swarm practice, then a real PX4 quad with a Pi 5 or Jetson companion. Study Aerostack2 as a reference design.

---

## 1. Layered architecture that transfers from ground robots to UAVs

### 1.1 Reference stack

```
 ┌──────────────────────────────────────────────────────────────────────────┐
 │ L8  Cognitive layer (LLM/VLM agents)          REUSABLE, off-board/cloud  │
 │     NL → mission plan (JSON DSL), replanning, scene Q&A, reflection      │
 ├──────────────────────────────────────────────────────────────────────────┤
 │ L7  Fleet coordination                        REUSABLE, GCS + on-board   │
 │     task allocation (auction/CBBA), shared world model, map merge,       │
 │     comm-loss policy, deconfliction, human-on-the-loop                   │
 ├──────────────────────────────────────────────────────────────────────────┤
 │ L6  Mission executive                         REUSABLE, on-board         │
 │     behavior trees (BehaviorTree.CPP), mission validation, safety        │
 │     supervisor (geofence, battery, heartbeat, e-stop)                    │
 ├════════════════════  NARROW WAIST: Skill API (ROS 2 actions)  ═══════════┤
 │     NavigateTo · Explore(area) · Search(class) · Hold · ReturnHome ·     │
 │     Dock/Land · Capture · Follow · (UAV only: Arm, Takeoff)              │
 ├──────────────────────────────────────────────────────────────────────────┤
 │ L5  Skill implementations                     VEHICLE-SPECIFIC adapters  │
 │     UGV: Nav2 BT navigators, explore_lite   UAV: PX4 modes/executors,    │
 │                                                  Aerostack2 behaviors    │
 │ L4  Navigation & motion planning              VEHICLE-SPECIFIC           │
 │     UGV: Nav2 planner + MPPI (Omni)         UAV: trajectory planner,     │
 │                                                  3D map (OctoMap/nvblox) │
 │ L3  Control                                   VEHICLE-SPECIFIC           │
 │     UGV: ros2_control mecanum controller    UAV: PX4 pos/att/rate loops  │
 │ L2  State estimation                          PATTERN reusable, impl not │
 │     UGV: robot_localization EKF + SLAM      UAV: PX4 EKF2 (+VIO)         │
 │ L1  Hardware abstraction / drivers            VEHICLE-SPECIFIC           │
 │ L0  Firmware (MCU motor PID / flight controller)                         │
 └──────────────────────────────────────────────────────────────────────────┘
 Cross-cutting: event log (domain events + MCAP telemetry), observability,
 configuration/parameters, simulation parity, security (SROS2/Zenoh ACL),
 learning loop (data → evaluate → propose → gate → deploy).
```

### 1.2 What is vehicle-specific and what is reusable

| Layer | Ground robot (now) | UAV (later) | Reuse |
|---|---|---|---|
| L0 Firmware | MCU running wheel PID (micro-ROS) or the Adeept HAT | PX4/ArduPilot flight controller | None, but the *contract* (heartbeat, failsafe) carries over |
| L1 HAL | ros2_control hardware interface, sensor drivers | uXRCE-DDS / MAVROS bridge | Patterns only |
| L2 Estimation | EKF (wheel/laser odom + IMU) + 2D SLAM | EKF2 + GPS/VIO | Concepts (frames, covariances, TF tree) |
| L3 Control | `mecanum_drive_controller` | PX4 controllers | None |
| L4 Navigation | Nav2 (2D costmaps) | 3D planners, PX4 setpoints | Concepts only |
| **Skill API** | Actions implemented with Nav2 | Actions implemented with PX4 modes or Aerostack2 | **Interface 100% reusable** |
| L6 Executive | BehaviorTree.CPP trees calling skills | Same trees with different skill providers | **High** |
| L7 Fleet | Allocation, world model, comms | Same, plus 3D airspace deconfliction | **High** |
| L8 Cognitive | LLM planner and validator | Same, with stricter validation | **High** |
| Cross-cutting | Event store, MCAP, KPIs, sim CI | Same | **High** |

**Key design principles [Opinion]:**

- **Skills are the contract.** Give each skill a typed goal, feedback, result, explicit preconditions, declared resource usage (battery, time estimate), a cancel semantic and a failure taxonomy. MAVLink already shows the value of this. A mission item such as `NAV_WAYPOINT` or `NAV_RETURN_TO_LAUNCH` is effectively a skill invocation, and MAVSDK's *Action/Mission* plugins are a skill API. Aerostack2 calls the same idea "behaviors", and its platform-independent, plugin-based, behavior-based mission control is exactly the property you want ([Aerostack2 paper](https://arxiv.org/html/2303.18237v2)).
- **Safety is owned low, never by the LLM.** Stopping on obstacles (Nav2 *Collision Monitor*), geofence, battery return-home and comm-loss behaviors must run on the vehicle without the network. PX4 formalises this: a pilot or GCS can always take control back from a ROS 2 mode, and failsafes stay in the flight controller ([PX4 ROS 2 Control Interface](https://docs.px4.io/main/en/ros2/px4_ros2_control_interface.html)).
- **Same launch files and same skill API in simulation and on the real robots.** Every mission must be runnable in Gazebo (or Nav2's *loopback simulator*, see §2) before it touches hardware. That is also the validator's dry-run.
- **Everything is an event.** Missions, task awards, skill starts and ends, failures, detections and human approvals are append-only domain events. This is what makes reflection, replay, KPIs and learning possible later.

### 1.3 Centralized or decentralized: a staged answer

The initial idea was a decentralized P2P swarm. **[Opinion]** Treat decentralization as a *later capability to earn*, not the starting point:

| Concern | Centralized (GCS) | Decentralized (P2P) |
|---|---|---|
| Optimality of allocation | Best (global view) | Near-optimal (CBBA has known bounds) |
| Robustness to comm loss | Single point of failure | Graceful |
| Debuggability | Easy | Hard (needs good event logs) |
| LLM integration | Natural (one planner) | Needs per-robot agents or a "leader" |
| Scale | Fine up to tens of robots | Better beyond that |

**Recommended pattern: "centralized planning, decentralized execution, decentralized fallback."**

- The ground station plans and allocates.
- Each robot executes autonomously and has a deterministic comm-loss policy: finish the current task, then go to a rendezvous point or return home.
- Tasks carry **leases**. If a robot misses heartbeats past its lease, its tasks are re-auctioned.
- Phase 6 then adds a CBBA mode where robots resolve allocation among themselves. The interface is the same, so the GCS becomes optional.

---

## 2. ROS 2 ecosystem: status in October 2026

### 2.1 Distro choice

- **Lyrical Luth** (12th ROS 2 release) went GA on **22 May 2026**. It is an **LTS supported until May 2031**. Its Tier 1 platforms are **Ubuntu 26.04 (amd64/arm64)** and Windows 11. Ubuntu 24.04 is Tier 3 ([release announcement](https://discourse.openrobotics.org/t/ros-2-lyrical-luth-released/55021), [release page](https://docs.ros.org/en/lyrical/Releases/Release-Lyrical-Luth.html)). Lyrical pairs with **Gazebo Jetty**, and Jazzy pairs with **Gazebo Harmonic** ([third-party comparison](https://openelab.io/blogs/learn/ros-2-lyrical-vs-jazzy); verify against REP-2000).
- **Kilted Kaiju** is a non-LTS release that reaches end of life in **November 2026**. Don't start on it ([Kilted announcement](https://www.openrobotics.org/blog/2025/5/23/ros-2-kilted-kaiju-released)).
- **Jazzy** is the 2024 LTS (to 2029). It is still receiving sync releases in 2026 ([Jazzy sync, Oct 2026](https://discourse.openrobotics.org/t/preparing-for-jazzy-sync-2026-09-09/57926/5)).

**[Opinion]** **Jazzy + Ubuntu 24.04 on the Pi 5s** gives you the most tutorials, the most third-party packages and the fewest surprises. Run it in Docker on the Pi if you want to switch later without reflashing. On Windows, develop in **WSL2 or a Docker devcontainer**, not natively. Re-evaluate Lyrical in Q2 2027.

### 2.2 Navigation: Nav2

- **Holonomic and mecanum support.** MPPI has an **Omni motion model** that uses the Y velocity ([Jazzy API](https://api.nav2.org/nav2-jazzy/html/classmppi_1_1OmniMotionModel.html), also present in [Lyrical](https://api.nav2.org/nav2-lyrical/html/classmppi_1_1OmniMotionModel.html)). Jazzy brought large MPPI optimisations and acceleration constraints ([Nav2 Jazzy release](https://discourse.openrobotics.org/t/nav2-jazzy-release/38321)).
- **Kilted** added ([announcement](https://discourse.openrobotics.org/t/nav2-kilted-release/44079)):
  - the **Route Server** (graph-based routes);
  - **error messages** from task servers to behavior trees;
  - `TwistStamped` as the default velocity type;
  - a **Loopback Simulator**, which treats velocity commands as odometry. This makes it ideal for fast mission and LLM-plan dry-runs without a physics simulator;
  - docking to static infrastructure;
  - MPPI about 45% faster;
  - a Plugin Container costmap layer.
- **Lyrical** added ([announcement](https://discourse.openrobotics.org/t/nav2-lyrical-release-key-features/57666)):
  - MPPI open-loop mode, dynamics support, asymmetric acceleration limits, **plugin motion models**, trajectory validators and delay compensation;
  - **Pause/Resume** via a BT node;
  - collision-monitor toggles and exclusion zones;
  - the `nav2_ros_common` package.
- **Mecanum reality check.** A 2026 preprint on a ROS 2 Jazzy + Nav2 mecanum robot reports motor deadzone problems on carpet. It notes that the standard kinematic models assume pure rolling and no lateral slip ([engrxiv preprint](https://engrxiv.org/preprint/download/7347/11996/10311)). TT motors without encoders will make this worse. **[Opinion]** Start with the DiffDrive model (no strafing) to get a working baseline, then enable Omni once you have closed-loop wheel control.
- **Multi-robot.** `nav2_bringup` ships `cloned_multi_tb3_simulation_launch.py`, which starts N namespaced robots sharing one params file. Remap `/tf` and `/tf_static` per robot ([nav2_bringup README](https://api.nav2.org/nav2-jazzy/html/md_nav2_bringup_README.html)).

### 2.3 Control: ros2_control

`mecanum_drive_controller` **exists in ros2_controllers**. It takes a `TwistStamped` (linear x, linear y, angular z), publishes odometry and TF, has a command timeout, and supports chaining ([Jazzy docs](https://control.ros.org/jazzy/doc/ros2_controllers/mecanum_drive_controller/doc/userdoc.html)). The package is at 4.41.0 on Jazzy and 5.16.0 on Kilted, released 2026-07-01 ([index](https://index.ros.org/p/mecanum_drive_controller/)).

To use it you write a **hardware interface** for the Adeept Robot HAT (PWM out; there are no state interfaces without encoders). The better option is an MCU firmware that closes the wheel loops. The **linorobot2_hardware** firmware (ESP32/Pico/Teensy, micro-ROS) has a mecanum config with encoder motors. Avoid the ESP32-C3, which lacks the PCNT encoder counters ([wiki](https://github.com/hippo5329/linorobot2_hardware/wiki)). Running PID and odometry on the MCU and publishing `/odom` and `/joint_states` is a common, sane split ([Discourse discussion](https://discourse.openrobotics.org/t/ros2-micros-ros-control-design-where-to-run-what/41681)).

### 2.4 Estimation and mapping

- **robot_localization** is actively maintained. Version 3.10.0 was released 2025-08-29, and lifecycle-node support for the EKF and UKF is in progress ([changelog](https://docs.ros.org/en/rolling/p/robot_localization/CHANGELOG.html)). Use it to fuse wheel odometry (or laser odometry) with the IMU.
- **slam_toolbox** is still *the* supported 2D LiDAR SLAM for ROS 2. It has a Jazzy branch and was last pushed in April 2026 ([index](https://index.ros.org/r/slam_toolbox/)). Its "multi-robot distributed mapping" is not something I could confirm as production-ready. **It needs a 2D laser scan**, so it won't work with your current sensors.
- **Without encoders**, use LiDAR scan-matching odometry (for example rf2o_laser_odometry or KISS-ICP) fused with an IMU. **[Opinion]** This is the cheapest path to usable odometry on TT motors.
- **Low-cost LiDAR drivers.** `rplidar_driver` lists Jazzy support and was verified on Jazzy at 1.4.0 ([index](https://index.ros.org/r/rplidar_driver/)). LD19/LD06 have community ROS 2 drivers whose Jazzy status I couldn't confirm, so test before you buy in bulk.

### 2.5 Exploration and map merging

- **m-explore-ros2** (robo-friends) ports `explore_lite` (frontier exploration on Nav2) and `multirobot_map_merge` to "Humble and newer". There are **no binaries**, so build from source ([repo](https://github.com/robo-friends/m-explore-ros2)). Map merge works with **known initial poses**, which is reliable, or **unknown poses** via feature matching, which is fragile. **[Opinion]** Start the robots from surveyed poses on an AprilTag-marked start line.
- **Swarm-SLAM** (MIST Lab) is decentralized, sparse-communication collaborative SLAM in ROS 2. It supports LiDAR, stereo and RGB-D, and was demonstrated with 3 robots on an ad-hoc network ([paper](https://arxiv.org/html/2301.06230v3), [repo](https://github.com/MISTLab/Swarm-SLAM)). Activity looks low lately. It is a good phase-6+ study target, but it needs better sensors than the kits have.
- ROSCon 2025 had a talk "Mapping Together: Collaborative SLAM for Distributed Robot Fleets" ([recordings](https://discourse.openrobotics.org/t/roscon-2025-recordings-now-available/51241)).

### 2.6 Middleware: rmw_zenoh for Wi-Fi fleets

- **Kilted was the first release with Zenoh as a Tier 1 RMW.** It is still **not the default** RMW ([Kilted announcement](https://www.openrobotics.org/blog/2025/5/23/ros-2-kilted-kaiju-released)). The motivation is DDS discovery storms (O(N²) discovery traffic), heavy tuning, and trouble with containers, VPNs and Wi-Fi ([ROSCon 2024 talk](https://roscon.ros.org/2024/talks/RMW_Zenoh-_An_alternative_middleware_for_ROS_2.pdf)).
- **Topology** ([rmw_zenoh README](https://github.com/ros2/rmw_zenoh)):
  - Run **one router per host or robot**.
  - Multicast discovery is off by default.
  - Bridge hosts by putting the other routers' endpoints in the router config's `connect` list.
  - Remote tools (RViz, the GCS) connect as **clients** to a robot's router.
  - Shared memory is available on-robot.
  - Humble does not interoperate with newer distros.
  - The default config listens on IPv6 only, so change it to `tcp/0.0.0.0:7447` on IPv4-only networks.
- Arm has a learning path on **isolating multiple robots with Zenoh** ([Arm](https://learn.arm.com/learning-paths/cross-platform/distributed-ros2-zenoh-lp2/5-isolate-multiple-robots/)). ROSCon 2025 had "From DDS to Zenoh: Migrating the Dexory Autonomy ROS Stack" ([recordings](https://discourse.openrobotics.org/t/roscon-2025-recordings-now-available/51241)).
- **UAV relevance.** **PX4 v1.17's in-tree Zenoh is compatible with rmw_zenoh**, so a PX4 vehicle shows up in a ROS 2 graph like any other node ([PX4 1.17 release notes](https://docs.px4.io/main/en/releases/1.17.html)). That is a strong argument for standardising on Zenoh across the whole fleet now.

**Recommended fleet topology [Opinion]:**

```
 robot1: zenohd + nodes (ns=/r1) ─┐
 robot2: zenohd + nodes (ns=/r2) ─┼──► GCS zenohd (laptop) ◄── .NET fleet mgr (client), Foxglove, RViz
 robot3: zenohd + nodes (ns=/r3) ─┘          │
                                             └─ only a whitelist of topics crosses the radio
                                                (status, map deltas, task msgs, low-rate images)
```

Keep `/rN/scan` and raw images on-robot. Send only *compressed, downsampled, or on-demand* data across Wi-Fi. The same rule applies to UAV radios.

### 2.7 Behavior trees and fleet management

- **BehaviorTree.CPP 4.7.0** is in Jazzy ([changelog](https://docs.ros.org/en/jazzy/p/behaviortree_cpp/__CHANGELOG.html)). **Groot2** is the editor and monitor for v4, and Nav2 uses BTs as its orchestration core ([Nav2 Groot tutorial](https://docs.nav2.org/tutorials/docs/using_groot.html)). **BehaviorTree.ROS2** gives you action, service and topic BT-node wrappers (BT.CPP ≥ 4.6, Humble+) ([repo](https://github.com/BehaviorTree/BehaviorTree.ROS2)). **[Opinion]** Use BT.CPP for the mission executive too, not only inside Nav2. LLMs generate BT XML or a plan DSL that compiles to BT XML well (see §3.4).
- **Open-RMF** is the OSRF framework for *heterogeneous fleet traffic management* in buildings (lifts, doors, traffic negotiation). It is still released on Jazzy ([rmf_fleet_msgs Jazzy](https://docs.ros.org/en/jazzy/p/rmf_fleet_msgs/)), and third-party fleet adapters keep appearing ([olive adapter, Oct 2026](https://index.ros.org/p/olive_openrmf_fleet_adapter/)). **[Opinion]** It is overkill for 3 robots in one room and doesn't map onto UAVs. Read its traffic-negotiation design, but don't adopt it.

### 2.8 Tooling worth adopting immediately

- **Foxglove plus `foxglove_bridge`** for visualisation and MCAP playback (also good from Windows). `rosbridge_suite` gives you a JSON/WebSocket gateway, which `ros-mcp-server` uses.
- **rosbag2 with the MCAP storage plugin** for all field runs. This is your raw event store (§4.5).
- **launch_testing plus Gazebo headless in CI** to regression-test missions.

---

## 3. LLM and foundation-model agents for robotics (2024–2026)

### 3.1 Landscape

| System or pattern | What it is | Relevance |
|---|---|---|
| **NASA JPL ROSA** | A LangChain agent that lets operators query and command ROS 1/2 systems in natural language, with parameter-validation safety checks. Tested on Spot, EELS and Nova Carter ([paper](https://arxiv.org/html/2410.06472v2), [repo](https://github.com/nasa-jpl/rosa)). Licence wording has been flagged as ambiguous, so check it. | Good **operator/debug assistant** pattern. Not a mission planner. |
| **ros-mcp-server** (robotmcp) | An MCP server that exposes ROS topics, services, actions and params via rosbridge to any MCP client (Claude, etc.). Apache-2.0, about 1.3k stars, v3.x in 2026 ([repo](https://github.com/robotmcp/ros-mcp-server)). | Great for **exploration and demos**. **Too broad for autonomous ops**, because it can publish any topic. |
| **SayCan → Code as Policies** | LLM chooses among affordance-scored skills, or writes Python against a perception/control API ([Google blog](https://research.google/blog/robots-that-write-their-own-code/)) | The foundational patterns. |
| **LLM → behavior trees** | Code-BT (IJCAI 2025): an LLM writes modular code and a BT is extracted, giving +16–29% generation quality ([paper](https://www.ijcai.org/proceedings/2025/980)). BETR-XP-LLM *expands* BTs on failure and keeps the fix, so later runs make fewer LLM calls ([paper](https://arxiv.org/html/2409.13356v1)). | **Directly applicable.** Verifiable and reusable. |
| **Multi-robot LLM planners** | SMART-LLM (centralized), RoCo (multi-agent dialogue, token-hungry), COHERENT (proposal–execution–feedback for heterogeneous teams) ([paper](https://arxiv.org/html/2409.15146v3)), MHRC (decentralized) ([paper](https://arxiv.org/html/2409.16030v2)), CoMuRoS (2025: hybrid hierarchical planning with **event-driven replanning**) ([paper](https://arxiv.org/html/2511.22354v1)) | CoMuRoS/COHERENT-style *central LLM planner with event-driven replanning* is the pattern to copy. |
| **Failure reflection** | REFLECT (CoRL 2023): summarises multisensory logs into a hierarchical record, has an LLM explain failures, and steers a corrected plan ([paper](https://arxiv.org/pdf/2306.15724)) | The basis for the self-improvement loop (§5). |
| **Open-vocabulary scene graphs** | ConceptGraphs: 2D segmentation fused into a 3D graph, with VLM-labelled objects and LLM-inferred relations ([paper](https://arxiv.org/pdf/2309.16650)) | A target for the "world model" the LLM queries. Heavy compute, so run it off-board. |
| **LLM + UAV** | 2026 survey of LLM-assisted UAV ops: navigation, mission planning, swarm control, safety, human-in-the-loop ([survey](https://arxiv.org/html/2602.19534v2)) | Reading list for phase 9. |

### 3.2 VLMs and VLAs: what runs where

- **VLMs on a Pi 5 CPU are too slow for anything but occasional queries.** Reports range from about 8 to 20 s per frame for Moondream on a Pi 5 ([Core Electronics forum](https://forum.core-electronics.com.au/t/guide-by-jaryd-getting-started-with-moondream-on-the-pi-5-human-like-computer-vision/23286)) to about 29 s image encoding plus about 21 s model load for Moondream 2B on a Pi 5 8 GB ([GoPiGo forum](https://forum.dexterindustries.com/t/moondream-vision-language-assistant-on-gopigo3-robot-kilted-dave/10750)).
- **Raspberry Pi AI HAT+ 2.** It uses a Hailo-10H (40 TOPS INT4, 8 GB of its own RAM) and runs small LLMs and VLMs offline ([product page](https://www.raspberrypi.com/products/ai-hat-plus-2/)). It launched at about **$130**, and Tom's Hardware's verdict was mixed: image inference is only somewhat better than the Pi CPU, and only supported models run ([review](https://tomshardware.com/raspberry-pi/raspberry-pi-ai-hat-plus-2-review)). **[Opinion]** If you buy one accelerator, buy it for **YOLO-class detection** (the 1st-gen AI HAT+ is enough). Do semantic VLM questions in the cloud.
- **VLAs.**
  - **pi0 and pi0.5** (Physical Intelligence, [openpi](https://github.com/Physical-Intelligence/openpi)) need more than 8 GB of GPU memory for inference and more than 22 GB for LoRA fine-tuning ([openelab guide](https://openelab.io/blogs/learn/vla-models-for-robot-arms)).
  - **SmolVLA** (about 450M parameters, LeRobot) needs about 2 GB of VRAM in FP16 ([Spheron](https://www.spheron.network/blog/smolvlm-smolvla-gpu-cloud-edge-ai-robotics/)). Both target **arm manipulation**.
  - **LeKiwi** (LeRobot's mobile manipulator: 3-omniwheel base, SO-101 arm, Pi 5) is the closest LeRobot platform to your robots ([repo](https://github.com/sigrobotics-uiuc/lekiwi)).
  - **[Opinion]** For mecanum navigation, VLAs don't beat Nav2 and don't run on a Pi. Skip them unless you add an arm. Then a LeKiwi-style arm with SmolVLA running on a laptop GPU over Wi-Fi is a fun side quest.

**Placement summary [Opinion]:**

| Function | Latency need | Where |
|---|---|---|
| Motor control, safety stop | < 10 ms | MCU or Pi, deterministic |
| Local planning (MPPI) | 20–50 ms | Pi 5 |
| Object detection (YOLO) | 50–200 ms | Pi 5 + Hailo |
| Scene Q&A (VLM) | 1–10 s, occasional | Cloud (Claude vision) or laptop GPU |
| Mission planning and replanning (LLM) | 2–60 s | Cloud (Claude), on the GCS |
| Reflection and analytics | minutes, batch | Cloud batch API or Claude Agent SDK on the GCS |

### 3.3 Safety guardrails: "LLM proposes, deterministic layer disposes"

The evidence is clear that the LLM must not be the last line of defence:

- **RoboGuard** grounds safety rules in the environment with a trusted LLM, then uses **temporal-logic control synthesis**. Under jailbreak attacks it cut unsafe plan execution from 92% to under 2.5% without hurting safe-plan performance ([paper](https://arxiv.org/abs/2503.07885)).
- Data-driven **reachability analysis** gives formal guarantees without a precise model ([paper](https://arxiv.org/html/2503.03911v1)).
- Surveys list temporal-logic specifications, control barrier functions and world-model-aware checks as the main defence families ([survey](https://arxiv.org/pdf/2601.02377)). They also note that NL→formal translation is only **24–35% semantically correct**, so don't let the LLM write the safety spec itself ([survey](https://arxiv.org/pdf/2608.14590)).

**Recommended guardrail pipeline [Opinion]:**

```
NL request ─► LLM planner (tools: read-only world queries + propose_plan)
            ─► Plan DSL (JSON Schema, strict) ─► Static validator
                 • schema + skill exists + robot has capability
                 • geofence / keep-out polygons (hand-written, NOT LLM-written)
                 • battery & time budget from skill estimates
                 • precondition/effect simulation (symbolic)
            ─► Dry-run: Nav2 loopback sim or Gazebo (fast, headless)
            ─► Risk tier: low → auto-execute; medium → operator 1-click approve;
                           high → reject/escalate
            ─► BT executor (deterministic) ─► Skills ─► Nav2/PX4 (with their own safety)
            ◄─ events (success/failure/anomaly) ─► LLM replanning (event-driven)
```

- Expose **skills, not topics**, as LLM tools. Use **read-only** tools for situational awareness (`get_fleet_status`, `get_map_summary`, `list_known_objects`, `query_vlm(robot, question)`) and a single **write** tool, `propose_mission(plan)`, which goes through the validator.
- **Keep ros-mcp-server for development only**, behind a flag. Its power (arbitrary publish) is what you don't want in autonomous operation.

### 3.4 Using Claude for high-level mission reasoning

- **Models and prices** (Anthropic API, cached October 2026; check [pricing](https://platform.claude.com/docs/en/about-claude/pricing)):

  | Model | Use for | Price (input / output, per 1M tokens) |
  |---|---|---|
  | `claude-opus-5-5` | Mission planning, replanning, reflection | $4 / $20 |
  | `claude-sonnet-5-5` | Mid-tier | $2 / $10 |
  | `claude-haiku-5-5` | Event summarisation, log triage, high-volume classification | $0.10 / $0.50 |

  All of them accept images, so the same API covers VLM scene questions.
- **Structured plans.** Use strict tool schemas or structured outputs so plans are schema-valid by construction. Note that the newest models don't support *forced* tool choice; use `auto` with an instruction plus `strict: true`. Run validation anyway.
- **Prompt caching.** Cache the large, stable prefix (skill catalogue, map summary, rules). Per-mission cost should then be cents. **[Speculation]** A typical plan is roughly 10–30k input tokens and 1–3k output tokens.
- **Surfaces:**
  1. **Messages API + tool use** from the .NET fleet manager, using the official C# SDK (§7). This is the mission planner in production.
  2. **Claude Agent SDK** (Python/TypeScript; the Claude Code harness as a library, with file, bash and MCP tools, hooks, subagents and permissions) ([docs](https://code.claude.com/docs/en/agent-sdk)). It is ideal for the **offline analyst agent** that reads MCAP exports and event logs, runs analysis scripts, and writes reflection reports and parameter-change proposals.
  3. **MCP.** Write your own *narrow* MCP server exposing the Skill API and read-only queries. Then Claude Desktop or Claude Code can act as an interactive operator console for free.
- **Latency and offline operation.** Cloud LLMs take seconds and need connectivity. That is fine for the mission layer but fatal for control. Robots must complete or abort safely with no LLM. **[Opinion]** A local fallback model on the GCS laptop (Ollama with a small Qwen or Llama) is optional and only worth it for demos without internet.

---

## 4. Multi-robot and swarm coordination

### 4.1 Task allocation

| Method | Nature | When |
|---|---|---|
| **Greedy / Hungarian** | Centralized, optimal for one-shot one-to-one assignment | Baseline, phase 6 |
| **Contract Net Protocol (CNP)** | Manager announces, contractors bid, award with timeout | Simple and very debuggable. Good first implementation |
| **Sequential single-item auctions** | Repeated CNP. Good approximation for routing-type tasks | Phase 6 |
| **CBBA** (Consensus-Based Bundle Algorithm) | Decentralized. Each robot builds a bundle, then a consensus phase resolves conflicts. Conflict-free with a provable approximation bound ([original MIT work](https://dspace.mit.edu/handle/1721.1/44926)) | Phase 6b/9: the UAV-relevant one |
| **Market + LLM** | The LLM decomposes the mission into tasks; the auction allocates them | Phase 7 |

- **Implementations.** There is no maintained ROS 2 CBBA package. Python options include **TrajAllocPy** (CBBA for trajectory and coverage tasks) ([repo](https://github.com/kasperg3/trajallocpy)) and small reference implementations. A GIST paper wraps asynchronous CBBA as ROS 2 nodes for UAV/UGV teams, but the code status is unclear ([record](https://scholar.gist.ac.kr/handle/local/31950)). **[Opinion]** Write your own. CNP and CBBA are a few hundred lines each, and writing them is half the learning.
- **Bid function trick [Opinion].** Compute bids with Nav2's `ComputePathToPose` cost (real travel cost through the current costmap), plus battery and current-queue penalties. This turns a toy auction into a meaningful one.

### 4.2 Coordinated exploration

- **Frontier allocation.** Assign frontiers with a utility minus cost, and discount utility near frontiers already claimed by teammates (the classic Burgard et al. coordinated-exploration scheme). Implement it as auctions over frontier clusters. Baseline: three independent `explore_lite` instances. Measure the speedup.
- **Communication-constrained exploration.** Rendezvous-based schemes (modelled as job-shop scheduling, with a temporal connectivity graph) beat relay chains and fixed base stations in simulation ([paper](https://arxiv.org/html/2309.13494v6); the [2025 MILP follow-up](https://arxiv.org/pdf/2511.12237v1)). Their code is ROS 1, so it would need porting. That makes it a good stretch topic for the UAV phase.

### 4.3 Communication-loss handling (design)

**[Opinion]**, based on standard practice:

1. **Liveliness.** Each robot publishes a heartbeat (Zenoh liveliness or a 2 Hz status topic). The GCS tracks `last_seen`.
2. **Task leases.** Every awarded task has a lease (for example 30 s, renewed by heartbeats). When it expires, the task goes back to *open* and is re-auctioned. The robot itself, when it reconnects, learns that its task was reassigned and stops.
3. **Robot-side policy.** If the link is lost for more than T₁, finish the current skill, then **hold**. If it is lost for more than T₂, **return to the last known comm point or home**. This mirrors UAV "datalink loss" failsafes.
4. **State reconciliation.** After reconnecting, the robot uploads buffered events and map deltas (event sourcing makes this natural: replay the outbox).
5. **Test it.** Inject packet loss with `tc netem` in simulation and in the lab. Make "comm-loss recovered correctly" a KPI.

### 4.4 Existing frameworks: adopt, study or skip

| Framework | Status (2026) | Verdict [Opinion] |
|---|---|---|
| **Aerostack2** | ROS 2 multi-aerial framework: platform-agnostic (PX4, Crazyflie, Tello, DJI), behavior-based missions ([paper](https://arxiv.org/html/2303.18237v2)). The `as2_platform_pixhawk` index entry was last updated 2025-12 but is listed as unreleased on Humble ([index](https://index.ros.org/r/as2_platform_pixhawk/)) | **Study now; possibly adopt for UAVs.** Its behavior abstraction is your Skill API |
| **Crazyswarm2** | ROS 2 Crazyflie stack with Jazzy packages (2025), a simulator, and a JOSS paper submitted November 2025 ([repo](https://github.com/IMRCLab/crazyswarm2)) | **Adopt** for indoor swarm practice in phase 9 |
| **Swarm-SLAM** | Decentralized C-SLAM in ROS 2, quiet lately | Study |
| **Buzz** | Swarm DSL (MIST Lab), ROS 1 era integration ([repo](https://github.com/buzz-lang/Buzz), [ROS+Buzz paper](https://arxiv.org/pdf/1710.08843)) | Read the ideas (virtual stigmergy, neighbor queries). Don't adopt |
| **Open-RMF** | Active, Jazzy | Skip (building-traffic oriented) |
| **MRS UAV System** (CTU) | Strong research UAV stack, ROS 1 based. I found no confirmed native ROS 2 port ([site](https://mrs.fel.cvut.cz/mrs-uav-system)) | Study its papers, not its code |
| **m-explore-ros2** | Source-only, Humble+ | **Adopt** (phases 5–6) |

### 4.5 Event sourcing: yes, but at the right level

**[Opinion]** The original idea is good, applied at the *mission domain* level:

- **Raw telemetry** goes in **rosbag2/MCAP** files per robot per run: high volume and immutable. That is already an event log.
- **Domain events** go to an append-only store (EventStoreDB/KurrentDB, Marten on PostgreSQL, or plain SQLite for a start). Events are small and semantic: `MissionRequested`, `PlanProposed`, `PlanValidated`/`Rejected(reason)`, `TaskAnnounced`, `BidSubmitted`, `TaskAwarded`, `SkillStarted`, `SkillFailed(code)`, `ObjectDetected`, `HumanApproved`, `CommLost`, `CommRestored`, `MissionCompleted(kpis)`.
- **Projections** feed dashboards, KPIs, LLM context ("what happened so far") and training datasets.
- Every domain event carries `robot_id`, `mission_id`, ROS time and **a pointer (bag file + timestamp) into MCAP**. Reflection can then jump from "SkillFailed at 12:03:14" to the exact sensor data.

This is where your .NET and event-sourcing experience pays off. It is also exactly what a UAV fleet needs for post-flight analysis and auditability.

---

## 5. Self-learning and self-improvement: realistic versus hype

### 5.1 What actually works on hobby-grade robots

| Technique | Value here | Effort | Notes |
|---|---|---|---|
| **Systematic benchmark + KPI tracking** | ★★★★★ | Low | A precondition for *any* "self-improvement". Without a fixed benchmark, "learning" is just drift |
| **Parameter auto-tuning (Bayesian optimisation, Optuna/TPE)** | ★★★★★ | Medium | Nav2 tuning is "more art than science" ([Nav2 Discourse](https://discourse.openrobotics.org/t/nav2-discussion-systematic-approach-for-tuning-nav2-planners-and-controllers/53254)). Offline TPE tuning of an MPC navigation controller in Gazebo transferred to hardware without retuning ([2026 preprint](https://arxiv.org/pdf/2606.14763)). BO improved a path-following controller within 32 trials ([paper](https://arxiv.org/html/2512.12649v1)) |
| **LLM post-mission reflection → proposals** | ★★★★ | Medium | REFLECT-style summarisation ([paper](https://arxiv.org/pdf/2306.15724)). Outputs are *proposals* (param diffs, BT patches, "lesson" memory entries), each evaluated in sim and on the benchmark, then promoted or discarded |
| **Fleet knowledge sharing** | ★★★★ | Low–medium | Shared map and keep-outs, a hazard memory ("carpet at x,y: slip"), shared tuned params, shared detection labels |
| **BT self-repair (BETR-XP-LLM style)** | ★★★ | Medium | Learned BT expansions persist ([paper](https://arxiv.org/html/2409.13356v1)) |
| **Imitation learning (LeRobot) for a narrow visual skill** | ★★ | Medium–high | For example camera-only docking or line following. LeRobot is built for arms and LeKiwi |
| **RL in simulation for a narrow skill** | ★★ (learning: ★★★★) | High | For example a learned local controller or docking. Use Isaac Lab or MuJoCo Playground |
| **End-to-end RL / VLA navigation** | ★ | Very high | Doesn't beat Nav2 on this hardware, and the TT motors and slip dominate the sim-to-real gap |

### 5.2 Simulation and sim-to-real tools

- **Gazebo** (Harmonic with Jazzy, Jetty with Lyrical). This is your **digital twin for integration testing**: same launch files, same Nav2, multi-robot, headless in CI. Model mecanum as planar-move or omni plugins rather than simulating roller physics. **[Opinion]**
- **Isaac Lab 3.0** (beta in late 2026) adds a multi-backend design (PhysX and **Newton**). It can run standalone on Newton without full Isaac Sim, and needs Python 3.12 ([release notes](https://isaac-sim.github.io/IsaacLab/release/3.0.0/source/refs/release_notes.html)). It needs an NVIDIA GPU. It is the best choice if you later do RL for UAV attitude or position control.
- **MuJoCo Playground** offers GPU-accelerated RL on MJX and MuJoCo Warp, trains policies in minutes, and reports zero-shot sim-to-real on six platforms. It targets legged robots, arms and hands, with **no wheeled-navigation environment** shipped ([paper](https://arxiv.org/html/2502.08844v1), [repo](https://github.com/google-deepmind/mujoco_playground)).
- **Domain randomisation** for a mecanum base: randomise wheel friction (including lateral), motor deadzone and gain per wheel, battery-voltage sag, command latency (Wi-Fi), and sensor noise and dropouts. **[Opinion]** Identify these first from real logs (system identification) before you randomise around them.

### 5.3 A practical self-improvement loop

```
 Missions (real + sim) ─► MCAP + domain events
        │
        ▼
 Nightly analyst agent (Claude Agent SDK on GCS)
   • compute KPIs per mission, compare to baseline
   • cluster failures (SkillFailed codes, recoveries, near-misses)
   • REFLECT-style explanation with pointers to bag segments
   • write PROPOSALS: param diffs, BT patches, hazard-map entries, new tests
        │
        ▼
 Gate: run benchmark suite in Gazebo (N seeds) → must improve target KPI
       and not regress safety KPIs → optional real-robot A/B on 1 robot
        │
        ▼
 Promote to fleet config (versioned, event: ConfigPromoted v42) ─► all robots
```

**[Opinion]** "Self-learning" should mean **closed-loop improvement with gates and versioning**. That is engineering you can trust, and it transfers unchanged to UAVs.

---

## 6. The UAV side

### 6.1 Autopilot: PX4 or ArduPilot

| | PX4 | ArduPilot |
|---|---|---|
| ROS 2 link | **uXRCE-DDS** built in. **Zenoh** compatible with rmw_zenoh in v1.17 ([1.17 notes](https://docs.px4.io/main/en/releases/1.17.html)) | **AP_DDS** (micro-ROS agent). Topics such as `/ap/cmd_vel`, arming and mode services ([docs](https://ardupilot.org/dev/docs/ros2-interfaces.html)). MAVROS is also common |
| Custom autonomy | **PX4 ROS 2 Interface Library**: write *modes* that appear in QGC and *mode executors* (state machines over modes), with PX4 fallback if a ROS mode fails ([docs](https://docs.px4.io/main/en/ros2/px4_ros2_control_interface.html)). 1.16 added experimental ROS-based waypoint missions ([1.16 notes](https://docs.px4.io/main/en/releases/1.16.html)) | Guided-mode setpoints via DDS or MAVLink, plus Lua scripting |
| Multi-vehicle SITL | Gazebo, with `px4_<n>` namespaces and `PX4_UXRCE_DDS_NS` ([docs](https://docs.px4.io/main/en/ros2/multi_vehicle)) | SITL multi-instance |
| Licence | BSD | GPLv3 |
| Community view | Cleaner modular architecture (uORB). Docs better for what it implements | Wider hardware, sensors and GCS options ([forum](https://discuss.ardupilot.org/t/px4-vs-ardupilot-when-to-choose-what/14262)) |

**[Opinion]** Use **PX4** for this project. ROS 2-native modes and executors are the cleanest way to implement the Skill API on a drone, and the Zenoh alignment matches the ground fleet.

**A bonus bridge.** **PX4 supports mecanum rovers** (experimental; custom rover firmware build; "Generic Rover Mecanum" airframe; Gazebo models exist) ([config](https://docs.px4.io/v1.16/en/config_rover/mecanum), [rover frames](https://docs.px4.io/main/en/frames_rover/index)). **[Opinion]** In phase 9, consider putting a cheap PX4 flight controller on one ground robot. You would then run *the exact UAV autopilot stack, QGC and MAVLink tooling* on a vehicle that can't crash from the sky.

### 6.2 Companion-computer software

- **Offboard companion:** Pi 5 (light) or Jetson Orin Nano/NX (for VIO, nvblox or VLMs). The companion runs the ROS 2 graph, the Skill API adapters and perception. The FC runs estimation, control and failsafes.
- **Interfaces:**
  - **uXRCE-DDS / Zenoh** gives native ROS 2 topics, and the px4_ros2 library handles NED↔ENU frame conventions.
  - **MAVSDK** is a C++ core exposed as **`mavsdk_server` gRPC** for other languages ([FAQ](https://mavsdk.mavlink.io/v3/en/faq.html)). The C# client is a stale 2019 proof of concept, but you can generate C# gRPC clients from **MAVSDK-Proto** ([repo](https://github.com/mavlink/MAVSDK-Proto)).
  - **MAVROS** is the mature MAVLink↔ROS bridge.
- **Simulation:** PX4 SITL + Gazebo (Harmonic LTS since PX4 1.16), multi-vehicle with namespaces. Point the *same* fleet manager and LLM planner at it.
- **Swarm hardware for learning:** **Crazyflie + Crazyswarm2** is safe indoors, ROS 2 on Jazzy and well documented ([repo](https://github.com/IMRCLab/crazyswarm2)). Aerostack2 also supports Crazyflie and Tello.

### 6.3 Designing the ground stack so it maps onto UAVs

| Ground concept | UAV counterpart | Design rule [Opinion] |
|---|---|---|
| `NavigateTo(x,y,θ)` | `GoTo(x,y,z,yaw)` via a PX4 Goto setpoint or Aerostack2 `go_to` | Make the goal a **3D pose with optional z** from day one |
| `Explore(polygon)` | Coverage / lawnmower / 3D frontier | Areas are polygons with altitude bands |
| `Hold` | Hold / loiter | Same semantics |
| `ReturnHome` | RTL | Same, with **the FC owning the RTL failsafe** |
| `Dock` | Land / precision-land | Skill result codes aligned |
| e-stop | Kill vs. land-now | Two-level stop: *controlled stop* and *kill* |
| Costmap 2D | OctoMap / voxel map | World-model API returns *occupancy queries*, not a specific map type |
| Battery % → task bid | Same, but with much steeper consequences | Energy model per skill in the bid |
| Heartbeat | MAVLink HEARTBEAT, datalink-loss failsafe | Mirror MAVLink semantics now |
| — | Arm, Takeoff, airspace deconfliction, geofence altitudes | Reserve them in the Skill API enum; on the UGV, `Arm` maps to "enable motors" |

**Rule:** the mission executive, fleet manager and LLM layer must not reference anything Nav2- or ground-specific. Enforce it with a separate package or interface layer (`fleet_interfaces`: skills, events, world-model queries) and a *capability descriptor* per vehicle (`{type: ugv-mecanum, skills: [...], max_speed, endurance, sensors}`).

---

## 7. Where C#/.NET fits

### 7.1 Status of the .NET options

| Need | Option | Status (Oct 2026) |
|---|---|---|
| Native ROS 2 from .NET | **rclnet** (noelex) | MIT. **Lyrical, Kilted, Jazzy, Humble**. .NET 8/9/10, Linux and Windows. Topics, services and actions supported, with async/await, `IAsyncEnumerable` and `TimeProvider`. Parameters partial, no lifecycle nodes ([repo](https://github.com/noelex/rclnet)). **The best choice.** |
| | ros2_dotnet / ros2cs | ros2_dotnet forks are Foxy-era. ros2cs (Robotec) targets Foxy–Humble ([fork](https://github.com/miyakoshi-dev/ros2cs)). Not recommended for Jazzy+ |
| | Mori.Ros2Sharp | A pure-.NET DDS implementation tested against Humble ([NuGet](https://www.nuget.org/packages/Mori.Ros2Sharp)). Interesting but not with Zenoh |
| Bridge | rosbridge (JSON/WebSocket), foxglove_bridge | Robust and language-neutral. Fine for UIs and low-rate control. Adds latency |
| Bridge | Custom gRPC service in a Python node | Clean typed contract. Good if you want to hide ROS from .NET entirely |
| MAVLink | **Asv.Mavlink** (NuGet, v3.10.x, net8.0, with vehicle helpers for PX4 and ArduPilot) ([NuGet](https://feed.nuget.org/packages/Asv.Mavlink)) | Active. Good for a .NET GCS |
| MAVSDK | Generate C# gRPC clients from MAVSDK-Proto and run `mavsdk_server` | Feasible. The official C# client is a stale PoC |
| LLM | **Official `Anthropic` NuGet** (official from v10). Implements **`IChatClient`** (Microsoft.Extensions.AI), works with MCP tools via the ModelContextProtocol package, and includes a `BetaToolRunner` ([docs](https://platform.claude.com/docs/cli-sdks-libraries/sdks/csharp)) | Production-ready |
| Agent framework | **Microsoft Agent Framework 1.0** (GA around April 2026 per secondary sources). The successor to Semantic Kernel and AutoGen, with MCP/A2A support ([MS devblog](https://devblogs.microsoft.com/agent-framework/migrate-your-semantic-kernel-and-autogen-projects-to-microsoft-agent-framework-release-candidate/)). Semantic Kernel is in maintenance mode | Optional. **[Opinion]** Start with plain `IChatClient` + tools. Add a framework only when you need multi-agent workflows |

### 7.2 Recommendation

```
 ┌──────────────── Ground Control Station (laptop, .NET 10) ────────────────┐
 │ Fleet Manager service  (rclnet over rmw_zenoh)                           │
 │   • task allocation (CNP/auction), lease mgmt, capability registry       │
 │ Mission Service  • NL → Claude (Anthropic SDK, IChatClient, tools)       │
 │                  • plan DSL + validator + risk tiering + approval        │
 │ Event Store (Marten/PostgreSQL or KurrentDB) + projections + KPI engine  │
 │ UI: Blazor dashboard (+ Foxglove for 3D/sensor views)                    │
 │ Narrow MCP server (C# ModelContextProtocol SDK) exposing Skill API       │
 └──────────────────────────────────────────────────────────────────────────┘
 ┌──────────── Robot (Pi 5, Ubuntu 24.04, ROS 2 Jazzy) ─────────────┐
 │ C++: ros2_control HW interface, Nav2 + plugins                   │
 │ Python: drivers, skill servers, BT nodes (or C++), perception    │
 │ MCU (C/C++): micro-ROS wheel PID                                 │
 └──────────────────────────────────────────────────────────────────┘
 Offline/analytics: Python (MCAP, pandas, Optuna, RL), Claude Agent SDK (Py/TS)
```

**[Opinion]** Don't fight the ecosystem on the robot. Nav2 plugins, ros2_control and almost every tutorial and driver are C++ or Python. .NET is a **first-class choice for the supervisory, transactional and agentic layer**, which is also the most transferable layer and the one where your strengths matter most.

---

## 8. Staged learning roadmap

Each phase has a **demo milestone**, an **exit KPI** and a **"learn" goal**. Timeboxes assume part-time work and are guesses.

### Phase 0: Bring-up and baseline (1–2 weeks)

- Assemble one robot and run the Adeept stock code to validate the hardware. Flash Ubuntu 24.04 and install ROS 2 Jazzy (or Docker). Set up the Windows dev environment (WSL2 or devcontainer, VS Code, Foxglove). Set up a git monorepo (`robot/`, `gcs/`, `sim/`, `docs/`).
- **Milestone:** wheels turn from a ROS 2 `cmd_vel` via a minimal Python node driving the Robot HAT.
- **KPI baseline:** straight-line drift over 2 m; strafe drift; battery runtime.

### Phase 1: Single-robot ROS 2 foundations (2–3 weeks)

- HAL node with mecanum inverse kinematics; ultrasonic as `sensor_msgs/Range`; camera via `camera_ros`/libcamera; URDF and TF; gamepad teleop; rosbag2/MCAP recording.
- **Order hardware now:** a 2D LiDAR and an IMU per robot. Optionally, encoder gear motors and an ESP32/Pico per robot.
- **Ground-truth rig:** an overhead webcam over the arena, an AprilTag on each robot, and a pose node. This is your "motion capture".
- **Milestone:** teleop with live sensor views in Foxglove; bag replay.
- **Learn:** nodes, QoS, TF, launch, parameters.

### Phase 2: State estimation (2–3 weeks)

- LiDAR driver; laser odometry (rf2o or KISS-ICP) and/or encoder odometry from the MCU; IMU; robot_localization EKF.
- **Milestone and KPI:** odometry drift < 2% of distance traveled, measured against the AprilTag ground truth.
- **Learn:** covariances, frames (`map`/`odom`/`base_link`), time sync.

### Phase 3: SLAM + Nav2 on one robot (3–4 weeks)

- slam_toolbox mapping and localization; Nav2 with MPPI (DiffDrive first, then **Omni**); Collision Monitor; keep-out zones; custom BT; Groot2.
- **Milestone:** 20 random goals in the arena.
- **KPIs:** success ≥ 95%, final position error ≤ 5 cm, yaw error ≤ 5°, zero collisions, and average speed recorded.
- **Learn:** costmaps, planners vs. controllers, recoveries, tuning.

### Phase 4: Digital twin and CI (2 weeks, then ongoing)

- Gazebo Harmonic model; the same launch files with `use_sim_time`; Nav2 loopback sim for fast tests; launch_testing in CI; a **benchmark suite v1** (10 navigation scenarios).
- **Milestone:** one command runs the benchmark in simulation and prints KPIs. The sim-vs-real KPI gap is recorded.

### Phase 5: Single-robot autonomy and the Skill API (3–4 weeks)

- Define `fleet_interfaces` (skills, events, capability descriptor). Implement `NavigateTo`, `Explore` (explore_lite), `Search(class)` (YOLO on the Pi, optionally with a Hailo HAT), `ReturnHome` and `Capture`. Mission executive in BehaviorTree.CPP. Domain events go to the GCS event store (.NET + rclnet).
- **Milestone:** "Explore the arena and report where the red ball is" as a hand-written mission.
- **KPIs:** coverage rate (m²/min), time-to-detection, object position error vs. ground truth.

### Phase 6: Multi-robot (4–6 weeks)

- **6a:** namespaces, rmw_zenoh router topology, three robots, map merge with known initial poses, the .NET fleet manager, **Contract Net** with Nav2-cost bids, leases and comm-loss policy, and `tc netem` fault injection.
- **6b:** **CBBA** mode with no GCS needed for allocation; coordinated frontier auctions.
- **Milestone:** cooperative exploration with three robots.
- **KPIs:**
  - **Exploration speedup** vs. one robot: target ≥ 2× with three robots.
  - Overlap/redundancy < 25%.
  - Map merge error < 10 cm.
  - Allocation latency < 1 s.
  - Correct recovery in 100% of injected comm-loss scenarios.

### Phase 7: Natural-language missions with an LLM (4–6 weeks)

- Plan DSL (JSON Schema) → Claude Opus 5.5 planner with read-only world tools → validator (geofence, battery, capability, preconditions) → loopback-sim dry run → risk tier → BT execution → **event-driven replanning** on `SkillFailed`. Use Claude vision for occasional VLM questions ("is the door open?"). Add a narrow MCP server for an operator console.
- **Benchmark v2:** 30 NL missions ("Two robots search the north half for a red ball; the third waits at the door"), including **adversarial and unsafe requests**.
- **KPIs:**
  - Mission success ≥ 80%.
  - **Plan first-pass validity** ≥ 90%.
  - **Unsafe-plan execution = 0**.
  - Planning latency p95 < 15 s.
  - Cost per mission (tokens/$).
  - Human interventions per mission.

### Phase 8: Self-improvement (ongoing, 4+ weeks)

- Optuna tuning of MPPI and costmap params in simulation, then validation on one real robot. Nightly analyst agent (Claude Agent SDK) producing reflection reports and **proposals**, gated by benchmarks v1 and v2. Fleet hazard memory. Optionally, one RL or imitation side quest (for example camera-based docking, trained in Isaac Lab or MuJoCo, or a LeRobot dataset).
- **Milestone:** benchmark KPIs improve across ≥ 3 promoted config versions with no manual tuning and no safety regression.
- **KPIs:** Δ success, Δ time, Δ error per version; proposal acceptance rate; sim-to-real gap trend.

### Phase 9: Port to UAVs (8+ weeks)

- **9a:** PX4 SITL + Gazebo with 3 quads. Implement Skill API adapters with the PX4 ROS 2 Interface Library (custom modes and executors), or wrap Aerostack2 behaviors. Run the **same fleet manager, LLM planner, validator and event store unchanged**, adding 3D geofences and altitude bands.
- **9b (optional):** put a PX4 flight controller on one mecanum robot (PX4 rover mecanum) to rehearse the real autopilot workflow on the ground.
- **9c:** a Crazyflie swarm with Crazyswarm2 indoors, running the same NL missions.
- **9d:** one real PX4 quad with a Pi 5 or Jetson companion, then a mixed UGV+UAV mission ("drone scouts, ground robots inspect").
- **KPIs:** the same mission DSL runs on both vehicle types with **zero changes above the Skill API**; plus UAV-specific KPIs: position-hold error, mission time, battery margin at landing, and failsafe test pass rate.

### 8.1 KPI definitions: what "fast, accurate, intelligent" means

| Dimension | KPI | How to measure |
|---|---|---|
| **Fast** | Mission completion time; exploration rate (m²/min); time-to-first-detection; planning latency (LLM p50/p95); replan latency after failure; allocation latency; multi-robot speedup (T₁/T₃) | Domain-event timestamps |
| **Accurate** | Goal pose error (cm/°); localization ATE/RPE vs. AprilTag ground truth; map IoU vs. a measured floor plan; detection precision/recall; object localization error | Overhead-cam ground truth, evo/evaluation scripts |
| **Intelligent** | NL mission success rate on a fixed benchmark; plan first-pass validity; recovery success rate; human interventions/hour; exploration redundancy; generalisation to held-out missions; $ per mission | Benchmark harness, event projections |
| **Safe and robust** | Collisions and near-misses (Collision Monitor triggers); unsafe-plan execution (must be 0); comm-loss recovery success; MTBF; battery margin | Events plus fault-injection tests |
| **Self-improving** | KPI delta per promoted config version; proposal acceptance rate; sim-to-real KPI gap | Versioned benchmark runs |

---

## 9. Opinionated decisions (ADR-style shortlist)

1. **ROS 2 Jazzy on Ubuntu 24.04, with rmw_zenoh for multi-robot.** Revisit Lyrical in 2027.
2. **Buy a 2D LiDAR and an IMU per robot.** Encoders plus an MCU come second. An AprilTag ground-truth rig comes third.
3. **Nav2 + MPPI (Omni once closed-loop) + slam_toolbox + robot_localization + explore_lite/map_merge.**
4. **The Skill API (ROS 2 actions) is the narrow waist.** Nothing above it knows the vehicle type.
5. **BehaviorTree.CPP for both navigation and the mission executive.** The LLM generates a plan DSL that compiles to BT, never raw control.
6. **Centralized planning, decentralized execution.** Contract Net first, CBBA second, leases and comm-loss policy from day one.
7. **Event-sourced mission domain in .NET plus MCAP raw logs.**
8. **Claude** (Opus 5.5 planner, Haiku 5.5 summariser) through the official C# SDK, with a strict validator, sim dry-run and human approval tiers. ros-mcp-server for development only.
9. **Self-improvement = benchmark + auto-tuning + gated LLM proposals.** RL and VLA are optional side quests.
10. **PX4 for UAVs.** SITL first, Crazyswarm2 second, a real quad third. Aerostack2 as the reference design.

---

## 10. Open questions and things to verify

- **Adeept Robot HAT V3.2 specifics.** Now answered in [01-adeept-mecanum-kit.md](01-adeept-mecanum-kit.md) §2: PCA9685 @0x5F, ADS7830 @0x48, DRV8833 (inferred), full GPIO map. The 5V regulator rating is still unknown; measure it on the bench.
- **OS choice.** This report assumes Ubuntu 24.04 + native Jazzy. Reports 01/02 favour Raspberry Pi OS + ROS 2 in Docker, for camera, Hailo and Adeept-code compatibility. Resolved in [00-overview.md](00-overview.md).
- **Jazzy build status of m-explore-ros2** (source-only), and which LD19/LD06 driver to use.
- **Exact Microsoft Agent Framework GA date and language support.** Secondary sources disagree.
- **Zenoh .NET bindings.** Not researched. If they exist and are mature, the GCS could speak Zenoh directly without rclnet.
- **Aerostack2 Jazzy/Lyrical release status.** The index shows Humble-era packages. Check GitHub.

---

## Appendix A: Key links

- ROS 2 Lyrical: <https://discourse.openrobotics.org/t/ros-2-lyrical-luth-released/55021>
- Nav2 Kilted / Lyrical releases: <https://discourse.openrobotics.org/t/nav2-kilted-release/44079>, <https://discourse.openrobotics.org/t/nav2-lyrical-release-key-features/57666>
- mecanum_drive_controller: <https://control.ros.org/jazzy/doc/ros2_controllers/mecanum_drive_controller/doc/userdoc.html>
- rmw_zenoh: <https://github.com/ros2/rmw_zenoh>
- m-explore-ros2: <https://github.com/robo-friends/m-explore-ros2>
- linorobot2_hardware (micro-ROS firmware): <https://github.com/hippo5329/linorobot2_hardware/wiki>
- ROSA: <https://github.com/nasa-jpl/rosa>, ros-mcp-server: <https://github.com/robotmcp/ros-mcp-server>
- RoboGuard: <https://arxiv.org/abs/2503.07885>, REFLECT: <https://arxiv.org/pdf/2306.15724>, CoMuRoS: <https://arxiv.org/html/2511.22354v1>
- PX4 ROS 2 Control Interface: <https://docs.px4.io/main/en/ros2/px4_ros2_control_interface.html>, PX4 multi-vehicle ROS 2: <https://docs.px4.io/main/en/ros2/multi_vehicle>
- Aerostack2: <https://arxiv.org/html/2303.18237v2>, Crazyswarm2: <https://github.com/IMRCLab/crazyswarm2>
- rclnet: <https://github.com/noelex/rclnet>, Asv.Mavlink: <https://feed.nuget.org/packages/Asv.Mavlink>, Anthropic C# SDK: <https://platform.claude.com/docs/cli-sdks-libraries/sdks/csharp>
- Isaac Lab 3.0: <https://isaac-sim.github.io/IsaacLab/release/3.0.0/source/refs/release_notes.html>, MuJoCo Playground: <https://github.com/google-deepmind/mujoco_playground>
