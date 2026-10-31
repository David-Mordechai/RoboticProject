# Robot Car Team Project — Plan & Decisions

> Summary of a planning session (claude.ai, October 2026). Intended as context for a Claude Code agent running on the user's training PC.
> Suggested use: place this file in the project root as `CLAUDE.md` (or reference it from there) so the agent loads it automatically.
> Hardware and software research for these cars, including findings that affect this plan (F1–F12): [research/00-overview.md](research/00-overview.md).
> Status: **planning complete for the core system; some communication-hardware decisions are still open** (see section 10).

---

## 1. Project goal

Train two robot cars to operate as a team using simulation-based reinforcement learning (inspired by the Nvidia / Hugging Face "Microduck" sim-to-real approach):

1. **Phase A goal:** drive in formation (column, side-by-side, diagonal), switch formations on command, avoid obstacles while holding formation.
2. **Extended goal:** operate inside a building up to **~100 m** from the central PC, through **concrete walls**, with:
   - live **video** from each car to the central PC,
   - real-time **telemetry** sharing,
   - **shared mapping**: when one car discovers a new room, the other car receives the updated map immediately.
3. **End goal (user, 2026-10-31): fully autonomous office robots.** See §13. The driving and formation skills from Phase A serve this goal. Without any human in the loop, the cars:
   - **find a specific person** in the office and **deliver a message** to them;
   - **greet people they already know** in a funny way when they meet them;
   - **go home to the user's office when the battery is low** and ask *the user* for "food" because they're hungry. This act is only for the user, and only on low battery; never toward other people.

**User's working style / expectations (important for the agent):**
- Wants real engineering solutions to the actual problem, **not simplified approaches or workarounds**.
- Considers conservative time estimates unnecessary: expects a first working formation policy within **a day or two** of starting, assuming hardware cooperates. Physical steps (charging, test runs, hardware surprises) are the real time gates.
- The agent (Claude Code) will have **network access and full control of the cars** via an MCP server. The user handles physical tasks.

---

## 2. Hardware (confirmed)

### Cars (×2): HIWONDER MentorPi A1 Advanced Kit (Amazon ASIN B0FLJRW5XN)
- Ackermann (car-style) steering chassis
- Raspberry Pi 5, 8 GB
- ROS2 Humble (factory software; may be Docker-based, to be confirmed)
- TOF lidar, 3D depth camera
- Closed-loop encoder motors, high-torque steering servo
- Note: **Pi 5 has no hardware H.264 encoder** — video compression runs on the CPU and competes with SLAM / policy for compute.

### Training PC: ASUS Ascent GX10 ("GB10")
- NVIDIA GB10 Grace Blackwell superchip, ARM CPU (aarch64), GPU compute capability sm_121, 128 GB unified memory, CUDA 13
- Likely runs DGX OS (Ubuntu 24.04-based) — verify.

### Suggested additions
- Overhead camera (phone or webcam mounted above the test area) for ground-truth video of real runs — used for simulator validation and debugging.
- Gamepad (or similar) as a **physical kill switch** that bypasses the agent.

---

## 3. GB10 software compatibility (researched)

| Component | Status | Notes |
|---|---|---|
| PyTorch (core of our simulator + training) | ✅ Works | Use CUDA 13.0 (cu130) aarch64 wheels from **PyTorch's own index URL**, not plain PyPI. A warning about sm_121 vs max supported 12.0 can be ignored (sm_120/sm_121 binary compatible). |
| Isaac Sim / Isaac Lab (optional, later) | ✅ Works with limits | Runs natively on ARM (Arm has published a full Isaac Lab humanoid training workflow on DGX Spark/GB10). Limits: some features unsupported on DGX Spark, incl. camera-based (visuomotor) policy inference — not needed for our lidar/pose-based policy. **Driver matters**: Isaac Sim 6.0.1 crashes reported with driver 595.x; validated driver was 580.159.03 via the DGX Spark dashboard (use ASUS's equivalent update tool). |
| JAX | ⚠️ CPU-only by default on aarch64 | Avoid; use PyTorch. |
| ROS2 | ⚠️ Version mismatch | Cars run Humble (Ubuntu 22.04). GB10 OS is Ubuntu 24.04-based (native = Jazzy). **Decision: run ROS2 Humble in Docker on the GB10.** |
| onnxruntime-gpu | ⚠️ No official aarch64 CUDA 13 wheel | Not needed on GB10 for our use; policies run on the cars' Pi 5 CPU. |

---

## 4. Way of working (decided)

- **Claude Code + the MCP server are development-stage tools** (user, 2026-10-31). They measure metrics and bring the cars up when they arrive. The cars' autonomous behaviour at the end runs on **our own brain with a local model on the GB10** (§13), not on Claude.
- **Claude Code runs on the GB10**, with access to the local network, the cars (SSH + ROS2), the simulator, and the training jobs.
- **MCP server runs locally on the GB10** — nothing exposed to the internet.
- **Division of work:**
  - User: charging, placing cars, clearing the test area, holding the kill switch, hardware fixes, being physically present when cars move.
  - Agent: all code, experiments via MCP tools, data analysis, simulator, training, deployment. Agent asks before anything new (first test at higher speed, first real run of a new policy).

---

## 5. System architecture (decided)

### Control layers on each car
0. **Mission layer ("brain", §13)**. It decides what to do next: search for a person, deliver a message, greet, go home. It runs our own logic plus a local model on the GB10, and gives the layers below a goal on the map.
1. **Localization** — each car knows its own pose and the teammate's pose (lidar SLAM on a shared map; later collaborative SLAM, see §9.3). ArUco markers on cars are a supplementary option for direct relative pose.
2. **Formation policy (trained in simulation)** — small neural network.
   - Inputs: own speed + steering, pose relative to target formation slot, teammate relative pose + velocity, downsampled lidar distances, (later) link-quality measurements (§9.2).
   - Outputs: speed + steering commands.
3. **Low-level control** — Hiwonder's existing ROS2 stack converts `cmd_vel` into motor/servo signals.

### Machines
- **GB10:** Claude Code, MCP server, simulator + training, data store (ROS bags, models), ROS2 Humble in Docker.
- **Each car:** Hiwonder ROS2 stack, our safety node (watchdog + limits), later the policy node (ONNX on Pi 5 CPU at ~20 Hz).

---

## 6. MCP server design (decided)

### Tools
- **Status:** `list_cars`, `get_status` (battery, connection, mode), `get_pose`, `get_lidar_snapshot`
- **Motion:** `drive(car, speed, steering, duration)`, `stop(car)`, `stop_all()`
- **Experiments:** `run_test(car, test_type)` — predefined calibration maneuvers (speed steps, full-lock turns, circles); `start_recording` / `stop_recording` (ROS2 bags saved on GB10)
- **Deployment:** `deploy_policy(car, model_file)`, `start_policy`, `stop_policy`, `set_formation(offset)`

### Safety rules (non-negotiable, built in from day one)
- **Server-side hard limits** on speed, command duration, and allowed area (geofence on the SLAM map) — enforced regardless of what the agent requests.
- **Watchdog on each car:** no command for 0.5 s → stop (covers link loss, crashes, frozen sessions).
- **Physical kill switch** held by the user, independent of the agent.
- **Low speed limits** initially; raised only by the user's decision.
- **User physically present** whenever cars move, at least until behavior is proven stable.
- First tests of new MCP tools done with **wheels off the ground**.

---

## 7. Simulation & training (decided)

### Simulator ("virtual lab")
- **Primary:** custom **batched GPU simulator in plain PyTorch** using the **kinematic bicycle model** (accurate for Ackermann cars at low speed on flat floors). Tens of thousands of car pairs in parallel; training in minutes.
- Parameters come from **measured car data** (Phase 2): wheelbase, max steering angle, max speed, acceleration curve, command response delay, battery-dependent speed drop.
- **Validation requirement:** replay recorded real commands in the sim and compare trajectories with real runs (overhead camera + logs). Trust the sim only once they match.
- **Optional later:** MuJoCo/mjlab or Isaac Lab for collisions / 3D terrain (check aarch64 support at that time).

### Training
- Algorithm: **MAPPO** (multi-agent PPO), **centralized training, decentralized execution** — critic sees everything in training; each car's policy uses only its own real sensor inputs.
- **Formation as an input:** target offset (e.g., 1 m behind, 0.8 m left) is part of the observation → one policy handles all formations; switch at runtime via `set_formation`.
- **Reward:** + small distance to target slot, + heading match with leader, + route progress; − collisions (walls, teammate), − closer than safe distance, − jerky steering. (Extended: connectivity terms, §9.2.)
- **Domain randomization:**
  - communication delay and dropped messages for teammate pose (most important for teamwork)
  - SLAM pose noise and jumps
  - steering servo delay and limits, speed limits
  - floor friction, battery-voltage effects on speed
  - track layouts and obstacle positions
- **Deployment:** export to **ONNX**, run as ROS2 node on Pi 5 CPU at the same control frequency as in sim (~20 Hz).

---

## 8. Project phases (decided)

| Phase | Owner | Content |
|---|---|---|
| 0 — Setup | User (agent guides) | Claude Code on GB10; SSH + ROS2 connectivity GB10↔cars (Humble in Docker); clear test area; build SLAM map; mount overhead camera |
| 1 — MCP server | Agent | Server + safety node; test every tool with wheels off the ground |
| 2 — Data collection | Agent drives, user supervises | Calibration maneuvers; measure steering limits, response delay, acceleration, battery effects; measure inter-car latency and pose noise |
| 3 — Virtual lab | Agent | Build batched PyTorch sim from measured data; validate against real recordings |
| 4 — Training | Agent | Leader-follower → obstacles + formation switching → full two-car MAPPO; report with plots + sim videos |
| 5 — Deploy & iterate | Together | Low-speed real tests, recorded; differences feed back into Phase 3 |

Recommended progression inside Phases 4–5: classical baseline first (car A follows a Nav2 path, car B holds offset with PID) to test localization + networking without AI; then RL leader-follower; then obstacles + switching; then both cars as equals.

---

## 9. Building-scale communication & mapping (direction decided: points 3, 4, 5)

Requirements: ~100 m inside a building, **concrete walls**, video to central PC, real-time telemetry and map sharing between cars.

### 9.1 Network layer — DECIDED
- **Mesh routing** across both cars, the PC, and any fixed nodes; every node forwards traffic (e.g., **BATMAN-adv**).
- **Traffic manager with QoS classes:**
  1. Control + telemetry — highest priority, guaranteed, low latency
  2. Map updates — **reliable multicast** to all cars, with retransmission
  3. Video — **adaptive bitrate** with error correction; never starves classes 1–2
- **Continuous link-quality measurement** between every pair of nodes (signal strength, throughput) → input to §9.2.
- Time synchronization across nodes.

### 9.2 Connectivity-aware formation / exploration policy — DECIDED
- **Radio propagation model in the simulator:** distance loss + per-concrete-wall loss, with wall positions taken from the SLAM building map, **calibrated with a real site survey** (drive a car through the building recording signal strength + throughput everywhere).
- Policy observations include link quality to teammate and to the PC.
- Reward includes keeping a working data path to the PC and avoiding disconnections, alongside the task reward.
- Expected learned behavior: e.g., car B automatically holds a relay position when car A enters a shielded room.
- Stress tests: video + map bursts + telemetry simultaneously at worst locations.

### 9.3 Collaborative SLAM — DECIDED
- **Decentralized collaborative SLAM** (e.g., **Swarm-SLAM**, ROS2): each car maps on its own; cars exchange compact place descriptors, detect shared places, compute relative poses, merge maps — no requirement to start from known positions.
- New map areas distributed via the reliable map channel (§9.1).
- **Engineering item:** Pi 5 compute budget (SLAM + map merging + CPU video encoding + policy). Profile each component during setup and decide placement.

### 9.4 Dependency order
1. Radio hardware choice (sets available bandwidth) — **OPEN, see §10**
2. Network layer (§9.1)
3. Collaborative SLAM (§9.3) → building map
4. Site survey → calibrated radio model
5. Connectivity-aware training (§9.2)

---

## 10. Open decisions (to resolve with the user)

### 10.1 Radio / relay hardware — NOT YET DECIDED
Options discussed:

| Option | Pros | Cons |
|---|---|---|
| **Wi-Fi mesh (802.11s) with cars as relays** — latest direction the user raised | Cheap, available hardware; no router or building infrastructure; auto-routing, no reconnection delays while moving | 2.4/5 GHz penetrates concrete poorly → shorter hops; 100 m may need 1–2 fixed mesh nodes in corridors |
| Professional MIMO mesh radios (Doodle Labs Mesh Rider, Silvus StreamCaster) | Built for this exact job; sub-GHz options; tens of Mbps; self-healing mesh | Hundreds to thousands of USD per radio |
| COFDM video links | Designed for no-line-of-sight, heavy multipath video | Mainly video-focused; cost |
| Custom software-defined radio (own OFDM + FEC on sub-GHz) | Full control | A serious research project on its own |
| Wi-Fi HaLow (802.11ah, sub-GHz) | Much better wall penetration | Few Mbps per hop at distance; regional band rules |

**If 802.11s Wi-Fi mesh is chosen (recommended implementation):**
- **Do not** use plain hotspot chaining (slow AP reconnection while moving, fixed topology, throughput halves per hop on one radio).
- Add **USB Wi-Fi adapters with external antennas** with solid Linux mesh support (e.g., **MediaTek mt76-based** chipsets); the Pi 5 built-in Wi-Fi is not suitable for mesh mode.
- **Two radios per car** on different channels (receive/forward) to avoid halving throughput per hop.
- Run **BATMAN-adv** on top for link-quality-based routing.

**Unresolved point:** the user earlier asked for a solution "without Wi-Fi", then proposed cars acting as hotspots. Confirm whether Wi-Fi hardware is acceptable as long as no router / building infrastructure is used.

### 10.2 Link-budget inputs
- Walls: **concrete** (confirmed). Still needed: reinforced or not, and approximate number of walls between the farthest point and the PC.
- Rough link-budget reference (very approximate, real values depend on building): free-space loss over 100 m ≈ 71 dB @ 900 MHz, 80 dB @ 2.4 GHz, 88 dB @ 5.8 GHz; each concrete wall adds roughly 10–15 dB @ 2.4 GHz, more at 5 GHz, less at sub-GHz.
- **Regulation check required:** allowed sub-GHz bands, power, and channel widths in Israel (Ministry of Communications) before buying any sub-GHz hardware.

### 10.3 Other items to confirm at start
- Whether the cars' Hiwonder ROS2 stack runs natively or in Docker, and whether it has been modified.
- Cars' IP addresses and login credentials.
- GB10 OS version and NVIDIA driver version.

---

## 11. Earlier communication ideas (considered, superseded or supplementary)

- **ESP32 + ESP-NOW** for car-to-car telemetry: low latency, router-independent, ~250-byte packets, good for pose/e-stop data — but **cannot carry video** and does not solve 100 m through concrete. Superseded by the mesh network direction for the building-scale goal.
- **ESP32 + LoRa** (e.g., SX1262): kilometers of range, few kbps — candidate for an **independent emergency-stop / heartbeat channel** that works even if the main network fails. Recommended as a safety supplement; not formally decided.
- ESP32 Wi-Fi itself cannot provide high-bandwidth video or significantly beat Wi-Fi range (it *is* a 2.4 GHz Wi-Fi radio).

---

## 12. First steps for the agent when the session starts

1. Read this file; confirm open items in §10.3 with the user.
2. Phase 0 checks: GB10 OS/driver, install PyTorch cu130 (PyTorch index), set up ROS2 Humble in Docker, verify SSH + ROS2 topic visibility to both cars (shared `ROS_DOMAIN_ID`).
3. Explore the Hiwonder software layout on the cars (launch files, topics, message types, Docker usage).
4. Build the MCP server + safety node (Phase 1), test with wheels off the ground.
5. In parallel, resolve §10.1 (radio hardware) with the user, since it gates the building-scale work in §9.

---

## 13. Autonomous office behaviours (end goal, 2026-10-31)

### 13.1 Behaviours
| Behaviour | Trigger | What the car does |
|---|---|---|
| Deliver a message | A task: "find X, tell them Y" | 1. Searches the office; with two cars, they split the map. 2. Finds X by face. 3. Approaches and stops at a polite distance. 4. Speaks the message. 5. Reports back. |
| Greet a known person | It sees an enrolled face, and hasn't recently greeted that person | Says something funny by name, possibly with a small motion. Never blocks the person's path. |
| Go home when hungry | Battery below a threshold, from a **measured** discharge curve | Abandons or parks the current task and drives home to the user's office. **Only to the user** (recognized face) does it say it's hungry and ask for "food" (charging). |
| Idle / patrol | No task | Parks at home, or patrols, as the user decides. |

### 13.2 Components: what the Advanced kit ships vs what we build
Checked 2026-10-31 against Hiwonder's code (`docs/vendor/hiwonder/code/`), the docs' table of contents and the product page. See `research/02` and `research/03`.

| Component | Out of the box | We build |
|---|---|---|
| Face **detection** | ✅ MediaPipe FaceDetector (`mediapipe_example/face_detect.py`), plus pose, hands, face mesh | – |
| Face **recognition** (who it is) | ❌ Not in the code, docs or product page | Face-embedding model and an enrolled-people store. Detection runs on the car, matching on the GB10 or the Pi (profile it) |
| Person / object detection | ✅ YOLOv5 (product page: YOLOv11); VLM "track anything" | Person-search behaviour |
| Voice | ✅ WonderEcho Pro, speech-to-text, text-to-speech, LLM function calling (`large_models`) | Our phrases and personality; local speech models |
| Language / vision model | ✅ OpenAI-compatible client with a configurable `base_url`. Defaults are cloud APIs (Alibaba Qwen, StepFun) | **Local model served on the GB10**, with no cloud |
| Map, localization, route to a room or person | ✅ slam_toolbox, AMCL, Nav2 | Shared map and named places such as "home = the user's office" (`research/03` §3) |
| Local driving among obstacles | – | Our RL driving policy (§7; trained in sim) |
| Battery level | ✅ RRC Lite battery voltage (`/ros_robot_controller/battery`) | Measured discharge curve → "hungry" threshold with a reserve to reach home |
| Mission logic | – | Our brain: a task/behaviour layer on the GB10 that talks to each car over the network |

### 13.3 Rules
- **Privacy:** only people who agreed are enrolled for recognition. Faces and embeddings stay on our machines (car and GB10); nothing goes to the cloud.
- **"Hungry" is owner-only:** it's triggered only by low battery and addressed only to the recognized user.
- **Safety (§6):** the safety rules still apply to every autonomous behaviour: watchdog, speed limits, kill switch and geofence.
- **Network loss:** the brain runs on the GB10, so a car that loses the link must stop safely or drive home on its own. Define this together with the mesh work (§9).

### 13.4 Training replan (2026-10-31): use the car's stock stack, train only the gaps
The car ships the "go somewhere safely" skill: Nav2 with AMCL and LiDAR costmaps (`navigation/`), named-place
navigation (`large_models/vllm_navigation.py`), LiDAR avoid/follow/guard (`app/lidar_controller.py`) and person
following with YOLO + depth (`example/.../body_track.py`). We use these as they are. The RL navigation task
(`Robotlab-Navigate-MentorPiA1`) duplicated Nav2 and is shelved; its walking-people simulation is reused.

Hiwonder's two-car formation code (`multi/formation_update.py`, row/column/triangle) sends lateral velocity and
assumes aligned start poses, so it doesn't fit Ackermann cars. Team driving is the gap. Architecture: **the leader
runs stock Nav2; the follower runs our trained policy** and holds a formation slot relative to the leader.

| Step | Skill |
|---|---|
| T1 | Formation follower: slot as an input (column, diagonal left/right, side-by-side), Nav2-like leader, walking people |
| T2 | People seen through the stock YOLO + depth detections, with realistic noise and delay |
| T3 | Formation switching on command (e.g. side-by-side to column when the hallway narrows) |
| Later | Relay positioning for the mesh network (§9.2) |

Measure on the real cars: how stock Nav2 drives an Ackermann A1 (car-like planner settings), which calibrates the sim
leader, and the range and rate of the stock person detector, for T2.
