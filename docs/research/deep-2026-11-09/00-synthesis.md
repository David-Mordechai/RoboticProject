# Deep research 2026-11-09: why we failed, and the solution

Tracks:
- 01: Nav2 on a small Ackermann car and a Pi 5
- 02: learned local planners that worked on real robots
- 03: low and thin obstacle perception

Our own data: floor logs goto17-23, the office twin, and the history in `docs/lessons-learned.md`.

Notation: **[V]** = verified (source code, a paper, or our own logs). **[J]** = my judgement.

## 1. Why the last days did not converge

1. **[V] We dropped the classical stack because of a config error, not a real limit.**
   - Nav2 on 11-04 stopped with "Starting point in lethal space".
   - Cause: our `car/nav/make_params.py` used `robot_radius 0.14`. Humble Smac and MPPI check only the centre cell against the inscribed cost, so any obstacle within 0.14 m blocked every plan.
   - The same file also had:
     - MPPI `consider_footprint: False`;
     - an MPPI turning radius (0.30) different from the planner's (0.40);
     - default lifecycle bonds, which cause the idle CPU load (#5714, #5784).
   - Hiwonder's own A1 config is not a car config: NavFn, Spin, 0.05 m radius.
   - We switched to end-to-end RL after one day instead of fixing these.
2. **[V] Hand-typed waypoints, three of them on obstacles.**
   - The first waypoint is on the desk legs, the door waypoints are on the door frame, and the hall lanes are 0.1-0.3 m from the left wall.
   - No planner ever computed a path from the LiDAR and depth data.
   - Every BARN team uses a global planner. A LiDAR-only learned policy reached 40% in confined spaces (2507.04086).
3. **[V] RL with a clipped Gaussian produced bang-bang steering.**
   - goto17 log: 24% of driving time at full lock, 3.4 steering reversals per second. That is the weaving.
   - Known in the literature (Seyde 2021, Chou 2017). Reward shaping does not remove it.
4. **[V] Imitation of a privileged teacher that sees the true map.**
   - The student ignored its sensors: causal confusion (de Haan 2019).
   - LiCS avoids this: a sensor-based expert, noisy executed actions, clean labels.
5. **[V] Exams not calibrated to the real site until 11-09.**
   - Random-office numbers rose from 5/16 to 12/16 while the floor stayed a failure.
   - The office twin later matched the floor: v6p median 63 s and 13 back-ups, against 63-67 s and 13-14 on the floor.
6. **[V] Interacting hand rules grew** (back-ups, multi-point turn, steer-around, off-route back-up, memory checks). The U-turn in goto23 had 21 back-ups from the off-route rule alone.
7. **[V] One floor run per change, and no classical baseline on the same course.** BARN uses 5 trials per course and always compares against a baseline.

## 2. What works elsewhere (evidence)

- **Hybrid architecture is the norm.** BARN 2024: "All teams adopted a hybrid paradigm"; "a separate safety check layer is still required".
- **Learning has won physically only once, by imitation.** BARN 2024 was won by learning once (LiCS):
  - behaviour cloning of a classical expert;
  - LiDAR plus an A* local goal as input;
  - noise on the executed actions;
  - plus a safety layer.
  - Classical approaches won in 2022, 2023 and 2025.
- **Modular systems transfer better.** Modular: 90% real. End-to-end: 77% in sim, 23% real (2212.00922).
- **Smooth steering comes from the right action setup:**
  - integrated (rate) actions;
  - L2C2 / LipsNet;
  - a Beta or squashed policy;
  - an actuator model, which matters most for transfer (2504.02420, 2410.16632).
- **Perception for low obstacles:**
  - LiDAR obstacle layer, plus an STVL layer for the depth points (15 s decay, measured FOV, 0.2 m min_z);
  - inflation that covers the chair base around the LiDAR-visible post;
  - Collision Monitor;
  - a bumper or low ToF sensor for the 0-0.2 m blind zone.

## 3. The solution: phases, each with a gate

### Phase A: the classical baseline on the car
This becomes the expert and the bar to beat.
- **Planner:** Nav2 Smac Hybrid-A*, REEDS_SHEPP, turning radius 0.40.
- **Footprint:** a polygon footprint, no `robot_radius`.
- **Inflation:** 0.40 / 4.0 global and 0.30 / 5.0 local.
- **Controller:** RPP with reversing (cheap on the Pi). MPPI only if the CPU allows it.
- **Costmaps:** STVL for depth plus the LiDAR obstacle layer, and Collision Monitor.
- **Behaviour tree:** no Spin. Clear, BackUp 0.2 m, Wait, replan, at most 4 retries.
- **CPU:** `bond_timeout 0`, composition.
- **The route:** goals only (door, hall end, charger), no hand-typed waypoints. The planner finds the gap between the table and the chair from the costmap.
- **Gate A:** on the real floor, 5 trials per course:
  - C1: charger to door
  - C2: door, hall end, U-turn
  - C3: back to the charger
  - Measured: success count, time, contacts. Against v6p (5 trials on the same courses).
  - Pass = it beats v6p on success count and has no contacts.
- **Checks before the floor:**
  - planner output on saved maps, offline;
  - CPU profile on the Pi with the camera at 15 fps;
  - the minimum speed at which the car moves.

### Phase B: the learned driver (the "AI for robots" part)
- **Expert:** the Phase A controller logic, run in our GPU sim on the planner's local goal and sensor-built costmaps.
- **Training:** behaviour cloning, LiCS-style: noisy executed actions (sigma ~0.25), clean labels, then DAgger rounds.
- **Inputs:** LiDAR sectors, depth sectors, local goal from the global path, previous action.
- **Outputs:** speed and steering rate (integrated), with an L2C2 smoothness term. Actuator model from car measurements.
- **Runs inside the stack:** global planner, then the learned local driver, then Collision Monitor. Recovery stays in the behaviour tree.
- **Gate B:**
  - Office twin first: at least the Phase A numbers.
  - Then 5-10 floor trials per course against Phase A on the same courses.
  - Promoted only if better; otherwise Phase A stays.

### Phase C: learned skills only where Phase A/B measurably fail
For example narrow dead-end escape (RL, arXiv 2511.22338; sim results only so far).

## 4. Known risks (not proven by any source)
- **[J] No published success rates** exist for small Ackermann cars in offices; Phase A may need tuning on the floor.
- **[J] Pi 5 CPU** with STVL, the depth camera and Nav2 has no published measurement; it must be profiled.
- **[J] Odometry** is commanded speed plus gyro (no encoders), so error builds along the corridor. Fuse IMU yaw in an EKF with the gyro bias removed.
- **[J] Chair bases are invisible below 0.2 m and under the LiDAR.** Without a bumper or low ToF sensor, a chair moved into the path while the car turns can still be hit.
- **[J] The twin lacks depth-only obstacles** (chair bases, door stopper), so floor trials remain the final judge.

## 5. Transfer to other robots
The same structure applies to drones and legged robots:
1. map and plan;
2. a learned local controller cloned from a working classical expert;
3. an independent safety layer;
4. a site twin from the robot's own map;
5. N real trials against a baseline.
