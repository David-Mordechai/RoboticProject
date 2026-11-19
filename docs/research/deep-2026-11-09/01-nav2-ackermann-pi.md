# Deep research 2026-11-09, track 1: Nav2 on a small Ackermann car with a Raspberry Pi 5

Source: research agent report. [V] = checked in source code or docs (URL given); [I] = inference, to be tested.

## Why our Nav2 trial of 11-04 stalled ("Starting point in lethal space")
- [V] **Humble Smac start check:** Humble's Smac `a_star.cpp` throws "Starting point in lethal space" when the start fails the collision check ([src](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_smac_planner/src/a_star.cpp)).
- [V] **Centre-point check with `robot_radius`:** with `robot_radius` set, the checker looks only at the centre cell and treats cost >= 253 (INSCRIBED) as collision. It also treats UNKNOWN as collision unless `allow_unknown` is set ([collision_checker.cpp](https://raw.githubusercontent.com/ros-navigation/navigation2/humble/nav2_smac_planner/src/collision_checker.cpp)).
- [I] **Our `robot_radius 0.14` made it worse:** the real inscribed radius is ~0.08 m (width 0.159 m), so any obstacle within 0.14 m of the centre blocks every plan. A polygon footprint checks the true shape.
- [V] **MPPI fails the same way:** its ObstaclesCritic counts INSCRIBED as collision when `consider_footprint` is false, and fails after retries. [I] That is our stall after BackUp.
- [V] **Jazzy removed the start check** ([src](https://raw.githubusercontent.com/ros-navigation/navigation2/jazzy/nav2_smac_planner/src/a_star.cpp)). Maintainers explain the radius-mode behaviour in [#4271](https://github.com/ros-navigation/navigation2/issues/4271).
- [V] **Hiwonder's own MentorPi-A1 navigation config is not a car config** ([repo](https://github.com/Hiwonder/MentorPi-ROS/tree/MentorPi-A1/navigation)):
  - planner: NavFn
  - behaviour tree: includes Spin
  - footprint: `robot_radius 0.05`
  - controller: TEB, `min_turning_radius 0.785`, `use_teb` true

## CPU
- [V] **Idle load comes from bond heartbeats.** The container used 140% CPU idle at a 0.1 s heartbeat ([#5784](https://github.com/ros-navigation/navigation2/issues/5784)). The fix is `bond_timeout: 0.0`.
- [V] **slam_toolbox (async) on a Pi 5:** about 1% of a core.
- [V] **No Pi 5 MPPI measurement exists.** MPPI uses 49% of a core on a Ryzen desktop and 65% on an Orin. It crashed with "illegal instruction" on a Pi 4 running Humble (#4380).
- [I] **Verdict:** MPPI is borderline on a Pi 5; RPP is cheap.

## Recovery
- [V] **Humble BackUp / DriveOnHeading drive straight** (`angular.z = 0`).
- [I] **Recovery sequence:**
  1. ClearCostmapAroundRobot
  2. BackUp 0.2 m
  3. Wait
  4. Replan
  - No Spin; cap the retries.
  - Let a REEDS_SHEPP planner do the real escape with reverse arcs.

## Success rates
- [V] No published rates exist for small Ackermann robots in offices.
- [V] Differential-drive references: MPPI 53% / DWB 50% on DynaBARN; MPPI 51/54, RPP 48/54 in narrow corridors.
- **Common failures:** wrong footprint, bad inflation, rotate-in-place recoveries, compute too slow.

## Recommended configuration (estimates until measured)
| Parameter | Value |
|---|---|
| footprint | Rectangle 0.213 x 0.159 m + 1 cm, relative to base_footprint (measure where base_footprint sits); no robot_radius |
| inflation | Global 0.40 / scale 4.0; local 0.30 / scale 5.0 |
| resolution | 0.05 m; local rolling 3 x 3 m at 5 Hz; global at 1 Hz |
| unknown | `track_unknown_space` true, `allow_unknown` true |
| planner | SmacPlannerHybrid REEDS_SHEPP, `minimum_turning_radius` 0.40, `reverse_penalty` 2.1, `cost_penalty` 2.0, 64 angle bins, `cache_obstacle_heuristic` |
| controller | RPP, `allow_reversing` true, `use_rotate_to_heading` false, `use_collision_detection` true, 20 Hz. (MPPI alternative: Ackermann `min_turning_r` 0.40, batch 1000, time_steps 40, `model_dt` 0.05, `consider_footprint` true) |
| behaviour tree | No Spin: clear costmap -> BackUp 0.2 m -> Wait 2 s -> replan, max 4 retries |
| CPU | composition, `bond_timeout` 0.0, `visualize` false |
| distro | Jazzy if possible (start check removed, Smac speedups) |
