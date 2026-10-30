# Open issues

Next steps after the 2026-10-30 session, in priority order. Each has a goal, its current state and what "done" means. Follow the CLAUDE.md hard rule: look up the spec, cite it, never assume.

**State at hand-off:**
- Branch `mentorpi-virtual-lab`, commit `7c7c561`.
- `Robotlab-Office-MentorPiA1` is solved in sim. Run `logs/rsl_rl/office/2026-10-30_13-19-00`, `model_200.pt`, completed 100% of 512 one-minute evaluation drives with 0 crashes (`lab/scripts/evaluate_office.py`).

---

## 1. Domain randomization (sim-to-real), Microduck style
**Why:** the policy has only seen one ideal car. Microduck randomizes physics so its policies survive the real robot (`microduck_velocity_env_cfg.py`: `ENABLE_*` flags, mjlab events).

**Do:** add mjlab event terms behind `ENABLE_*` flags for:
- tyre friction
- mass and inertia (±5%, as Microduck does)
- drive-motor torque: unknown, so use a wide range
- steering servo delay
- policy-to-actuator latency
- LiDAR noise and dropouts
- leader pose noise

Steering randomization (29–37°) is already done, inside `mdp.AckermannAction`. Consider moving it to an event for consistency.

**Done when:** the policy is retrained with the randomization on, and still passes `evaluate_office.py` at 95% or better.

## 2. ONNX export and policy runtime on the Pi 5
**Why:** the cars run the policy on board at 20 Hz.

**Do:**
- Follow Microduck's `export.py` and `checkpoints.py`: export the actor with the obs normalizer baked in.
- Write a parity test: torch vs ONNX Runtime on recorded observations.
- Write down the exact observation layout. It's 54 values: trail error 4, trail preview 4, car motion 4, leader 6, LiDAR 36.

**Done when:** the ONNX file matches torch within 1e-5, and its inference time is measured on a Pi 5.

## 3. Measure the real cars and update the model
**Why:** several values are unpublished or contradictory. See `docs/research/00-overview.md` "Open items" and `robot/mentorpi_a1_constants.py`.

**Measure, per car:**
- **Steering build:** 2024 (29°) or 2025 (40°).
- **Turning circle** at full lock.
- **Wheel diameter:** Hiwonder's drawing says 65 mm, their code says 67 mm.
- **Wheelbase and track:** Hiwonder's code says 0.145 / 0.133 m, their URDF says 0.137 / 0.139 m.
- **Top speed and acceleration:** this tells us the motor torque, which is unpublished. The sim value of 0.3 N·m is an estimate.
- **LiDAR model and scan-plane height:** the sim uses an estimated 0.14 m.
- **Steering servo response time.**

**Done when:**
- every measured value in the constants cites its measurement
- the steering randomization is narrowed to the measured build
- the sim turning circle matches the measured one

## 4. Retrain the follow task
`Robotlab-Follow-MentorPiA1` (`logs/rsl_rl/follow/2026-10-30_09-23-00`) was trained before the steering fix, with the car steering about 5°. That checkpoint is invalid.

**Done when:** it's retrained on the fixed car model and evaluated.

## 5. Two learning cars (multi-agent)
**Goal:** project plan Phase A, with both cars controlled by policies.

**Do:**
- MAPPO, which mjlab doesn't provide: write a wrapper and a runner.
- Each car observes the other's relative pose.
- Shared or per-car actors.

**Done when:** two learning cars follow each other through the office with 0 collisions in evaluation.

## 6. Formations task
**Goal:** project plan: column, side-by-side and diagonal formations; switching formation on command; keeping formation around obstacles.

**Depends on:** #5.

## 7. Leader pose on the real cars
**Problem:** the policy observes the leader's relative pose. On real cars that needs localization and car-to-car communication. Stock odometry is open-loop (research finding F1), so this needs a real source: LiDAR odometry or SLAM, or the encoders via a custom RRC firmware path.

**Done when:** we have a chosen and tested source of relative pose, with its noise level fed back into #1.

## 8. LiDAR fidelity
**Problem:** the sim gives 36 rays, clipped at 3 m, refreshed every policy step (20 Hz). The real LD19 (or MS200 / COIN-D6) spins at about 10 Hz, ranges about 12 m, and has noise and dropouts.

**Do:** model the scan rate, the downsampling and the noise from the sensor datasheet (`docs/vendor/hiwonder/datasheets/LDROBOT_LD19_Development_Manual_EN_v2.4.pdf`).

## 9. Stale docs and files
- **`docs/docker-env.md` and `docs/virtual-lab-plan.md`:** they still describe the pre-mjlab design. Update them for mjlab, the viewer, cu130, driver 617, the Warp cache volume and the Microduck layout.
- **CLAUDE.md:** the quick-start command and the doc map need refreshing.
- **`lab/scripts/gpu_check.py` and `lab/tests/test_environment.py`:** written before mjlab. Verify or remove them.
- **Training-speed note:** the 6 GB RTX 2060 can't train and run the viewer at the same time (4.4× slowdown). Write this down.

## 10. Run the lab on the GB10
**Do:**
- Build the image natively on aarch64 (`docker/env/gb10.env`, sm_121, cu130).
- Run `pytest` and a short training run.
- Compare throughput with the PC: 18–21k steps/s at 1,024 envs on the RTX 2060.

## 11. Office realism (later)
Ideas for later:
- doors and open rooms
- hallway widths varied around measured real hallways
- moving obstacles, such as people
- glass and dark surfaces that a LiDAR misses

## 12. Showcase video upkeep
`lab/scripts/make_showcase.py` rebuilds everything from a run directory: shots, segments, master, and an SSIM-checked delivery encode with aq-mode 3. Re-run it after #1 and #3 to show the randomized, measured car. Its stage captions are written for v3 (checkpoints 0 / 50 / 100 / final), so check them against new footage.
