# Open issues

Next steps, in priority order (updated 2026-10-31). The end goal is fully autonomous office robots (`project-plan.md` §1.3, §13); #13–#18 cover it. Each has a goal, its current state and what "done" means. Follow the CLAUDE.md hard rule: look up the spec, cite it, never assume.

**State at hand-off:**
- Branch `mentorpi-virtual-lab`, commit `7c7c561`.
- `Robotlab-Office-MentorPiA1` is solved in sim. Run `logs/rsl_rl/office/2026-10-30_13-19-00`, `model_200.pt`, completed 100% of 512 one-minute evaluation drives with 0 crashes (`lab/scripts/evaluate_office.py`).

---

## 1. Domain randomization (sim-to-real), Microduck style
**Why:** the policy has only seen one ideal car. Microduck randomizes physics so its policies survive the real robot (`microduck_velocity_env_cfg.py`: `ENABLE_*` flags, mjlab events).

**Round 1 (2026-10-31): ✅ done.** Run `logs/rsl_rl/office/2026-10-31_17-49-48`, `model_599.pt`: `evaluate_office.py` 100% completed, 0 crashed, 0 lost (512 × 30 s), both without and with `--obs-noise`; mean trail error 1 cm. Implemented in `lab/src/robotlab/tasks/randomization.py`. Each term has its own `ENABLE_*` flag and a cited source, or is marked "NO SOURCE". Tested in `lab/tests/test_randomization.py`. Terms:
- tyre friction
- mass and inertia ±5%
- drive torque and speed-loop gain
- steering servo dead time and speed
- policy-to-motor latency
- LiDAR noise, bias, dropouts and 10 Hz hold
- leader pose noise, bias and lag

The wheels have contact `priority`, so tyre friction really sets the floor contact (checked: μ 0.05 vs 1.2 gives 0.04 vs 0.64 m/s).

Turning circle vs R = H / tan θ: +10–12% without randomization (the existing model), up to +20% at 37° with low-friction cars (understeer). Resolve with the real measurement (#3).

The first retrain hit NaN observations at iteration 213; root cause found and fixed:
- **Cause:** MuJoCo Warp has no analytic cylinder-box collider. Tyre-on-floor (the floor was a 14 m box) went through float32 convex CCD.
- **Failure:** for a grazing tyre (≈0.01 mm overlap) CCD sometimes returned a contact 40 m away with a horizontal normal, and the step went NaN.
- **Evidence:** a deterministic one-step repro from the dumped state. CPU MuJoCo is correct on the same state, and mujoco-warp 3.17 still fails (`lab/scratch/nan_*.py`, `logs/nan_car.npz`).
- **Fix:** the wheels now touch one ground plane, which uses the exact plane-cylinder collider; the carpet boxes are visual only (`office_terrain.py`).
- **Verified:** 0 NaN in 4.9M car-steps on the failing corner and across all 32 DR corners (`lab/scratch/dr_stress.py`).

Upstream report: not filed (user's decision, 2026-10-31). The ready repro is in the git-ignored `lab/scratch/repro/`.

**Round 2 (from `docs/research/03` §4.3):**
- **Leader pose:** jumps on relocalization, every 0.25 m or 0.5 s.
- **Leader dropouts:** burst dropouts, plus a new `leader_age` observation (54 → 55 values).
- **LiDAR scan rate:** 10–15 Hz (the MS200 runs at 15 Hz in stock).
- **LiDAR sectors:** min-pooled 10° sectors, computed the same way in sim and on the car.
- **LiDAR geometry:** min range 0.02–0.1 m; plane height 0.14–0.16 m.
- **Own speed:** scale error ±3%.

Steering randomization (29–37°) stays in `mdp.AckermannAction`.

**Done when (round 2):** the round-2 terms are in, and the retrained policy still passes `evaluate_office.py --obs-noise` at 95% or better.

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

**Recommended path (`docs/research/03` §3, G5):**
1. Map the area once and load the map on both cars.
2. Each car localizes in it, with rf2o laser odometry launched into the EKF (G1).
3. Each car sends its pose, speed and steering at 20 Hz.
4. The follower corrects for message age; this needs chrony time sync.

An AprilTag on each car's rear, seen by the follower's HP60C, is the cross-check and fallback. Live map sharing (slam_toolbox decentralized multi-robot, which isn't in Jazzy; or Swarm-SLAM) comes later.

**Done when:** we have a chosen and tested source of relative pose, with its noise level fed back into #1.

## 8. LiDAR fidelity
**The model is still unresolved:** the product page lists both the MS200 and the COIN-D6, and the stock code drives the MS200 or LD19 only (`docs/research/03` §1.1).
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
- low and overhanging obstacles that the LiDAR plane misses, together with ~12 depth-camera rays in the observation (`docs/research/03` §2.2, G7)
- doors and open rooms
- hallway widths varied around measured real hallways
- moving obstacles, such as people (needed for #17 and #18)
- glass and dark surfaces that a LiDAR misses

## 12. Showcase video upkeep
`lab/scripts/make_showcase.py` rebuilds everything from a run directory: shots, segments, master, and an SSIM-checked delivery encode with aq-mode 3. Re-run it after #1 and #3 to show the randomized, measured car. Its stage captions are written for v3 (checkpoints 0 / 50 / 100 / final), so check them against new footage.

---

## End goal: autonomous office robots (`project-plan.md` §13)

## 13. Face recognition
**Problem:** the Advanced kit ships face *detection* (MediaPipe) but not *recognition*: no code, no docs, no product-page claim (checked 2026-10-31).

**Do:**
- Choose a face-embedding model. Check its license, and its accuracy on our own enrolled people.
- Build an enrolment tool and a local store. Only people who agreed are enrolled.
- Pipeline: car detection → crop → matching on the GB10 (or on the Pi, if profiling allows).

**Done when:** in office lighting, known people are recognized and unknown people are reported as unknown. Accuracy, false-match rate and latency are measured.

## 14. Local brain on the GB10
**Do:**
- Serve a local LLM/VLM on the GB10 behind an OpenAI-compatible API. Hiwonder's `large_models` client then works unchanged by setting `base_url`.
- Design our mission layer: tasks, behaviours, and the car-side interface (named goals, speech, perception events).
- Claude + MCP is for development only (plan §4).

**Done when:** a car takes a spoken or typed task, runs it through the local model with no cloud, and reports back.

## 15. Voice and personality
**Do:**
- Local speech-to-text and text-to-speech on the WonderEcho Pro, replacing the default cloud ASR/TTS.
- A library of funny greetings by name.
- The "hungry" lines, for the user only.

## 16. Battery → go home
**Do:**
- Measure the discharge curve: voltage vs time while driving, read from `/ros_robot_controller/battery`.
- Set the "hungry" threshold with enough reserve to reach home from the farthest point on the map.
- Define home (the user's office) as a named pose on the shared map.
- Return home; recognize the user; ask for food.

**Depends on:** #3 (real cars), #7 (map and localization), #13.

## 17. Find a person and deliver a message
**Do:**
- Search over the shared map, splitting rooms between the two cars (#5).
- Recognize the person (#13), approach to a polite distance, speak the message (#15), and report back.

**Sim:** add people (moving obstacles, #11) so the driving policy learns to keep a safe distance from humans.

## 18. Greet known people
**Do:** on a recognized face that hasn't been greeted recently, say a funny greeting (#15) and maybe make a small motion. Never block the person's path; cool down per person.

## 19. 3D look of the car in the sim (optional)
Hiwonder's URDF has STL meshes of the A1 (`docs/vendor/hiwonder/code/MentorPi-A1/simulations/mentorpi_description/meshes/`). Their code has no license, so the meshes can't be committed.
- **Option A:** load them, visual-only, from the git-ignored clone, falling back to our boxes when it's absent.
- **Option B:** ask Hiwonder for permission to include them.

Physics stays on our measured shapes.
