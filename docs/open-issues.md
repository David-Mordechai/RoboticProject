# Open issues

Next steps, in priority order (updated 2026-10-31). The end goal is fully autonomous office robots (`project-plan.md` §1.3, §13); #13–#18 cover it. Each has a goal, its current state and what "done" means. Follow the CLAUDE.md hard rule: look up the spec, cite it, never assume.

**State at hand-off:**
- Branch `mentorpi-virtual-lab`, commit `7c7c561`.
- `Robotlab-Office-MentorPiA1` is solved in sim. Run `logs/rsl_rl/office/2026-10-30_13-19-00`, `model_200.pt`, completed 100% of 512 one-minute evaluation drives with 0 crashes (`lab/scripts/evaluate_office.py`).

---

## 00000000000. ALL OPEN ISSUES, consolidated 2026-11-19 (START HERE)

Everything still open, found up to the end of 2026-11-16. Day plan: `docs/plan-2026-11-17.md`. Evidence and numbers:
`docs/progress-live.md` (bottom = latest) and `docs/lessons-learned.md`. Older sections below keep their history.

### A. Phase A / Gate A (car #1, stock Nav2) — current work
| # | Issue | State | Next / done when |
|---|---|---|---|
| A1 | Gate A not passed: 5 clean trials per leg needed (RESULT ok, 0 recoveries, 0 guard blocks, no contact) | clean so far: C1 2, C2 1, C3 2; no contact since the 2026-11-16 fixes | `gate_a_loop.sh` until 5/5/5; Gate A report vs v6p |
| A2 | C1 starts with a 3-point turn among boxes/cables (C3 parks facing the charger) -> "Failed to make progress", 1 recovery (g31_c1) | cause found | move C3's end to a roomy spot facing the door, check the path offline on the map |
| A3 | Depth-camera phantoms at 0.28-0.33 m (camera range just above the HP60C 0.2 m minimum), only while driving -> "Starting point in lethal space", 4-8 recoveries per trip | fixed in `depth_scan.py` (SCAN_DEPTH_MIN 0.28 m + 2-frame near confirmation), tests pass, NOT yet confirmed on the floor | one attended round: no "lethal" lines; review `near phantom dropped` log |
| A4 | New blind zone from A3: a NEW obstacle below the LiDAR plane appearing within ~0.22 m of the bumper is not seen | accepted trade-off (design estimate) | watch in trials; real fix = bumper / low ToF sensor (see C1) |
| A5 | Nav2 crash "client will not receive response" (rmw_response.cpp:154, Humble bug, ros2/rmw_fastrtps#751) when a new client sends its first goal | worked around: 3 s discovery wait in `nav_trip.py` (13/13 ok) | **user OK needed**: update ROS Humble packages in the car container (real fix) |
| A6 | After a Wi-Fi switch under a running Nav2, a new rclpy client never sees the action server (CLI still does) | workaround: restart Nav2 (`nav_a_ready.sh`) after any Wi-Fi change | automate: restart Nav2 when `wifi_return` switches networks |
| A7 | AMCL drifted ~1 m / 112 deg while the car stood on the charger for ~2 h | workaround: always `nav_a_ready.sh` / scan match right before a trial | find why (scan gaps during the Wi-Fi flap?) |
| A8 | C3 ends 0.31-0.36 m from goal (limit 0.4) and crawls the last metre | open | check goal-approach params after A2 |
| A9 | `nav_trip.py` 17-20 % of a Pi core (BasicNavigator polling, not /tf) | accepted | only if CPU becomes the limit |
| A10 | CPU spikes at recorder start (load 8.5, depth_scan 6.9 Hz for a moment) | open | profile `rec_run.sh` start; stagger recorder start |
| A11 | Parking at the charger (`home_legs.sh` waypoint) never camera-checked | open | camera-check the park waypoint before installing |
| A12 | Steering trim: +11 applied; trim test suggested ~+17 (steering error -0.57 deg) | open, not approved | measure again, then decide |
| A13 | Planner turning radius 0.4 m vs measured steering 28 deg L / 24 deg R (right side tighter limit) | open | recompute R = wheelbase / tan(24 deg) with the measured wheelbase; bench re-check |
| A14 | Odometry is commanded speed + gyro (no encoders): error grows along the hall | open (research deep-2026-11-09 sec. 4) | EKF with IMU yaw, gyro bias removed |
| A15 | People in the hall: a person in front -> 120 s timeout (att_c2) | expected behaviour; no contact | decide waiting / re-route policy for people |

### B. Network / operations
| # | Issue | State | Next |
|---|---|---|---|
| B1 | Phone hotspot drops (twice on 2026-11-16); hotspot subnet changes | car rejoins by itself (`wifi-return.timer`), PC finder `find_cars.ps1` | **user**: small travel router (~$30-50) with fixed IPs per car |
| B2 | `pkill`/`pgrep -f` over ssh matches the ssh shell's own command line (killed my shell 3 times) | lesson recorded | make `nav_a_stop.sh` / `stop_trial.sh` skip their caller's process tree |
| B3 | Car #2: on old setup (AndroidAP 10.71.88.121, ROS domain 2), trim 0, no hostname/priority/wifi-return | open | same network setup as car #1 (user OK per change) |
| B4 | Lots of work was uncommitted until 2026-11-19 | this commit | commit after each working step (with user approval) |

### C. Hardware / sensing gaps (from research, not fixed)
| # | Issue | Next |
|---|---|---|
| C1 | Chair bases below the LiDAR plane; depth camera sees no office floor and nothing in A4's zone | bumper switch or low ToF sensor (decision + purchase: user) |
| C2 | Battery: no measured discharge curve; "hungry" threshold unknown (research/01: charge below 6.4 V) | log voltage over full Gate A sessions -> curve |
| C3 | Cables on the floor (1-2 cm) are below the depth cut (FLOOR_H 0.03 m) | accepted; keep the route clear |

### D. AI training path (after Gate A)
| # | Issue | State | Next / done when |
|---|---|---|---|
| D1 | Learned student failed on the floor 2026-11-09: it ignores its sensors (sim routes always avoid obstacles = shortcut) | diagnosed | Phase B gates: sensor-ablation probe, car-recording replay, car-realistic sim exam |
| D2 | Phase B learned local driver: behaviour cloning from the Nav2 controller in the office twin + DAgger | not started (needs Gate A expert) | beats Phase A on the same floor courses |
| D3 | Office twin lacks depth-only obstacles (chair bases, door stopper) | open | add from car recordings |
| D4 | Teacher map option `ROBOTLAB_TEACHER_MAP=built` (copyable teacher) | not tested | test in sim exam |
| D5 | Team skills T1-T3 (formation follower, people from detector, switching), then MAPPO | sim T1 exists; not on cars | needs car #2 (B3) + leader pose source (#7 below) |

### E. End goal (project-plan.md sec. 13) — not started
Face recognition (#13), local brain on the GB10 (#14), voice (#15), battery -> go home (#16, needs C2), find a person and
deliver a message (#17), greet (#18); building scale: mesh network, relay positioning, shared map (project-plan sec. 9,
radio hardware decision open, sec. 10.1).

---

## 0000000000. 2026-11-09 PIVOT: Phase A = classical Nav2 for a car (research `docs/research/deep-2026-11-09/`)

**State (2026-11-09 night):** the learned student failed on the floor (ignores its sensors). Phase A Nav2 drove car #1 on
11-09: C1 charger -> door 17.8 s; C2 / C3 slow, reversing, weaving; parking failed. Causes measured from the bags and
fixed + tested on the Nav2 bench (`car/nav/bench/`, ASUS): full story and numbers in `docs/report-2026-11-09.md`.

**Next, in order (needs car #1 + the user):**
1. Measure the real steering limit: `car/measure/steer_limit.py` (29 deg = the known weak spot: doorway on the way back).
2. `pwsh car/nav/install_phase_a.ps1`, then `~/nav_a_ready.sh` (Nav2 up + scan-to-map fit >= 80 % + AMCL verified).
3. Depth floor-fit check standing still (`depth_scan` log "floor plane yes", `nav/dsnow.py` no phantom hits).
4. Gate A: 5 trials each of C1 (`nav_a_c1.sh`), C2 (`nav_a_trial.sh ... 2.95 -7.0 -90`), C3 + park (`nav_a_home.sh`);
   success, time, back-ups, flips, contacts vs v6p (63-88 s, 13-32 back-ups).
5. Then Phase B (learned local driver cloned from this Nav2 driver, inside the stack), per the research synthesis.

**Done when:** Gate A passed (beats v6p on success, no contacts) -> Phase A is the baseline / expert.

---

## 000000000. 2026-11-08 PIVOT: privileged teacher + imitation student (DAgger) instead of PPO for goto

**Why:**
- goto v7 / v7b / v7c (PPO + penalties / CAPS / weave penalty) never moved the steering wobble on straights (9.3-10.8 deg) and each broke route following (runs lost up to 35%).
- Root cause in the literature: a clipped-Gaussian PPO policy piles up at the action bounds and steers bang-bang (arXiv 2608.24488). On the car the ONNX steering output was +-4..10.

**Teacher** (`lab/src/robotlab/tasks/teacher.py`): sees the true map, pose and people.
- Route path, shifted sideways around clutter / people, with hysteresis.
- Pure pursuit with Nav2 RPP's look-ahead (1.5 s x v, 0.6-0.9 m).
- DWA-style candidate steering checked against the drive shield's corridor (`_shield_arc`).
- Multi-point turn; minimum 0.6 s reverses; waits for a person ahead.
- Exam: `tmp_goto_check.py teacher`. Teacher v8 passes 11/16 goals (lost 0.2%, U-turns 91%, steering reversals 0.35/s).
- Still failing:
  - person contacts 1.0% (on the line);
  - chair hits 1.5%;
  - reverse blips 2.3/min;
  - wall-start held 1.16 s;
  - steering std 5.4 deg. The trace shows this is smooth settling after turns, not weaving; the std goal measures it too.
- Bugs found on the way:
  - the shield's `approaching()` keeps state, so calling it for look-ahead corrupted the drive's own stop; `pure=` flag added, the teacher uses the corridor test only;
  - free space was first measured on the route line instead of the car's arc.

**Student** (`lab/scripts/train_student.py`): DAgger (Ross et al. 2011), Learning-by-Cheating style (Chen et al. 2019).
- Setup: 8192 cars. 16384 gave the same samples/s; the teacher's look-ahead saturates the GPU at ~24k samples/s.
- **student_v1** (MSE, goto obs):
  - beats the old PPO model on lost (1.3%) and contacts by round 5;
  - plateaus at |student - teacher| 0.19;
  - hesitates: U-turns 74-78%, wall starts held 2-3.6 s, reversals 2.4-3.7/s.
  - Cause: the teacher's hidden modes (multi-point turn, reverse hold) give forward and reverse labels in the same state, and MSE averages them to ~0 speed.
- **student_v2:**
  - L1 loss (the median picks the majority move);
  - + `last_drive` input (the executed command), task `Robotlab-GotoStudent-MentorPiA1`, car node layout `GOTO_LAST_LAYOUT`.
- Next if needed: a multimodal head (diffusion policy, Chi et al. 2023) or forward / stop / reverse classification.

## 00000000. FLOOR RUN 2026-11-08: goto17 (model v6p 8750)

**Result:** goto17 (`go17.sh` = go16 with `goto_v6p_8750.onnx`, the updated `explore_policy.py`, `--ref-map goto6_map_after.npz`) did the whole trip charger -> office door -> hall end -> U-turn -> back -> charger in **63.4 s**, with **no freeze**. The longest stop was 0.5 s. Total stopped 5.8 s, reversing 13.9 s, mean forward speed 0.63 m/s, 14 back-ups. The saved-map alignment matched 93% (rotate +3 deg, shift 5 cm). Logs: `lab/logs/car_runs/goto17/` (run.log, goto17.npz, cam/).

**Office door, way out: passed** with one 0.45 s stop (depth 0.34 m) and back-up #2. Back-up #1 (0.35 s stop) came earlier, near the charger. That is 1 of the 3 runs needed for "3 runs in a row with no door stop".

**Where the time went (camera checked):**
| Part | Time | Stopped / reversing | Cause seen on the camera |
|---|---|---|---|
| Charger -> door | 17.0 s | 1.7 / 1.9 s | 2 short stops, back-ups #1-#2 |
| Hall, way out | 9.4 s | 0.8 / 0.1 s | clean |
| Hall end U-turn + hall back | 20.0 s | 1.2 / 6.7 s | **back-ups #4-#6 "0.71 m off the route"**: the driving-away trigger fired DURING the U-turn, which swings the car wide on purpose (false trigger). At (2.6, -4.15) **people standing in the hall** (frame cam/0048.9) -> 2 stops + back-ups #7-#8 (correct). At (2.8, -3.2) **a person standing in the doorway** (cam/0054.5) -> stalled, back-up #9 (correct) |
| Door -> charger | 16.3 s | 2.1 / 5.3 s | **office chair with its legs in the path** at (1.1, -0.85) (cam/0062.4, 0065.8): about 10 s of stop/back-up #10-#14 before getting around it |

**Video + data review of goto17 (2026-11-08, user's report: wobble on straights, bumped the chair wheel, reverses for no reason).** Frame sheets `lab/logs/car_runs/goto17/sheet1-4.jpg` (camera clock = run clock + 11.3 s, matched by motion).
- **A. Wobble on straights. Cause: the model steers bang-bang.** On the straight hall (t 18.7-24.1) the raw ONNX steering output is +-4...+-10 (the valid range is +-1) and flips sign every ~0.6 s. The car's steering-command rate limit (0.1 per step) turns that into a +-15 deg triangle wave: executed steer std 10 deg, 1.7-2.8 flips/s, yaw-rate std 26 deg/s. goto16 (v6d) did the same (std 7.6-8.9 deg). It is NOT input noise: smoothing or freezing any obs group (route, lidar, depth, maps, motion) leaves the output std at ~27 deg (`ONNX replay`, script in session scratchpad wob.py). The sim's steer_rate penalty (-20, on the executed command) doesn't stop it, and no sim goal measures it.
- **B. Reverses for no reason, two kinds:**
  - The model's own short reverses: 22 of them, -0.03...-0.30 m/s for 0.05-0.5 s, 1-9 cm, mostly with ~1 m free ahead. The car is back on the same pose and heading 0.1-0.4 s later. Same bang-bang cause, on the speed output.
  - Safety-layer back-ups: 14 of 0.8 s / ~0.2 m. 10 of them end with the car back on the same pose and heading ~1 s later, so they bought nothing:
    - #4-6: the false off-route trigger during the U-turn;
    - #7-9: people standing in the hall and the doorway (waiting would have been better);
    - #11-14: at the chair.
- **C. Chair wheel bump (t ~52.8-55, near the charger). Cause: a sensor blind spot.** The chair base and casters (~0.05-0.12 m high) are below the LiDAR plane (~0.16 m, research/03 sec. 2), so the LiDAR saw only the chair's centre column (0.42-0.54 m). The depth camera saw the legs (0.22-0.34 m), but at full lock the front corner sweeps outside its 73.8 deg cone and inside its 0.2 m blind zone (frame cam/0066.3: caster at the bottom-right edge). The model drove at it at 0.7 m/s and full steer (t 52.8). The sim has no low obstacles: chairs are solid boxes the LiDAR sees (office_terrain.py).
- **D. False back-ups in the U-turn** (above, #4-6): the driving-away trigger fires while the car swings wide on purpose.

**Proposed fixes, all in one training run (sim and car changed together):**
1. Penalise raw actions outside +-1 (both outputs) and executed speed sign flips. Add sim goals: steering std on straight route segments <= 4 deg, model-chosen reverses <= 1/min.
2. Driving-away back-up off on the U-turn segment (sim `moving_away` + car AWAY_*).
3. Back-up escalation: if the car comes back to the same pose and is blocked again, the next back-up is longer and uses the other steering side. A stop by a standing person waits first (no back-up for ~2 s).
4. Remember depth obstacle points for ~3 s in the odom frame for the stop corridor, so a chair leg that leaves the camera cone while turning still stops the car. Add low chair bases (5 legs + casters, 0.10 m high, visible only to depth) to the office sim.

**Coded 2026-11-08 (goto v7, training on the ASUS from v6p 8800, phases_x ONLY_PHASE=goto_v7, run name goto_v7):**
- Sim:
  - `goto_env_cfg`: steer_flip -6, action_bound -0.1, policy_reverse -2; TURN_ZONE 0.8 m (no driving-away back-up near the U-turn corners); CHAIR_PROB 0.5; depth map on; depth fan 48 columns.
  - `office_terrain`: CHAIR_* chair bases. Five legs below the LiDAR plane, a column the LiDAR sees; sizes are ESTIMATES.
  - `explore_env_cfg`: update_depth_map / DEPTH_MAP_S 8 s / 1.5 m.
  - `mdp`: ESCALATE_* back-up escalation, `backing` flag.
  - `camera`: DepthFanPatternCfg.sub.
- Car:
  - `explore_obs.DepthLayer` in map_fine / map_coarse;
  - `explore_policy`: TURN_ZONE turn-back corners (on the goto17 route: (2.4, -7.4), (2.8, -7.4); false back-ups #4-6 were 0.18-0.38 m from them), ESCALATE_*.
  - Both are switched on by the model's ONNX metadata (`depth_map`, `unstick_escalate`), so older models drive as before.
- Checker `tmp_goto_check.py`, new goals:
  - steering reversals per second on straight stretches <= 0.5;
  - steering std on straights <= 4 deg;
  - policy reverses <= 1/min;
  - chair-base hits <= 1%.
- **Baseline: v6p 8750 in the new sim (one check, 2048 cars).**
  - **Matches the floor:** steering std on straights 10.8 deg (floor 10), reversals 5.2/s, policy reverses 9.2/min.
  - Untrained for the chairs and depth map: wall/obstacle contacts 7.2% (was 0.2%), lost 7.2%, turn-backs 87%, blocked held 1.36 s.
- **Not done:** wait for a standing person instead of backing up. The car can't tell a person from a chair; person detection (YOLO) isn't wired into the car node.
- **2026-11-08 slip:** the first start of the phase runner replayed old phases (no done markers in phases_x) and ran turnaround_v2b for ~1.5 min. It was stopped with the STOP file. Left behind: run dir `turnaround/2026-11-08_10-47-21_turnaround_v2b` (model_900.pt) and a re-export of turnaround_v2 749. Restarted with ONLY_PHASE=goto_v7.

**Next:**
1. Driving-away trigger: don't fire while the route is turning back (U-turn segment), or raise its distance near the turn point. Same rule in the sim (goto v6i/v6k), so the sim and the car stay matched.
2. Two more runs with the hall clear of people and the chair moved, to count door passes (need 3 in a row).
3. The chair case (legs + wheels close in front) is worth a sim scenario only if it keeps happening; for now it is a clutter case the car solved by itself.

## 0000000. STATE AT END OF 2026-11-05 (floor route trip + training)

**Floor result:** goto16 (v6d 6750 + saved-map route + back-and-forth rule) did the whole trip, charger → office door →
5 m into the hall → U-turn → back → charger, in **42.2 s** (0.61 m/s mean, 3.0 s stopped, 1.6 s reversing). Logs:
`lab/logs/car_runs/goto3..16*` (npz, cam, run logs); every run's analysis is in this session's notes below.

**Open, in priority order:**
1. **Battery of car #1 is empty** (user, 2026-11-05 evening). Charge before the next floor test.
2. **Stops at the office door must not happen** (user: "it should not happen").
   - What we saw:
     - goto11 and goto14: frozen 15+ s with the door frame 0.38-0.45 m at 45-55 deg right. The model asked full speed + full right turn, cutting the corner.
     - goto16: one stop plus one back-up at the door, then through.
   - Causes found:
     - The policy cuts the corner into the door.
     - It never chose reverse by itself; the stop_held input is ignored (`lab/scripts/tmp_stuck_probe.py`).
   - In training: corner starts (v6d/v6e) and harder corners (v6e).
   - Done when: 3 floor runs in a row pass the door with no stop.
3. **Repeat the floor trip 2-3 times**, to see whether 42 s is repeatable. Use the newest exported model.
4. **Back-and-forth rule on the car (scripted):** `car/policy/explore_policy.py` UNSTICK_*.
   - What it does: held 0.5 s → back up 0.8 s with steering opposite to the model's, then the model drives again; it repeats until through.
   - The training should make it unnecessary. Count the "back up #" lines per run as the measure.
   - Evening 2026-11-05 (PC copy only, **copy `car/policy/explore_policy.py` to the car before the next drive**):
     - the log said "stuck 0.0 s" (fixed);
     - two more triggers, as in the sim training (goto v6i/v6k):
       - **stalled:** moved < 0.25 m in 2 s;
       - **driving away:** > 0.7 m off the route and moving away at > 0.1 m/s.
     - Watch for false back-ups when SLAM corrects the pose by a jump (route distance jumps).
5. **Route on a saved map:** `--ref-map goto6_map_after.npz`.
   - How: at the start the car matches its first SLAM map to the saved map (FFT correlation, ±25 deg, ±1 m) and moves the route onto it.
   - Before this: run-to-run start-heading errors of up to 11 deg put a fixed route 0.5-3.7 m beside the hall (goto3-13).
   - Route (saved-map frame): `1.2 -0.3 2.0 -0.6 2.45 -1.1 2.45 -1.9 2.45 -3.0 2.4 -6.5 2.4 -7.4 2.8 -7.4 2.8 -6.5 2.6 -3.0 2.55 -1.9 2.55 -1.1 2.0 -0.6 1.2 -0.3 0 0`. Drive scripts are in the session scratchpad (`go16.sh`).
6. **Training (ASUS), goto v5 → v6e, all from goto v4 4850.**
   - **v5e** (way back 0.4 m to one side; fixed the left/right dithering): mid-hall turn-backs went 3.6% → 79%.
   - **v6** blocked starts (nose to the wall), stop penalty −10.
   - **v6c** `stop_held` input.
   - **v6d** corner + boxed-in starts.
   - **v6e** halls 1.0-1.8 m, so a turn needs several back-and-forth moves (user: "keep going back and forward until it succeeds"), and harder corners.
   - Goal check: `lab/scripts/tmp_goto_check.py` (mid-hall turn-backs, blocked / corner starts and their held time).
7. **Final model of the night: goto v6p 8750.**
   - All 12 sim goals met on the mean of 3 × 2048-car checks; full table in `docs/report-2026-11-05.md` §3.
   - Next: copy `car/policy/explore_policy.py` and `lab/logs/rsl_rl/goto/v6p/model_8750.onnx` to car #1, then do floor runs 2-3 (item 3).
8. **Not done today:** car #2 factory restore, stock-Nav2 investigation (plan 000000 below, still open).

---

## 000000. PLAN FOR 2026-11-05 (user's direction, end of 2026-11-04)

1. **Car #2 → factory state, then find out why the stock software does not drive.**
   - List every change we made on car #2 and undo it (Wi-Fi STA config, lowload / camfps patches, start_mapstack, ORT, our scripts), or re-flash the Hiwonder image (SD card swap = a user step).
   - Follow Hiwonder's documented mapping + navigation procedure exactly (docs/vendor, `research/02` §2: `slam.launch.py`, save map, `navigation.launch.py map:=...`) and record where it fails.
2. **Car #1 stays the training car.**
3. **Training only for NEW skills worth something.** No more re-learning what Nav2 or stock does.
   - Stopped runs:
     - goto_v4 4350: 2.35 goals/min, 25% of runs lost, 1.4% at-fault contacts.
     - gotosub_v1 4300: 0.65 goals/min, 90% of runs lost.
     - Kept in GB10 logs. Not to be continued unless stock Nav2 proves unusable.
   - **Proposal for the user** (`project-plan.md` §13.4, written 2026-10-31 and drifted from on 11-03/04):
     - Stock driving first. Train only the gap stock cannot do: **team driving**.
     - **T1 (worth it):** formation follower. The leader runs stock Nav2; the follower is our policy holding a slot (column, diagonal, side-by-side) among walking people. The task exists (`Robotlab-Formation-MentorPiA1`, commit 174bc03); its sim leader must be calibrated to how stock Nav2 actually drives once (1) works.
     - **T2:** people from the stock YOLO + depth detections, with real noise and delay.
     - **T3:** switching formation on command.
     - **Explore (v7g 3600, done):** keep as the frontier/mapping skill only if stock `explore_lite`-style exploration is not enough. Decide with the user.
   - See `docs/car2-factory-restore.md` for the car #2 change list and the stock-test questions.

**Nav2 on car #1 today (logs: `lab/logs/car_runs/nav2_trips/`):**
- **Map:** hall4's SLAM map saved as `slam/maps/hallway_01` (from `map_after.npz`).
- **Stock `navigation.launch.py`:**
  - Run on top of the autostart, it doubled the base nodes; it must run alone.
  - TEB: "trajectory is not feasible", then Spin recoveries (a car cannot spin). The car stopped 2.2 m before the door.
- **Our `car/nav/nav_map.launch.py`** (stock base + map_server + AMCL, our Smac Hybrid-A* + MPPI Ackermann):
  - `component_container_isolated` used 154% CPU at idle (load 18). Use `component_container_mt`.
  - The `/initialpose` quaternion must be normalized exactly, or AMCL rejects it.
  - MPPI drove 0.75–1.3 m smoothly, then stopped with "Starting point in lethal space" next to furniture or people.
  - After a BackUp it stalled at ~0 m/s with only the steering moving.
- **Not yet found:** why stock fails. Was the factory config ever meant for the Acker version, and is Hiwonder's own tutorial for it different?

---

## 00000. START HERE (end of 2026-11-04): explore v7g done; goto (route following) training with doorways

**Explore v7 → v7g (GB10, all uncommitted):**
- Cause of the floor pauses: 85% of the idle time with a clear path was the shield's approach stop and its 5 s hold. It fired 20% of the time with no person within 2 m (`lab/scripts/tmp_idle_cause.py`).
- v7 fix: approach stop only within `SHIELD_APPROACH_HALF_W` 0.35 m of the car's axis, and no hold. The car node (`car/policy/explore_policy.py`) has the same change.
- Person contacts after the hold was removed:
  - 98–100% were the bumper on a standing person's feet. The feet are below the LiDAR plane and up to 0.19 m ahead of the legs (`tmp_contact_geom.py`).
  - A larger contact penalty (v7b) did not help. CaT terminations (v7c; Chane-Sane et al. 2024) did not help on their own.
  - Root cause: the action std had collapsed to 0.0016 for steering, because `steer_rate` was charged on the sampled actions.
  - v7d moved `steer_rate` to the executed steering command, reset the std (`lab/scripts/reset_std.py`) and set entropy to 0.001 (v7e). With that, contacts fell from 10% to about 5%.
  - v7f: CaT margin plus 0.5 s × closing speed. v7g: 25% of people stand still (`people.STAND_PROB`).
- At-fault contacts are now counted as "the car closed the gap faster than the person over the last 0.5 s".
  - v7g 3500: 81% of contacts were walkers still walking 0.5 s before contact (walkers never yield to an oncoming car by design).
  - Old rule ("car moving toward the person") at v7g 3600: 4.7%. The user has not yet confirmed the new definition.
- v7g 3600 passed all goals: idle with a clear path 1%, speed with people beside 0.95, coverage 0.79, at-fault contacts 0.8%. Exported. The door-freeze replay of v5_drive1 drives through.

**drive_to.py is retired.** In hall5–7 it stopped and boxed itself in every few metres: remembered depth points, a ±25° heading window, and circling turnarounds. The user wants AI, not code.

**Goto task (`Robotlab-Goto-MentorPiA1`, `mentorpi_goto_env_cfg.py`):**
- Explore v7g, plus navigate's route preview (17 inputs appended), and progress / goal / on_route rewards.
- 50% of new goals are behind the car, so the model learns to turn back.
- Warm start from explore 3600:
  - `warm_start.py`, then `tmp_route_stats.py`. The copied normalizer stats for the route columns were wrong and never adapt after explore's millions of samples.
  - Entropy 0.003.
- v3: `LOST_DISTANCE` 2.5 m and `GOAL_DIST` 1.0 m. Turn-back routes had been lost within 2.2 s, and route ends were missed by 0.6 m.
- Check: `lab/scripts/tmp_goto_check.py`. At goto_v3 4050: 1.87 goals/min (goal ≥ 2), 53% of runs lost (goal ≤ 5%). Speed and safety pass.
- Car side: `explore_policy.py --route X1 Y1 ...` (map frame) feeds the same preview. `--memory` is now off by default: remembered depth points stopped goto1 on clear floor.
- Floor goto2 (v3 4000): drove 1.8 m smoothly toward the door, then froze 0.7 m in front of the wall with the doorway about 60° to its right. The sim had no doorways.
- **goto v4 (running from 4050):** `office_terrain.door_prob` 0.35 adds a wall across the hallway with a 0.7–0.9 m gap off to one side, and the route goes through it.

**Next:**
1. When goto v4 meets its goals, export and replay goto2's door moment.
2. Floor trip, same route as goto2: `--route 1.5 -0.6 3.0 -1.1 3.2 -1.8 3.3 -2.7 3.6 -7.6 3.3 -2.7 3.2 -1.8 3.0 -1.1 1.5 -0.6 0 0`, with the map restarted at the charger.
3. The turnaround model is not used with goto.
4. Car #2 still needs today's software.

Recordings on car #1: `~/docker/tmp/runs/{v7g_drive1, hall5, hall6, hall7, goto1, goto2}`. Not yet copied to the PC.

---

## 0000. Retrain plan from the 2026-11-04 floor drives (proposed, not started)

**Issues seen on the floor, and why the sim did not catch them:**

| # | Floor issue (run) | Root cause in the sim |
|---|---|---|
| 1 | Explore froze ~25 s facing a door 1.1 m away, ~0 speed, no person (`v5_drive1`) | Standing still costs almost nothing when no person is near: `forward` is +0.1 only; nothing penalises idling |
| 2 | Turnaround only reverses; stuck when the back is blocked (`v5_drive3`); hit a chair base in reverse (`v5_drive2`) | Turnaround episodes always start facing a dead end with 2.5-4 m of empty corridor behind, so "reverse first" always works; no shield in the turnaround sim, while the car stops it (rear 0.20 m) |
| 3 | 5 s pauses during turnaround (`v5_drive2`) | Car applied the approaching-object hold in turn mode (fixed car-side; the sim turnaround never had it) |
| 4 | Explore stops a lot near people, slow to go on | Approach hold 5 s + `shield_stop` -3 + `speed_near_people`: conservative by design; no reward for going around or on once clear |
| 5 | Person contacts plateau 2.1% (sim), 95% with the car moving toward the person | Not analysed yet for this checkpoint |
| 6 | Hallway trip: jerky, pauses, slow 3-point turn (`hall1-4`) | Hand-written `drive_to.py`, not trained (fixes in code, untested) |

**Step 0 (before training, ~20 min, CPU/GPU):** a replay check from real drives. Feed the recorded car observations
(`v5_drive1.npz` 35-58 s, the door freeze; `v5_drive3.npz`, the blocked turnaround) to a policy and report its commands.
Gate: the new model must command motion (turn away or reverse with a clear rear) where v5 froze. Plus
`tmp_contact_detail.py` on checkpoint 2149 to see what the remaining person contacts look like.

**Phase A: turnaround v2 (warm start from v1 model_450, est. 300-450 iterations, ~40-60 min on the GB10):**
- Scenes: in 50% of episodes clutter behind the car (wall / box / chair base 0.25-0.8 m behind); open rooms, not only
  dead ends; target heading 90-180 deg either way.
- The car's turnaround shield in the sim (front 0.22, rear 0.20 m + 0.3 s x v, band 0.13 m), the same rule as
  `explore_policy.blocked` in turn mode, so the policy learns what the car will actually allow.
- Rewards: time -0.3 -> -0.6 per s (efficiency), switch -0.5 -> -1.0 (fewer legs), a penalty when the shield stops it
  (as explore's `shield_stop`).
- Goal check per checkpoint (new, like `tmp_goal_check.py`): success >= 95% with clutter behind, median time to turn
  <= 8 s, median forward/reverse switches <= 3, contacts <= 1%.

**Phase B: explore v6 (warm start from v5 model_2149, est. 400-600 iterations, ~50-70 min):**
- Idle penalty: speed < 0.05 m/s with no person within 1.5 m (fixes the freeze; the sim's `CAR_STILL` already tells a
  stopped car apart).
- More starts facing a wall or door at 0.5-1.5 m, and in dead ends, so it learns to turn away itself.
- "Go on when clear": a reward for speeding up once the nearest person has moved away (the hold ends), so it doesn't
  linger after people pass.
- Person contacts: act on Step 0's finding (e.g. TTC horizon 2 -> 3 s, or approach range 1.5 -> 2.0 m), only one change.
- Goal check: the current 7 goals + new: idle share with no person near <= 5%; the replay gate from Step 0.

**Phase C (bigger, next): a trained go-to-goal policy to replace `drive_to.py`.** Same sensors + the goal's relative
position; office scenes with doors, hallways, chairs and people; rewards for progress to the goal, smoothness, people
safety; the shield as in explore. That is the real fix for issue 6 (smooth driving and K-turns learned, not
hand-tuned). Estimate: half a day to build and check the task, 2-4 h training.

**Order:** Step 0, then A and B can share one GB10 session (A then B, ~2 h total with checks). Car test after each:
the same recorded runs (`rec_run.sh`): turnaround in the hallway with a chair behind; explore 60 s with a person walking
by. Commit nothing without approval.

## 000. Floor results 2026-11-04 (car #1, both cars on the phone hotspot)

**Hallway trip works** with `car/nav/drive_to.py`, started from the charging spot (SLAM origin, map frame of a fresh
`start_mapstack.sh` there): out `1.5 -0.6 3.0 -1.1 3.2 -1.8 3.3 -2.7 3.6 -7.6`, back `3.3 -2.7 3.2 -1.8 2.0 -0.7 1.0 -0.3`
(run `hall3` + `hall4`): through the door, 5 m down the hallway, 3-point turn, back to the office; waited for people.
Fixes made to `drive_to.py` on the way:
- 3-point turn ends at 20 deg off the target, not 60 (the gap step undid the turn in a hallway).
- Reversing is checked (`REAR_CLEAR` 0.30 m): it had no check and backed into a chair base.
- Depth obstacle memory (`MEM_S` 15 s) for low objects only: depth points with a LiDAR return within 0.10 m are not kept
  (remembered legs of people became ghosts).
- Boxed in front and back: wait up to `WAIT_S` 10 s (people pass) instead of giving up; exit code 1 on failure.

**Explore v5 (checkpoint 1900) on the car:** drives straight and smoothly, but in a 60 s drive it froze ~25 s in front of
a door (policy outputs ~0 speed; no person there). With `--turn-model turnaround_v1.onnx` the turnaround only reverses
and gets stuck when the back is blocked. Car-side fixes in `car/policy/explore_policy.py`: no approaching-object
stop/hold in turnaround mode (rotation looked like an approaching object: 5 s pauses), turnaround rear stop 0.10 -> 0.20
m and band 0.13 m, depth obstacle memory (as above), stop reason logged (`why` in the npz), mixed-length log save fixed,
`self.hist` name clash (scan history vs stuck check) fixed. **Next for explore:** retrain the turnaround with blocked rear
/ forward-first K-turns; the explore freeze in front of walls.

**Car setup fixes:** `~/start_mapstack.sh` (both cars) also kills `third_party_ws` and `shared/perception` processes (3
LiDAR drivers ran at once -> 4 Hz scans, camera "busy"); car #2 moved to the phone hotspot (STA, 10.71.88.121) and ROS
domain 2; `~/estop.sh` kills `drive_to.py`, `explore_policy.py`, `goal.py`. The stock `web_video_server` segfaulted
mid-run (dashboard video gone until the stack restarts).

**Recording every run** (`car/measure/rec_run.sh NAME 'command'` on the car): ROS bag (scan, odom, imu, tf, cmd_vel, map,
depth, route, battery), camera JPEGs at 2/s from the ROS topic (`cam_saver.py`), run log, map before/after. Copied to the
PC in `lab/logs/car_runs/<name>/` (git-ignored): `hall1..4`, `out_of_desk`, `hall1_return` (car carried: invalid).

**GB10:** `gb10_supervise.sh` looped at the end (rsl_rl's last save is `model_<END-1>`); fixed (`-ge END-1`).

## 00. START HERE (2026-11-04): explore v5 is ready for its first car test drive
Stopped on 2026-11-03 at 16:50 at the user's request; the GB10 was shut down. Nothing is committed (ask first).

**Next step:** test-drive explore v5, checkpoint 1900, on car #1. The user switches on the phone hotspot and car #1 and
puts the car on the floor. Then:
1. `netsh wlan connect name=AndroidAP`.
2. `~/start_mapstack.sh` on the car; check the odometry yaw is stable (`~/yawsrc.sh`).
3. Copy `car/policy/explore_policy.py`, `explore_obs.py` and
   `lab/logs/rsl_rl/explore/v5w_1900/explore_v5w_1900.onnx` to `~/docker/tmp/policy`.
4. Dry run, then `python3 explore_policy.py explore_v5w_1900.onnx --seconds 60` (optionally `--turn-model
   turnaround_v1.onnx`). The user walks toward the car to test stopping for people.
5. Analyse the log.

The car node's new code (scan history, steering rate limit, approaching-object stop and hold, rear stop 0.35 m) has
**never run on a car**. Check it in the dry run.

**Explore v5 (the `explore_v5w` run, a warm start from v4f):**
- Input: `car_motion | lidar_sectors | depth | map_fine | map_coarse | lidar_hist` (1284 values). `lidar_hist` is the 4
  scans before the current one, 0.1 s apart (DRL-VO, arXiv 2301.06512).
- Steering command rate limit: 0.1 per step, applied in the sim and in the car node.
- Shield, identical in the sim (`mentorpi_explore_env_cfg.shield` / `approaching`) and the car (`blocked` /
  `approach_stop`):
  - arc obstacle stop (front 0.30 m, rear 0.35 m, + 0.4 s × v);
  - approaching-object stop: within ±60°, closer than 1.5 m, approaching faster than 0.6 m/s, confirmed in two scans in
    a row, with the car's own motion compensated from odometry;
  - 5 s hold toward the direction where the approach was seen (±30°, objects within 0.7 m).
- Rewards: `person_ttc` −20; `shield_stop` −3; `hit_person` −300; `personal_space` −10; steer rate −10.
- Sim people (`people.py` `pass_by`):
  - they don't stop for a car approaching them (faster than 0.15 m/s); half step aside;
  - they walk around a stopped or slow car;
  - 30% are distracted and stop only at 0.55 m.

**Goal check at checkpoint 1900** (`lab/scripts/tmp_goal_check.py`, 1024 cars × 60 s, sensor noise):

| Goal | Result | Target |
|---|---|---|
| Steering change per step | 0.033 | ≤ 0.06 ✓ (v3: 0.152) |
| Wall contacts | 0.1% | ✓ |
| Speed with a person ahead | 0.14 m/s | ✓ |
| Stopped beside people | 23% | ✓ |
| Floor explored in 60 s | 64.1% | ≥ 65%, just short |
| Person contacts | 2.7% | ≤ 1%, short |

The last two have plateaued since checkpoint 1850. The full history is in `lab/logs/rsl_rl/explore/v5w_1900/goals_explore_v5w.log`.

**Files:**
- On the PC: `lab/logs/rsl_rl/explore/v5w_1900/`, with `model_1900.pt`, the `.onnx`, the goal log and a copy of
  `gb10_supervise.sh`.
- On the GB10: run `logs/rsl_rl/explore/2026-11-03_16-30-18_explore_v5w`.

**Continue training on the GB10:** run `rm ~/RoboticProject/runs_state/explore_v5w/STOP; setsid nohup bash
~/gb10_supervise.sh &`.
- It resumes from 1900, runs to END=2150, runs a goal check at every checkpoint, evaluates and exports at the end, and
  logs temperatures.
- It restarts training after a crash, but **not after a reboot**: the `@reboot` crontab line needs the user's OK.

**Learned on 2026-11-03:**
- Sim people that always yield taught the policy to wait for or brush past people. Sim realism matters more than reward
  weights.
- Raising a penalty 5× did nothing while the policy couldn't perceive the motion, so the scan history was needed.
- Most of the gain on person contacts came from the deterministic shield plus training with it, not from reward
  shaping.
- A 10 ms physics step is rejected: braking and tight K-turns differ from 5 ms.
- Warm start (`lab/scripts/warm_start.py`) widens a checkpoint for appended inputs, so there's no need to train from
  scratch.

**Ideas if the last two goals matter:**
- left/right symmetry augmentation (rsl_rl `symmetry`);
- a curriculum on how many people there are and how fast they walk;
- a person-specific hold that tracks the person rather than a direction;
- analyse the remaining contacts with `lab/scripts/tmp_contact_detail.py` and `tmp_precontact.py`.

Clean up later: promote the `lab/scripts/tmp_*.py` checks into tests or proper scripts.

## 0. State at the end of 2026-11-02 and next steps
Nothing from 2026-11-02 after commit `274349e` is committed: ask the user before any commit (standing rule).

**Goal the user set:** the cars explore and map on their own and drive autonomously; no manual driving. Work toward
that with the virtual lab; ask the user only for physical steps (charge, place a car, watch a first run).

**Cars (both checked 2026-11-02; details in the per-car memory notes):**
| | Car #1 | Car #2 |
|---|---|---|
| Network | phone hotspot "AndroidAP" (STA mode), 10.71.88.196; falls back to its own hotspot HW-54007663 if the phone hotspot is off at boot; ROS_DOMAIN_ID 0 | phone hotspot "AndroidAP" (STA mode since 2026-11-04), 10.71.88.121 (AP config backup `~/hiwonder-toolbox/wifi_conf_ap_backup.py`); **ROS_DOMAIN_ID 2** (both cars on one hotspot saw each other's topics on domain 0; revert `~/docker/tmp/domain_backup/revert.sh`). No LAN cable on either car. |
| Motor direction | fixed: stock `ackermann.py` drives both rear motors backward on + (wiring matches the manual: M2 left, M4 right). `[0, vl, 0, -vr]` -> `[0, -vl, 0, vr]`; user confirmed wheels forward + stop | same fix, confirmed |
| Steering | servo on S5, trim -9 saved (floor test: yaw rate 0.018 -> 0.0002 rad/s) | S5 OK; **trim 0, needs `car/measure/trim_drive.py` on the floor** |
| Camera fps fix / low-load (no app nodes, camera 15 fps) | both | both (load ~3.4-5 -> ~1) |
| Drive command watchdog (`car/watchdog`, stops motors 0.5 s after commands stop) | installed, tested 0.49 s | installed |
| ONNX Runtime 1.19.2 (user site) | installed; formation map_v2 runs in 0.06 ms | installed 2026-11-04 |
| Explore v5 stack (2026-11-04) | `~/start_mapstack.sh` (LF; kills stale third_party_ws + perception copies), v5 dry run OK | same files copied (old ones in `~/docker/tmp/stale_backup_2026-11-04`), stack + v5 dry run OK |
Revert scripts for every change are on each car under `~/docker/tmp/*_backup/revert.sh` (camfps, motorfix, trim, lowload, mapprep) and `~/watchdog/uninstall.sh`, `~/wifi_backup/revert.sh` (car #1).

**Measured / learned 2026-11-02:**
- The stock software does NOT explore on its own: Hiwonder's mapping = you drive with the keyboard; navigation = click
  goals in RViz on a saved map (`docs/vendor/hiwonder/manuals/web-7.mapping_lesson.html`, `web-8.navigation_lesson.html`).
- Battery: 2S 2200 mAh LiPo, charge below 6.4 V, ~1 h charge, charge with the car OFF (Hiwonder Charging & Usage
  Guide). Idle drain was high (8.25 -> 7.38 V in 42 min) until the low-load settings; the board alarm (beep) is 6.9 V.
- The LiDAR plane is ~0.16 m up: office chair bases are invisible to it (car #1 hit / got pinned on chairs). Fixed with
  `car/perception/depth_scan.py` (depth camera -> `/depth_scan` for Nav2 + `/depth_sectors` = the sim's 12 sectors).
  The depth camera's measured FOV is 56.6 deg; the sim used the product page's 73.8 deg (the explore task uses 56.6).
- Stock Nav2 on the A1: TEB `min_turning_radius` 1.0 m made it reverse in loops; even with 0.4 m TEB would not follow a
  valid Smac Hybrid-A* path. NavFn plans paths a car cannot drive. Nav2 refuses goals outside the map bounds.
- Car #2's `install/bringup/share/.../bringup.launch.py` is a real copy, not a link to src: patch both (lowload does).
- Never flood `cmd_vel` (thousands/s made the board miss the stop). `pkill -f` inside `zsh -c` kills its own shell.
- PC Wi-Fi to the phone hotspot drops every few minutes: run drive scripts on the car, not over a live ssh session.

**What worked on the floor (car #1):** a 1 m Nav2 goal; `car/nav/drive_to.py` (SLAM pose + LiDAR + depth, follow the gap,
stall detection) drove from the room through the door into the hallway (3 waypoints, 23 s). **What failed:** turning
around in the narrow hallway with `drive_to.py` (heading did not change during the 3-point-turn mode; unexplained:
steering not applied, or SLAM heading lost in the featureless hallway).
Saved map `slam/maps/office_01` (car #1, copy in `car/maps/`) is distorted (room duplicated/rotated: the car was carried
while SLAM ran): a record only, remap before using it for localization.

**Next steps, in order:**
1. **Nav2 controller for a car:** replace TEB with MPPI (`nav2_mppi_controller`, installed on the cars), Ackermann motion
   model, min_turning_r 0.4 m, up to 0.25 m/s, LiDAR + depth costmaps (`car/nav/make_params.py`, `nav_slam.launch.py`
   already set Smac Hybrid-A* 0.4 m, robot_radius 0.14, no back-up/spin BT `navigate_ackermann.xml`, depth source).
   Test the config in the virtual lab first, then with the car standing still (`plan_check.py`), then goals.
2. **Frontier exploration:** `explore_lite` from robo-friends/m-explore-ros2 (BSD, Humble from source; its map merge is
   NOT slam_toolbox-compatible yet, issue #10: write our own merge for two cars). One clean mapping run per area, never
   carrying the car while SLAM runs.
3. **Exploration policy from the virtual lab:** task `Robotlab-Explore-MentorPiA1` (built 2026-11-02 by a subagent;
   trained on the GB10, 1500 iterations x 8192 envs, run `explore/2026-11-02_16-30-23_explore_v1`; reward plateaued
   by ~iteration 300). Evaluation of `model_1499.pt` (1000 runs x 90 s, sim): floor seen 47.7% at 30 s, 70.0% at 60 s,
   78.5% at 90 s (median 82.9%, p10 55.7%); with sensor noise 77.7%; wall contact 3.6-3.8%, person contact 1.6-1.8%,
   94.4-94.8% finish 90 s without contact. ONNX exported (`model_1499.onnx`, input `obs` [1, 1140], parity 1.6e-6,
   0.034 ms/step on the GB10 CPU). Checkpoint, ONNX, params, tfevents and train log copied to
   `lab/logs/rsl_rl/explore/2026-11-02_16-30-23_explore_v1/` on this PC (git-ignored).
   **Blockers before the car:** (a) the policy drives ~0.49 m/s (V_MAX 0.7) but the stock driver clips /cmd_vel to
   0.2 m/s: retrain with V_MAX 0.2 or raise the driver cap after measuring; (b) ~5% contact runs: keep the watchdog +
   LiDAR stop layer; (c) the car must rebuild the exact observation layout: car_motion 4 | lidar_sectors 36 | depth 12
   (56.6 deg FOV) | map_fine 800 (2x20x20, 0.15 m, x -0.9..2.1, y -1.5..1.5) | map_coarse 288 (2x12x12, 0.6 m) from
   slam_toolbox `/map` at the SLAM pose (unknown = 0 on both planes, outside map = occupied); the exact definitions are in
   `mentorpi_explore_env_cfg.py` (built_map_fine/built_map_coarse) and in the ONNX file's metadata.
   **2026-11-03:** (a) retraining as `explore_v2_slow` (V_MAX 0.2, reverse 0.15, `AckermannActionCfg.v_max`), auto-stop
   at reward plateau, then eval + export (`~/gb10_plateau_v2.sh`, `~/gb10_finish_v2.sh`; export now writes v_max /
   v_reverse metadata). (c) car-side node written: `car/policy/explore_policy.py` + `explore_obs.py`
   (`lab/tests/test_car_explore_obs.py`: map crops, pooling, LiDAR sectors, action mapping match the sim, 4/4 pass).
   **v2 result** (`model_550`, auto-stopped at plateau, iteration ~550; 1000 runs x 90 s): coverage 20% / 28% / 34%
   at 30 / 60 / 90 s (p90 52%), same with noise; wall contact 0.1-0.2%, person contact 0%, 99.8% full runs; mean
   forward speed 0.12 m/s (60% of the 0.2 cap). Lower coverage than v1 because it drives ~4x less distance in 90 s;
   ~25x fewer contacts. ONNX `model_550.onnx` has v_max 0.2 / v_reverse 0.15 metadata; copied to
   `lab/logs/rsl_rl/explore/2026-11-03_07-51-47_explore_v2_slow/`. Possible next: longer episodes (180 s) or a
   speed reward to use the full 0.2 m/s.
   **Car #1 tests 2026-11-03:** v2 crawled and dithered; v1 drove but steered in place. Root cause: stale stock
   `imu_filter` + `robot_state_publisher` left running next to our stack -> two /imu publishers -> /odom yaw jumped
   84 deg up to 3.5x/s with the car still (probably also yesterday's hallway turn-around failure). Fixed: start script
   kills every /opt/ros/humble/lib + ros2_ws process before launching. After the fix v1 drove a smooth 2.3 m path but
   pushed toward a chair star base seen only by depth (the arc obstacle stop held it): the sim's chairs are boxes,
   so the policy never learned depth-only obstacles -> add chair bases below the LiDAR plane (explore v4).
   Motor dead band measured: 0.02 m/s -> 1.1 of 3 cm, 0.03 m/s and up follow within ~10%.
   Next skill (user, 2026-11-03): reverse / U-turn / multi-point turn in narrow places, quick and precise ->
   task `Robotlab-Turnaround-MentorPiA1` (built and trained 2026-11-03: `mentorpi_turnaround_env_cfg.py`,
   `lab/tests/test_turnaround.py` 6/6, run `turnaround/2026-11-03_10-27-46_turnaround_v1`, `model_450` auto-stopped at
   plateau; 2000 runs: success 98.2% / 97.3% with noise, contact 1.4% / 1.9%, median 4.1 s, median 2 switches; widths
   < 0.45 m 89-91%. ONNX `model_450.onnx` (obs 56: car_motion 4 | lidar_sectors 36 | depth 12 | heading_error 2 |
   last_action 2). Sim turning radius is 13-19% wider than H/tan(theta) (parallel-steering scrub): measure on the car.
   Car-side: `explore_policy.py --turn-model` switches to it when the obstacle stop holds > 1 s or the car moves
   < 10 cm in 3 s while commanding motion; target = heading + 180 deg; hands back at < 10 deg error and still for
   0.3 s (max 20 s, 3 s cooldown); shorter obstacle-stop reaches while turning (estimates). Not yet run on a car.
   Explore v3 (0.7 m/s, action_rate -0.5, contact -50): coverage 76% at 90 s, 0.46 m/s, wall contact ~5%, smoother
   actions (std 0.06); run `explore/2026-11-03_09-15-33_explore_v3_fast/model_550`.)
   First run procedure: copy both files + the v2 ONNX to `~/docker/tmp/policy` on car #1 (has ONNX
   Runtime), start nav_slam stack + depth_scan.py, `--dry-run` with the car still, then wheels up, then floor.
   GB10 was powered off 2026-11-02 after training/export (user-approved).
4. Car #2: floor trim test, phone-hotspot Wi-Fi, ONNX Runtime; mark the cars "1" / "2".
5. Two cars: separate `ROS_DOMAIN_ID`s (namespaces break Hiwonder's launch files) + a relay on the PC for maps/poses.
6. **One driving model instead of explore + turnaround (later; recommendation 2026-11-03).** For now the car runs two
   policies: explore v3 drives and maps; the turnaround skill takes over when the car is blocked (facing a wall, dead
   end or chair base, or no progress for ~2 s) and hands back once turned. Two models were a shortcut, not the goal:
   - Why split first: the explore task's offices (hallways 1.2-2.4 m, box-shaped clutter) almost never need a turn in
     a tight spot, so v3 never practised it; a dedicated task trains and measures the skill per corridor width in
     ~1 h per round; and the rewards differ (new floor + speed vs. precise heading + few maneuvers).
   - Cost of the split: hand-written switching logic on the car (when to hand over, when to hand back), glitches at
     the handover, and two models to keep in sync with the car-side observation code.
   - Plan: (1) prove the turnaround skill on the real car on its own; (2) train **explore v4** as one model: start
     from v3's weights, add narrow dead ends (0.4-1.4 m, as in the turnaround task) and office-chair star bases below
     the LiDAR plane to the explore offices, keep the turnaround task's anti-dither terms (action_rate -0.5, penalty
     per forward/reverse switch); optionally distil the turnaround policy into it (behaviour cloning on dead-end
     episodes) if plain RL learns the turns too slowly; (3) evaluate v4 on both the explore metrics and the turnaround
     metrics per width; (4) replace the two-model switching on the car with v4 once it matches both.

**Tools added 2026-11-02 (uncommitted):** `car/measure/` (safe_drive, go_dist, stop_test, motor_id, steer_check,
trim_drive, map_peek), `car/watchdog/`, `car/config/` (camfps_apply, lowload_apply/patch), `car/perception/depth_scan.py`,
`car/nav/` (nav_slam.launch.py, make_params.py, navigate_ackermann.xml, goal.py, plan_check.py, drive_to.py,
find_gaps.py), `car/policy/onnx_timing.py`, `car/dashboard/index.html` (live map with cars, trail, routes, battery %
estimate, camera on click; open via `python -m http.server 8765` in that folder), `car/pc/` (ROS 2 Humble in WSL
Ubuntu-22.04: `setup_ros2_humble.sh`, launchers `rviz2.cmd` / `rqt_graph.cmd` / `rqt.cmd` / `ros_shell.cmd`, `car.rviz`;
WSL mirrored networking + Hyper-V firewall rule ROS2-DDS-in done; RViz needs LIBGL_ALWAYS_SOFTWARE=1), the lab's
map-based formation (`localization.py`, `lidar_rate.py`, map_v2 = best: 99.2% completed with noise), the follow
retrain (100%, `lab/scripts/evaluate_follow.py`), the explore task.

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

**Status (2026-10-31):** `lab/scripts/export_onnx.py` exports with the normalizer baked in and writes the observation and action layouts into the ONNX metadata. On the office policy (`model_599.onnx`, 186 KiB):
- **Parity:** 1.2e-6 max difference over 6,400 recorded noisy observations (PASS).
- **Speed on this laptop's CPU:** 0.01 ms per step on 1 thread.

**Remaining:** time it on a Pi 5 once the cars arrive.

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

**Status (2026-11-02), car #1 wheels-up** (`docs/measurements/car1-2026-11-02.md`):
- **Measured and fed into the sim:**
  - LiDAR model: MS200
  - LiDAR rate 10 Hz and noise
  - IMU noise
  - camera intrinsics and rate
  - command latency 15–73 ms (`randomization.py`, `camera.py`)
- **Camera fps fix applied to car #1:** readers now get 24.8 fps, up from 5–6. Apply it to car #2 too.
- **New conflict:** the depth image's FOV is 56.6° × 43.9° (from its intrinsics), not the product page's 73.8° × 58.8°. The sim still uses 73.8°. Verify with a tape measure, then change `camera.py`.
- **Still open:**
  - hand measurements (wheels, wheelbase, track, mass, sensor heights, full-lock angle)
  - floor tests (top speed, acceleration, turning circle)

## 4. Retrain the follow task
`Robotlab-Follow-MentorPiA1` (`logs/rsl_rl/follow/2026-10-30_09-23-00`) was trained before the steering fix, with the car steering about 5°. That checkpoint is invalid.

**Done when:** it's retrained on the fixed car model and evaluated.

## 4b. Training replan, T1–T3 (2026-10-31, `project-plan.md` §13.4)
- **Leader:** stock Nav2.
- **Follower:** a trained formation policy.
- **Shelved:** the RL navigate task, because Nav2 already does that. A diagnosis from its evaluation is kept: v1 clipped standing people's feet at ~0.6 m/s.

**T1 status (end of the 2026-10-31 session): trained and evaluated, NOT done yet.**

Task `Robotlab-Formation-MentorPiA1` (`lab/src/robotlab/tasks/mentorpi_formation_env_cfg.py`):
- **Leader (Nav2-like):** slows in turns, stops for people ahead, accelerates at a limited rate, and cruises at 0.2–0.6 m/s. The source is Hiwonder's stock Nav2: velocity smoother caps at 0.32 m/s, TEB at 0.6 m/s.
- **Slot input:** column, diagonal L/R, side-by-side L/R.
- **Adaptive slot:** the rewarded target slides in behind the leader where there's no room beside it (ground-truth occupancy grid), and always at intersections ("single file").
- **People:** walking people that yield, and turn back after 3 s.
- **Sensors:** 360-ray LiDAR min-pooled to 36 sectors; depth camera (12 height-filtered sectors, `tasks/camera.py`), which sees feet the LiDAR can't (256/256 in the smoke test); stock-YOLO-style person detections (2 nearest).

Best checkpoint: `logs/rsl_rl/formation/2026-10-31_22-22-47_t1b_adaptive_fast/model_998.pt` (T1b, fine-tuned from T1 `2026-10-31_21-21-12/model_599.pt`). `lab/scripts/evaluate_formation.py`, 1000 runs × 60 s, without / with sensor noise:

| Slot | Completed | Wall | Person | Error to adapted slot | Fell back |
|---|---|---|---|---|---|
| column | 100 / 99.5% | 0% | 0 / 0.5% | 3.6 cm | 0% |
| diagonal L/R | 97–98 / 96–98% | 2–4% | 0–0.4% | 12–14 cm | 64% |
| side-by-side L | 87.5 / 87% | 12.5% | 0–0.5% | 42 cm | 74% |
| side-by-side R | 74 / 69% | 25–29% | 0.5–2.2% | 45 cm | 72% |
| all | 91.7 / 90.2% | 8–9% | 0.1–0.7% | 21 cm | — |

Person contacts were 1 / 7 per 1000 minutes, all with standing people. Before the camera input, v1 navigate had 21% of cars hitting someone within 2 minutes.

**Remaining problems (next session):**
1. **Side-by-side crashes** (12–29%) and loose tracking (~45 cm). Every simulated hallway is 1.4 m, which doesn't fit two cars side by side with 0.25 m clearance, so side-by-side is mostly falling back and spreading out again, and it fails during those transitions.
   - **Next:** randomize hallway width (e.g. 1.2–2.4 m), then narrow to the **measured real office hallways** (user to measure; ties to #11).
   - Also look at why the right side is worse than the left.
2. **Standing-person contacts aren't zero** (up to 7 per 1000 min with noise).
   - **Next:** diagnose as for v1, with car speed, bearing and leg vs foot at contact.
   - Then decide: more depth weight, a speed limit near people, or longer training.
3. **Done bar for T1:** ≥95% completed on every slot with sensor noise, and 0 standing-person contacts.
4. **No pytest yet** for the formation task. Smoke and stress scripts are in the git-ignored `lab/scratch/` (`formation_smoke.py`, `formation_stress.py`); promote the key checks to `lab/tests/`.
5. **Training throughput on the RTX 2060:** ~7.8–13k env-steps/s at 1024 envs, 600 iterations ≈ 45 min. Move training to the GB10 (#10) and measure the speedup.

**Shelved:** the RL navigate task `Robotlab-Navigate-MentorPiA1` (Nav2 already does this). Its `people.py` walkers, `camera.py` sensors and `evaluate_navigate.py` are reused or kept.

**Tools added this session:**
- `lab/scripts/export_onnx.py` (#2)
- `lab/scripts/evaluate_navigate.py`
- `lab/scripts/evaluate_formation.py`
- `lab/scripts/tour_viewer.py`: live viewer with a scripted camera path from `lab/scratch/tour_plan.json`. Use still shots: live camera moves shake in the browser.
- **Demo video:** `runs/video/virtual_lab_tour.mp4`.

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
**Model: MS200 on car #1** (stock driver node `/MS200`, 10.0 Hz, 449 points at 0.80°; `docs/measurements/car1-2026-11-02.md` §1). Check car #2.
**Problem:** the sim gives 36 rays, clipped at 3 m, refreshed every policy step (20 Hz). The real LD19 (or MS200 / COIN-D6) spins at about 10 Hz, ranges about 12 m, and has noise and dropouts.

**Do:** model the scan rate, the downsampling and the noise from the sensor datasheet (`docs/vendor/hiwonder/datasheets/LDROBOT_LD19_Development_Manual_EN_v2.4.pdf`).

## 9. Stale docs and files
- **`docs/docker-env.md` and `docs/virtual-lab-plan.md`:** they still describe the pre-mjlab design. Update them for mjlab, the viewer, cu130, driver 617, the Warp cache volume and the Microduck layout.
- **CLAUDE.md:** the quick-start command and the doc map need refreshing.
- **`lab/scripts/gpu_check.py` and `lab/tests/test_environment.py`:** written before mjlab. Verify or remove them.
- **Training-speed note:** the 6 GB RTX 2060 can't train and run the viewer at the same time (4.4× slowdown). Write this down.

## 10. Run the lab on the GB10
**Expected speedup: unknown until measured.** No published MuJoCo Warp / mjlab benchmark compares the GB10 with an RTX 2060. From the specs:

| | RTX 2060 laptop (this PC) | GB10 | Ratio |
|---|---|---|---|
| CUDA cores | 1920 | 6144 | 3.2× |
| FP32 | ~4.6–6.0 TFLOPS (derived: 1920 × 2 × 1.2–1.56 GHz boost) | 31 TFLOPS (NVIDIA, Hot Chips 2025) | ~5–7× |
| Memory bandwidth | 336 GB/s | 273 GB/s | 0.8× |
| Memory | 6 GB | 128 GB unified | ~21× |

So compute-heavy parts (rays, contacts, the MLP) should gain several times over, while memory-bound kernels may not gain at all. The 128 GB also allows 4–8× more parallel worlds, and running the viewer while training. Benchmark the formation task at 1024 / 4096 / 8192 envs on both machines; first runs are slow while the SM 12.1 kernels compile.

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

**Parked (user, 2026-10-31).** Research is done: `docs/research/04-face-recognition.md`. Key points:
- **Camera geometry:** the stock camera (0.12 m up, no tilt) only sees a standing person's eyes beyond ~3 m, and gives 6–16 px between the eyes at 640×480 (≥48 px are needed).
- **Proposal:** add an upward-tilted Pi Camera Module 3 for recognition.
- **Decisions pending:** that camera, and the weights license (AdaFace/InsightFace are research-only; SFace and dlib are clean).

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
