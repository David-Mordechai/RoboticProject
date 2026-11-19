# Lessons learned: training robot driving skills (living record)

Purpose: the project's real goal is a **method** for training AI skills for any robot (car today; walking robots,
UAVs, drones later): sensors and video in, actions / function calls out. This file records every path we took
until success: what we tried, what worked, what failed and why, and how each lesson transfers to other robots.
Updated at every step. Newest entries at the bottom of each section. Details and numbers: `docs/progress-live.md`,
daily reports `docs/report-*.md`.

---

## 1. Timeline of approaches

| # | Date | Approach | Result | Why (root cause) |
|---|---|---|---|---|
| 1 | 11-02..11-06 | **RL (PPO) end-to-end** on sim cars (mjlab, 8192 cars, GB10), plus scripted safety layer (stop, back-ups) | Drove the full hall round trip on car #1 (goto17, 63 s), but: weaving on straights, chair-base bump, pointless reverses | PPO with a Gaussian policy clipped to [-1, 1]: exploration std 1.67 > the whole steering range, so it learned all-or-nothing (bang-bang) steering. The chair base was under the LiDAR plane. |
| 2 | 11-08 | RL fixes: smoothness rewards (CAPS), straight-line penalties (goto v7, v7b, v7c) | Failed: steering stayed 9-11 deg, route following got worse | Reward shaping cannot undo the bang-bang optimum of a clipped Gaussian; penalties also punished needed manoeuvres (U-turn) |
| 3 | 11-08 | **Privileged teacher + imitation (DAgger)**: scripted teacher with the true map drives perfectly; a sensor-only student copies it | Teacher reached 16/16 sim goals; student alone plateaued; student + teacher-as-skill 16/16 | See 4 and 5 |
| 4 | 11-08 | Hybrid: learned student + scripted manoeuvre skill | 16/16 in sim; **failed on the floor 11-09** (never left the charger in 120 s) | (a) The sim exam used the privileged teacher as the skill; the car has only its own sensors. (b) The student ignored its sensors: in sim every route already went around the obstacles, so "follow the route" was enough (causal confusion). |
| 5 | 11-09 | Desk legs on the route + route noise; honest exams; sensor-use probe; car skill ported to sim | In progress | |

---

## 2. What worked (keep doing)

- **Diagnostics before fixes.** Every real fix came from a trace or a classifier of failure cases (per-step state of
  the worst cases), not from guessing. Example 11-09: a trace of the 3 worst cars showed the skill commanding reverse
  while the stop held it; root cause was a sign bug in the sim port, found in minutes.
- **Mean of 3 independent checks** with 2048 sim cars each; single checks passed by luck several times.
- **Recording every real run** (camera 2 fps, ROS bag, logs) and looking at the frames myself. The camera showed the
  desk legs are slanted A-frames, which the sim did not model.
- **Replaying real sensor recordings through a model offline** (`replay_car_obs.py`, `probe_sensors.py`): finds
  sim-to-real problems without driving.
- **Privileged teacher** (scripted, sees the true map): easy to make perfect, easy to debug, gives clean labels.
- **Output conditioning** of a learned policy (steering smoothing 0.5, dead band, forward-only + explicit reverse
  skill): fixed zig-zag and reverse blips without retraining.

## 3. What failed (do not repeat)

- **Reward shaping to fix a structural RL problem** (bang-bang from clipped actions). Fix the action distribution or
  change method instead.
- **Declaring success on an exam that uses something the robot does not have** (the privileged teacher as the
  skill in the 16/16 exam). Rule: the exam must use exactly what runs on the robot.
- **Training scenes where the shortcut works.** If the goal/route signal alone solves the sim, the network ignores
  the sensors (de Haan et al. 2019, causal confusion). Scenes must contain cases where only the sensors give the
  answer (obstacles the route does not know about, noisy routes).
- **Simplified obstacles.** Vertical posts in sim vs slanted A-frame legs in the office; low chair bases under the
  LiDAR plane. Model the real geometry seen by the camera.
- **Porting code by hand without a check.** The sim copy of the car skill had a reverse-steering sign bug; it
  produced invalid exam numbers for an hour.
- **Fixing on the floor by trial and error.** Two floor runs (goto18/19) with a guessed trigger change wasted a
  charge; the offline replay showed the real cause in minutes.

## 4. Gates before any real-robot test (adopted 11-09)

1. **Honest sim exam**: the same code path as the robot (no privileged information at run time), mean of 3 checks,
   all goals met.
2. **Replay of real recordings** of the last failure: the new model must command something better there.
3. **Sensor-use probe**: blanking each sensor group must change the actions near obstacles (sum >= 0.6 on goto19).
4. **Sim matches reality** for what the sensors see: compare camera frames of the failure spot with the sim scene.

## 5. Transfer to other robots (UAV, drone, walking robot, arm)

| Lesson | Car (now) | How it transfers |
|---|---|---|
| Teacher-student | Scripted teacher with true map, student with LiDAR/depth/camera | Drone: teacher with true obstacle map / state (as UZH drone racing and "Learning by Cheating"); student with camera/IMU. Legged: privileged terrain teacher (as ETH ANYmal). |
| Avoid clipped-Gaussian bang-bang | Steering std > action range | Any continuous control (thrust, joint torque): check exploration std vs action range; use squashed (tanh) or Beta policies, or imitation first. |
| Learned skill + explicit safety/manoeuvre layer | Stop corridor, multi-point turn | Drone: geofence + hover-hold + return-home as explicit layers; the net flies, the rules guarantee safety. |
| Shortcut / causal confusion | Route alone was enough | Drone waypoint following: put obstacles between waypoints that the plan does not know; randomize the plan. |
| Honest exam | Exam must use the robot's real code path | Same for any platform: hardware-in-the-loop or the exact onboard stack in sim. |
| Sensor-use probe | Blank LiDAR / depth / map | Blank camera / depth / IMU channels; the policy must react. |
| Model what the camera shows | Slanted desk legs, chair bases | Thin wires, glass, branches for drones; model the real failure geometry. |

## 6. Open questions

- Reactive manoeuvre rules (back-up and turn) can fall into local minima; research (arXiv 2511.22338; Nav2 Smac
  Reeds-Shepp + RPP allow_reversing) suggests planning the multi-point turn on the map. Decide after the current
  skill exam.

---

## 7. Log (newest last)

- 11-09: floor test of the hybrid failed (goto18/19). Replay: student ignores sensors (probe 0.30 vs old RL 1.52).
- 11-09: research: causal confusion (de Haan 2019); realizable students (Cornell, IROS 2025); lidar-to-depth student
  works when scenes contain random obstacles and noisy inputs (Fraunhofer 2026).
- 11-09: sim: desk legs on the route (slanted A-frames from camera frames), route noise, a 0.55 m passage kept.
  Teacher 15/16 at 30% leg segments.
- 11-09: U-turn time goal was a measurement bug (median 0 s); real value 5-6 s.
- 11-09: car skill port bug (reverse steering) found by trace; fixed. Teacher + car skill: 7/16 -> 9/16. Next trace
  running.
- 11-09: trace after the fix: worst cars face an obstacle with the route ahead (< 45 deg); the skill does not start
  (held trigger needs 45 deg), the back-up rule backs off, the driver goes straight back in: a loop (= floor goto18).
  Fix under test: "steer around" = DWA-style choice among 8 steering candidates checked against LiDAR/depth arcs,
  closest to the driver's steering, 0.25 m/s for 0.6 s. Lesson: a reactive back-up without a lateral choice loops.
- 11-09: steer-around: teacher + car skill 9/16 -> 12-13/16 (blocked hold 4.0 -> 1.1-2.4 s). Chair hits rose (2.4%):
  classified by state: 28/50 in the multi-point turn (reverse legs blind: depth camera looks forward, chair legs under
  the LiDAR). Added: reverse legs check remembered depth hits along the reverse arc (car + sim). Chair 2.4 -> 1.6%.
  Lesson: a sensor's field of view defines what a manoeuvre may do; any move into a blind zone needs memory of what
  was seen there (drones: rear/side blind zones, same rule).
- 11-09: user: "why react at 0.2 m? plan ahead". Right: the skill chose moves from the current sensor frame only, so
  a chair leg seen at 1.5 m was forgotten once it was < 0.2 m (depth camera minimum range) or out of view. Now every
  skill move (turn legs forward + reverse, steer-around candidates) is checked against all depth hits of the last
  8 s (the depth layer), the emergency stop stays reactive. Lesson: plan with memory of what was seen; reactive
  checks are only the last safety net (same for drones: obstacles leave the camera FOV before you pass them).
- 11-09: an independent line-by-line audit (a separate agent, not told my conclusions) of car code vs the sim exam
  found 4 more mismatches: the sim smoothed the skill's steering (1 s per lock flip vs ~0.15 s on the car), judged
  the model's unclipped output, kept counting "held" across back-ups, and a car bug (turn give-up timer not reset).
  All fixed; every exam re-run. Lesson: when the exam re-implements the robot's code, audit the port line by line
  BEFORE trusting any number; better: run the robot's own code in the sim loop (one implementation, no port).
- 11-09: exams after the audit fixes (mean of 3): teacher + car skill 13/16, student v9 + car skill 10/16 (was 5/16
  for the floor model). Shared failures = the skill. Idle classified: 37% "stop holds, route ahead, no free forward
  steering, nothing acts" until the 0.3 s back-up; 24% the student commands 0 itself. Fix: no free forward candidate
  -> start the multi-point turn at once (it reverses). Lesson: classify every idle / failure step by "who is in
  control"; gaps between rules (no rule owns a state) show up as idle time.
- 11-09: made the car skill one shared module (robotlab/tasks/carskill.py) used by the exam and the video recorder
  (record_clip.py --carskill), so what the user sees is what is examined.
- 11-09: rule patching of the skill plateaued (3 rounds: 12-13/16 teacher+skill, 10/16 student+skill). Switched
  method: PPO fine-tuning of the imitation student WITH the car's skill layer inside the training env (hook in the
  action term), on the same scenes and goals. Rationale (UZH DAgger -> RL fine-tune; Weihs 2021): the policy can learn
  to avoid the states where the rules get stuck. Lesson: when hand rules plateau, put them in the training loop and
  let the learned part adapt to them, instead of adding more rules.
- 11-09: found why the student stood still: it was trained on ALL teacher actions, including reverses and turn moves
  that the car's skill owns; under forward-only "reverse" became "stand still" and nothing triggered the skill.
  Fix: label only the states the learned part owns (teacher driving forward); run the skill in the DAgger loop.
  Lesson (any robot with a learned policy + scripted skills): split ownership explicitly, and train the network only
  on the states it controls; otherwise it learns the skill's moves badly and the hand-over between them breaks.
- 11-09 evening: floor goto20 (student v10 + fixed car skill, sim 12/16): total failure. 20 s and 5 back-ups to pass
  the charger desk legs, then 90 s stuck facing the cabinet beside the door (11 turns, 42 back-ups). Yesterday's RL
  model v6p did the full trip in 63 s. A day of sim-only progress (5/16 -> 12/16) did not transfer.
  Lessons: (1) never replace a floor-proven baseline with one judged only in sim; change one thing at a time on top
  of the working baseline and floor-test each. (2) Calibrate the sim exam against the floor before optimising it:
  a model that is better on the floor must also score better in the exam, else the exam is wrong.
- 11-09 evening: floor control runs. Plain v6p: goto17 63 s / 14 back-ups, goto22 67 s / 13 back-ups: consistent.
  v6p + steering smoothing (goto21): 74 back-ups, not finished. Lesson: a fix that helps a smooth policy (EMA on the
  student) can break a bang-bang one (v6p relies on full-lock flips to get round the desk); and always keep a
  control run of the baseline in the same session (battery, alignment) before judging a change.
- 11-09 evening: built an office twin from the car's own SLAM map (walls + legs the LiDAR saw, unknown = wall), the
  floor route and the charger start. It reproduced the floor: v6p 63 s median / 13 back-ups (floor 63-67 s / 13-14)
  and ranked the failed student+skill clearly below v6p. Random procedural hallways never did. It also exposed a bug
  in the task itself: the route's first waypoint lies on the desk leg. Lesson (any robot): build the exam from the
  robot's own map of the real site and calibrate it on real runs (ranking + numbers) before optimising anything;
  procedural scenes are for training variety, not for acceptance.
- 11-09 night: floor goto23 (v6p, hand-corrected route): finished 88 s but worse than goto17/22: weaving, drove under
  the table, bad U-turn (21 back-ups), 32 back-ups total. User: research what we do wrong.
  Research: BARN Challenge 2024 (Xiao et al., arXiv 2407.01862): every top team = global planner on a costmap with
  inflation (A* / move_base) giving a local goal + a local planner + an explicit safety layer with footprint arcs and
  recovery (reverse / rotate). The winner's local planner was LEARNED by behaviour cloning of a good classical expert
  (KUL+FM), LiDAR + local goal in, (v, w) out, noise augmentation; 6/9 physical runs, fastest. "Only LiCS-KI used
  learning." Root causes on our side: (1) no planner: hand-typed waypoints, three of them on obstacles; (2) RL policy
  that weaves (bang-bang); (3) a scripted teacher no one validated on the floor; (4) exams not calibrated to the site
  until today; (5) more and more interacting hand rules instead of planning; (6) one floor run per change.
- 11-09 night: deep research (docs/research/deep-2026-11-09/: 01 Nav2 Ackermann Pi, 02 learned local planners, 03 low
  obstacles, 00 synthesis). Verified in our make_params.py: the 11-04 Nav2 failure ("Starting point in lethal space")
  was our config (robot_radius 0.14 = centre-point check; MPPI consider_footprint False; mismatched turning radii;
  bonds -> idle CPU), not a limit of the approach; we abandoned the classical stack after one day. New plan: Phase A
  classical baseline correctly configured (polygon footprint, Smac REEDS_SHEPP, RPP, STVL depth, Collision Monitor,
  goals not waypoints) measured on fixed floor courses x5 vs v6p; Phase B learned local driver = BC of that expert
  (LiCS recipe) inside the stack; Phase C learned skills only where measured failures remain.
- 11-09 evening, Phase A on car #1 (Nav2 Humble 1.1.15): config bugs only the real stack showed, each fixed in
  make_params_a.py: LiDAR topic is /scan_raw (no /scan: stock filter commented out); rcl rejects YAML aliases; Humble
  collision-monitor polygons are flat double arrays and use max_points; the through-poses BT needs Spin unless replaced;
  a Humble "stop" polygon blocks ALL directions (the charger behind the docked car held it) -> use "approach" (motion-
  simulated); AMCL can silently drop /initialpose -> set and VERIFY the pose. Lesson (any robot): bring the real stack
  up stationary and check every topic/frame/param against the vendor stack before the first drive.
  Trials: C1 charger->door 17.8 s (works). C2 crawled and reversed: the depth "obstacle below the LiDAR" node saw the
  FLOOR as an obstacle 0.3 m ahead all run (camera pitched ~1 deg vs its TF; fixed z cut 2 cm) -> RPP "collision
  ahead" -> patience -> recovery BackUp + my script's back-up = "reversing for no reason". C3 on LiDAR only: no
  recoveries but the 1 Hz Smac Hybrid replan put a tiny reverse cusp 8-16 cm ahead of the car in 39/40 plans at the
  U-turn -> 22 reverse dithers; 26/27 steering flips happened while reversing; the velocity smoother (separate v and w
  limits) turned slow commands into full-lock flicks (driver steers atan(L w / v)).
  Lessons: (1) a depth "low obstacle" layer needs a per-frame floor fit, never a fixed height cut; (2) with a
  kinematic (Reeds-Shepp) planner replan only when the path is invalid, not on a timer; (3) recoveries that back up
  first look like random reversing to people: clear and wait first; (4) a car cannot fix its heading at a goal ->
  plan the final heading from far away, never turn on the spot at the dock; (5) for (v, w) -> steering drivers keep
  v and w scaled together.
- 11-09 night: Nav2 bench on the ASUS (fake car with the driver's real steering law, LiDAR ray-cast in the car's map,
  steering limit 29/33/40, 3 repeats). Calibrated on the day's floor runs (C1 time/flips match; C3 failure reproduced),
  then tree B (replan only if invalid) took C2 from a failure to 9/9 in 15 s and C3 from a failure to 6/8 in 44 s, with
  no hand rules. Lesson (any robot, UAV too): a cheap, calibrated software-in-the-loop bench of the REAL navigation
  stack beats tuning on the floor; one change at a time, N repeats, report medians and failures.
- 11-09 night, bench tuning (each step one change, 9-27 runs, medians + every failure looked at):
  RPP's own collision check (costmap cells, arc to the look-ahead point) gave false "collision ahead" in a tight
  S-bend past desk legs -> patience -> recovery loops; shorter look-ahead + check off (safety left to the Collision
  Monitor "approach" = real points, exact footprint, simulated motion) took C3 from 4/6 to 27/27. Top speed 0.35 -> 0.5
  m/s was only safe WITH the smoother limiting v and w together (our driver steers atan(L w / v)): without it the
  S-bend failed again. The safety polygon needed +3 cm: the car turns ~6 % tighter than Nav2 assumes (driver wheelbase
  0.145 vs URDF 0.137) and touched a leg 5 x in one run; docking moved outside Nav2 so the margin costs nothing there.
  A "more room" idea (pre-dock 0.6 m out) was WORSE (5/9): measure, don't assume.
  Tooling lessons: ROS_DOMAIN_ID must be <= 232 (Fast DDS) or runs fail silently; Windows edits leave CRLF that breaks
  bash on Linux (check with `file`); a lost lifecycle change_state reply can stall a Nav2 start under load -> always
  check "Managed nodes are active" before driving.
  Transfer (UAV / any robot): treat the vendor driver's command-to-actuator law as part of the plant in the sim; keep
  commanded components scaled together when the actuator uses their ratio.

## 2026-11-11: depth camera and floors
- Tried: per-frame floor-plane fit to stop the depth camera reporting the floor as an obstacle (fix designed from hall data).
- Found on the car: the glossy office laminate gives NO depth return at all at grazing angles; the only near-floor points were
  furniture bases, and the fit sometimes accepted a plane through them, which would have hidden their lowest 6 cm.
- Worked: accept a floor plane only with enough inliers close to the car (real floor returns are strongest there) and a
  small offset; otherwise fall back to the fixed height cut. Added an office test case next to the hall case.
- Transfer (UAVs/drones): a depth/stereo floor or ground model must be checked per surface type; "no return" is not
  "free" and object bases are not ground. Validate any ground fit on each real surface before trusting it.

## 2026-11-11: a sign fix that hid a side swap
- Tried (11-02): car drove backward, so we negated both rear-motor commands. Straight driving looked right, so we stopped.
- Failed: the motors were on the opposite sides from Hiwonder's build, so the turn differential was mirrored: at full
  lock the rear wheels pushed against the steering and the car barely turned. Small Nav2 turns only looked "a bit wide".
- Worked: isolate the parts (servo alone with the car standing, then servo + EQUAL wheel speeds on the floor), measure
  heading with gyro + LiDAR ICP (they agreed within 0.4 deg), then fix the mapping and verify through the driver.
- Also: a safety watchdog tuned for 20 Hz senders stopped an event-driven joystick every 0.5 s. Any timeout must know
  every legitimate sender's rate.
- Transfer (UAVs/drones): after any motor-mapping fix test every axis and every combination (pure throttle AND turning,
  both directions), not just the one that showed the bug. A mirrored mixer flies straight but fights every turn.

## 2026-11-11: re-test the planner after measuring the real turning limit
- Tried: Nav2 config tuned on the bench at 29-40 deg steering (vendor range). Car #1 measured only ~28 L / ~24 R.
- Failed: at 24-28 deg the one-goal park (turn ~180 deg next to the dock wall) failed 4/14 (shuffling, 64-104 s
  reversing). A first two-leg waypoint picked by eye was 0.25 m from a chair base: 0/14, the car pinned against it.
- Worked: do the U-turn in the middle of the open room to a waypoint already facing out, then reverse straight to the
  pre-dock pose (straight reverse leg 15/15). Pick waypoints from the map with clearance >= turning diameter, not by eye.
- Transfer (UAVs/drones): when a measured limit (turn rate, bank angle) is below the design range, re-run every
  manoeuvre test at the measured value; put tight manoeuvres where there is room, then approach the target straight.

## 2026-11-16: Gate A round 2 hit a table leg (CPU overload -> stale TF -> blind Nav2)
- Tried: 5 unattended Nav2 rounds (C1/C2/C3) after one clean round (13.8 / 15.1 / 49.0 s).
- Failed: round 2 degraded (45 s, then 106 s), the car hit a table leg; Nav2 later crashed. Logs: TF extrapolation errors
  growing from 10 s into the trial; load average 7 on a 4-core Pi 5 (Nav2 + depth_scan 53 % + bag + camera recorder).
- Our success check trusted Nav2's SUCCEEDED, which it reported with a broken TF tree -> the loop kept driving.
- Lesson: one clean round says nothing about load over time; measure CPU and TF latency WHILE driving before unattended
  loops; a robot needs a watchdog on its own localisation freshness (stop if TF/pose is stale), not only on commands.
- Transfer to UAVs/drones: same failure mode (compute starvation -> stale state estimate -> controller acts on old pose).
  Budget compute with all logging on, and fail safe on stale estimates (hover/land), never on "planner says done".
- Root cause of the overload (same day): rclpy busy-loop when one Python node has an Image subscription AND a tf2
  TransformListener (~3900 executor wakeups/s, 70-86 % of a Pi 5 core; each alone 1-4 %). Profiling (cProfile:
  _wait_for_ready_callbacks on top) found it in minutes after hours of guessing on 11-09/11-11. Fix: cache static TFs and
  drop the listener. Transfer: on embedded robots/drones, profile every Python node under the real topic load; prefer
  cached static transforms; budget CPU with all logging on.
- Safety pattern added: a freshness gate on the final command topic (stale scan/odom/localisation -> zero), tested by
  pausing a fed topic with renamed topics on the real car without moving it.
- Same day, 2nd Nav2 crash: Humble rclcpp action server aborts the whole process when a fresh client's first goal comes
  before discovery matched the reply path ("client will not receive response"). Reproduced standing still with a no-motion
  goal (goal = current pose): 1/1 crash without a wait, 0/13 with wait-for-server + 3 s. Lesson: reproduce a crash with a
  harmless input before and after the fix; no-motion goals are a free test of the whole navigation command path.
  Transfer: any ROS 2 Humble vehicle (drones too) talking to a long-running server from short-lived clients.

## 2026-11-16: depth-camera phantoms at the minimum range
- Tried: Nav2 costmaps + collision_monitor fed by a depth-camera "scan" (/depth_scan) next to the LiDAR.
- Failed: every run had near "obstacles" at a fixed 0.28-0.33 m that nothing real explained (LiDAR, camera frames). They
  were readings just above the HP60C's 0.2 m minimum range, only while driving. When one landed on a replan, the
  planner saw the robot's own cell as lethal -> costmap clears counted as recoveries (4-8 per trip).
- Worked (to be confirmed by g31-33): cut the obstacle scan at 0.28 m camera range + require near returns to persist
  for 2 frames. First a one-frame filter alone: replay showed it removed only ~half (phantoms last 2-4 frames).
- How we found it: histogram of near returns by bearing and RANGE in recorded bags; a tight range band independent of
  where the car was = sensor artifact, not the world. Replay the bags through a candidate filter before driving.
- Transfer (UAVs/drones): stereo/ToF depth is least trustworthy right at its min range; set the obstacle cut above the
  spec minimum from logged data, and keep a time-persistence check on anything very close.
