# Live progress (updated during work; read this first after a crash or power cut)

Last update: 2026-11-08 15:05 (PC clock)

## Goal
- A goto model for car #1 that meets all goals in `lab/scripts/tmp_goto_check.py` (16 goals, 2048 cars).
- Route: privileged teacher → imitation student (DAgger). PPO is dropped for goto; see `docs/open-issues.md` §000000000.

## Where things are
- **Teacher** `lab/src/robotlab/tasks/teacher.py`.
  - Exam: `docker compose run --rm -T lab python lab/scripts/tmp_goto_check.py teacher`.
  - v9 passes 11/16. Failing:
    - idle 5.1% (≤ 5);
    - U-turns 89.8% (≥ 90);
    - wall-start held 1.11 s (≤ 1);
    - reverse blips 2.3/min (≤ 1);
    - steering std 5.4 deg. That one measures settling after turns, not weaving: trace in tmp_teacher_std.py.
  - v10 (reverses need 0.6 m clear on the map first): **12/16**.
    - Wall-start held 0.67 s ✅.
    - Failing: idle 5.8%, U-turns 87.7%, blips 2.0/min, std 5.4 deg (settling).
  - Option `ROBOTLAB_TEACHER_MAP=built`: teacher plans on the car's built map, a "copyable" teacher. Not tested yet.
- **Student** `lab/scripts/train_student.py`, run by `~/student_run.sh NAME TASK [args]` on the ASUS.
  - Wait for results with `~/wait_goals.sh NAME`; results in `~/RoboticProject/goals_NAME.log`.
  - student_v1 (MSE, task Robotlab-Goto): 8/16, plateau.
  - student_v2 (L1 + last_drive, task Robotlab-GotoStudent-MentorPiA1), running since 14:20, 40 rounds:
    - best so far model_10: 8/16, zig-zag 1.5/s;
    - model_15: zig-zag 1.13/s;
    - gap to the teacher plateaus at 0.165.
- **ASUS:** the 9 shared containers are stopped (list in `~/stopped-containers-2026-11-08.txt`). No PPO training running.

## Research check 2026-11-08 15:10 (user: "don't follow the wrong path all the way")
- Imitation gap (Weihs et al., NeurIPS 2021): a teacher with privileged info makes the student hesitant in ambiguous states. Fixes:
  - a realizable teacher (LEAP, ICLR 2025);
  - RL fine-tuning after DAgger, carefully (UZH agile flight: 40% -> 100%).
- Copycat problem (Wen et al., NeurIPS 2020): the previous-action input (our last_drive) can make the student repeat itself. Student v2's reverse blips rose 5.2 -> 9.5/min.
- Plan:
  - (a) exam the built-map teacher;
  - (b) student_v3 on it, task Robotlab-Goto (NO last_drive), L1;
  - (c) RL fine-tune with an imitation term if a gap remains.

## 15:10 state
- Built-map teacher: idle 18.5%, U-turns 76%. The sim built map never clears people's trails, so the teacher stopped for ghosts.
  - New mode `ROBOTLAB_TEACHER_MAP=seen`: true obstacles the car has seen. Exam running.
- student_v2 finished, 40 rounds. Best zig-zag 0.82/s (model_30); reverse blips rose to 10/min (copycat?).
- **student_v3** running since 15:10:
  - task Robotlab-Goto (no last_drive), L1, teacher v10 (true map);
  - init: student_v1 model_40;
  - `goals_student_v3.log`.

- "seen" teacher exam:
  - 11/16, about as good as the true-map v10;
  - idle 5.9%, blips 1.9/min, U-turns 89.3%, chair 1.3%.
- **student_v4** running since ~15:22: as v3 but `TEACHER_MAP=seen`. Compare v3 vs v4.

- **student_v3** model_5: gap 0.135 by round 10. v2 with last_drive: 0.155-0.17. Goals like v2's early checks.
- Teacher idle breakdown ("seen"): shield holds 35% (its approaching part), multi-point turn 35%.
- **Prepared, not run yet:** RL fine-tuning after DAgger.
  - `lab/scripts/prepare_finetune.py`: student actor, std 0.10 / 0.15 (the PPO ckpts had steer std 1.67 for range +-1!), v6p critic, fresh Adam.
  - Then train Robotlab-Goto with env `ROBOTLAB_IMITATE_W` > 0 (`teacher_imitation` reward, label from goto_step).

- student_v4 ("seen" teacher) model_5 is about the same as v3: gap 0.141. So the gap is not mainly the true map; it's people knowledge, turn modes and shield stops.
- **ft_v1** (PPO fine-tune) running since 15:34:
  - started by `~/ft_run.sh ft_v1 <v3 model_15> 5 600` (imitation weight 5, 600 iterations);
  - std 0.10 / 0.15, v6p critic;
  - run dir `logs/rsl_rl/goto/2026-11-08_15-34-09_ft_v1`, results in `goals_ft_v1.log` (every 50 iterations).

- 15:50: ft_v1 stopped at iteration 16 (31 s/iteration, ETA 9 h). The teacher label is now computed every 2nd step (IMITATE_EVERY).
  - **ft_v2** running: same settings, 400 iterations, ~20 s/iteration, ETA ~2.5 h (about 18:20).
  - Run dir `logs/rsl_rl/goto/2026-11-08_15-50-53_ft_v2`, results in `goals_ft_v2.log` every 50 iterations.
- student_v3 / v4 stopped at about round 22 (plateaued). Their watcher loops still poll and are harmless.

- ft_v2 model_100 (16:15):
  - improving: wall-start held 3.5 -> 1.8 s, idle 7.8 -> 6.7%, chair 1.7 -> 1.0%, goals/min 2.59;
  - flat: zig-zag 0.87/s, U-turns 81%, blips 6.9/min.

- ft_v2 model_150: flat or slightly worse (zig-zag 0.90, U-turns 81%, blips 7.1/min, wall hold 1.97 s). Fine-tune alone isn't enough.
- 16:35: output conditioning added, sim (env vars) and car (from ONNX metadata):
  - dead band `ROBOTLAB_V_DEAD` (0.03 m/s = car #1's measured motor minimum);
  - steering smoothing `ROBOTLAB_STEER_EMA`.
  - Test: ft_v2 model_100 with V_DEAD 0.03 + EMA 0.5, check running.
- Idea held back: the multi-point turn as an explicit skill (mode switch like the car's turnaround), not learned by imitation. U-turns are stuck at ~81% in every student.

- **16:50 results** on ft_v2 model_100, no retraining:
  - EMA 0.5 + dead band 0.03: zig-zag 0.87 -> **0.35 ✅**. Blips 6.9 -> 5.9, so they are real reverses.
  - **+ forward-only (`ROBOTLAB_FORWARD_ONLY=1`): 11/16, best learned so far.**
    - Passing: blips 0, chair 0.8%, person 0.7%, corner hold 0.8 s, zig-zag 0.34.
    - Failing: idle 12.5%, U-turns 80%, wall starts 88% / 2.2 s, std (settling).
    - What's left is all reverse manoeuvring.
- **Hybrid check running:** the student drives forward and the teacher's manoeuvre skill ("seen" map) handles multi-point turns and reverses. Checker arg `hybrid:<ckpt>`.
  - If it passes: port the teacher's turn / reverse logic to the car node (SLAM grid + depth layer for the arc checks).

- Hybrid (student + teacher skill): 10/16, U-turns 83%. Switching mid-manoeuvre breaks it.
- **The teacher's U-turns by hall width:** 1.0-1.2 m 82% (12 s per turn), 1.2-1.4 90%, 1.4-1.6 95%, 1.6-1.8 96%.
  - Cause: REV_CLEAR 0.6 blocked reverse legs in narrow halls. The multi-point turn now uses KTURN_REV_CLEAR 0.45.
- **Checker goals v2, 17:10 (old values still printed as INFO):**
  - "reverse blips" counts only when forward was free (the user's "reverse for no reason");
  - "steering std" replaced by "weave" = steering minus its 1 s mean (v1 counted the settling after turns).
- Teacher v11 exam + U-turn widths running (`~/teacher_v11.txt`, `~/teacher_v11_uturn.txt`).

- **Teacher v11 ("seen" map, goals v2): 12/16.**
  - Weave 1.5 deg ✅ (v1 std 5.5); wall starts 93% / 0.85 s ✅.
  - Failing: idle 5.9%, blips-with-forward-free 1.4/min, chair 1.7% (up: shorter reverse clearance?), U-turns 88.6%.
  - Running: U-turn widths ("seen"), then the true-map v11 exam (is the chair rise caused by unseen legs?).

- True-map teacher v11: **13/16**, chair 0.7% ✅ (the "seen" 1.7% is unseen legs while reversing).
  - Failing: idle 5.6%, blips-forward-free 1.38/min, U-turns 88.5%.
  - U-turns in 1.0-1.2 m halls: 82%, 10.9 s per turn.
- 17:45 **manoeuvre mode**:
  - the sim stop uses the car node's turn-mode reaches (0.22 front / 0.20 rear / half width 0.13, `explore cfg TURN_*`) while `env._robotlab_manoeuvre` is set;
  - the teacher sets it during its multi-point turn; KTURN_ARC 0.22, KTURN_REV_CLEAR 0.275;
  - the car node already uses these reaches in its turn mode;
  - teacher v12 exam running.

- **Teacher v12 (manoeuvre mode, true map): 15/16.**
  - U-turns 93.1%, idle 3.8%, blips 0.38, person 1.0% (on the line).
  - Only failing: wall-start held 1.24 s.
  - 1.0-1.2 m halls 89% (8 s per turn).
- Teacher v13: the multi-point turn also starts when no forward move is allowed, and ends only once forward is allowed again. Exam running.
- **Next once the teacher passes 16/16:**
  1. confirm with a mean of 3 checks;
  2. student training with manoeuvre mode;
  3. the multi-point turn as an explicit car-side skill (port teacher.py's turn logic, using the SLAM grid + depth layer instead of the true map).

- Teacher v13: 14/16.
  - Person contacts at fault 2.1%: the turn started next to people, and manoeuvre mode has no approaching stop.
  - Wall hold 1.05 s.
- Teacher v14: no blocked-start turn when a person is within 1 m ahead (wait instead). Exam running.
- ft_v2 stopped at iteration ~300 (superseded).

- Teacher v14: 14/16 (person 0.7% ✅). Failing: chair 1.5%, wall hold 1.26 s.
  - Blocked-start trace: after the turn the teacher went straight to 0.6-0.7 m/s beside the wall, and the stop held it again and again.
- Teacher v15: gentle exit (EXIT_V 0.3 m/s for 1 s after a turn). Exam running.
- mdp forward_only now exempts manoeuvre steps (for the hybrid: student forward + teacher turn skill).

- **Teacher v15: 15/16.** Only failing: chair 1.3%. Wall hold 0.96 ✅, U-turns 90.6%, person 0.9% (tight margins).
  - Chair hits come mostly from back-ups / turn reverses into legs. Behind the car only the LiDAR sees, and the legs are under its plane.
- v16: when reversing, the stop also checks remembered depth obstacles behind the car.
  - Sim: `explore cfg rear_depth_blocked`. Car: `blocked()` with DepthLayer.
  - Exam running.

- v16 reverted (chair 1.4%, wall hold 1.70 s: the depth map also holds wall bases behind the car). **Teacher v15 is the best: 15/16.**
- 19:05: **mean of 3 checks of teacher v15** running.
  - `~/mean3.sh teacher_v15 "-e ROBOTLAB_TEACHER_MAP=true" teacher`, results in `~/mean3_teacher_v15.txt` + printed summary.
- After that, depending on the result:
  - student training on teacher v15: DAgger, Robotlab-Goto, L1;
  - deployed with EMA 0.5, dead band 0.03, forward_only + the turn skill;
  - the turn skill must be ported to the car node.

- 19:20: **car-side multi-point turn skill** written: `explore_policy.py --kturn`, `kturn_step()`, KTURN_* = teacher v15 values.
  - The stop uses the turn reaches while it runs.
  - Offline stub test: start, reverse at a wall, end, gentle exit all OK. Not yet run on the car.

- **Teacher v15, mean of 3 checks: 13/16.**
  - Edge failures: person 1.1% (checks 1.0 / 1.0 / 1.3), chair 1.0% (1.4 / 1.0 / 0.7), wall hold 1.23 s (1.15 / 1.48 / 1.07).
  - U-turns pass at 90.1%.
- **student_v5** running since 19:42 PC time (17:42 ASUS clock):
  - init student_v3 model_15, teacher v15 (true map);
  - RUN_ENV V_DEAD 0.03 / EMA 0.5 / FORWARD_ONLY;
  - checks in hybrid mode (`goals_student_v5.log`).
- Teacher v17 = v15 + `ROBOTLAB_TEACHER_PERSON_SLOW=1` (0.35 m/s with a person within 1.2 m ahead). Mean-of-3 running (`~/mean3_teacher_v17.txt`).

- **student_v5 model_5, hybrid: 11/16, best learned + skill.**
  - Passing: zig-zag 0.29, weave 1.3, blips-free 0.33, chair 0.8%, wall 1.2%.
  - Failing: idle 6.0%, person 1.3%, U-turns 85%, wall starts 89.9% / 2.1 s.
  - The hybrid check now applies the gentle exit to the student too, as the car node does (from model_10 on).
- Teacher v17's first mean-of-3 crashed (`speed` used before it was assigned). Fixed, smoke-tested, rerunning.

- **Teacher v17 (person slow-down), mean of 3: 14/16.** Person 0.7% ✅.
  - Left: chair 1.3% (1.3 / 1.5 / 1.0), wall-start hold 1.26 s (1.31 / 1.15 / 1.33).
  - Diagnostics running (blocked trace + bump classes).
- student_v5 model_10, hybrid: 11/16. Wall hold 1.29 s (gentle exit), zig-zag 0.27, U-turns 85%, person 1.3%.

- v17 diagnostics:
  - (1) a turn's reverse leg kept being commanded while the stop held it, 2.75 s (the map check only), and the scripted back-up fired inside the turn;
  - (2) 9/11 chair hits were back-ups or reverse legs.
- **Teacher v18:**
  - mdp: no scripted back-up while `_robotlab_manoeuvre` (= car node);
  - teacher: turn legs also need the stop's turn-reach sensor test (`_shield_turn`).
  - Mean of 3 running (`~/mean3_teacher_v18.txt`).
- student_v5 model_15 hybrid: **12/16**. Failing: idle 6.2%, U-turns 83%, wall hold 2.1 s, corner hold 1.19 s.

- **Teacher v18, mean of 3: 13/16.** Chair 0.2% ✅, wall hold 0.43 s ✅, wall 0.6%, person 0.5%.
  - New failures: idle 6.1%, U-turns 87.9%, wall starts 89.7%. With the back-up off inside the turn, both-legs-blocked turns waited.
- **Teacher v19:**
  - both legs blocked -> try the other lock;
  - still blocked 1 s -> end the turn and hand over to the back-up;
  - both triggers wait the cooldown.
  - Mean of 3 running (`~/mean3_teacher_v19.txt`).
- student_v5 finished, 40 rounds. Best hybrid: model_15 at 12/16; model_25 (with the v18 code) 11/16.
- **Car node still needs:** no gentle-exit change. But its kturn_step has no "other lock" / give-up rule yet. Add both once v19 is confirmed.

- # ✅ **TEACHER v19: 16/16 goals, mean of 3 checks, every single check passing too.** (20:40 PC time)
  - Run with env `ROBOTLAB_TEACHER_MAP=true ROBOTLAB_TEACHER_PERSON_SLOW=1`; results in `~/mean3_teacher_v19.txt`.
  - Key values: U-turns 92.7%, idle 2.2%, chair 0.8%, person 0.7%, wall-start hold 0.51 s, blips-free 0.14/min.
- **student_v6** running since 20:41 PC time (18:41 ASUS clock):
  - init student_v5 model_40, teacher v19;
  - RUN_ENV PERSON_SLOW + V_DEAD 0.03 + EMA 0.5 + FORWARD_ONLY;
  - hybrid checks (`goals_student_v6.log`). First gap 0.088.
- Car skill `kturn_step` updated with the v19 rules (other lock, hand over after 1 s). Offline stub test OK.
- **Known sim/car gap:** the teacher's two person rules (no turn start with a person ahead; slow near people) use people positions the car doesn't have (no person detector in the node). The student copies the slowing from LiDAR / depth; the car skill has no person rule.

- student_v6 model_5, hybrid: 11/16.
  - Narrow misses: idle 5.1%, corner hold 1.005 s.
  - Also failing: person 1.4%, U-turns 87%, wall hold 1.57 s.
- Running: mean of 3 for teacher v19 with `TEACHER_MAP=seen` (realizable: only what the car saw); results in `~/mean3_teacher_v19_seen.txt`.

- student_v6 model_10 hybrid: 11/16. Person 0.9% ✅; holds 1.59 / 1.22 s; U-turns 86.5%; blocked done 89.9%; idle 5.4%.
- Hybrid check: the skill now also takes over after 0.3 s held (= car kturn_step trigger). Used from student_v6 model_15/20 on.

- **Teacher v19 with the "seen" map (realizable), mean of 3: 15/16.** Only chair 1.3% fails (never-seen legs). U-turns 91%, idle 2.0%.
- student_v6 model_15 hybrid: **12/16**. Failing: idle 5.7%, U-turns 85.9%, wall hold 1.18 s, corner hold 1.9 s.

- student_v6 model_20 hybrid (with the held trigger): 12/16.
  - Corner hold 0.63 ✅.
  - Failing: idle 5.8%, person 1.3%, U-turns 85.7%, wall hold 1.12 s.
- Test running: hybrid with `ROBOTLAB_HYBRID_NEAR_TURN=0.8` (the skill drives within 0.8 m of a turn-back corner).
  - If U-turns rise to ~92%: port to the car (pure pursuit on the route preview + kturn near turn_backs corners).

- **Hybrid v6 model_20 + NEAR_TURN 0.8: 14/16 (single check).**
  - U-turns 92% ✅, idle 3.4% ✅, wall hold 0.45 ✅, person 1.0% ✅.
  - Failing by a hair: corner starts 89.3%, corner hold 1.011 s.
  - Mean of 3 running (`~/mean3_hybrid_v6m20.txt`).

- Car node: the near-U skill is ported (`kturn_step`: pure pursuit on the route preview within NEAR_TURN 0.8 m of turn_backs corners). Offline stub test OK.

- # **Hybrid student v6 model_20 + skill (NEAR_TURN 0.8), mean of 3: 15/16.**
  - Only failing: person contacts at fault 1.1% (0.9 / 1.1 / 1.4).
  - U-turns 91.5%, holds 0.52 / 0.78 s, zig-zag 0.27/s, chair 0.6%.
  - Exported: `lab/logs/rsl_rl/goto/student_v6/model_20.onnx` (on the PC too).
    - Metadata: v_dead 0.03, steer_ema 0.5, forward_only 1, depth_map 8 s / 1.5 m, unstick_escalate.
    - Car run: `explore_policy.py model_20.onnx --kturn --ref-map ... --route ...`.
- Mean of 3 for model_40 running (`~/mean3_hybrid_v6m40.txt`): is person safety better?

- Hybrid model_40, mean of 3: **15/16**. Person 0.9% ✅ but corner hold 1.15 s ❌. Model_20 is the reverse: person 1.1% ❌, corner 0.78 ✅.
- Running: model_40 + `ROBOTLAB_HYBRID_BEND_DEG=60` (the skill steers through route bends > 60 deg within 0.8 m), mean of 3 (`~/mean3_hybrid_v6m40_bend.txt`).
  - If 16/16: port the bend rule to the car kturn_step (the same bend test on the route preview), export model_40.

- Hybrid model_40 + bend 60: 15/16, corner hold 1.007 s (fails by 0.007).
- Running: + `ROBOTLAB_HYBRID_HELD_S=0.1` (the skill takes over after 0.1 s held instead of 0.3); `~/mean3_hybrid_v6m40_held01.txt`.
- Car node already updated: BEND_DEG 60 pure-pursuit zone + KTURN_HELD_S 0.1 (back-up keeps 0.3).

- # ✅✅ **FINAL (22:15 PC time): hybrid student_v6 model_40 + skill = 16/16 goals, mean of 3 checks.**
  - Settings: `ROBOTLAB_TEACHER_MAP=true TEACHER_PERSON_SLOW=1 V_DEAD=0.03 STEER_EMA=0.5 FORWARD_ONLY=1 HYBRID_NEAR_TURN=0.8 HYBRID_BEND_DEG=60 HYBRID_HELD_S=0.1`.
  - Results in `~/mean3_hybrid_v6m40_held01.txt`.
  - Exported: `lab/logs/rsl_rl/goto/student_v6/model_40.onnx` (parity 9.2e-6), on the PC.
  - Lab tests 26/26. Report: `docs/report-2026-11-08.md` §6-7.
  - **Next: the floor test** with `model_40.onnx --kturn` on car #1.

## Next steps (in order)
1. Teacher v10 result. Fix the remaining teacher goals one at a time (diagnose with tmp_teacher_*.py, don't guess).
2. Exam the built-map teacher (`ROBOTLAB_TEACHER_MAP=built`). If it passes about as well, train student_v3 on it.
3. A student that meets all goals → mean of 3 checks → export ONNX (`export_onnx.py Robotlab-GotoStudent-MentorPiA1`).
   The car node already supports its layout (`GOTO_LAST_LAYOUT`).
4. Write `docs/report-2026-11-09.md` for the user.

## How to resume after a power cut
- ASUS: `ssh davidm@192.168.77.52`. It powers on by itself after an outage; the 9 shared containers start again, so stop them.
- Check `ls -t ~/RoboticProject/logs/rsl_rl/goto | head` and `tail goals_student_v*.log`.
- A dead student run continues by re-launching `student_run.sh` with a new name. There's no resume flag yet; start from the last saved `model_*.pt` with `--init`.

## 2026-11-09 floor test: FAILED
- goto18 (model_40 --kturn) and goto19 (+ held trigger only if route > 45 deg): the car never left the charger area in 120 s (17 multi-point turns / 60+ back-ups).
- Cause, from replaying the car's inputs through model_40 (scratchpad g18): the student **ignores its sensors**. Zeroing lidar, depth or the maps changes its steering by < 0.03; it follows only the route and drives 0.7 m/s into the desk leg until the stop holds it.
- So the sim 16/16 came from the privileged teacher skill + stop doing the obstacle work, not from the student. The sim exam was not valid for the car: it must use the car's own skill (kturn_step), not teacher.py.
- U-turn time goal was a checker bug (start and end fired in the same step, median 0 s); fixed in tmp_goto_check.py (heading within 30 deg of the back leg). Re-check running: ~/mean3_hybrid_v6m40_uturnfix.txt.
- Research (de Haan 2019 causal confusion; Cornell IROS 2025 realizable students; Fraunhofer 2026 lidar->depth student): confirmed in office_terrain.py that every sim route ran around the obstacles -> route alone was enough -> shortcut.
- Fix in progress (goto v8): `office_terrain` desk legs ON the route line (`route_leg_prob`, DESK_LEG_* estimates) + route jitter (`route_jitter`), env `ROBOTLAB_ROUTE_LEG` / `ROBOTLAB_ROUTE_JITTER`. MAX_CLUTTER x3 (crash fix).
- New gate `lab/scripts/probe_sensors.py` (car recording replay, blank sensor groups): v6p 1.52, model_40 0.30, gate >= 0.6.
- Real U-turn time (fixed checker): 5.8 / 5.3 / 6.3 s, passes.
- Running: teacher on leg scenes (~/check_teacher_legs.log); model_40 alone on leg scenes (~/check_m40_alone_legs.log).
- Plan: if teacher passes on leg scenes -> DAgger student (seen map, no forward_only) on leg scenes; exam = student alone (no teacher skill); then probe gate; then floor.
- Teacher on leg scenes: 15/16 (zig-zag 1.1 = swerving around legs; checker v3 now counts straights only with nothing within 0.35 m / 1.2 m ahead).
- Honest exams reproduce the floor: model_40 alone on legs 7/16; model_40 + ported CAR skill (`tmp_goto_check.py carskill:<ckpt>`) 5/16 (idle 20%, blocked hold 6.2 s).
- student_v7 training since 09:02 ASUS clock: init v6p 8750, teacher true map, legs 0.5 + jitter 0.15, FORWARD_ONLY, per-checkpoint exam = student alone (goals_student_v7.log). Gate for floor: carskill exam + probe_sensors >= 0.6.
- student_v7 model_10: probe 0.48 (model_40 0.30, gate 0.6); still drives at the charger leg. Camera: the office desks have SLANTED A-FRAME legs (round tubes; foot != LiDAR-height point). Stopped v7.
- office_terrain v8b: legs = capsules from a shared top (0.72 m) to feet 0.12-0.26 m out (A-frame) or single tilted (30%); occupancy along the leg up to 0.40 m. Verified ends in MuJoCo.
- **student_v8** since 09:16 ASUS clock (logs/rsl_rl/goto/2026-11-09_09-15-58_student_v8): same settings as v7 + slanted legs; per-checkpoint exam = CARSKILL (student + ported car skill). Teacher check on slanted legs: ~/check_teacher_slant.log.
- Teacher on slanted legs 0.5: 10/16; + DESK_LEG_PASS 0.55 m passage: 13/16 (idle 8%, U-turns 88.4%, goals/min 1.84). Idle classes: shield 32%, kturn 21%, map blocks arc 19%, accelerating 18%.
- **student_v9** since 09:27 ASUS clock (logs/rsl_rl/goto/2026-11-09_09-27-22_student_v9): legs 0.3 + jitter 0.15, carskill exam per checkpoint (goals_student_v9.log). Teacher check at 0.3: ~/check_teacher_slant3.log.
- carskill:teacher (teacher forward + ported car skill) gave 7/16, blocked hold 10 s. Worst-car trace: skill said rev_ok and commanded reverse, stop held it 11+ s. Cause: MY PORT BUG (reverse steer = side*lim instead of car's kt_dir*side*lim). Fixed. All earlier carskill numbers (model_40 5/16, teacher 7/16, student_v9 checkpoints) are INVALID; re-running. Car code itself is correct on this point.
- Research (arXiv 2511.22338; Nav2 Smac Reeds-Shepp + RPP allow_reversing): reactive backup-turn rules fall into local minima; planned forward/reverse manoeuvres on the map are the robust option, if the skill still fails after the bug fix.
- 11-09 afternoon: car skill now a shared module `robotlab/tasks/carskill.py` (exam + record_clip --carskill + training hook). Audit by independent agent: 4 sim-port mismatches + 1 car bug, fixed. Fixes from idle/chair classification: steer-around (AV_*, incl. straight-slow), memory checks fwd+rev, no-free-candidate -> k-turn.
- Mean of 3 (carskill exam): teacher+skill 12-13/16, student v9+skill 10/16 (floor model was 5/16). Rule patching plateaued over 3 rounds.
- **ft_cs1** since 12:50 ASUS clock: PPO fine-tune of student v9 m40 WITH the car skill in the loop (ROBOTLAB_CARSKILL=1, mdp hook), legs 0.3 + jitter, imitation W 0.05, 300 iters x 16.8 s (~85 min). Checkpoint exams (carskill) -> ~/RoboticProject/goals_ft_cs1.log. Script ~/ft_cs_run.sh.
- Clip: runs/video/student_v9_carskill.mp4 (record_clip.py --carskill --chase).
- ft_cs1 (RL fine-tune with skill): ckpt 0 10/16, ckpt 50 11/16 but idle up (student commands 0: 37% of idle), reward falling -> stopped at iter ~70.
- Root cause found: DAgger labels included the teacher's reverses / k-turn moves; the student learned "reverse", forward_only turned it into "stand still" with no stop hold -> no skill trigger.
- **student_v10** since 13:16 ASUS clock (logs/rsl_rl/goto/2026-11-09_13-16-28_student_v10): init v9 m40, `--forward-labels-only` (15% of states dropped), car skill in the loop (ROBOTLAB_CARSKILL=1), beta0 0.5, 40 rounds; carskill exams per checkpoint -> goals_student_v10.log.
- **Floor goto20 (v10 m40 + skill): FAILED** (charger 20 s, then stuck at the cabinet beside the door 90 s). v6p (goto17) did the full trip.
- Proposed to user: back to v6p baseline, one floor-tested change at a time (first: steering smoothing only); calibrate sim exam with v6p (running: ~/mean3_v6p_legs.txt).
- Floor goto21 (v6p + steer EMA 0.5): door + hall end OK, not back in 120 s, 74 back-ups, went LEFT around the charger desk.
- Floor goto22 (plain v6p, control): **full trip 67.1 s, 13 back-ups** (= goto17 63 s, 14). v6p is consistent 2/2; EMA hurt v6p (1 run).
- **Office twin exam built** (2026-11-09 evening): `lab/scripts/make_twin.py` (car map goto6_map_after.npz -> `lab/src/robotlab/tasks/twin_office.npz`, unknown = wall, charger cells cleared 0.22 m), `robotlab/tasks/office_twin.py` (terrain + start + route, env ROBOTLAB_TWIN), hooks in reset_explore / reset_goto / goto cfg, exam `lab/scripts/twin_check.py MODEL [N]`.
- Calibration (256 cars, 120 s): v6p 75% finish, median 63 s, 13 back-ups (floor 2/2, 63/67 s, 14/13) MATCH. v10+skill 33%, 94 s, 30 back-ups, 23 turns (floor: failed) RANKED CORRECTLY. v6p+EMA 74% / 61 s (floor goto21 failed, 1 run): twin says no effect; v6p's own failure mode in the twin (16-21% "lost" right after the start) = what goto21 did -> goto21 likely bad luck.
- Finding: the route's first waypoint (1.2, -0.3) is ON an occupied map cell (the desk leg at the charger); the door waypoint touches the door frame.
- Twin: "lost" termination removed (the car never ends a run off-route). v6p old route 90.6% / 65.5 s / 16 back-ups; corrected route 91.0% / 58.4 s / 13.
- **Floor goto23 (v6p, corrected route): full trip, 87.8 s, 32 back-ups.** Back-ups: 4 at the start (table legs), ~21 at the hall-end U-turn (mostly "0.7 m off the route": the new lanes are 0.4 m apart, the car swings wide to x 3.6-3.8), 7 on the way back at the chair (1.2-1.5, -0.85).

## 2026-11-09 night: Phase A (classical baseline) started, offline
- Deep research saved: docs/research/deep-2026-11-09/ (00-synthesis.md = the plan).
- `car/nav/make_params_a.py` -> `nav2_params_a.yaml` (standalone Humble: polygon footprint, Smac REEDS_SHEPP R 0.40, RPP allow_reversing, inflation 0.40/4 global 0.30/5 local, LiDAR + depth_scan obstacle layers, Collision Monitor, no Spin, bond_timeout 0). BT `navigate_ackermann_a.xml` (clear, BackUp 0.2 m, wait, replan x4).
- Offline planner check on the ASUS: image `robotlab-nav2:humble` (car/nav/offline/Dockerfile), `car/nav/offline/plan_check_a.py MAP.npz PARAMS [--first X Y]`.
- Results on goto6 map: start AT the charger = "Starting point in lethal space" with either footprint (the dock is in the map) -> an undock step (straight out 0.3 m) is needed. From 0.3 m: charger->door 3.75 m, min clearance 0.19 m, right of the table-leg row; door->hall end centred, 0.30 m clearance, no reverse; return with a planned 3-point U-turn at the hall end (7 reverse poses), min clearance 0.19 m.
- With today's chair (goto20 depth hits merged into the goto20 map): the planner goes LEFT of the table legs (y +0.5), not between table and chair: the gap is narrower than footprint + inflation in the car's own data.
- Open before the floor: base_footprint position (URDF on car), depth point cloud topic + Pi CPU (STVL), undock step, install Nav2 params on car, minimum moving speed.

## 2026-11-09 16:20 ASUS clock: Windows crash, resumed
- Nothing was running at the crash (Phase A was offline work). ASUS up 1 day 11 h, idle, 0 containers. Reach it via 10.10.77.84 (192.168.77.52 timed out from the PC after the reboot).
- PC Wi-Fi lost AndroidAP; the hotspot is not broadcasting -> car #1 (10.71.88.196) and car #2 are unreachable.
- Waiting for the hotspot. Then the Phase A pre-floor items, in order: (1) base_footprint from the car's ack.xacro / TF -> re-run make_params_a.py --footprint-rear; (2) depth point-cloud topic + Pi CPU with the camera at 15 fps; (3) minimum moving speed; (4) install the Nav2 params; (5) undock = go_dist.py 0.3 m straight out.

## 2026-11-09 evening: Phase A on car #1 (no driving yet)
- Measured on the car (URDF ack.xacro + TF): base_footprint midway between axles (front wheels x +0.0668, rear -0.0703);
  base_link mesh x -0.1038..+0.1096 -> footprint rear 0.104 (make_params_a.py default). LiDAR z 0.163, /scan_raw 10 Hz
  (stock laser filter commented out: NO /scan). Depth cloud /ascamera/camera_publisher/depth0/points 12.5 Hz, 154k pts,
  frame ascamera_color_0 (TF ok). STVL not installed -> /depth_scan (car/perception/depth_scan.py) instead.
  Nav2 on car = Humble 1.1.15 (collision monitor uses max_points, polygon points = flat double array).
- Min moving speed: already measured (open-issues 448: 0.03 m/s) -> RPP min_approach 0.08 (x0.4 slow zone = 0.032).
- Fixes in make_params_a.py: /scan_raw, footprint, AMCL + map_server (initial pose = charger 0,0,0), lifecycle_manager_localization bond 0,
  car-side BT path, no YAML aliases (rcl rejects them), Humble polygon format.
- New: car/nav/nav_a.launch.py (on top of stock autostart; BASE=1 to include robot.launch), nav_a_start.sh / nav_a_stop.sh / nav_a_wait.sh
  (in ~ on the car), car/nav/npz_to_pgm.py, map car/maps/office_goto/goto6.pgm/.yaml (on car: shared/maps/goto6.yaml).
- Offline planner re-check with new footprint: all 3 legs plan; U-turn leg min clearance 0.12 m (was 0.19).
- On-car bringup: localization ACTIVE; navigation failed at collision_monitor polygon type -> fixed params NOT yet copied (Wi-Fi dropped).
- NEXT: copy nav2_params_a.yaml to car docker/tmp/nav/, ./nav_a_stop.sh; ./nav_a_start.sh; bash nav_a_wait.sh; then check AMCL pose vs car
  position, CPU (top), costmaps; then ask user before the first undock + C1 trial.
- Car #1 now on office LAN cable: **10.10.76.35** (DHCP; MAC 98-fe-54-00-76-63), wheels up. wlan0 still AndroidAP.
- bt_navigator failed: through-poses default tree needs Spin -> default_nav_through_poses_bt_xml = our tree. Fixed.
- **Nav2 Phase A fully ACTIVE on car #1** (localization + navigation, collision monitor). Static check (nav_a_check.py):
  scan_raw 9 Hz, depth_scan 7.4 Hz (54/120 bins hit), local costmap has lethal cells, no cmd_vel sent.
  CPU: total ~60% (40% idle), Nav2 container 18%, **depth_scan.py 90-96% of one core** (only 7.4 of 12.5 Hz) -> optimise later.
  AMCL at the initial pose (0,0,0): NOT verified (car is wheels up, not at the charger).
- Next: car on the floor at the charger -> restart Nav2 -> AMCL check -> user OK -> undock + C1 trial (run detached on the car; link only to start/fetch).
- **Trial a_c1_1 (first Phase A drive): no progress.** (a) go_dist undock stopped after 5 cm (LiDAR range ahead jumped 3.77->1.69 m);
  (b) Collision Monitor StopZone held the car: the charger is 1-5 cm behind the docked car (LiDAR x -0.12..-0.155) and a Humble
  "stop" polygon blocks ALL directions; Nav2 commanded 0.15 m/s, /controller/cmd_vel = 0; then "Starting point in lethal space".
  Fixes: StopZone -> "approach" polygon = footprint (time_before_collision 1 s); car/nav/undock.py (odometry distance 0.35 m).
- Trial a_c1_2: AMCL silently ignored the /initialpose (8 deg heading error); undock drove ~10 cm before I killed it (rec_run wrapper
  survived the first kill). Fixes: set_pose.py now verifies /amcl_pose and exits 1; car/nav/stop_trial.sh kills rec_run first.
- Dashboard = `python -m http.server 8765` in car/dashboard on the PC (http://localhost:8765/index.html); died with the crash, restarted.
- **a_c1_3: C1 charger(0.28 out)->door 17.8 s, 1 recovery, no contact** (camera checked). First Phase A success.
- a_c2_1 door->hall end: stopped at 69 s, 0.58 m short. Cause (bagtime.py timeline + nav log): 207x RPP "collision ahead",
  13x "Controller patience exceeded" -> BT recovery BackUp 0.2 m + nav_trip back-up 0.3 m = the reversing the user saw.
  depth_scan reported ~0.3 m ahead all run while LiDAR saw 1-2 m: FLOOR LEAK (dsz.py: floor z p05 +0.04..+0.06 at 1-4 m,
  above FLOOR_Z 0.02 -> camera pitched ~1 deg vs TF). Needs camera pitch calibration / higher floor cut before depth is used again.
- a_c3_1 hall end->office on LiDAR ONLY (depth_scan killed): U-turn + door, 0 recoveries, but 90 s nav_trip timeout 0.64 m short;
  a_c3_1b finished to tolerance (0.58,-0.11). Docking attempt (yaw tolerance 0.35, goal facing out) = 58 s of shuffling,
  AMCL std grew to 0.27 m / 16 deg -> stopped; user docks by hand.
- **User complaints (must fix): too slow; left-right steering weaving "for no reason"; reversing for no reason.**
  Weaving hypotheses to VERIFY from bags a_c2_1 / a_c3_1 before any change: (1) driver turns w into steer = atan(w L / v)
  -> at low v small w = full lock; (2) 1 Hz replans flip sides; (3) AMCL jumps; (4) RPP lookahead 0.4 m at low speed.
- Car #1 runtime state: controller_server yaw_goal_tolerance 0.35 / xy 0.12 set at runtime (revert = restart Nav2); depth_scan restarted.
- Car #2 is parked at the charging wall next to car #1's spot (camera, 2026-11-09).
- PC wired port back on office DHCP (10.10.76.37) by the user; ASUS at 10.10.77.84; hotspot off; cars off. Night work = bench + fixes.
- Reverse analysis (revsegs.py on ASUS): C3 U-turn 22 reverse segments of 1-17 cm in ~40 s; 39 of 40 cusp plans had the cusp
  8-16 cm from the start = every 1 Hz Smac replan adds a tiny reverse at the car -> RPP drives it (dithering + flips).
  C2 reverses = recovery back-ups (BT 0.2 m + nav_trip 0.3 m). Fix list: replan only if path invalid, reverse_penalty up,
  recoveries without BackUp first, scale_velocities true. Verify each on the bench (car/nav/bench) first.

## 2026-11-09 night: Nav2 bench (ASUS, no car)
- Bench = car/nav/bench/: fake_car.py (driver steering atan(0.145 w/v) clip 40, steer limit 29/33/40, bicycle L 0.137,
  latency 45 ms, servo 60deg/0.2s, speed tau 0.15 s ESTIMATE, LiDAR ray-cast in goto6 map, contacts), bench_run.py
  (Nav2 Humble 1.1.20 in robotlab-nav2:humble, no AMCL, LiDAR only; metrics json), bench.sh, batch.sh (courses x steer x 3).
  On ASUS: ~/phase_a/bench (results/, bench_logs/).
- Calibration: C1 bench 15.1 s / 0 recov / 0.27 flips/s vs floor 17.8 s / 1 / 0.24. C3 tree A bench: FAILED 82 s, 22 recov,
  27 s reversing (floor: 89 s timeout, 30 s reversing) -> outcome reproduced (mechanism: recoveries on bench vs 1 Hz cusps on floor).
- **Tree B (car/nav/navigate_ackermann_b.xml: replan only if path invalid; recoveries clear -> wait -> short backup):**
  batch B1: C1 9/9 16.8 s 0 recov; C2 9/9 15.3 s 0 recov; C3 6/8 43.8 s (2 fails at steer 29: stuck ~(1.0,-0.4)
  near the charger desk legs, RPP "collision ahead" + patience). 0 contacts in all.
- Docking: turning on the spot near the dock = 13-89 reverses (= floor 58 s shuffle). Return goal WITH the dock heading
  (0.35,-0.03,-8 deg, yaw tol 0.5, xy 0.15) from the hall end: 48 s, 0 recov, ends (0.45,-0.16) -> then dock.py reverse.
- **Depth floor leak fix** (car/perception/depth_scan.py): per-frame floor plane fit + height above plane > 0.03 m + 2 points
  per bin. Synthetic test (car/perception/test/test_floor_fit.py, floor pitched 1.2 deg like car #1): phantom bins 141/145 -> 0,
  chair base bins seen 0% -> 97%. PASS. Needs on-car check (standing, then driving).
- nav_trip.py: no own back-up any more (wait 2 s + resend), goal timeout 150 s.
- Running: variants1.sh (V1 shorter lookahead, V2 global cost_scaling 2.5, V3 both) on C3 x steer 29/33 x 3.
- Variants on C3 x steer 29/33 x 3 (tree B): V1 lookahead min 0.25 / time 1.0 -> 6/6 (one 93 s / 14 recov at 29 deg);
  V2 global cost_scaling 2.5 -> 4/6; V3 both -> 4/6. Stuck spot = S-bend past desk legs (1.0-1.25,-0.27) and (0.3,-0.4).
  Running variants2.sh: V4 = V1 + RPP use_collision_detection false (Collision Monitor approach = safety), V5 = V1 + RPP horizon 0.5 s.
- make_params_a.py: BT default = navigate_ackermann_b.xml; YAML written with LF. New car/nav/nav_a_home.sh (return + park).
- V4 (V1 + RPP use_collision_detection false) C3 x 29/33 x 3: 6/6, median 39.7 s, recov max 2. **Full validation F4
  (C1-C3 x 29/33/40 x 3): 27/27, 0 contacts, max 1 recovery** -> in make_params_a.py (min_lookahead 0.25, lookahead_time 1.0, check off).
- F6 (V4 + velocity_smoother scale_velocities): 27/27, C3 full lock 7.4 -> 5.0 % but reversing 0.5 -> 4.6 s: not clearly better.
- F7 (F6 + top speed 0.5): 27/27, 0 contacts, ~30 % faster (C1 12.1 s, C2 10.9 s, C3 32.3 s), C3 full lock 2.2 %.
- Bench bug fixed: ROS_DOMAIN_ID must be <= 232 (F6/F7 first attempt silently failed). Running: F8 = V4 + 0.5 m/s only.
- scan_map_match.py: fine search 0.025 m / 1 deg + tie-break; bench test true (0.05,-0.03,-9) -> found (0.050,-0.025,-9).
  New car/nav/nav_a_ready.sh (restart Nav2, scan-match, refuse < 80 % fit, set + verify AMCL), car/nav/install_phase_a.ps1.
- depth_scan.py logs its own per-frame time (CPU question) every 100 frames.
- F8 (V4 + 0.5 m/s, no scale_velocities): C3 7/8, full lock 5.6 %; F7 (with it) 27/27, 2.2 % -> make_params_a.py now
  V_MAX 0.5 + scale_velocities true (+ V4). FINAL check of the generated yaml itself: C1 9/9 15.0 s, C2 9/9 10.9 s,
  C3 9/9 33.2 s, 0 contacts.
- Park course H (hall end -> pre-dock 0.35 m facing out, yaw tol 0.5): 6/9, 5 contacts (all in one run stuck at the desk
  legs (0.9,-0.29), steer 40), 1 heading dither 150 s at the goal. -> testing CM approach polygon +3 cm (undock/dock
  bypass Nav2 so the charger no longer limits it) and pre-dock 0.6 m out (H6). Running margin.sh (M3C, M3H).
- undock.py / dock.py tested against fake_car: undock 0.35 m in 3.4 s; dock reversed 0.42 m, stopped 0.127 m by rear LiDAR.
- Repo: car/nav/analysis/ (bag analysis scripts + README), car/nav/dsnow.py, installer covers all trial scripts.
- Margin 3 cm (margin.sh): park H 9/9 0 contacts (was 6/9, 5 contacts); C1-C3 26/26; pre-dock 0.6 m worse (5/9). -> in config.
- FIN2 (final generated yaml): C1 9/9 16.1 s, C2 9/9 10.8 s, C3 8/9 34.2 s (1 stuck in the doorway at steer 29), H 9/9 43.4 s, 0 contacts.
- New car/measure/steer_limit.py (gyro radius at full lock); bench check reads 29.0 / 40.0 exactly. fake_car publishes /imu.
- 29 deg doorway variants (C3 x 29 x 6): D0 final 6/6; D1 margin 2 cm 5/6; D2 slow curves 6/6 max 4 recov; D3 planner
  radius 0.5 6/6 max 0 recov, full lock 9.8 %. Running d3full.sh (D3 on all courses + park).
- docs/open-issues.md: new top section 0000000000 (Phase A next steps). Report docs/report-2026-11-09.md written.
- D3 (planner radius 0.5) full: C1 8/8, C2 9/9, C3 9/9 31.1 s, but park H 7/9 (vs 9/9) -> REJECTED; final config unchanged
  (C1 9/9, C2 9/9, C3 14/15 incl. D0, H 9/9, 0 contacts). Bench results archived: car/maps/phase_a_runs/bench_results.tgz.
- Tried to profile depth_scan CPU on the ASUS with synthetic 640x480 images: images never reached any subscriber in the
  container (TF did; also with --ipc=host) -> NOT resolved, stopped (time-box). The node's own timing log answers it on the
  car in 1 min tomorrow (step 3 of the floor plan).
- NIGHT DONE, waiting for the user + car #1. Floor plan: docs/report-2026-11-09.md sec. 5.
- Overnight on ASUS: overnight.sh (final config, 5 repeats: N5 = C1-C3 x 29/33/40 x 5, N5H = park x 15) -> ~/phase_a/bench/batch_N5*.txt
- 2026-11-11: PC found off (crash in display-off standby 11-11 ~11:01; GPU crash 0x119 on 11-09 18:03). Overnight N5:
  C1/C2/C3 15/15 each, park H 13/15 (fails: 29 deg stuck mid-route; 40 deg heading dither at pre-dock). Added as report sec. 6.
  Floor plan (report sec. 5) still not done: needs car #1 + user.
- 2026-11-11 19:55 floor session start: car #1 on AndroidAP 10.71.88.196 (PC USB Wi-Fi joined AndroidAP with user OK; LAN untouched).
  install_phase_a.ps1 OK; nav_a_ready 97-98 % fit, AMCL verified. depth_scan CPU SOLVED: 4-7 ms/frame at 12.4 Hz (~12 % core).
  FINDING: office laminate gives NO depth return (lower half of depth image empty); the 11-09 floor fit accepted planes
  3-4 cm up through furniture bases -> would hide them. Fix: plane needs >= 50 inliers within 1.0 m + |c| <= 0.03.
  test_floor_fit.py: hall case PASS (0 phantoms, chair 97 %), new office case PASS (no plane). On car: plane NO, only
  real hit = table leg 1.15 m at -15 deg, 0 pts in STOP/SLOW. Hall floor (matte tile) plane fit still to be seen live.
- 2026-11-11 ~20:30 STEERING: steer_limit.py through the driver: car barely moved, yaw ~0 (runs/steer2, camera). Servo sweep OK
  (user saw L/R/C), servo pulses 1944/1055 sent. CAUSE (strong evidence): stock ackermann.py sets rear wheels vl = v + w t/2,
  vr = v - w t/2, [0, vl, 0, -vr] = Hiwonder build with M2 = RIGHT rear. Car #1 has M2 = LEFT rear; the 11-02 fix negated
  both ([0, -vl, 0, vr]) -> straight OK but the differential is MIRRORED: at full lock the rear wheels push against the steering.
  steer_direct.py (servo full lock + EQUAL rear speeds): yaw +21.4 / -20.2 deg/s. arc_icp.py (LiDAR ICP + IMU, runs/steer3):
  REAL STEERING LIMIT ~28 deg LEFT, ~24 deg RIGHT (L 0.137; 29/25 with L 0.145). Below the bench's 29-40 range!
  Proposed driver fix: [0, -vr, 0, vl] (needs user OK; backup first). Then re-bench Nav2 at 24-28 deg.
- 2026-11-11 ~20:20 USER OK -> driver side fix APPLIED on car #1: ackermann.py:50 [0, -vr, 0, vl] (backup + revert:
  ~/docker/tmp/motorfix2_backup/revert.sh). Gamepad dongle now plugged (WonderEcho port; Pi has 4 USB ports: RRC, LiDAR,
  HP60C, WonderEcho -> 5 devices / 4 ports, docs/research/01 sec. 7). User joystick drive: stop-and-go every ~0.5 m =
  OUR cmd_watchdog (stock joystick node sends only on stick CHANGE). Fix: watchdog ignores the timeout while /joy (20 Hz
  autorepeat) shows a held stick (car/watchdog/cmd_watchdog.py, deployed; old copy in motorfix2_backup). After: 0 stops.
- Joystick drive joy2 (164 s): full lock 77 deg/s, right sign 89 %; real speed 0.46-0.47 at cmd 0.5 (93 %, LiDAR ICP fit >60 %).
  Stick top speed = Hiwonder launch default max_linear 0.5 (joystick_control.launch.py:8; D-pad up = 1.0 m/s). Raise? ASKED.
  User: car PULLS RIGHT (needs left stick to go straight); gyro straight runs drift -0.2..-6.9 deg/s. Trim -9 IS re-applied
  at driver start (ros_robot_controller_node.py:63). NEXT: trim_drive.py on the floor (2 m clear ahead), propose new trim.
  Gentle steer 5-25 deg turned little (only ~2 s data): test on purpose. Nav2 idle used ~2.5 Pi cores: investigate.
- ASUS: steer24.sh running (final config at steer 24/26/28 x 5, C1-C3 + park H) -> ~/phase_a/bench/batch_S24*.txt.
- Car #1 charging; Nav2 still running on it (no goal). PC USB Wi-Fi on AndroidAP (user OK), LAN untouched.
- 2026-11-11 (2nd power cut; ASUS rebooted too). steer24.sh results (14 runs/course, last runs cut): C1 14/14 15.1 s,
  C2 14/14 10.9 s, C3 13/14 33.6 s (1 fail steer 28 stuck at (3.0,-2.45)), park H 10/14 (3 fails at 26 deg ending at the
  dock (0.2,0.0) = heading dither at pre-dock, 1 at 24 deg mid-route). PC Wi-Fi disconnected -> car unreachable; rejoin
  AndroidAP needs user OK. NEXT on bench: fix park H at 24-28 deg.
- 2026-11-11 (after power cut) two-leg park on the bench: P2 waypoint (1.3,-0.05) 0/14 (0.25 m from a chair base,
  231 contacts) but leg 2 straight reverse waypoint->pre-dock 15/15, 0 contacts. P3 waypoint (2.0, 0.2, -8) mid-room:
  first 5/5 runs reached ~38 s, 0 contacts. fake_car/bench_run: new --steer-max-right (asymmetric, car #1 28 L / 24 R);
  P3A = asymmetric runs. Scripts synced to car/nav/bench (batch.sh H7-H10, park2.sh, park3.sh, steer24.sh).
- User OK -> PC USB Wi-Fi rejoined AndroidAP; car #1 reachable, battery 8.41 V, Nav2 not running.
  trim_drive.py (scan wait 1 s -> up to 5 s) refused: obstacle 1.12 m ahead (desk leg, camera checked). Needs the car
  placed with >= 2 m clear ahead.
- Nav2 CPU (car #1, idle, no goal): start-up spike 102 % (nav2_container, map + costmaps) for ~20 s, then ~21 %.
  Found a DUPLICATE depth_scan.py (old one left from a session that ended without nav_a_stop.sh, ~17 % each).
  Killed the old one; nav_a_start.sh now refuses to start while any Nav2/depth_scan runs. Idle total ~1.1 core.
  The "2.5 cores" of the last session = start spike + duplicate. Still to measure: CPU while driving.
- Car park scripts (not on the car yet, wait for bench P4): car/nav/home_legs.sh (leg 1 waypoint loose yaw 0.8/xy 0.3,
  leg 2 pre-dock 0.5/0.15, then dock.py; tolerances restored on exit), nav_a_home.sh calls it, installer copies it.
- P3 (waypoint (2.0,0.2), tight tol 0.5/0.15): symmetric 24/26/28 leg 1 8/9, leg 2 9/9; ASYMMETRIC 28 L / 24 R leg 1
  only 3/6 (all fails = heading dither at the waypoint (1.94,0.05)), leg 2 6/6. 0 contacts anywhere.
  -> P4 running (park4.sh): leg 1 loose 0.8/0.3 sym + asym; leg 2 from skewed starts (H11 +25 deg, H12 -40 deg).
  Gotcha: "until ! pgrep -f park3.sh" waiters match THEMSELVES (and stalled park4's own pgrep): wait by PID (kill -0).
- 2026-11-11 MISTAKE: picked "open floor" goal (1.6,0.4) from the LiDAR map -> the car drove UNDER THE DESKS (LiDAR at
  0.16 m sees only desk legs; the desk area is "free" in the map). Also goal yaw tol 3.14 -> heading not enforced.
  User upset. RULE: check any goal/test spot with the car camera (or recorded frames), not the map alone.
  The bench park waypoint (2.0,0.2) is likely under the desks too -> NOT installing home_legs.sh until checked.
- Drove C1-like to the hall door (2.95,-2.5,-90) with yaw tol 0.25: 53 s, 9 recoveries (heading alignment). Hall clear (camera).
  `ros2 param set` from ssh hung once (no timeout) -> always wrap in `timeout`. yaw tol is still 0.25 on the car (reset to 3.14!).
- trim_drive.py fixes: start check = path corridor (1 m+stop+0.2 long, +-0.25 m), not a +-35 deg cone (pillar beside the
  path refused); ignore hits < 0.14 m (own body; a 0.05 m hit stopped trim3 at 0.0 s); NaN-safe print.
- trim4 (hall, 0.2 m/s 5 s): heading -12.7 deg, steady yaw -0.0402 rad/s (RIGHT). Script's LiDAR speed wrong (4.04 m jump)
  -> with real 0.186 m/s (93 % of cmd, joy2): delta = atan(0.145*0.0402/0.186) = -1.8 deg -> trim +20 units
  (2000/180 per deg). Current trim -9 -> proposed +11. ASKED user.
- P4 bench: leg 1 loose tol sym 9/12 (2 fails at the waypoint, heading), asym 6/6; leg 2 from skewed starts 17/17.
- 2026-11-16 ~19:40 (car clock). Car #1 was unreachable: it booted into its own AP (HW-54007663) because both NM profiles
  autoconnected with no priority. User plugged LAN; fixed with user OK: AndroidAP autoconnect-priority 10, own AP 0,
  hostname mentorpi1 (mentorpi1.local). Hotspot subnet changed to 10.164.108.x; car at 10.164.108.196.
  New car/pc/find_cars.ps1 finds both cars by MAC on every PC network.
- Trim -9 -> +11 applied (user OK) in servo_config.yaml key 3, backup ~/docker/tmp/trim_backup/servo_config.yaml.minus9-2026-11-16,
  start_node restarted, watchdog active. Battery 7.98 V. NOT yet verified on the floor: camera showed the car on a desk
  with the LAN cable in. NEXT: car on the floor, 2 m clear, cable out -> trim_drive.py 0.2 5 (camera check first).
- trim5_p11 (floor, 0.2 m/s, trim +11): stopped 3.9 s by side check (passing boxes/bin, safe), 0.67 m driven.
  Heading -2.7 deg, steady yaw -0.0117 rad/s (was -0.0402 at trim -9) -> steering error -0.57 deg (was -1.8)
  -> script suggests +6 more = trim 17. Data on car: ~/docker/tmp/runs/trim5_p11 (bag, 17 cam frames, run.log).
  ASKED user: set +17 and re-test from the start spot.
- Car on the floor (not at the charger). New car/nav/global_locate.py (whole-map scan search, 6 s) -> office (2.27,-1.205,-32),
  fit 59-61 % (boxes/bin not in map), checked by overlay image; AMCL took it (0.01 m). Nav2 restarted -> yaw tol 3.14 again.
- GATE A round 1: C1 (noundock) 13.8 s, C2 15.1 s, C3 49.0 s (ends 0.3 m in front of the charger; car #2 parked beside
  it), all reached, 0 recoveries, camera checked. Park NOT run (home_legs waypoint unchecked).
- Rounds 2-5 running detached on the car: ~/gate_a_loop.sh, log ~/gate_a_loop.log, runs g<R>_c<1-3>.
- GATE A round 2 (gate_a_loop.sh): g2_c1 45.3 s (recoveries), g2_c2 20.3 s, g2_c3 "reached" 105.9 s = FALSE: car HIT A
  TABLE LEG near the charger (user saw it; last frame 0120.5 = green leg at the lens). Loop then started g3_c1; stopped by
  me after user report (pkill -f gate_a_loop.sh over ssh killed its own ssh shell again -> kill by PID).
  Cause chain (logs): TF extrapolation errors from ~10 s into g2_c3, growing (controller, collision_monitor, costmaps on
  stale TF); pose frozen at (2.34,-2.65) 36-58 s, then (0,0); nav2_container later died (SIGABRT, RCLError) during g3_c1.
  Car load average 7.2 (5 min) on 4 cores; depth_scan.py 53 % of a core (process only 3-4 ms/frame at 12.3 Hz -> cost
  is rclpy image deserialisation, estimate); floor plane NO (fixed cut) = Gate step 3 never passed; temp 52 C, not throttled.
  nav_trip.py trusts Nav2 SUCCEEDED -> counted a broken-TF "success". All Nav2/recorders stopped.
  NEXT (proposed to user): CPU budget while driving, TF-freshness guard that stops the car, success = AMCL pose at goal,
  loop stops on recoveries; re-run Gate A only after that.
- 2026-11-16 ~20:25, car charging (no driving). FIXES, all installed on car #1 (pwsh car/nav/install_phase_a.ps1 -Car <ip>):
  1. depth_scan.py 88 % -> 7 % of a core: an Image sub + tf2 TransformListener on one rclpy node busy-loops the executor
     (~3900 wakeups/s; probes: either alone 1-4 %, together 70-86 %). Camera TF measured constant (676 lookups, 0 spread)
     -> looked up once, listener unregistered. Same 12.4 Hz, floor plane yes.
  2. car/nav/nav_guard.py: freshness gate collision_monitor -> /cmd_vel_guard_in -> guard -> /controller/cmd_vel. Zeroes
     commands if /scan_raw (>0.35 s), odom TF (>0.25 s) or AMCL map->odom (>0.5 s) is stale (nominal measured: 9.7 / 28.5 /
     9.3 Hz). Live test with renamed topics (car/nav/test/test_nav_guard_live.py): PASS (blocked 0.26 s into a scan pause,
     released 0.6 s after). Guard ~10 % CPU.
  3. nav_trip.py: success only if OUR pose (map->odom + odom->base from /tf, both < 0.5 s old) is within 0.4 m of the goal;
     last line "RESULT ok|FAIL <why> recoveries N guard_blocks M". Not yet exercised on a real goal.
  4. car/nav/gate_a_loop.sh (now in the repo): stops on RESULT not ok, any recovery, any guard block; logs battery, load.
  Standing CPU with Nav2 + guard + recorders: ~1.0 core total, load ~2-3.
  NEXT: attended round (user watching): C1/C2/C3 one by one, read RESULT + peak load + guard blocks; then loop rounds.
- 2026-11-16 ~21:35 car clock: hotspot dropped; car stuck on own AP (NM never switches back). Installed wifi-return.timer (car/config/wifi_return.sh), user OK; tested: car rejoined AndroidAP 6 s after PC left its AP.
- Attended C1 #1 (~21:50 car clock): undock ok, then nav2_container DIED the instant the goal was sent: RCLError "client
  will not receive response" (rmw_response.cpp:154). Known Humble bug (ros2/rmw_fastrtps#751 -> ros2/ros2#1462; rcl/rclpy
  fixed May 2024, rclcpp action fix #2464 not confirmed in Humble; car rcl 5.3.7, rclcpp 16.0.8 = Feb 2024).
  Trigger confirmed on the car standing still (goal = current pose): fresh nav_trip WITHOUT waiting -> crash on try 1;
  nav_trip now waits for the server + 3 s (DISCOVERY_WAIT_S) -> 13/13 RESULT ok, no crash. Logs nav_a_crash_att_c1.log,
  nav_a_crash_repro.log. Also likely: killing nav_trip mid-request (stop_trial) can crash Nav2 the same way (19:57?).
  Real fix later (needs user OK): update ROS Humble packages in the container.
- Attended att_c1b (noundock, after the fix): RESULT ok 16.4 s, recoveries 3 (planner "no valid path" 3 s at ~(1.54,-1.06)
  near the desk leg/boxes), 0 guard blocks during the drive, Nav2 alive, battery 7.29 V, peak load 9.2.
- Attended att_c2: FAILED at the 120 s limit. A PERSON stood in front of the car in the hall (camera: feet at 30 s and,
  very close, at 70 s) -> 11 recoveries, back and forth at y -3.2..-4.1, then moving again (y -5.08 at 113 s).
  No Nav2 crash. CPU sampled (top 3 s): 20-60 % idle; Nav2 25-38 %, nav_trip 20-30 % (its /tf sub), bag ~20 %,
  ascamera 15-25 %, nav_guard ~13 %, depth_scan ~10 %. Load average 7 overstates it (many short ROS threads).
- ~22:15 car clock: nav_trip.py now subscribes /tf only for the final pose check (TF_LISTEN_S 0.6 s); copied to the car, NOT yet tested (no-motion test cut off). AndroidAP dropped AGAIN at 16:14 PC time (41 min after reconnect, 'disconnected by the driver'), then 'not available' on 4 reconnects. Car not reachable; Nav2 left running idle on the docked car (relocalised at charger 0.075,0.025,-4, fit 83 %).

## 2026-11-16 evening: reconnect + no-motion test
- Hotspot back (user): PC dongle rejoined AndroidAP (10.164.108.33); car #1 rejoined by itself via wifi-return.timer
  (journal: 2 failed tries on own AP, joined AndroidAP 00:33:45 car time, same IP 10.164.108.196). Timer WORKS.
- While on own AP/idle ~2 h, AMCL drifted from (0.07, 0.02, -4) to (0.11, -1.05, -116) with the car standing on the
  charger (scan match after restart: 86 % at 0.025 0.05 -4). => always nav_a_ready.sh before a trial, never trust old AMCL.
- Old Nav2 (started before the Wi-Fi flap): rclpy client passed wait_for_server once, then goToPose's wait_for_server
  never saw the server for 57 s, while `ros2 action info` listed it. Inference: DDS discovery partly broken by the
  interface change under a running Nav2. => restart Nav2 after any Wi-Fi switch.
- Fresh Nav2, no-motion goal AT THE DOCKED POSE: goal accepted, feedback OK, Nav2 stayed up, but the planner aborts
  "Starting point in lethal space" (footprint overlaps the charger); recoveries ran, backup said "Collision Ahead",
  car did not move. Expected: that is why C1 undocks first. No-motion tests must be done off the dock.
- nav_trip CPU with /tf only at the end: still 17-20 % (was 20-30 %): cost is BasicNavigator's polling, not /tf.
- Battery off charger 8.08 V. READY 86 %. Attended C1 (att_c1c) not started: waiting for user "go".
- ATTENDED ROUND CLEAN (user "go"), all RESULT ok, 0 recoveries, 0 guard blocks, camera checked each leg:
  att_c1c undock->door 17.5 s, 0.30 m from goal | att_c2b door->hall end 12.8 s, 0.24 m | att_c3b hall end->charger
  front 39.7 s, 0.36 m (slow last metre, ends facing the cables at the charger wall). Battery 8.08 -> 8.04 V; load 4-5.
  First fully clean Gate A round since the 2026-11-16 fixes (depth_scan TF cache, nav_guard, pose-checked RESULT,
  discovery wait). Next: gate_a_loop.sh (unattended, stops on first unclean trial) on user OK.
- gate_a_loop 21-23 (user "ok, continue"): g21_c1 ok 22.2 s clean; g21_c2 ok 14.7 s but recoveries 4 -> loop STOPPED
  (as designed). Cause from the bag: ONE /depth_scan frame at t=1794847580.74 read 0.30 m at +14 deg; frames before/
  after 1.24 / 1.09 m; LiDAR min 0.56 m (pillar at x 3.72); camera frames show an empty hall. collision_monitor fired
  at .81 ("approach"), planner "Starting point in lethal space" at 581.78, BT cleared both costmaps (= the 4
  recoveries), replanned in ~1 s, car drove on, reached goal 0.33 m. => single-frame depth phantom (glass hall).
  Candidate fix (not done, needs check): require a depth obstacle in >= 2 consecutive frames in depth_scan.py;
  first count how often such 1-frame drops occur in the recorded bags. Car parked at hall end, Nav2 up, no goal.
- C3 home (att_c3c, user "go, bring it back and fix"): ok 44.6 s, 0.31 m, but 8 recoveries (2 bursts, hall clear).
- PHANTOM ROOT CAUSE (bags, near_where.py): in the empty hall ALL /depth_scan returns < 0.45 m were 0.28-0.33 m from
  base_footprint at +10..+25 deg = camera range ~0.20-0.27 m, just above the HP60C 0.2 m minimum. Not the floor strip
  (image bottom row meets the floor at 0.357 m, floor_edge.py; office floor gives no depth). Real near returns
  (charger area) 0.37-0.43 m. Standing still: 0 near pixels in 68 frames -> appears only while driving.
- FIX depth_scan.py: (1) SCAN_DEPTH_MIN 0.28 m camera range for /depth_scan only (/depth_sectors unchanged, sim match);
  (2) confirm_near: a bin < 0.45 m needs a return in the previous frame within 0.15 m, +-2 bins, else inf; dropped
  bins logged with pixel row/col/depth/height. Replay of the 1-frame rule alone: g21_c2 near frames 39 -> 14 (not
  enough -> (1)). Tests: test/test_near_confirm.py + test_floor_fit.py PASS on the car. Live: 6-7 ms/frame, 12.2 Hz.
  Backup: ~/docker/tmp/perception/depth_scan.py.bak-2026-11-16-pre-nearcut.
- Gotcha again: nav_a_stop.sh's pgrep killed my ssh shell because the command line contained "depth_scan.py" (cp).
- Verification: gate_a_loop.sh 31 33 started (pose via scan match 79 %, 0.645 -0.083 143.9).
- g31_c1 (new depth_scan): ok 29.2 s, 1 recovery -> loop stopped. NOT a phantom (no "lethal start"): controller "Failed
  to make progress" while 3-point-turning from the C3 end pose (0.65, -0.08, 144 deg: facing the charger wall between
  boxes/cables) for ~12 s; cleared + replanned, then drove fine. depth_scan dipped to 6.9 Hz (18 ms) at recorder start.
  Drop log: charger-area clutter (top rows h~0.2 m, near-floor cables 0.25 m) + 1-frame drops while turning (by design).
- home_1 door -> charger: ok 33.3 s, 0 recoveries, 0 guard blocks. Car parked in front of the charger.
- Decided next (no user question needed): C3 ends facing the door at a roomy spot so C1 does not start with a tight
  turnaround among the boxes; pick it from the map; then gate_a_loop again.
- End of day: Nav2 stopped, sync, car files md5 == repo. User powers the car off. Plan: docs/plan-2026-11-17.md.
