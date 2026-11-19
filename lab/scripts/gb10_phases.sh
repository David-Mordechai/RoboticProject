#!/bin/bash
# Supervised training on the GB10, one phase after another (2026-11-04: A = turnaround v2, B = explore v6).
# Per phase: warm start from a checkpoint, resume from the newest checkpoint after a crash, a check at every new
# checkpoint (logged to goals_<name>.log), stop at END (rsl_rl's last save is model_<END-1>) or when the explore goal
# check says ALL GOALS MET, then export ONNX of the last checkpoint. Temperatures every minute.
#   setsid nohup bash ~/RoboticProject/lab/scripts/gb10_phases.sh > /dev/null 2>&1 < /dev/null &
#   touch ~/RoboticProject/runs_state/phases/STOP      -> stop training and the supervisor on purpose
cd ~/RoboticProject
# A second supervisor can run one phase in parallel:  PHASES_DIR=runs_state/phases_b ONLY_PHASE=<name> bash ...
S=${PHASES_DIR:-runs_state/phases}; mkdir -p $S; LOG=$S/supervisor.log
exec 9>$S/supervisor.lock; flock -n 9 || { echo "already running"; exit 0; }
say() { echo "$(date '+%F %T') $*" >> $LOG; }
thermal() {
  G=$(nvidia-smi --query-gpu=temperature.gpu,power.draw,clocks.sm,clocks_event_reasons.hw_thermal_slowdown,clocks_event_reasons.sw_thermal_slowdown --format=csv,noheader)
  B=$(cat /sys/class/thermal/thermal_zone*/temp | sort -n | tail -1); B=$((B/1000))
  echo "$(date '+%F %T') gpu: $G | board max ${B}C" >> $S/thermal.log
  T=$(echo "$G" | cut -d, -f1 | tr -d ' ')
  if [ "$T" -ge 85 ] || [ "$B" -ge 90 ] || echo "$G" | grep -q ", Active"; then say "WARNING hot: $G board ${B}C"; fi
}

# phase NAME TASK EXP INIT_RUN INIT_CK END CHECK(turn|explore)
phase() {
  local NAME=$1 TASK=$2 EXP=$3 INIT_RUN=$4 INIT_CK=$5 END=$6 CHECK=$7
  local C=robotlab-train-${NAME//_/-} P=$S/$NAME RUN IT
  mkdir -p $P
  [ -n "$ONLY_PHASE" ] && [ "$NAME" != "$ONLY_PHASE" ] && return 0
  [ -f $P/done ] && return 0
  latest() {
    for d in logs/rsl_rl/$EXP/$INIT_RUN logs/rsl_rl/$EXP/*_$NAME; do
      [ -d "$d" ] && ls $d/model_*.pt 2>/dev/null | sed "s#.*model_\(.*\)\.pt#$(basename $d) \1#"
    done | sort -k2 -n | tail -1
  }
  say "phase $NAME start"
  while [ ! -f $S/STOP ]; do
    thermal
    read RUN IT < <(latest)
    if [ "$IT" -gt "$INIT_CK" ] && ! grep -qx "$IT" $P/checked 2>/dev/null; then
      sleep 5; echo "$IT" >> $P/checked
      echo "=== checkpoint $IT $(date +%T)" >> goals_$NAME.log
      if [ "$CHECK" = explore ] || [ "$CHECK" = goto ]; then
        G=lab/scripts/tmp_goal_check.py; [ "$CHECK" = goto ] && G=lab/scripts/tmp_goto_check.py
        docker compose run --rm -T lab python $G logs/rsl_rl/$EXP/$RUN/model_$IT.pt $TASK 2>&1 \
          | grep -a "PASS\|FAIL\|person contacts\|GOALS\|NOT YET" >> goals_$NAME.log
        if tail -1 goals_$NAME.log | grep -q "ALL GOALS MET"; then say "$NAME goals met at $IT"; docker stop $C; break; fi
      else
        docker compose run --rm -T lab python lab/scripts/evaluate_turnaround.py --checkpoint logs/rsl_rl/$EXP/$RUN/model_$IT.pt \
          --obs-noise 2>&1 | grep -a "^all\|^width\|^rear\|^chair\|^open\|runs, first" >> goals_$NAME.log
      fi
      continue
    fi
    if ! docker ps --format '{{.Names}}' | grep -q "^$C\$"; then
      if [ "$IT" -ge "$((END - 1))" ]; then break; fi
      say "$NAME not running: (re)start from $RUN model_$IT.pt for $((END - IT)) iterations"
      setsid nohup docker compose run --rm -T --name $C lab train $TASK --env.scene.num-envs 8192 \
        --agent.max-iterations $((END - IT)) --agent.resume True --agent.load-run $RUN --agent.load-checkpoint model_$IT.pt \
        --agent.run-name $NAME >> train_$NAME.log 2>&1 < /dev/null 9>&- &
      sleep 90; continue
    fi
    sleep 30
  done
  if [ -f $S/STOP ]; then say "STOP file: stopping $NAME"; docker stop $C >/dev/null 2>&1; return 1; fi
  read RUN IT < <(latest)
  docker compose run --rm -T lab python lab/scripts/export_onnx.py $TASK --checkpoint logs/rsl_rl/$EXP/$RUN/model_$IT.pt \
    2>&1 | tail -3 >> goals_$NAME.log
  touch $P/done; say "phase $NAME done at $IT ($RUN)"
}

say "supervisor start"
phase turnaround_v2 Robotlab-Turnaround-MentorPiA1 turnaround 2026-11-03_10-27-46_turnaround_v1 450 750 turn || exit 0
# v2b: REAR_GAP from 0.05 m (v2: 0.25); the car's drive 3 replay showed v2 still reversing into a ~0.1 m rear gap
phase turnaround_v2b Robotlab-Turnaround-MentorPiA1 turnaround 2026-11-04_10-01-48_turnaround_v2 749 950 turn || exit 0
# v2c: open front with the rear blocked; last_action = executed speed (the replay still reversed into the blocked rear)
phase turnaround_v2c Robotlab-Turnaround-MentorPiA1 turnaround 2026-11-04_10-13-43_turnaround_v2b 900 1150 turn || exit 0
phase explore_v6 Robotlab-Explore-MentorPiA1 explore 2026-11-04_08-17-29_explore_v5w 2149 2650 explore || exit 0
# v7: the shield's approach stop in the path only, no 5 s hold (v6: 85% of idle was that stop; tmp_idle_cause.py)
phase explore_v7 Robotlab-Explore-MentorPiA1 explore 2026-11-04_10-35-28_explore_v6 2649 3150 explore || exit 0
# v7b: hit_person -2000 (v7 -300: at-fault person contacts flat at ~10% from 2650 to 2800)
phase explore_v7b Robotlab-Explore-MentorPiA1 explore 2026-11-04_13-07-51_explore_v7 2800 3300 explore || exit 0
# v7c: CaT person_close termination (training only); v7b 2850: 98% of contacts were feet, -100 per contact did not help
phase explore_v7c Robotlab-Explore-MentorPiA1 explore 2026-11-04_13-36-58_explore_v7b 2900 3400 explore || exit 0
# v7d: steer_rate on the executed steering, action_rate speed only, action std reset to 0.25 / 0.2 (reset_std.py; the
# steering std had collapsed to 0.0016, so PPO could not learn to steer around people). Init run made by hand:
# logs/rsl_rl/explore/2026-11-04_14-10-00_explore_v7d_init/model_<v7c latest>.pt
phase explore_v7d Robotlab-Explore-MentorPiA1 explore 2026-11-04_14-10-00_explore_v7d_init 2950 3600 explore || exit 0
# v7e: steer_rate -20, entropy_coef 0.001, std reset again to 0.25 / 0.2 (v7d: steering 0.031 -> 0.070/step, std 0.38).
# Init run made by hand: logs/rsl_rl/explore/2026-11-04_14-35-00_explore_v7e_init/model_3150.pt
phase explore_v7e Robotlab-Explore-MentorPiA1 explore 2026-11-04_14-35-00_explore_v7e_init 3150 3800 explore || exit 0
# v7f: CaT margin + 0.5 s x closing speed (v7e: contacts flat ~5.5%, car at 0.65 m/s 0.5 s before contact)
phase explore_v7f Robotlab-Explore-MentorPiA1 explore 2026-11-04_14-37-48_explore_v7e 3250 3900 explore || exit 0
# v7g: 25% of people stand still all episode (people.STAND_PROB); std reset to 0.2 / 0.15. v7f: contacts flat ~5.6%,
# mostly the car's corner on a standing person's feet at 45-90 deg. Init run made by hand (reset_std.py).
phase explore_v7g Robotlab-Explore-MentorPiA1 explore 2026-11-04_15-20-00_explore_v7g_init 3400 4100 explore || exit 0
# goto v1: explore v7g 3600 + route preview (warm_start.py, 17 inputs), progress / goal rewards, U-turn goals.
# Replaces car/nav/drive_to.py. Init run made by hand.
phase goto_v1 Robotlab-Goto-MentorPiA1 goto 2026-11-04_16-30-00_goto_init 3600 4400 goto || exit 0
# goto v2: route normalizer stats from sim samples + std reset (tmp_route_stats.py), entropy 0.003. goto_v1: route
# inputs scaled with copied LiDAR stats that never updated, std 0.09: 0.06 goals/min after 100 iterations.
phase goto_v2 Robotlab-Goto-MentorPiA1 goto 2026-11-04_16-20-00_goto_init2 3600 4400 goto || exit 0
# goto v3: LOST_DISTANCE 2.5 m, GOAL_DIST 1.0 m (goto_v2 3800: 0.49 goals/min; lost on turn-back routes after 2.2 s and
# just past route ends). Continues from goto_v2's latest checkpoint.
phase goto_v3 Robotlab-Goto-MentorPiA1 goto 2026-11-04_16-15-58_goto_v2 3850 4600 goto || exit 0
# goto v4: doorways (office_terrain.door_prob 0.35: a wall across the hallway with a 0.7-0.9 m gap off to one side).
# goto2 on the floor froze facing the wall beside the office door. Continues from goto_v3's latest checkpoint.
phase goto_v4 Robotlab-Goto-MentorPiA1 goto 2026-11-04_16-56-05_goto_v3 4050 5000 goto || exit 0
# gotosub v1 (research 2026-11-04: DRL-VO sub-goal, Rudin arrive-not-track): goto_v4's network with the 17 route inputs
# swapped for a 3-value sub-goal (tmp_swap_obs.py), no on_route reward. Runs in parallel with goto_v4 (second supervisor).
phase gotosub_v1 Robotlab-GotoSub-MentorPiA1 gotosub GOTOSUB_INIT GOTOSUB_CK 5000 goto || exit 0
# turnaround v3: open-room episodes (30%: walls 3 m apart, free ahead, a thin post 5-30 cm behind the car), drive 3 case
# v2c still reversed into. From v2c 1149.
phase turnaround_v3 Robotlab-Turnaround-MentorPiA1 turnaround 2026-11-04_10-21-54_turnaround_v2c 1149 1450 turn || exit 0
# goto v5 (2026-11-05): 30% mid-hallway turn-back routes (goto_env_cfg.MIDTURN_*), hall width 1.2-1.8 m, progress point
# may not jump > 1 m ahead (navigate.MAX_ADVANCE). Floor goto3-8: v4 never turned back mid-hall; sim v4 4850: 18% done.
phase goto_v5 Robotlab-Goto-MentorPiA1 goto 2026-11-04_18-56-11_goto_v4 4850 5850 goto || exit 0
# goto v5b: mid-hallway routes lost 1.0 m off the route (MIDTURN_LOST): v5 4950 turned at the next junction instead.
phase goto_v5b Robotlab-Goto-MentorPiA1 goto 2026-11-05_10-15-20_goto_v5 4950 5850 goto || exit 0
# goto v5c: dense route_approach (+1) / route_heading (+0.3) on mid-hallway routes: v5b 5050 1.6% done, 90% of the
# failures never reversed (tmp_midturn_trace.py).
phase goto_v5c Robotlab-Goto-MentorPiA1 goto 2026-11-05_10-36-16_goto_v5b 5050 5850 goto || exit 0
# goto v5d: after each goal, the next route is a mid-hallway turn-back with MIDTURN_PROB too (v5c: only at reset, ~50
# turn-backs per iteration; 1.8-3.1% done 5100-5200).
phase goto_v5d Robotlab-Goto-MentorPiA1 goto 2026-11-05_10-56-55_goto_v5c 5200 6000 goto || exit 0
# goto v5e: the way back of a mid-hallway route runs 0.4 m to a random side (MIDTURN_SIDE): v5c/d steering flipped left /
# right on a route straight back (tmp_midturn_steps.py), 3.5% done at 5250-5300.
phase goto_v5e Robotlab-Goto-MentorPiA1 goto 2026-11-05_11-23-58_goto_v5d 5300 6100 goto || exit 0
# goto v6: 25% blocked mid-turn starts (nose 0.22-0.42 m to the hallway wall, BLOCKED_*), shield_stop -10 (was -3).
# Floor goto9/10 (2026-11-05): v5e 6000 turned, stopped at the wall and never reversed.
phase goto_v6 Robotlab-Goto-MentorPiA1 goto 2026-11-05_11-46-08_goto_v5e 6099 7000 goto || exit 0
# goto v6b: blocked starts skip spots where the car would overlap a wall / clutter (v6 6200: some spawned inside clutter
# and ended as a bump at the first step, tmp_blocked_place.py).
phase goto_v6b Robotlab-Goto-MentorPiA1 goto 2026-11-05_13-52-17_goto_v6 6200 7000 goto || exit 0
# goto v6c: + stop_held input (how long the stop has held the car, 0 -> 1 over 2 s; warm_start.py --copy-stats unit).
# v6b 6300 step trace: held by the stop the observation froze and the policy asked the same thing forever.
phase goto_v6c Robotlab-Goto-MentorPiA1 goto 2026-11-05_14-45-00_goto_v6c_init 6300 7100 goto || exit 0
# goto v6d: + corner starts (15%: just before an intersection, turned early, the corner 0.32-0.45 m at 25-60 deg; floor
# goto11 door frame) and half of the blocked starts boxed in mid-turn (45-80 deg off the wall normal; floor goto12).
phase goto_v6d Robotlab-Goto-MentorPiA1 goto 2026-11-05_14-41-08_goto_v6c 6700 7500 goto || exit 0
# goto v6e: halls 1.0-1.8 m (narrow ones need several back-and-forth moves to turn: user 2026-11-05 "keep going back and
# forward until it succeeds"), corner starts closer / more turned in (floor goto14 froze at the office door again).
phase goto_v6e Robotlab-Goto-MentorPiA1 goto 2026-11-05_16-04-15_goto_v6d 6850 7650 goto || exit 0
# goto v6f: the car's back-and-forth rule in the sim too (mdp.AckermannActionCfg.unstick; floor goto16 full trip with it).
# v6e 6950: narrow-hall (<1.3 m) turn-backs 38%, failures rarely reversed.
phase goto_v6f Robotlab-Goto-MentorPiA1 goto 2026-11-05_16-34-51_goto_v6e 7000 8000 goto || exit 0
# goto v6g: left / right mirror data augmentation (goto_symmetry.py, rsl_rl Symmetry). v6f 7250: left turn-backs 76%,
# right 94% in a mirror-symmetric task; overall 84-86% flat from 7100 to 7300.
phase goto_v6g Robotlab-Goto-MentorPiA1 goto 2026-11-05_16-58-31_goto_v6f 7300 8300 goto || exit 0
# goto v6h: v6f continued from 7300 without the mirror augmentation (v6g collapsed: 84% -> 67.5% turn-backs).
phase goto_v6h Robotlab-Goto-MentorPiA1 goto 2026-11-05_16-58-31_goto_v6f 7300 8300 goto || exit 0
# goto v6i: back-up also when stalled (moved < 0.25 m in 2 s; mdp unstick). v6h 7350: in 1.0-1.3 m halls the car turned
# left / right in place, never reversed (tmp_midturn_narrow.py), 56% done there.
phase goto_v6i Robotlab-Goto-MentorPiA1 goto 2026-11-05_18-25-52_goto_v6h 7400 8400 goto || exit 0
# goto v6j: MIDTURN_LOST 1.3 m (was 1.0): v6i 7450 83.5% -> 89.4% turn-backs with the same model at 1.3 m.
phase goto_v6j Robotlab-Goto-MentorPiA1 goto 2026-11-05_18-42-13_goto_v6i 7450 8500 goto || exit 0
# goto v6k: back up also when driving away from a turn-back route (goto_env_cfg.moving_away; eval on v6j 7800: runs
# lost 10% -> 5.7%, untrained). v6j 7550-7800: runs lost flat at 8.4-10%.
phase goto_v6k Robotlab-Goto-MentorPiA1 goto 2026-11-05_18-56-18_goto_v6j 7800 8500 goto || exit 0
# goto v6l: back-up after 0.3 s held, person_ttc -30, goal check on 2048 cars. v6k 8200 met all goals in one check but
# 10/12 as the mean of 3 (person contacts 1.1%, blocked held 1.25 s).
phase goto_v6l Robotlab-Goto-MentorPiA1 goto 2026-11-05_19-57-25_goto_v6k 8200 9000 goto || exit 0
# goto v6m: blocked back-ups cycle their steering (opposite / straight / same side; mdp unstick). Eval on v6l 8250: blocked
# starts held mean 1.26 -> 0.32 s, max 29.9 -> 2.1 s, none stuck.
phase goto_v6m Robotlab-Goto-MentorPiA1 goto 2026-11-05_21-08-48_goto_v6l 8250 9000 goto || exit 0
# goto v6n: back-ups only with the LiDAR's rear quarter clear 0.6 m (or after waiting 1.5 s). v6m 8300: person contacts
# 1.2%, 10% of them during a back-up; with the rule (untrained) 8300 met all goals at 1.0%.
phase goto_v6n Robotlab-Goto-MentorPiA1 goto 2026-11-05_21-25-30_goto_v6m 8350 9000 goto || exit 0
# goto v6o: person_ttc -50 (was -30). v6n 8400-8500: at-fault person contacts 1.0-1.3%, every other goal met.
phase goto_v6o Robotlab-Goto-MentorPiA1 goto 2026-11-05_21-46-43_goto_v6n 8500 9200 goto || exit 0
# goto v6p: shield corridor half width 0.14 m (was 0.10). v6o 8600: 3-check mean 11/12 goals, at-fault person contacts
# 1.4% (mostly standing people clipped by the car's corner); eval with 0.14: 0.6-0.9%, but blocked held / lost worse
# until the policy adapts.
phase goto_v6p Robotlab-Goto-MentorPiA1 goto 2026-11-05_22-15-06_goto_v6o 8600 9300 goto || exit 0
# goto v7 (2026-11-08, floor goto17 review): steer_flip / action_bound / policy_reverse penalties (straight-hall zig-zag,
# reverse blips), chair bases under the LiDAR plane (office_terrain.CHAIR_*), depth hits in the built map for 8 s,
# 48 depth columns, no driving-away back-up near the U-turn corners, back-up escalation. New goals in tmp_goto_check.py.
phase goto_v7 Robotlab-Goto-MentorPiA1 goto 2026-11-05_22-44-06_goto_v6p 8750 10000 goto || exit 0
# goto v7b: CAPS smoothness on the policy mean (caps_ppo.py; Mysore et al. 2021) instead of strong reward penalties on the
# sampled actions. v7 8800-9000: steering std on straights flat 10.6-10.8 deg, runs lost 7% -> 14.5%; the penalties
# mostly shrank the exploration std (0.87 -> 0.56). steer_flip -6 -> -1, action_bound -0.1 -> -0.02.
phase goto_v7b Robotlab-Goto-MentorPiA1 goto 2026-11-08_10-50-46_goto_v7 9000 10200 goto || exit 0
# goto v7c: from v6p 8800 (v7b 9100: runs lost 27.6%, turn-backs 63%, weave still 10.1 deg). The weave is a feedback loop,
# not output jitter: straight_yaw (-5, yaw rate^2 on straight stretches) instead of steer_flip / action_bound; CAPS light
# (0.3 / 0.1 / 0.3, 1 step); TURN_ZONE 0.5 m.
phase goto_v7c Robotlab-Goto-MentorPiA1 goto 2026-11-05_22-44-06_goto_v6p 8800 10000 goto || exit 0
say "all phases done"
