#!/bin/bash
# batch.sh TAG BT "extra args" : courses x steering limits x 3 repeats, 6 in parallel -> results/TAG_*.json, summary
# env COURSES (default "C1 C2 C3"), STEERS (default "29 33 40"), REPS (default 3), K0 (ROS domain base, default 50;
# every K0+n must stay <= 232, the Fast DDS limit: F6/F7 with K0 230/260 silently failed on 2026-11-09)
TAG=$1; BT=$2; EX="$3"
C1="0.28 -0.09 -11 2.95 -2.3 -90"; C2="2.92 -1.95 -90 2.95 -7.0 -90"; C3="2.36 -7.42 -95 0.3 0 180"
# H = home + park pose in front of the charger facing out (nav_a_home.sh; run with its goal-checker tolerances)
H="2.36 -7.42 -95 0.35 -0.03 -8"
H6="2.36 -7.42 -95 0.6 -0.05 -8"   # pre-dock 0.6 m out (more room to align)
H7="2.36 -7.42 -95 1.3 -0.05 -8"   # 2026-11-11: U-turn in the open first (waypoint 1.3 m out, facing out)
H8="1.3 -0.05 -8 0.35 -0.03 -8"    # then straight reverse to pre-dock
H9="2.36 -7.42 -95 2.0 0.2 -8"     # U-turn waypoint in the middle of the open room (>= 0.55 m from chair bases)
H10="2.0 0.2 -8 0.35 -0.03 -8"     # then reverse 1.65 m to pre-dock
H11="2.0 0.35 25 0.35 -0.03 -8"    # leg 2 from a skewed waypoint arrival (+25 deg, 0.15 m off)
H12="1.85 0.1 -40 0.35 -0.03 -8"   # leg 2 from -40 deg
jobs=(); k=${K0:-50}
for c in ${COURSES:-C1 C2 C3}; do for sm in ${STEERS:-29 33 40}; do for r in $(seq ${REPS:-3}); do
  k=$((k+1)); jobs+=("${TAG}_${c}_s${sm}_r$r $k $BT ${!c} --timeout 150 --steer-max $sm $EX"); done; done; done
printf '%s\n' "${jobs[@]}" | xargs -P 6 -I{} bash -c './bench.sh {} > /dev/null'
python3 - "$TAG" <<'PY'
import json, glob, sys, statistics as st
tag = sys.argv[1]
for c in ("C1", "C2", "C3", "H", "H6", "H7", "H8", "H9", "H10", "H11", "H12"):
  R = [json.load(open(f)) for f in sorted(glob.glob(f'results/{tag}_{c}_*.json'))]
  if not R: print(c, 'no results'); continue
  ok = [r for r in R if r.get('reached')]
  g = lambda k: st.median([r.get(k, 0) for r in R])
  print(f'{c}: reached {len(ok)}/{len(R)}, time med {st.median([r["time_s"] for r in ok]) if ok else float("nan"):.1f} s, '
        f'recov med {g("recoveries")}, max {max(r.get("recoveries", 0) for r in R)}, reverse s med {g("reverse_s")}, '
        f'flips/s med {g("flips_per_s")}, full lock med {g("full_lock_pct")}%, speed med {g("mean_speed")}, '
        f'contacts total {sum(r.get("contacts", 0) for r in R)}')
  for r in R:
    if not r.get('reached'): print('   FAILED:', {k: r.get(k) for k in ('steer_max', 'time_s', 'recoveries', 'end')})
PY
