"""Mean of 3 goal checks (the PASS/FAIL lines of tmp_goto_check.py, as mean3.sh collects them): python mean3_summary.py FILE"""
import collections
import re
import sys

g = collections.OrderedDict()
for line in open(sys.argv[1], encoding="utf-8", errors="replace"):
  m = re.match(r"(PASS|FAIL)\s+(.+?): ([0-9.]+) \(goal (<=|>=) ([0-9.]+)", line)
  if m:
    g.setdefault(m[2], []).append((float(m[3]), m[4], float(m[5])))
ok = 0
for name, vals in g.items():
  v = sum(x[0] for x in vals) / len(vals)
  op, thr = vals[0][1], vals[0][2]
  good = v <= thr if op == "<=" else v >= thr
  ok += good
  checks = ", ".join(f"{x[0]:.3f}" for x in vals)
  print(f"  {'PASS' if good else 'FAIL'} {name[:60]}: {v:.3f} (goal {op} {thr}; checks: {checks})")
print(f"== {ok}/{len(g)} goals met (mean of {min(len(v) for v in g.values())} checks)")
