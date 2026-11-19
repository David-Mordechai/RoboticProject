# For each L/R steering flip in the controller's own output (/cmd_vel_nav), what happened just before (0.6 s):
#  - a new /plan arrived, and how far its point 0.6 m ahead moved sideways vs the previous plan (H2: replans);
#  - an /amcl_pose correction (map->odom jump) and its size (H3: localization jumps);
#  - the car was reversing (U-turn manoeuvre: flips expected).
import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
nav, plans, amcl, odom = [], [], [], []; t0 = None
while r.has_next():
  topic, data, t = r.read_next()
  if topic not in ('/cmd_vel_nav', '/plan', '/amcl_pose', '/odom'): continue
  t0 = t0 or t; s = (t - t0) / 1e9; m = deserialize_message(data, get_message(types[topic]))
  if topic == '/cmd_vel_nav': nav.append((s, m.linear.x, m.angular.z))
  elif topic == '/plan': plans.append((s, np.array([[p.pose.position.x, p.pose.position.y] for p in m.poses])))
  elif topic == '/amcl_pose': amcl.append((s, m.pose.pose.position.x, m.pose.pose.position.y))
  elif topic == '/odom': odom.append((s, m.pose.pose.position.x, m.pose.pose.position.y))
amcl = np.array(amcl); odom = np.array(odom)
# AMCL jump = change of (amcl - odom) offset between consecutive amcl poses
jumps = []
for i in range(1, len(amcl)):
  def off(k):
    j = np.argmin(np.abs(odom[:, 0] - amcl[k, 0])); return amcl[k, 1:] - odom[j, 1:]
  jumps.append((amcl[i, 0], float(np.linalg.norm(off(i) - off(i - 1)))))
# plan shift: lateral move of the point ~0.6 m along the new plan from its start vs nearest point of previous plan
shifts = []
for i in range(1, len(plans)):
  a, b = plans[i - 1][1], plans[i][1]
  if len(a) < 2 or len(b) < 2: continue
  cum = np.r_[0, np.cumsum(np.linalg.norm(np.diff(b, axis=0), axis=1))]; p = b[np.searchsorted(cum, min(0.6, cum[-1]))]
  shifts.append((plans[i][0], float(np.min(np.linalg.norm(a - p, axis=1)))))
n = np.array(nav); mv = np.abs(n[:, 1]) > 1e-3; n = n[mv]
d = np.degrees(np.arctan(0.145 * n[:, 2] / n[:, 1])); sg = np.where(np.abs(d) < 3, 0, np.sign(d))
idx = [i for i in range(1, len(sg)) if sg[i] != 0 and sg[i - 1] != 0 and sg[i] != sg[i - 1]]
cnt = {'replan_shift>0.10': 0, 'amcl_jump>0.05': 0, 'reversing': 0, 'none': 0}
for i in idx:
  t = n[i, 0]; tags = []
  ps = [s for (ts, s) in shifts if t - 0.6 <= ts <= t]
  if ps and max(ps) > 0.10: tags.append('replan_shift>0.10')
  js = [j for (ts, j) in jumps if t - 0.6 <= ts <= t]
  if js and max(js) > 0.05: tags.append('amcl_jump>0.05')
  if n[i, 1] < 0 or n[i - 1, 1] < 0: tags.append('reversing')
  for k in tags or ['none']: cnt[k] += 1
print(f'{len(idx)} flips; plans {len(plans)} (lateral shift at 0.6 m: median {np.median([s for _, s in shifts]):.3f}, '
      f'p90 {np.percentile([s for _, s in shifts], 90):.3f} m); amcl updates {len(amcl)} (jump median {np.median([j for _, j in jumps]):.3f}, max {max(j for _, j in jumps):.3f} m)')
print('flips preceded (0.6 s) by:', cnt)
