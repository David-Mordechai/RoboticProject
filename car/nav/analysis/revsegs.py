# Reverse segments the controller drove (/cmd_vel_nav v<0): when, where (odom), how long; plus how many /plan messages
# contain a direction change (cusp) and how far along the plan the first cusp is.
import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
nav, odom, cusps = [], [], []; t0 = None
while r.has_next():
  topic, data, t = r.read_next()
  if topic not in ('/cmd_vel_nav', '/odom', '/plan'): continue
  t0 = t0 or t; s = (t - t0) / 1e9; m = deserialize_message(data, get_message(types[topic]))
  if topic == '/cmd_vel_nav': nav.append((s, m.linear.x))
  elif topic == '/odom': odom.append((s, m.pose.pose.position.x, m.pose.pose.position.y))
  else:
    P = np.array([[p.pose.position.x, p.pose.position.y] for p in m.poses])
    if len(P) < 3: continue
    v = np.diff(P, axis=0); c = np.where((v[1:] * v[:-1]).sum(1) < 0)[0]
    L = np.r_[0, np.cumsum(np.linalg.norm(v, axis=1))]
    cusps.append((s, len(c), float(L[c[0] + 1]) if len(c) else None, float(L[-1])))
odom = np.array(odom); segs = []; cur = None
for s, v in nav:
  if v < -1e-3 and cur is None: cur = s
  if v >= -1e-3 and cur is not None: segs.append((cur, s)); cur = None
print(f'{len(segs)} reverse segments, {sum(b - a for a, b in segs):.1f} s reversing of {nav[-1][0] - nav[0][0]:.1f} s driving')
for a, b in segs:
  i, j = np.argmin(abs(odom[:, 0] - a)), np.argmin(abs(odom[:, 0] - b))
  print(f'  {a:5.1f}-{b:5.1f} s ({b - a:3.1f} s) odom ({odom[i, 1]:+.2f},{odom[i, 2]:+.2f}) -> ({odom[j, 1]:+.2f},{odom[j, 2]:+.2f}) {np.hypot(*(odom[j, 1:] - odom[i, 1:])):.2f} m')
wc = [c for c in cusps if c[1]]
print(f'plans with a cusp: {len(wc)} of {len(cusps)}; first cusp < 0.5 m from the plan start: {sum(1 for c in wc if c[2] < 0.5)}')
for c in wc[:25]: print(f'  plan at {c[0]:5.1f} s: {c[1]} cusp(s), first at {c[2]:.2f} m of {c[3]:.2f} m')
