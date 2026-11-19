# Steering the driver actually sets (ackermann.py: atan(0.145 w / v), clip +-40 deg) for each command stage of a bag.
import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
S = {k: [] for k in ('/cmd_vel_nav', '/cmd_vel_smoothed', '/controller/cmd_vel')}; t0 = None
while r.has_next():
  topic, data, t = r.read_next()
  if topic not in S: continue
  t0 = t0 or t; m = deserialize_message(data, get_message(types[topic]))
  S[topic].append(((t - t0) / 1e9, m.linear.x, m.angular.z))
for k, v in S.items():
  a = np.array(v)
  if not len(a): print(k, 'none'); continue
  mv = np.abs(a[:, 1]) > 1e-3
  d = np.degrees(np.clip(np.arctan(0.145 * a[mv, 2] / a[mv, 1]), -math.radians(40), math.radians(40)))
  sg = np.sign(np.where(np.abs(d) < 3, 0, d)); sg = sg[sg != 0]
  flips = (np.diff(sg) != 0).sum(); dur = a[mv, 0][-1] - a[mv, 0][0]
  lowv = np.abs(a[mv, 1]) < 0.1
  print(f'{k:22s} {mv.sum():5d} moving msgs over {dur:5.1f} s: steer |d|>=39 deg {np.mean(np.abs(d) >= 39) * 100:4.1f}%, '
        f'L/R flips {flips} ({flips / dur:.2f}/s), mean |d| {np.mean(np.abs(d)):.1f} deg; at |v|<0.1: {lowv.mean() * 100:.0f}% of msgs, '
        f'{np.mean(np.abs(d[lowv]) >= 39) * 100 if lowv.any() else 0:.0f}% full lock')
