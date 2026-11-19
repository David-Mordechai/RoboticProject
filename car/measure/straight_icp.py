"""Real vs commanded speed on straight stretches of a recording (2026-11-11): LiDAR ICP between the scan at the start and
at the end of each stretch where the commanded steering is < 3 deg and |v| > 0.05 m/s for >= 1 s.
  python3 straight_icp.py RUN/bag"""
import math, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
sys.path.insert(0, '/home/ubuntu/shared')
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
cmd, scans = [], []
while r.has_next():
  tp, data, t = r.read_next(); t *= 1e-9
  if tp == '/controller/cmd_vel': m = deserialize_message(data, Twist); cmd.append((t, m.linear.x, m.angular.z))
  elif tp == '/scan_raw': scans.append((t, deserialize_message(data, LaserScan)))
cmd = np.array(cmd)


def pts(m):
  rr = np.array(m.ranges); a = m.angle_min + np.arange(len(rr)) * m.angle_increment
  ok = np.isfinite(rr) & (rr > m.range_min) & (rr < min(m.range_max, 5.0))
  return np.c_[rr[ok] * np.cos(a[ok]), rr[ok] * np.sin(a[ok])]


def icp(A, B):
  th = tx = ty = 0.0
  for it in range(50):
    c, s = math.cos(th), math.sin(th)
    Bt = B @ np.array([[c, s], [-s, c]]) + [tx, ty]
    d = ((Bt[:, None, :] - A[None, :, :]) ** 2).sum(-1); j = d.argmin(1); dm = d[np.arange(len(B)), j]
    keep = dm < (0.05 ** 2 if it > 15 else 0.3 ** 2)
    P, Q = Bt[keep], A[j[keep]]; mp, mq = P.mean(0), Q.mean(0); H = (P - mp).T @ (Q - mq)
    dth = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1]); th += dth; c, s = math.cos(dth), math.sin(dth)
    t = mq - mp @ np.array([[c, s], [-s, c]]); tx, ty = np.array([tx, ty]) @ np.array([[c, s], [-s, c]]) + t
  return th, tx, ty, keep.mean()


# command segments (hold last command until the next one)
seg = []
for i in range(len(cmd)):
  t, v, w = cmd[i]; t_end = cmd[i + 1, 0] if i + 1 < len(cmd) else scans[-1][0]
  dl = math.degrees(math.atan(0.145 * w / v)) if abs(v) > 1e-3 else 0
  seg.append((t, t_end, v, abs(dl) < 3 and abs(v) > 0.05))
runs, cur = [], None
for a, b, v, ok in seg:
  if ok and cur and abs(v - cur[2]) < 1e-6 and abs(a - cur[1]) < 1e-3: cur[1] = b
  else:
    if cur: runs.append(cur)
    cur = [a, b, v] if ok else None
if cur: runs.append(cur)
for a, b, v in runs:
  a2 = a + 0.6                               # skip the acceleration
  if b - a2 < 0.8: continue
  s0 = min(scans, key=lambda s: abs(s[0] - a2)); s1 = min(scans, key=lambda s: abs(s[0] - b))
  th, tx, ty, fit = icp(pts(s0[1]), pts(s1[1])); dt = s1[0] - s0[0]
  print(f'cmd {v:+.2f} m/s for {b - a:.1f} s: real {math.copysign(math.hypot(tx, ty) / dt, tx):+.3f} m/s '
        f'({math.hypot(tx, ty) / dt / abs(v) * 100:.0f}% of command), turned {math.degrees(th):+.1f} deg, fit {fit:.0%}')
