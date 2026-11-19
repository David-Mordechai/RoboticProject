"""Real turning radius from a recorded arc run (rec_run.sh bag), without trusting commanded speed (2026-11-11).
Heading change = integrated /imu yaw rate over the arc; position change of the LiDAR = 2D ICP between the last scan
before and the first scan after the arc (seeded with the IMU heading). Chord c and heading change dpsi give the LiDAR's
path radius R_l = c / (2 sin(dpsi/2)); rear-axle radius R_r = sqrt(R_l^2 - d^2) with d = LiDAR x ahead of the rear axle;
steering delta = atan(L / R_r).  Run in the MentorPi container:  python3 arc_icp.py RUN_DIR/bag
"""
import math, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, LaserScan
from tf2_msgs.msg import TFMessage

L = 0.137                     # URDF wheelbase (driver uses 0.145: both reported)
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
imu, scans, tfs = [], [], {}
while r.has_next():
  topic, data, t = r.read_next(); t *= 1e-9
  if topic == '/imu': imu.append((t, deserialize_message(data, Imu).angular_velocity.z))
  elif topic == '/scan_raw': scans.append((t, deserialize_message(data, LaserScan)))
  elif topic == '/tf_static':
    for tr in deserialize_message(data, TFMessage).transforms:
      tfs[(tr.header.frame_id, tr.child_frame_id)] = tr.transform.translation
imu = np.array(imu)
for k, v in tfs.items():
  if 'lidar' in k[1] or 'laser' in k[1] or 'wheel' in k[1]: print('tf', k, round(v.x, 4), round(v.y, 4))


def pts(m):
  rr = np.array(m.ranges); a = m.angle_min + np.arange(len(rr)) * m.angle_increment
  ok = np.isfinite(rr) & (rr > m.range_min) & (rr < min(m.range_max, 4.0))
  return np.c_[rr[ok] * np.cos(a[ok]), rr[ok] * np.sin(a[ok])]


def icp(A, B, th):
  """Pose (th, tx, ty) mapping scan B (after) into A's frame (before)."""
  tx = ty = 0.0
  for it in range(40):
    c, s = math.cos(th), math.sin(th)
    Bt = B @ np.array([[c, s], [-s, c]]) + [tx, ty]
    d = ((Bt[:, None, :] - A[None, :, :]) ** 2).sum(-1); j = d.argmin(1); dm = d[np.arange(len(B)), j]
    keep = dm < (0.05 ** 2 if it > 10 else 0.2 ** 2)
    P, Q = Bt[keep], A[j[keep]]
    mp, mq = P.mean(0), Q.mean(0); H = (P - mp).T @ (Q - mq)
    dth = math.atan2(H[0, 1] - H[1, 0], H[0, 0] + H[1, 1])
    th += dth; c, s = math.cos(dth), math.sin(dth)
    t = mq - mp @ np.array([[c, s], [-s, c]]); tx, ty = np.array([tx, ty]) @ np.array([[c, s], [-s, c]]) + t
  return th, tx, ty, keep.mean()


turning = np.abs(imu[:, 1]) > math.radians(8)
edges = np.flatnonzero(np.diff(turning.astype(int)))
segs = [(imu[edges[i] + 1, 0], imu[edges[i + 1], 0]) for i in range(0, len(edges) - 1, 2)]
lid = next((v for k, v in tfs.items() if 'lidar' in k[1] or 'laser' in k[1]), None)
for t0, t1 in segs:
  if t1 - t0 < 0.5: continue
  sel = (imu[:, 0] >= t0 - 0.3) & (imu[:, 0] <= t1 + 0.3)
  dpsi = np.trapz(imu[sel, 1], imu[sel, 0])
  before = max((s for s in scans if s[0] < t0 - 0.3), key=lambda s: s[0])[1]
  after = min((s for s in scans if s[0] > t1 + 0.3), key=lambda s: s[0])[1]
  th, tx, ty, fit = icp(pts(before), pts(after), dpsi)
  c = math.hypot(tx, ty); Rl = c / (2 * abs(math.sin(th / 2)))
  print(f'arc {t1 - t0:.2f} s: IMU dpsi {math.degrees(dpsi):+.1f} deg, ICP dpsi {math.degrees(th):+.1f} deg, '
        f'LiDAR moved {c:.3f} m (fit {fit:.0%}) -> LiDAR path radius {Rl:.3f} m, mean speed {c / (t1 - t0):.3f} m/s')
  for name, dx in (('lidar x from TF (base_footprint)', lid.x if lid else 0.0),):
    for rear in (-L / 2, 0.0):          # base_footprint at axle midpoint (URDF) or at the rear axle
      d = dx - rear; Rr = math.sqrt(max(Rl * Rl - d * d, 1e-6))
      print(f'   rear axle at x={rear:+.3f}: rear radius {Rr:.3f} -> steering {math.degrees(math.atan(L / Rr)):.1f} deg '
            f'(L {L}); with L 0.145: {math.degrees(math.atan(0.145 / Rr)):.1f} deg')
