"""Does the live LiDAR scan fit the saved map at a pose? For each scan hit (placed at pose X Y YAW, LiDAR at
base_footprint +(-0.010, 0), TF car #1) take the distance to the nearest occupied map cell. Also searches +-0.3 m /
+-12 deg for the best-fitting pose; last line 'BEST x y yaw_deg percent' for scripts. Car must stand still.
  python3 scan_map_match.py MAP.yaml [X Y YAW_DEG]"""
import math, sys, time
import numpy as np, yaml, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from scipy.ndimage import distance_transform_edt

y = yaml.safe_load(open(sys.argv[1])); res = float(y['resolution']); ox, oy = y['origin'][:2]
with open(sys.argv[1].rsplit('/', 1)[0] + '/' + y['image'], 'rb') as f:
  f.readline(); w, h = map(int, f.readline().split()); f.readline()
  img = np.frombuffer(f.read(), np.uint8).reshape(h, w)[::-1]          # row 0 = min y
occ = img < 50
dist = distance_transform_edt(~occ) * res
X, Y, YAW = (float(v) for v in sys.argv[2:5]) if len(sys.argv) > 4 else (0.0, 0.0, 0.0)
rclpy.init(); n = rclpy.create_node('scan_map_match'); got = []
n.create_subscription(LaserScan, '/scan_raw', lambda m: got.append(m), qos_profile_sensor_data)
t = time.time()
while len(got) < 5 and time.time() - t < 5: rclpy.spin_once(n, timeout_sec=0.1)
m = got[-1]; r = np.array(m.ranges); a = m.angle_min + np.arange(len(r)) * m.angle_increment
ok = np.isfinite(r) & (r > m.range_min) & (r < 6.0); r, a = r[ok], a[ok]
lx, ly = -0.010 + r * np.cos(a), r * np.sin(a)


def score(x, yy, yaw):
  c, s = math.cos(yaw), math.sin(yaw)
  gx, gy = x + c * lx - s * ly, yy + s * lx + c * ly
  i, j = ((gy - oy) / res).astype(int), ((gx - ox) / res).astype(int)
  inside = (i >= 0) & (i < h) & (j >= 0) & (j < w)
  d = np.full(len(gx), 1.0); d[inside] = dist[i[inside], j[inside]]
  return d


d = score(X, Y, math.radians(YAW))
print(f'{len(d)} hits at ({X}, {Y}, {YAW} deg): median dist to map wall {np.median(d):.3f} m, '
      f'within 0.10 m {np.mean(d < 0.10) * 100:.0f}%')
# finer search around the given pose: +-0.30 m in 0.025 m steps, +-12 deg in 1 deg steps, scored by the share of hits
# within 0.10 m of a map wall (a median on a 5 cm map is too coarse to rank poses).
best = None
for dyaw in range(-12, 13):
  for dx in np.arange(-0.30, 0.301, 0.025):
    for dy in np.arange(-0.30, 0.301, 0.025):
      dd = score(X + dx, Y + dy, math.radians(YAW + dyaw))
      f = np.mean(dd < 0.10); m = np.mean(np.minimum(dd, 0.2))   # tie-break: mean distance (clipped 0.2 m)
      if best is None or (f, -m) > (best[0], -best[4]): best = (f, X + dx, Y + dy, YAW + dyaw, m)
print(f'best fit: ({best[1]:+.3f}, {best[2]:+.3f}) {best[3]:+.0f} deg, {best[0] * 100:.0f}% of hits within 0.10 m')
print(f'BEST {best[1]:.3f} {best[2]:.3f} {best[3]:.1f} {best[0] * 100:.0f}')
