"""Where on the saved map is the car? Global search of the live LiDAR scan over the whole map (car standing still),
for when the car is NOT at the charger (nav_a_ready.sh assumes it is). Same score as scan_map_match.py: share of scan
hits within 0.10 m of an occupied map cell, LiDAR at base_footprint +(-0.010, 0) (TF car #1).
Coarse grid: every free cell 0.10 m apart x yaw every 3 deg, on a 2-in-3 subsample of hits; prints the TOP distinct poses.
Refine the best with: python3 scan_map_match.py MAP.yaml X Y YAW (+-0.3 m / +-12 deg).
  python3 global_locate.py MAP.yaml [TOP=5]"""
import math, sys, time
import numpy as np, yaml, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from scipy.ndimage import distance_transform_edt, binary_erosion

y = yaml.safe_load(open(sys.argv[1])); res = float(y['resolution']); ox, oy = y['origin'][:2]
TOP = int(sys.argv[2]) if len(sys.argv) > 2 else 5
with open(sys.argv[1].rsplit('/', 1)[0] + '/' + y['image'], 'rb') as f:
  f.readline(); w, h = map(int, f.readline().split()); f.readline()
  img = np.frombuffer(f.read(), np.uint8).reshape(h, w)[::-1]          # row 0 = min y
occ = img < 50
free = img > 250
near = distance_transform_edt(~occ) * res
# candidate positions: free cells at least 0.10 m from a wall (car half-width ~0.08 m, research/01 sec. 2)
cand = free & (near > 0.10)
step = max(1, int(round(0.10 / res)))
ci, cj = np.nonzero(cand[::step, ::step]); ci, cj = ci * step, cj * step
px, py = ox + (cj + 0.5) * res, oy + (ci + 0.5) * res

rclpy.init(); n = rclpy.create_node('global_locate'); got = []
n.create_subscription(LaserScan, '/scan_raw', lambda m: got.append(m), qos_profile_sensor_data)
t = time.time()
while len(got) < 5 and time.time() - t < 5: rclpy.spin_once(n, timeout_sec=0.1)
m = got[-1]; r = np.array(m.ranges); a = m.angle_min + np.arange(len(r)) * m.angle_increment
ok = np.isfinite(r) & (r > m.range_min) & (r < 6.0); r, a = r[ok], a[ok]
lx, ly = -0.010 + r * np.cos(a), r * np.sin(a)
sub = np.arange(len(lx)) % 3 != 2
lx, ly = lx[sub], ly[sub]
print(f'{len(px)} positions x 120 headings, {len(lx)} hits', flush=True)

best = []
t = time.time()
for yd in range(0, 360, 3):
  c, s = math.cos(math.radians(yd)), math.sin(math.radians(yd))
  gx = px[:, None] + c * lx[None, :] - s * ly[None, :]
  gy = py[:, None] + s * lx[None, :] + c * ly[None, :]
  i, j = ((gy - oy) / res).astype(np.int32), ((gx - ox) / res).astype(np.int32)
  inside = (i >= 0) & (i < h) & (j >= 0) & (j < w)
  d = np.ones(gx.shape, np.float32)
  d[inside] = near[i[inside], j[inside]]
  sc = (d < 0.10).mean(axis=1)
  k = np.argsort(sc)[-10:]
  best += [(float(sc[q]), float(px[q]), float(py[q]), yd) for q in k]
best.sort(reverse=True)
out = []
for b in best:  # distinct poses: > 0.5 m or > 20 deg apart
  if all(math.hypot(b[1] - o[1], b[2] - o[2]) > 0.5 or abs((b[3] - o[3] + 180) % 360 - 180) > 20 for o in out):
    out.append(b)
  if len(out) >= TOP: break
print(f'search {time.time() - t:.1f} s')
for sc, x, yy, yd in out:
  print(f'CAND {x:.2f} {yy:.2f} {((yd + 180) % 360) - 180} {sc * 100:.0f}')
