# Points of /scan_raw (lidar at base_footprint +(-0.010,0)) and /depth_scan (frame base_footprint) inside the
# Collision Monitor stop zone (x -0.164..0.219, |y| <= 0.119) and slow zone (x -0.314..0.519, |y| <= 0.189).
import math, time, numpy as np, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
rclpy.init(); n = rclpy.create_node('zone_pts'); got = {}
n.create_subscription(LaserScan, '/scan_raw', lambda m: got.__setitem__('scan', m), qos_profile_sensor_data)
n.create_subscription(LaserScan, '/depth_scan', lambda m: got.__setitem__('depth', m), qos_profile_sensor_data)
t = time.time()
while len(got) < 2 and time.time() - t < 6: rclpy.spin_once(n, timeout_sec=0.1)
for k, dx in (('scan', -0.010), ('depth', 0.0)):
  m = got.get(k)
  if not m: print(k, 'none'); continue
  r = np.array(m.ranges); a = m.angle_min + np.arange(len(r)) * m.angle_increment
  ok = np.isfinite(r) & (r >= m.range_min) & (r <= m.range_max); r, a = r[ok], a[ok]
  x, y = dx + r * np.cos(a), r * np.sin(a)
  stop = (x > -0.164) & (x < 0.219) & (abs(y) < 0.119)
  slow = (x > -0.314) & (x < 0.519) & (abs(y) < 0.189)
  print(f'{k} ({m.header.frame_id}): range_min {m.range_min}, {stop.sum()} pts in STOP, {slow.sum()} in SLOW')
  for xi, yi, ri, ai in list(zip(x[stop], y[stop], r[stop], a[stop]))[:15]:
    print(f'   stop pt x {xi:+.3f} y {yi:+.3f}  (r {ri:.3f} at {math.degrees(ai):.0f} deg)')
