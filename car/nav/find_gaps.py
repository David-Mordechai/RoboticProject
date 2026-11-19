# Find doorway candidates in the current LiDAR scan: runs of rays that reach far (> FAR_M) bounded by nearer returns,
# whose opening is at least DOOR_MIN_M wide where it meets the near wall. Prints them in the car frame and the map
# frame (via TF map -> base_footprint). Run inside the MentorPi container.
# Door width: common interior doors are ~0.8-0.9 m clear; we accept >= 0.6 m (our threshold, the car is 0.16 m wide).
import math, rclpy, time
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import tf2_ros

FAR_M, DOOR_MIN_M = 2.5, 0.6
rclpy.init(); n = rclpy.create_node('find_gaps')
got = []
n.create_subscription(LaserScan, '/scan_raw', got.append, qos_profile_sensor_data)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
pose = None; t0 = time.monotonic()
while (len(got) < 3 or pose is None) and time.monotonic() - t0 < 8:
  rclpy.spin_once(n, timeout_sec=0.1)
  try:
    t = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()); q = t.transform.rotation
    pose = (t.transform.translation.x, t.transform.translation.y,
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
  except Exception:
    pass
m = got[-1]
r = [x if m.range_min < x < m.range_max else float('inf') for x in m.ranges]
N = len(r); ang = [m.angle_min + i * m.angle_increment for i in range(N)]
far = [x > FAR_M for x in r]
print(f'pose in map: {pose}')
# Rotate so the scan starts on a near ray, then collect runs of far rays.
start = next(i for i in range(N) if not far[i])
runs, i = [], 0
while i < N:
  k = (start + i) % N
  if far[k]:
    j = i
    while j < N and far[(start + j) % N]: j += 1
    a, b = (start + i - 1) % N, (start + j) % N       # the near rays bounding the run
    pa = (r[a] * math.cos(ang[a]), r[a] * math.sin(ang[a])); pb = (r[b] * math.cos(ang[b]), r[b] * math.sin(ang[b]))
    width = math.dist(pa, pb)
    mid_i = (start + (i + j) // 2) % N
    if width >= DOOR_MIN_M:
      runs.append((width, ang[mid_i], (pa[0] + pb[0]) / 2, (pa[1] + pb[1]) / 2, min(r[(start + x) % N] for x in range(i, j))))
    i = j
  else:
    i += 1
for w, a, cx, cy, deep in sorted(runs, key=lambda z: -z[0]):
  line = f'gap {w:.2f} m wide, bearing {math.degrees(a):+.0f} deg, opening centre car-frame ({cx:.2f}, {cy:.2f}), clear beyond >= {deep:.1f} m'
  if pose:
    c, s = math.cos(pose[2]), math.sin(pose[2])
    line += f', map ({pose[0] + c * cx - s * cy:.2f}, {pose[1] + s * cx + c * cy:.2f})'
  print(line)
if not runs: print('no gaps >= %.1f m with far returns' % DOOR_MIN_M)
