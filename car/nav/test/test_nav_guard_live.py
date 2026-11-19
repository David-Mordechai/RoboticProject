"""Live test of nav_guard.py on the car WITHOUT moving it: start a second guard with renamed topics
  python3 nav_guard.py --ros-args -r __node:=nav_guard_test -r /controller/cmd_vel:=/gt_out -r /cmd_vel_guard_in:=/gt_in \
     -r /scan_raw:=/gt_scan -r /nav_guard/blocked:=/gt_blocked
then run this: it feeds /gt_scan at 10 Hz (real /tf keeps odom + AMCL fresh), sends Twist x=0.123 on /gt_in at 20 Hz,
pauses the scan feed for 1.0 s, resumes. Expect: commands pass, then zeros within ~0.35 s of the pause, then pass again
~0.5 s after the scan resumes."""
import time, rclpy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
rclpy.init(); n = rclpy.create_node('gt_driver'); out = []
sp = n.create_publisher(LaserScan, '/gt_scan', 10); cp = n.create_publisher(Twist, '/gt_in', 10)
n.create_subscription(Twist, '/gt_out', lambda m: out.append((time.monotonic(), m.linear.x)), 50)
t0 = time.monotonic(); next_scan = next_cmd = t0
while time.monotonic() - t0 < 5.0:
  now = time.monotonic(); t = now - t0
  if now >= next_scan:
    if not (2.0 <= t < 3.0):                       # scan pause 2.0-3.0 s
      s = LaserScan(); s.header.stamp = n.get_clock().now().to_msg(); s.header.frame_id = 'lidar_frame'; sp.publish(s)
    next_scan += 0.1
  if now >= next_cmd:
    m = Twist(); m.linear.x = 0.123; cp.publish(m); next_cmd += 0.05
  rclpy.spin_once(n, timeout_sec=0.005)
ev = [(round(a - t0, 2), x) for a, x in out]
passed = [t for t, x in ev if x > 0.1]; zeros = [t for t, x in ev if x == 0.0]
first_zero = min([t for t in zeros if t >= 2.0], default=None)
back = min([t for t in passed if t >= 3.0], default=None)
print(f'{len(ev)} outputs; passed before pause: {len([t for t in passed if 1.0 < t < 2.0])}/~20; '
      f'first zero after pause start: {first_zero} s (expect <= 2.4); passed during 2.4-3.0: '
      f'{len([t for t in passed if 2.4 <= t < 3.0])} (expect 0); passes again at {back} s (expect ~3.5)')
ok = first_zero is not None and first_zero <= 2.4 and not [t for t in passed if 2.4 <= t < 3.0] and back is not None and 3.3 <= back <= 3.8
print('PASS' if ok else 'FAIL')
