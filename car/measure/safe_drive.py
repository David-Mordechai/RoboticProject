# Floor driving for remote mapping: one short, speed-capped move with a LiDAR stop, then report clearances.
# Usage (in the MentorPi container): python3 safe_drive.py V W SECONDS [cmd_topic] [scan_topic]
#   V m/s (capped at 0.15), W rad/s angular.z as the stock Ackermann driver takes it; SECONDS 0 = only look.
# Stops at once if anything is within STOP_M in the direction of travel (front or rear sector), on Ctrl-C, or on exit.
import math, sys, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

V_MAX = 0.15          # m/s, our cap for driving near people and furniture
STOP_M = 0.30         # m from the LiDAR: car front is ~0.1 m ahead of it (213 mm long, research/01 §2)
SECTOR = math.radians(35)

v, w, dur = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
cmd_topic = sys.argv[4] if len(sys.argv) > 4 else '/controller/cmd_vel'
scan_topic = sys.argv[5] if len(sys.argv) > 5 else '/scan_raw'
v = max(-V_MAX, min(V_MAX, v))

rclpy.init(); n = rclpy.create_node('safe_drive')
pub = n.create_publisher(Twist, cmd_topic, 10)
scan = {}
n.create_subscription(LaserScan, scan_topic, lambda m: scan.__setitem__('m', m), qos_profile_sensor_data)


def min_in(m, centre, half):
  best = float('inf')
  for i, r in enumerate(m.ranges):
    a = m.angle_min + i * m.angle_increment
    d = math.atan2(math.sin(a - centre), math.cos(a - centre))
    if abs(d) <= half and m.range_min < r < m.range_max and math.isfinite(r):
      best = min(best, r)
  return best


def send(vx, wz):
  t = Twist(); t.linear.x = float(vx); t.angular.z = float(wz); pub.publish(t)


t_end = time.monotonic() + 3.0
while 'm' not in scan and time.monotonic() < t_end:
  rclpy.spin_once(n, timeout_sec=0.1)
if 'm' not in scan:
  print('NO SCAN on', scan_topic, '- not moving'); sys.exit(1)

reason = 'done'
try:
  t_end = time.monotonic() + dur
  while time.monotonic() < t_end:
    rclpy.spin_once(n, timeout_sec=0.05)
    ahead = min_in(scan['m'], 0.0 if v >= 0 else math.pi, SECTOR)
    if v != 0 and ahead < STOP_M:
      reason = f'STOPPED: obstacle {ahead:.2f} m {"ahead" if v >= 0 else "behind"}'; break
    send(v, w)
except KeyboardInterrupt:
  reason = 'interrupted'
finally:
  for _ in range(5):
    send(0.0, 0.0); time.sleep(0.02)

for _ in range(5):
  rclpy.spin_once(n, timeout_sec=0.05)
m = scan['m']
names = ['front', 'front-left', 'left', 'rear-left', 'rear', 'rear-right', 'right', 'front-right']
print(reason)
print('  '.join(f'{k}={min_in(m, i * math.pi / 4, math.pi / 8):.2f}' for i, k in enumerate(names)))
n.destroy_node(); rclpy.shutdown()
