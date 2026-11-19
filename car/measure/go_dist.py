# Drive straight a set distance, closed loop on the LiDAR range straight ahead (or behind), with a stop distance.
# Usage (in the MentorPi container): python3 go_dist.py METRES [SIGN]
#   METRES > 0 = physically forward, < 0 = backward. SIGN = sign of cmd_vel linear.x that moves the car physically
#   forward: car #1 drove backward on +x on 2026-11-02 (LiDAR 0 deg = front, confirmed by the user), so SIGN -1 there.
import math, statistics, sys, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

SPEED = 0.12          # m/s commanded
STOP_M = 0.30         # m, anything closer in the travel direction stops the car
TIMEOUT_S = 25.0

target, sign = float(sys.argv[1]), float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
centre = 0.0 if target > 0 else math.pi
rclpy.init(); n = rclpy.create_node('go_dist')
pub = n.create_publisher(Twist, '/controller/cmd_vel', 10)
scan = {}
n.create_subscription(LaserScan, '/scan_raw', lambda m: scan.__setitem__('m', m), qos_profile_sensor_data)


def ranges(half):
  m = scan['m']; out = []
  for i, r in enumerate(m.ranges):
    a = m.angle_min + i * m.angle_increment
    if abs(math.atan2(math.sin(a - centre), math.cos(a - centre))) <= half and m.range_min < r < m.range_max:
      out.append(r)
  return out


def send(v):
  t = Twist(); t.linear.x = float(v); pub.publish(t)


t0 = time.monotonic()
while 'm' not in scan and time.monotonic() - t0 < 3: rclpy.spin_once(n, timeout_sec=0.1)
start = statistics.median(ranges(math.radians(5)))
print(f'start: {start:.3f} m to the obstacle {"ahead" if target > 0 else "behind"}')
reason, travelled = 'timeout', 0.0
try:
  t0 = time.monotonic()
  while time.monotonic() - t0 < TIMEOUT_S:
    rclpy.spin_once(n, timeout_sec=0.05)
    near = min(ranges(math.radians(35)) or [99])
    v5 = ranges(math.radians(5))
    if v5: travelled = start - statistics.median(v5)
    if near < STOP_M: reason = f'STOPPED: obstacle {near:.2f} m'; break
    if travelled < -0.08: reason = 'STOPPED: moving the wrong way (check SIGN)'; break
    if travelled >= abs(target): reason = 'reached'; break
    send(sign * SPEED * (1 if target > 0 else -1))
except KeyboardInterrupt:
  reason = 'interrupted'
finally:
  for _ in range(5): send(0.0); time.sleep(0.02)
time.sleep(0.5)
for _ in range(5): rclpy.spin_once(n, timeout_sec=0.1)
end = statistics.median(ranges(math.radians(5)))
print(f'{reason}: travelled {start - end:.3f} m by LiDAR in {time.monotonic() - t0:.1f} s, now {end:.3f} m')
n.destroy_node(); rclpy.shutdown()
