"""Dock: reverse straight into the charging spot until the rear LiDAR sees the wall/charger close behind, or a maximum
odometry distance. Rear stop: nearest LiDAR hit in a +-30 deg cone behind < REAR_STOP (from the LiDAR, which sits
0.094 m in front of the body's rear edge: TF lidar x -0.010, body rear -0.104, URDF mesh) -> ~4 cm gap.
Docked car on 2026-11-09: charger points 1-5 cm behind the body (zone_pts.py). Speed 0.08 m/s (>= dead band 0.03).
  python3 dock.py [MAX_M=0.6]"""
import math, sys, time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

MAX_M = float(sys.argv[1]) if len(sys.argv) > 1 else 0.6
SPEED, REAR_STOP, TIMEOUT_S, CONE = 0.08, 0.135, 12.0, math.radians(30)
rclpy.init(); n = rclpy.create_node('dock'); st = {}
pub = n.create_publisher(Twist, '/controller/cmd_vel', 10)
n.create_subscription(Odometry, '/odom', lambda m: st.__setitem__('odom', m.pose.pose.position), 10)
n.create_subscription(LaserScan, '/scan_raw', lambda m: st.__setitem__('scan', m), qos_profile_sensor_data)
t = time.time()
while ('odom' not in st or 'scan' not in st) and time.time() - t < 3: rclpy.spin_once(n, timeout_sec=0.05)


def rear():
  m = st['scan']; best = 9.0
  for i, r in enumerate(m.ranges):
    a = m.angle_min + i * m.angle_increment
    if m.range_min < r < m.range_max and abs(math.atan2(math.sin(a - math.pi), math.cos(a - math.pi))) < CONE:
      best = min(best, r)
  return best


p0 = st['odom']; x0, y0 = p0.x, p0.y; t0 = time.time(); why = 'timeout'; d = 0.0
while time.time() - t0 < TIMEOUT_S:
  rclpy.spin_once(n, timeout_sec=0.05)
  p = st['odom']; d = math.hypot(p.x - x0, p.y - y0); rr = rear()
  if rr < REAR_STOP: why = f'rear {rr:.3f} m'; break
  if d >= MAX_M: why = 'max distance'; break
  c = Twist(); c.linear.x = -SPEED; pub.publish(c)
for _ in range(5): pub.publish(Twist()); time.sleep(0.05)
print(f'dock: reversed {d:.3f} m by odom in {time.time() - t0:.1f} s, stopped by {why}', flush=True)
