"""Undock: drive straight out of the charger by ODOMETRY distance (go_dist.py measured by the LiDAR range ahead, which
jumped 3.77 -> 1.69 m in trial a_c1_1 and stopped after 5 cm). Stops early if the LiDAR sees anything within STOP_M in a
+-25 deg cone ahead. Speed 0.12 m/s (>= dead band 0.03, open-issues 448); commands at 20 Hz (car #1 stop_test).
  python3 undock.py [METRES=0.35]"""
import math, sys, time
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

DIST = float(sys.argv[1]) if len(sys.argv) > 1 else 0.35
SPEED, STOP_M, TIMEOUT_S, CONE = 0.12, 0.25, 8.0, math.radians(25)
rclpy.init(); n = rclpy.create_node('undock'); st = {}
pub = n.create_publisher(Twist, '/controller/cmd_vel', 10)
n.create_subscription(Odometry, '/odom', lambda m: st.__setitem__('odom', m.pose.pose.position), 10)
n.create_subscription(LaserScan, '/scan_raw', lambda m: st.__setitem__('scan', m), qos_profile_sensor_data)
t = time.time()
while ('odom' not in st or 'scan' not in st) and time.time() - t < 3: rclpy.spin_once(n, timeout_sec=0.05)
p0 = st['odom']; x0, y0 = p0.x, p0.y; t0 = time.time(); why = 'timeout'
while time.time() - t0 < TIMEOUT_S:
  rclpy.spin_once(n, timeout_sec=0.05)
  p = st['odom']; d = math.hypot(p.x - x0, p.y - y0)
  if d >= DIST: why = 'distance'; break
  m = st['scan']; near = min((r for i, r in enumerate(m.ranges)
                               if m.range_min < r < m.range_max and abs(math.atan2(math.sin(m.angle_min + i * m.angle_increment),
                                                                                   math.cos(m.angle_min + i * m.angle_increment))) < CONE), default=9.0)
  if near < STOP_M: why = f'obstacle {near:.2f} m ahead'; break
  c = Twist(); c.linear.x = SPEED; pub.publish(c)
for _ in range(5): pub.publish(Twist()); time.sleep(0.05)
print(f'undock: {d:.3f} m by odom in {time.time() - t0:.1f} s, stopped by {why}', flush=True)
sys.exit(0 if why == 'distance' else 1)
