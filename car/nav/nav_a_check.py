"""Phase A static check (car not driving): AMCL pose, topic rates, local costmap obstacles from LiDAR and depth,
Collision Monitor output. Run in the MentorPi container while nav_a.launch.py runs:  python3 nav_a_check.py"""
import math, time
import numpy as np
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data

rclpy.init(); n = rclpy.create_node('nav_a_check')
got = {'amcl': None, 'local': None, 'depth': [], 'scan': [], 'cm': []}
tl = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: got.__setitem__('amcl', m), tl)
n.create_subscription(OccupancyGrid, '/local_costmap/costmap', lambda m: got.__setitem__('local', m), 1)
n.create_subscription(LaserScan, '/depth_scan', lambda m: got['depth'].append(m), qos_profile_sensor_data)
n.create_subscription(LaserScan, '/scan_raw', lambda m: got['scan'].append(time.time()), qos_profile_sensor_data)
n.create_subscription(Twist, '/controller/cmd_vel', lambda m: got['cm'].append(m), 10)
t0 = time.time()
while time.time() - t0 < 8.0:
  rclpy.spin_once(n, timeout_sec=0.1)
a = got['amcl']
if a:
  p, q = a.pose.pose.position, a.pose.pose.orientation
  yaw = math.degrees(2 * math.atan2(q.z, q.w)); c = a.pose.covariance
  print(f'AMCL pose ({p.x:+.2f}, {p.y:+.2f}) yaw {yaw:+.0f} deg, std x {math.sqrt(c[0]):.2f} y {math.sqrt(c[7]):.2f} m')
else:
  print('AMCL: no /amcl_pose yet (published after the first update / initial pose)')
print(f'/scan_raw {len(got["scan"]) / 8:.1f} Hz, /depth_scan {len(got["depth"]) / 8:.1f} Hz')
if got['depth']:
  r = np.array(got['depth'][-1].ranges); f = np.isfinite(r)
  print(f'depth_scan last: {f.sum()} of {len(r)} bins hit, nearest {r[f].min():.2f} m' if f.any() else 'depth_scan last: no hits')
m = got['local']
if m:
  g = np.array(m.data).reshape(m.info.height, m.info.width)
  print(f'local costmap {m.info.width}x{m.info.height}: lethal {(g >= 99).sum()} cells, inflated {((g > 0) & (g < 99)).sum()}, unknown {(g < 0).sum()}')
else:
  print('local costmap: none received')
nz = [c for c in got['cm'] if abs(c.linear.x) > 1e-6 or abs(c.angular.z) > 1e-6]
print(f'/controller/cmd_vel msgs {len(got["cm"])}, non-zero {len(nz)}')
