# Per-second timeline of a run bag: odom position, controller wish (cmd_vel_nav), motor command (/controller/cmd_vel),
# nearest LiDAR and depth_scan hit in a +-30 deg cone ahead (and behind for LiDAR).
import sys, math, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r = rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri=sys.argv[1], storage_id='sqlite3'), rosbag2_py.ConverterOptions('cdr', 'cdr'))
types = {t.name: t.type for t in r.get_all_topics_and_types()}
want = {'/odom', '/cmd_vel_nav', '/controller/cmd_vel', '/scan_raw', '/depth_scan'}
B = {}; t0 = None


def near(m, centre, half=math.radians(30), dx=0.0):
  rr = np.array(m.ranges); a = m.angle_min + np.arange(len(rr)) * m.angle_increment
  d = np.abs(np.arctan2(np.sin(a - centre), np.cos(a - centre)))
  ok = np.isfinite(rr) & (rr > m.range_min) & (d < half)
  return float(rr[ok].min()) if ok.any() else 9.0


while r.has_next():
  topic, data, t = r.read_next()
  if topic not in want: continue
  if t0 is None: t0 = t
  s = int((t - t0) / 1e9); b = B.setdefault(s, {'nav': [], 'mot': [], 'lf': [], 'lb': [], 'df': []})
  m = deserialize_message(data, get_message(types[topic]))
  if topic == '/odom': b['pos'] = (m.pose.pose.position.x, m.pose.pose.position.y)
  elif topic == '/cmd_vel_nav': b['nav'].append((m.linear.x, m.angular.z))
  elif topic == '/controller/cmd_vel': b['mot'].append((m.linear.x, m.angular.z))
  elif topic == '/scan_raw': b['lf'].append(near(m, 0)); b['lb'].append(near(m, math.pi))
  elif topic == '/depth_scan': b['df'].append(near(m, 0))
f = lambda v: f'{np.mean([x[0] for x in v]):+.2f}/{np.mean([x[1] for x in v]):+.2f}' if v else '   --    '
mn = lambda v: f'{min(v):.2f}' if v else ' -- '
print(' s   odom x,y        nav v/w       motor v/w     lidar fwd/back  depth fwd')
for s in sorted(B):
  b = B[s]; p = b.get('pos', (float('nan'),) * 2)
  print(f'{s:3d} ({p[0]:+.2f},{p[1]:+.2f})  {f(b["nav"]):>12}  {f(b["mot"]):>12}   {mn(b["lf"])}/{mn(b["lb"])}      {mn(b["df"])}')
