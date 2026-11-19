import math, time, numpy as np, rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
rclpy.init(); n = rclpy.create_node('dsnow'); got = []
n.create_subscription(LaserScan, '/depth_scan', got.append, qos_profile_sensor_data)
t = time.time()
while len(got) < 10 and time.time() - t < 6: rclpy.spin_once(n, timeout_sec=0.1)
m = got[-1]; print('frame', m.header.frame_id, 'angle', round(math.degrees(m.angle_min)), round(math.degrees(m.angle_max)), 'range', m.range_min, m.range_max)
for m in got[-3:]:
  r = np.array(m.ranges); f = np.isfinite(r)
  print('hits', f.sum(), 'min', np.round(np.sort(r[f])[:6], 3), 'at deg', np.round(np.degrees(m.angle_min + np.where(f)[0][np.argsort(r[f])[:6]] * m.angle_increment)))
