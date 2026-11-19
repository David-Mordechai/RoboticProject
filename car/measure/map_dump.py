import rclpy, numpy as np, time, math
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
import tf2_ros
rclpy.init(); n = rclpy.create_node('map_dump'); got = []
q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(OccupancyGrid, '/map', got.append, q)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n); pose = None; t0 = time.monotonic()
while (not got or pose is None) and time.monotonic() - t0 < 10:
  rclpy.spin_once(n, timeout_sec=0.1)
  try:
    t = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()); r = t.transform.rotation
    pose = (t.transform.translation.x, t.transform.translation.y, math.atan2(2*(r.w*r.z), 1-2*r.z*r.z))
  except Exception: pass
m = got[-1]; i = m.info
np.savez('/home/ubuntu/shared/map_now.npz', grid=np.array(m.data, np.int8).reshape(i.height, i.width), res=i.resolution,
         origin=(i.origin.position.x, i.origin.position.y), pose=pose)
print('saved', i.width, i.height, i.resolution, pose)
