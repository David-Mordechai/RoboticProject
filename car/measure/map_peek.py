# Snapshot of the live slam map with the car's pose, as a PNG for remote driving. Run in the MentorPi container.
# Usage: python3 map_peek.py OUT.png
import math, sys, time
import cv2, numpy as np, rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
import tf2_ros

rclpy.init(); n = rclpy.create_node('map_peek')
got = {}
qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(OccupancyGrid, '/map', lambda m: got.__setitem__('m', m), qos)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)
t_end = time.monotonic() + 8
pose = None
while time.monotonic() < t_end and ('m' not in got or pose is None):
  rclpy.spin_once(n, timeout_sec=0.1)
  try:
    t = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
    q = t.transform.rotation
    pose = (t.transform.translation.x, t.transform.translation.y,
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
  except Exception:
    pass
if 'm' not in got:
  print('NO MAP'); sys.exit(1)
m = got['m']; w, h, res = m.info.width, m.info.height, m.info.resolution
ox, oy = m.info.origin.position.x, m.info.origin.position.y
g = np.array(m.data, dtype=np.int16).reshape(h, w)
img = np.full((h, w, 3), 160, np.uint8); img[g == 0] = 255; img[g > 50] = 0
img = np.flipud(img).copy()
k = max(1, int(600 / max(w, h)))
img = cv2.resize(img, (w * k, h * k), interpolation=cv2.INTER_NEAREST)
def px(x, y): return int((x - ox) / res * k), int((h - (y - oy) / res) * k)
# 1 m grid with labels, so distances can be read off the picture.
for gx in range(math.floor(ox), math.ceil(ox + w * res) + 1):
  cv2.line(img, px(gx, oy), px(gx, oy + h * res), (220, 200, 200), 1)
  cv2.putText(img, str(gx), px(gx, oy) [0:1] + (h * k - 4,), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 200), 1)
for gy in range(math.floor(oy), math.ceil(oy + h * res) + 1):
  cv2.line(img, px(ox, gy), px(ox + w * res, gy), (220, 200, 200), 1)
  cv2.putText(img, str(gy), (2, px(ox, gy)[1] - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 200), 1)
if pose:
  x, y, a = pose
  cv2.circle(img, px(x, y), 6, (0, 0, 255), -1)
  cv2.arrowedLine(img, px(x, y), px(x + 0.4 * math.cos(a), y + 0.4 * math.sin(a)), (0, 0, 255), 2, tipLength=0.4)
  print(f'pose x={x:.2f} y={y:.2f} yaw={math.degrees(a):.0f}deg')
print(f'map {w}x{h} @ {res} m, origin ({ox:.2f},{oy:.2f})')
cv2.imwrite(sys.argv[1], img)
