"""Set the AMCL initial pose (map frame, std 0.10 m / 5 deg) and VERIFY it: publish /initialpose until /amcl_pose is
within 0.1 m and 3 deg of it (one publish was silently lost on 2026-11-09: AMCL never logged initialPoseReceived and
trial a_c1_2 started with an 8 deg heading error). Exit code 1 if AMCL never takes it, so a trial script stops.
  python3 set_pose.py X Y YAW_DEG [STD_M STD_DEG]   (pose from scan_map_match.py; default std 0.10 m / 5 deg)"""
import math, sys, time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

x, y, yaw = float(sys.argv[1]), float(sys.argv[2]), math.radians(float(sys.argv[3]))
rclpy.init(); n = rclpy.create_node('set_pose'); last = {}
p = n.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: last.__setitem__('m', m),
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
m = PoseWithCovarianceStamped(); m.header.frame_id = 'map'
m.pose.pose.position.x, m.pose.pose.position.y = x, y
m.pose.pose.orientation.z, m.pose.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
SD, SA = (float(sys.argv[4]), float(sys.argv[5])) if len(sys.argv) > 5 else (0.10, 5.0)
m.pose.covariance[0] = m.pose.covariance[7] = SD ** 2; m.pose.covariance[35] = math.radians(SA) ** 2


def err():
  a = last.get('m')
  if not a: return 9, 999
  q = a.pose.pose.orientation; ay = 2 * math.atan2(q.z, q.w)
  return (math.hypot(a.pose.pose.position.x - x, a.pose.pose.position.y - y),
          abs(math.degrees(math.atan2(math.sin(ay - yaw), math.cos(ay - yaw)))))


t = time.time()
while p.get_subscription_count() == 0 and time.time() - t < 5: rclpy.spin_once(n, timeout_sec=0.1)
for k in range(10):
  m.header.stamp = rclpy.time.Time().to_msg()   # time 0 = "latest TF" (avoids "earlier than the transform cache")
  p.publish(m)
  t = time.time()
  while time.time() - t < 1.5: rclpy.spin_once(n, timeout_sec=0.05)
  d, a = err()
  if d < 0.1 and a < 3:
    print(f'AMCL took the pose ({x}, {y}, {math.degrees(yaw):.1f} deg) after {k + 1} publish(es): off {d:.3f} m {a:.1f} deg'); sys.exit(0)
print(f'AMCL did NOT take the pose: off {d:.2f} m {a:.1f} deg'); sys.exit(1)
