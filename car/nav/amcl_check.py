"""Ask AMCL for N no-motion updates (scan match without moving) and print its pose + std. Car must stand still.
  python3 amcl_check.py [N]"""
import math, sys, time
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Empty
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
rclpy.init(); n = rclpy.create_node('amcl_check'); last = {}
n.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', lambda m: last.__setitem__('m', m),
                      QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE))
cli = n.create_client(Empty, '/request_nomotion_update'); cli.wait_for_service(timeout_sec=5)
for _ in range(int(sys.argv[1]) if len(sys.argv) > 1 else 10):
  f = cli.call_async(Empty.Request()); rclpy.spin_until_future_complete(n, f, timeout_sec=2)
  t = time.time()
  while time.time() - t < 0.5: rclpy.spin_once(n, timeout_sec=0.05)
m = last.get('m')
if m:
  p, q, c = m.pose.pose.position, m.pose.pose.orientation, m.pose.covariance
  print(f'AMCL ({p.x:+.3f}, {p.y:+.3f}) yaw {math.degrees(2 * math.atan2(q.z, q.w)):+.1f} deg, '
        f'std x {math.sqrt(max(c[0], 0)):.3f} y {math.sqrt(max(c[7], 0)):.3f} m yaw {math.degrees(math.sqrt(max(c[35], 0))):.1f} deg')
else:
  print('no /amcl_pose')
