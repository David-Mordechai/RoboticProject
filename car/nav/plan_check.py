# Ask Nav2's planner for a path WITHOUT driving (ComputePathToPose action only) and check it against the car: the
# tightest turn along the path vs the planning turning radius, path length, and whether it reverses.
# Run inside the MentorPi container:  python3 plan_check.py X Y YAW_DEG [X Y YAW_DEG ...]
import math, sys
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose

rclpy.init(); n = rclpy.create_node('plan_check')
cli = ActionClient(n, ComputePathToPose, 'compute_path_to_pose')
if not cli.wait_for_server(timeout_sec=10.0):
  print('no planner'); sys.exit(1)
args = [float(a) for a in sys.argv[1:]]
for i in range(0, len(args) - 2, 3):
  x, y, yaw = args[i], args[i + 1], math.radians(args[i + 2])
  g = ComputePathToPose.Goal(); g.use_start = False; g.planner_id = 'GridBased'
  g.goal = PoseStamped(); g.goal.header.frame_id = 'map'
  g.goal.pose.position.x, g.goal.pose.position.y = x, y
  g.goal.pose.orientation.z, g.goal.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
  f = cli.send_goal_async(g); rclpy.spin_until_future_complete(n, f, timeout_sec=10.0)
  h = f.result()
  if not h or not h.accepted:
    print(f'goal ({x}, {y}, {math.degrees(yaw):.0f}): rejected'); continue
  rf = h.get_result_async(); rclpy.spin_until_future_complete(n, rf, timeout_sec=15.0)
  res = rf.result()
  pts = [(p.pose.position.x, p.pose.position.y) for p in res.result.path.poses] if res else []
  if len(pts) < 3:
    print(f'goal ({x}, {y}, {math.degrees(yaw):.0f}): NO PATH (status {res.status if res else None})'); continue
  length = sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
  # Turning radius from each set of three points ~0.15 m apart (circumradius).
  step = max(1, int(0.15 / max(1e-6, length / len(pts))))
  rmin, rev = float('inf'), 0
  for k in range(0, len(pts) - 2 * step, step):
    a, b, c = pts[k], pts[k + step], pts[k + 2 * step]
    ab, bc, ca = math.dist(a, b), math.dist(b, c), math.dist(c, a)
    area = abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1])) / 2
    if area > 1e-6: rmin = min(rmin, ab * bc * ca / (4 * area))
    if (b[0] - a[0]) * (c[0] - b[0]) + (b[1] - a[1]) * (c[1] - b[1]) < 0: rev += 1
  print(f'goal ({x}, {y}, {math.degrees(yaw):.0f}): {len(pts)} poses, {length:.2f} m, tightest turn radius '
        f'{rmin:.2f} m, direction changes {rev}', flush=True)
rclpy.shutdown()
