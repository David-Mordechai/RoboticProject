# Drive a trip as a chain of Nav2 goals (map frame), e.g. charger -> door -> 5 m into the hallway -> back. Nav2 refuses
# goals outside the map built so far ("Goal pose is out of costmap"), so a goal that fails to plan is retried every 2 s
# while the car keeps mapping, for up to RETRY_S. Run inside the MentorPi container:
#   python3 nav_trip.py X1 Y1 YAW1_DEG  X2 Y2 YAW2_DEG ...
# 2026-11-16: Nav2 reported SUCCEEDED with a broken TF tree (pose frozen, then 0,0) after car #1 hit a table leg, and the
# Gate A loop drove on. So a goal now counts only if OUR pose check agrees: map->odom (AMCL) and odom->base_footprint both
# arrived within POSE_FRESH_S, and the composed pose is within GOAL_XY_M of the goal. Last line:
#   RESULT ok|FAIL <reason> recoveries <max over the trip> guard_blocks <rising edges of /nav_guard/blocked>
import math, sys, time
import rclpy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

POSE_FRESH_S = 0.5    # s, design (nav_guard.py: AMCL 9.3 Hz, odom 28.5 Hz measured 2026-11-16)
TF_LISTEN_S = 0.6     # s: > 5 AMCL map->odom messages at 9.3 Hz (measured 2026-11-16)
GOAL_XY_M = 0.4       # m: Nav2 xy_goal_tolerance 0.3 (nav2_params_a.yaml) + 0.1 AMCL/odom slack (design)

GOAL_TIMEOUT_S, RETRY_S = 150.0, 150.0   # per goal incl. recoveries (2026-11-09: 90 s cut trial a_c3_1 0.64 m short)
rclpy.init()
nav = BasicNavigator()
# 2026-11-16: nav2_container died (SIGABRT, RCLError "client will not receive response", rmw_response.cpp:154) the moment
# a fresh nav_trip sent its first goal to an already-running Nav2. Known Humble bug: a server whose reply cannot be
# delivered throws (ros2/rmw_fastrtps#751 -> ros2/ros2#1462; fixed upstream Mar-May 2024 in rcl/rclpy, rclcpp action fix
# rclcpp#2464 not confirmed in Humble). Car has rcl 5.3.7 / rclcpp 16.0.8 (Feb 2024). Trigger (our inference): first
# request before discovery has matched the reply path. Round-1 trials that first waited ~2 s for the server all worked.
# So: wait for the server, then DISCOVERY_WAIT_S more before the first goal (design value).
DISCOVERY_WAIT_S = 3.0
nav.nav_to_pose_client.wait_for_server()
_t = time.monotonic()
while time.monotonic() - _t < DISCOVERY_WAIT_S: rclpy.spin_once(nav, timeout_sec=0.1)
TF = {}                                    # 'amcl' / 'odom' -> (arrival monotonic, x, y, yaw)
def _yaw(q): return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
def _on_tf(m):
  for t in m.transforms:
    k = {('map', 'odom'): 'amcl', ('odom', 'base_footprint'): 'odom'}.get((t.header.frame_id, t.child_frame_id))
    if k: TF[k] = (time.monotonic(), t.transform.translation.x, t.transform.translation.y, _yaw(t.transform.rotation))
GUARD = {'on': False, 'blocks': 0}
def _on_guard(m):
  if m.data and not GUARD['on']: GUARD['blocks'] += 1
  GUARD['on'] = m.data
nav.create_subscription(Bool, '/nav_guard/blocked', _on_guard, 10)
REC = [0]

def our_pose():
  """(x, y) in map from the latest map->odom and odom->base_footprint, or a reason string if stale/missing.
  /tf is subscribed only here, for TF_LISTEN_S: subscribed for the whole trip it cost 20-30 % of a Pi core (top, att_c2)."""
  TF.clear(); sub = nav.create_subscription(TFMessage, '/tf', _on_tf, 100); t = time.monotonic()
  while time.monotonic() - t < TF_LISTEN_S: rclpy.spin_once(nav, timeout_sec=0.05)
  nav.destroy_subscription(sub)
  now = time.monotonic()
  for k in ('amcl', 'odom'):
    if k not in TF: return f'no {k} TF'
    if now - TF[k][0] > POSE_FRESH_S: return f'{k} TF stale {now - TF[k][0]:.1f} s'
  _, ax, ay, ayaw = TF['amcl']; _, ox, oy, _ = TF['odom']
  return ax + math.cos(ayaw) * ox - math.sin(ayaw) * oy, ay + math.sin(ayaw) * ox + math.cos(ayaw) * oy

def finish(ok, why):
  print(f'RESULT {"ok" if ok else "FAIL"} {why} recoveries {REC[0]} guard_blocks {GUARD["blocks"]}', flush=True)
  sys.exit(0 if ok else 1)
v = [float(a) for a in sys.argv[1:]]
goals = [(v[i], v[i + 1], math.radians(v[i + 2])) for i in range(0, len(v), 3)]
t_trip = time.monotonic()
for k, (x, y, yaw) in enumerate(goals, 1):
  g = PoseStamped(); g.header.frame_id = 'map'
  g.pose.position.x, g.pose.position.y = x, y
  g.pose.orientation.z, g.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
  t0 = time.monotonic()
  while True:
    g.header.stamp = nav.get_clock().now().to_msg()
    nav.goToPose(g)
    last = 0.0
    while not nav.isTaskComplete():
      fb = nav.getFeedback()
      if fb and time.monotonic() - last > 1.0:
        p = fb.current_pose.pose.position
        print(f'{time.monotonic() - t_trip:6.1f} s goal {k} ({x}, {y}): {fb.distance_remaining:.2f} m left, at '
              f'({p.x:+.2f}, {p.y:+.2f}), recoveries {fb.number_of_recoveries}', flush=True); last = time.monotonic()
      if fb: REC[0] = max(REC[0], fb.number_of_recoveries)
      if time.monotonic() - t0 > GOAL_TIMEOUT_S:
        nav.cancelTask(); print(f'goal {k} timed out'); finish(False, f'goal_{k}_timeout')
    r = nav.getResult()
    if r == TaskResult.SUCCEEDED:
      q = our_pose()
      if isinstance(q, str):
        nav.cancelTask(); print(f'goal {k}: Nav2 says reached but {q}: NOT counted', flush=True); finish(False, q.replace(' ', '_'))
      d = math.hypot(q[0] - x, q[1] - y)
      if d > GOAL_XY_M:
        print(f'goal {k}: Nav2 says reached but our pose ({q[0]:+.2f}, {q[1]:+.2f}) is {d:.2f} m from the goal: NOT counted',
              flush=True); finish(False, f'goal_{k}_off_by_{d:.2f}m')
      print(f'{time.monotonic() - t_trip:6.1f} s goal {k} reached ({time.monotonic() - t0:.1f} s), our pose '
            f'({q[0]:+.2f}, {q[1]:+.2f}) {d:.2f} m from goal', flush=True); break
    if time.monotonic() - t0 > RETRY_S:
      print(f'goal {k} failed: {r}'); finish(False, f'goal_{k}_{r}')
    # 2026-11-09: no extra back-up here any more. Nav2's own tree (navigate_ackermann_b.xml) already recovers (clear,
    # wait, short back-up as the last resort); a back-up here on top was part of the "reversing for no reason" in
    # trial a_c2_1. Wait for the costmaps to refresh, then send the goal again.
    print(f'goal {k} not reached ({r}): waiting 2 s, then the same goal again', flush=True)
    time.sleep(2.0)
print(f'trip done in {time.monotonic() - t_trip:.1f} s')
finish(True, f'{time.monotonic() - t_trip:.1f}s')
