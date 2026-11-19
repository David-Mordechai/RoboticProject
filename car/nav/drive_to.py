# Drive to map-frame waypoints by steering directly on the SLAM pose, the LiDAR and the depth camera (no Nav2): for
# going through a doorway into space that is not yet on the map. Run inside the MentorPi container:
#   python3 drive_to.py X1 Y1 [X2 Y2 ...]
# "Follow the gap": every 0.1 s, score candidate headings within +-HEAD_SPAN of the car's heading by clearance
# (nearest LiDAR / depth return within a corridor the car's width along that heading) and closeness to the waypoint
# bearing; steer toward the best one (yaw rate capped to what the steering can do at this speed). If no heading has
# CLEAR_MIN of room, back up while steering the other way, then try again. Publishes /controller/cmd_vel at 10 Hz
# (the drive command watchdog stops the motors if this script dies).
import math, sys, time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
import tf2_ros

V = 0.25                 # m/s (user asked for faster, 2026-11-02; was 0.15)
V_BACK = -0.15           # m/s
TURN_AROUND = math.radians(90)   # target further off than this: 3-point turn toward it
TURN_DONE = math.radians(20)    # was 60: in a hallway the gap step (+-HEAD_SPAN) then steered away from the target
                                # and undid the turn, again and again (hall1 return leg, 2026-11-04). Keep < HEAD_SPAN.
WHEELBASE = 0.145        # m, ackermann.py (Hiwonder code; research/01 sec. 3)
STEER_MAX = math.radians(29)   # lower end of the 29-37 deg steering range until measured (research/01)
W_MAX = V * math.tan(STEER_MAX) / WHEELBASE    # yaw rate the steering can reach at V
HALF_W = 0.10            # m: half the car's 159 mm width (research/01 sec. 2) + 2 cm
REACHED = 0.25           # m
PASSED = 0.8             # m: an intermediate waypoint this close but behind the car counts as reached
CLEAR_MIN = 0.40         # m of free corridor needed to drive a heading (LiDAR is ~0.1 m behind the bumper)
LOOK = 1.2               # m: clearance beyond this counts as fully clear
HEAD_SPAN = math.radians(25)   # only drive headings inside the depth camera's view (56.6 deg FOV, measured car #1):
                               # a chair base outside it is invisible to both sensors (car #1 pinned on one, 2026-11-02)
BACKUP_S = 1.8           # s of reversing when boxed in
STALL_S = 1.2            # s: driving forward but the LiDAR picture does not change -> blocked by something unseen
STALL_CHANGE = 0.01      # m: median |change| of the LiDAR ranges below this = not moving
BLOCK_MEMORY_S = 30.0    # s a stall location stays an obstacle
TIMEOUT = 240.0          # s: whole out-and-back trip in one run (2026-11-04)
REAR_CLEAR = 0.30        # m free behind base_footprint before reversing (ESTIMATE: body ends 0.034 m behind it, + reach
                         # of 1 s at V_BACK + margin). Reversing had no check: car #1 backed into a chair base (2026-11-04)
V_FAST, V_SLOW = 0.35, 0.15   # m/s: speed with LOOK of clear corridor ahead / at CLEAR_MIN (0.15 drove fine on
                              # 2026-11-02); in between linear. Was a constant V with hard stops (user: "not smooth")
W_STEP = 0.15            # rad/s: largest change of the yaw-rate command per 0.1 s step (smooth steering)
KEEP_BONUS = 0.08        # score bonus for a heading within 5 deg of the last one (no jumping between equal gaps)
FRONT_WAIT_S = 1.5       # s to wait when only the front is blocked (people step aside) before backing up
TURN_V_BACK = -0.25      # m/s reversing in the 3-point turn (was V_BACK -0.15 for a fixed BACKUP_S: slow, many legs)
TURN_FWD_MIN = 0.30      # m: forward leg goes on until this little room is left at full lock (was CLEAR_MIN + 0.05)
TURN_BACK_MAX_S = 3.0    # s: longest reverse leg; it ends earlier when the rear check (REAR_CLEAR) sees something
WAIT_S = 10.0            # s to wait when boxed in front and back (a person walking past) before giving up
MEM_LIDAR_GAP = 0.10     # m: a depth point this close to a LiDAR return is at LiDAR height, not remembered
MEM_S, MEM_RANGE = 15.0, 1.5   # depth points (below the LiDAR plane) remembered in the map frame, so chair legs seen
                               # while facing them still count after the car turns or reverses

wps = [(float(sys.argv[i]), float(sys.argv[i + 1])) for i in range(1, len(sys.argv) - 1, 2)]
rclpy.init(); n = rclpy.create_node('drive_to')
pub = n.create_publisher(Twist, '/controller/cmd_vel', 10)
# Route for the dashboard: the car's position then the remaining waypoints, map frame.
route_pub = n.create_publisher(Path, '/drive_to/path', 10)
scan = {}
n.create_subscription(LaserScan, '/scan_raw', lambda m: scan.__setitem__('m', m), qos_profile_sensor_data)
# Depth camera obstacles below the LiDAR plane (car/perception/depth_scan.py), e.g. chair bases.
n.create_subscription(LaserScan, '/depth_scan', lambda m: scan.__setitem__('d', m), 10)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf, n)


def pose():
  t = buf.lookup_transform('map', 'base_footprint', rclpy.time.Time()); q = t.transform.rotation
  return t.transform.translation.x, t.transform.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y),
                                                                          1 - 2 * (q.y * q.y + q.z * q.z))


mem = []             # (time, map x, map y) of depth points (remember)


def remember(now, x, y, yaw):
  """Keep depth points the LiDAR does not see (no LiDAR return within MEM_LIDAR_GAP): low objects such as chair bases
  or door stops. What the LiDAR sees (walls, legs) it keeps seeing live all round; remembering it left ghosts of people
  who had walked away (hall3, 2026-11-04)."""
  if 'd' in scan and 'm' in scan:
    lm = scan['m']
    lid = [(r * math.cos(lm.angle_min + i * lm.angle_increment), r * math.sin(lm.angle_min + i * lm.angle_increment))
           for i, r in enumerate(lm.ranges) if lm.range_min < r < MEM_RANGE + 0.5 and math.isfinite(r)]
    m = scan['d']; c, s_ = math.cos(yaw), math.sin(yaw)
    for i, r in enumerate(m.ranges[::2]):
      if m.range_min < r < MEM_RANGE and math.isfinite(r):
        a = m.angle_min + 2 * i * m.angle_increment; px, py = r * math.cos(a), r * math.sin(a)
        if all((px - lx) ** 2 + (py - ly) ** 2 > MEM_LIDAR_GAP ** 2 for lx, ly in lid):
          mem.append((now, x + c * px - s_ * py, y + s_ * px + c * py))
  mem[:] = [p for p in mem if now - p[0] < MEM_S]


def points(x=None, y=None, yaw=None):
  """Obstacle points (x, y) in the car frame from both scans (LiDAR and depth are both near the base origin), plus the
  remembered depth points when the pose is given."""
  pts = []
  if x is not None:
    c0, s0 = math.cos(yaw), math.sin(yaw)
    pts += [((bx - x) * c0 + (by - y) * s0, -(bx - x) * s0 + (by - y) * c0) for _, bx, by in mem]
  for m in [scan[k] for k in ('m', 'd') if k in scan]:
    for i, r in enumerate(m.ranges):
      if m.range_min < r < min(m.range_max, LOOK + 0.5) and math.isfinite(r):
        a = m.angle_min + i * m.angle_increment
        pts.append((r * math.cos(a), r * math.sin(a)))
  return pts


def clearance(pts, h):
  """Free length along heading h of a corridor HALF_W either side (points behind the car are ignored)."""
  c, s = math.cos(h), math.sin(h); best = LOOK
  for x, y in pts:
    along, across = x * c + y * s, -x * s + y * c
    if 0.0 < along < best and abs(across) < HALF_W: best = along
  return best


def rear_free(pts):
  """Room to reverse: the straight corridor behind the car, HALF_W + 3 cm either side (the swinging nose ignored)."""
  return all(not (-REAR_CLEAR < px < 0.0 and abs(py) < HALF_W + 0.03) for px, py in pts)


def send(v, w):
  global stall_ref
  if v <= 0.0:
    stall_ref = None     # the stall check only makes sense while driving forward (waiting looked like a stall: hall4)
  t = Twist(); t.linear.x = float(v); t.angular.z = float(w); pub.publish(t)


cands = [math.radians(d) for d in range(-25, 26, 5)]
blocked = []         # (time, map x, map y): points in front of the car where it stalled
turning = False; phase = 'fwd'; phase_t = 0.0; boxed_since = None; front_since = None; last_h = 0.0; last_w = 0.0; turn_dir = None
stall_ref = None     # (time, LiDAR ranges) when forward driving started being checked
t0 = time.monotonic(); k = 0; reason = 'timeout'; last_print = 0.0; back_until = 0.0; back_w = 0.0
try:
  while time.monotonic() - t0 < TIMEOUT:
    rclpy.spin_once(n, timeout_sec=0.1)
    if 'm' not in scan: continue
    try: x, y, yaw = pose()
    except Exception: continue
    now = time.monotonic()
    remember(now, x, y, yaw)
    if now < back_until:
      if rear_free(points(x, y, yaw)):
        send(V_BACK, back_w); stall_ref = None; continue
      send(0.0, 0.0); back_until = 0.0
      print(f'{now - t0:5.1f} s something behind the car: stopped reversing', flush=True)
    # Stall check: commanded forward, yet the LiDAR ranges are unchanged -> blocked by something no sensor sees.
    rs = scan['m'].ranges
    if stall_ref is None:
      if last_w is not None and front_since is None and boxed_since is None and not turning:
        stall_ref = (now, list(rs))
    if stall_ref is not None and now - stall_ref[0] >= STALL_S:
      diffs = sorted(abs(a - b) for a, b in zip(rs, stall_ref[1]) if math.isfinite(a) and math.isfinite(b))
      if diffs and diffs[len(diffs) // 2] < STALL_CHANGE:
        for off in (-0.08, 0.0, 0.08):          # remember the car's front edge as an obstacle
          blocked.append((now, x + 0.2 * math.cos(yaw) - off * math.sin(yaw), y + 0.2 * math.sin(yaw) + off * math.cos(yaw)))
        tb0 = math.atan2(ty_prev - y, tx_prev - x) - yaw if 'tx_prev' in dir() else 0.0
        back_w = math.copysign(W_MAX * abs(V_BACK) / V, math.sin(tb0) or 1.0)
        back_until = now + BACKUP_S
        print(f'{now - t0:5.1f} s STALLED at ({x:.2f}, {y:.2f}) yaw {math.degrees(yaw):.0f}: marking it, backing up', flush=True)
        stall_ref = None
        continue
      stall_ref = (now, list(rs))
    tx, ty = wps[k]
    tx_prev, ty_prev = tx, ty
    if now - last_print > 1.0:
      route = Path(); route.header.frame_id = 'map'; route.header.stamp = n.get_clock().now().to_msg()
      for px, py in [(x, y)] + wps[k:]:
        ps = PoseStamped(); ps.header = route.header; ps.pose.position.x, ps.pose.position.y = px, py
        route.poses.append(ps)
      route_pub.publish(route)
    d = math.hypot(tx - x, ty - y)
    tb = math.atan2(math.sin(math.atan2(ty - y, tx - x) - yaw), math.cos(math.atan2(ty - y, tx - x) - yaw))
    # Passed it: an intermediate waypoint close by but behind the car counts as reached (hall6, 2026-11-04: the car
    # missed the doorway waypoint by 0.3 m, turned around for it, circled and boxed itself in).
    if d < REACHED or (k < len(wps) - 1 and d < PASSED and abs(tb) > math.radians(90)):
      print(f'{now - t0:5.1f} s reached waypoint {k + 1} ({tx}, {ty})', flush=True)
      k += 1
      if k == len(wps): reason = 'all waypoints reached'; break
      continue
    tb = math.atan2(math.sin(math.atan2(ty - y, tx - x) - yaw), math.cos(math.atan2(ty - y, tx - x) - yaw))
    pts = points(x, y, yaw)
    # Turn-around: the follow-the-gap headings only span the camera's view, so a target behind the car would never be
    # reached in a narrow hallway. Alternate full-lock forward / reverse arcs with the same yaw direction (3-point turn)
    # until the target is within TURN_DONE.
    if abs(tb) > TURN_AROUND or (turning and abs(tb) > TURN_DONE):
      if turn_dir is None:
        phase = 'fwd'
        # Turn the short way; when the target is nearly behind, turn toward the side with more room.
        if abs(tb) < math.radians(150):
          turn_dir = math.copysign(1.0, tb)
        else:
          turn_dir = 1.0 if clearance(pts, math.radians(60)) >= clearance(pts, math.radians(-60)) else -1.0
      turning = True
      fwd_room = clearance(pts, turn_dir * math.radians(25))
      if fwd_room >= TURN_FWD_MIN and phase != 'back':
        phase = 'fwd'; send(V, turn_dir * W_MAX * 0.999)
      else:
        if phase != 'back': phase = 'back'; phase_t = now
        if rear_free(pts) and now - phase_t < TURN_BACK_MAX_S:
          send(TURN_V_BACK, turn_dir * W_MAX * abs(TURN_V_BACK) / V)   # same yaw direction while reversing
        elif fwd_room >= TURN_FWD_MIN:
          phase = 'fwd'
        else:
          send(0.0, 0.0)
          boxed_since = boxed_since or now
          if now - boxed_since > WAIT_S: reason = 'boxed in front and back: gave up'; break
          continue
      if now - last_print > 1.0:
        print(f'{now - t0:5.1f} s turning around ({phase}) at ({x:.2f}, {y:.2f}) yaw {math.degrees(yaw):.0f} '
              f'target {math.degrees(tb):+.0f} deg', flush=True); last_print = now
      continue
    turning = False; turn_dir = None
    blocked[:] = [b for b in blocked if now - b[0] < BLOCK_MEMORY_S]
    c0, s0 = math.cos(yaw), math.sin(yaw)
    pts += [((bx - x) * c0 + (by - y) * s0, -(bx - x) * s0 + (by - y) * c0) for _, bx, by in blocked]
    scored = []
    for h in cands:
      cl = clearance(pts, h)
      if cl >= CLEAR_MIN:
        scored.append((cl / LOOK - 0.8 * abs(math.atan2(math.sin(h - tb), math.cos(h - tb))) / math.pi
                       + (KEEP_BONUS if abs(h - last_h) < math.radians(5.1) else 0.0), h, cl))
    if not scored and rear_free(pts):
      # Front blocked: wait FRONT_WAIT_S first (a person usually steps aside), then back up.
      if front_since is None:
        front_since = now
        print(f'{now - t0:5.1f} s front blocked at ({x:.2f}, {y:.2f}): waiting', flush=True)
      if now - front_since < FRONT_WAIT_S:
        send(0.0, 0.0); last_w = 0.0; continue
    if not scored:
      # Boxed in: reverse, steering so the nose swings toward the target side.
      # The twist's angular.z is the yaw rate in either direction of travel (the driver steers by w / v), so to swing
      # the nose toward the target while reversing keep the target's sign (a 3-point turn), at the reversing speed.
      if not rear_free(pts):
        send(0.0, 0.0)
        if boxed_since is None:
          boxed_since = now
          print(f'{now - t0:5.1f} s boxed in front and back at ({x:.2f}, {y:.2f}): waiting', flush=True)
        if now - boxed_since > WAIT_S: reason = f'boxed in front and back at ({x:.2f}, {y:.2f}): gave up'; break
        continue
      back_w = math.copysign(W_MAX * abs(V_BACK) / V, tb)
      back_until = now + BACKUP_S; front_since = None
      print(f'{now - t0:5.1f} s boxed in at ({x:.2f}, {y:.2f}): backing up', flush=True)
      continue
    boxed_since = None; front_since = None
    _, h, cl = max(scored)
    v = V_SLOW + (V_FAST - V_SLOW) * max(0.0, min(1.0, (cl - CLEAR_MIN) / (LOOK - CLEAR_MIN)))
    w_max = v * math.tan(STEER_MAX) / WHEELBASE
    w = max(-w_max, min(w_max, 1.5 * h * v / V))         # same path curvature as before at any speed
    w = max(last_w - W_STEP, min(last_w + W_STEP, w))
    last_h, last_w = h, w
    send(v, w)
    if now - last_print > 1.0:
      print(f'{now - t0:5.1f} s at ({x:.2f}, {y:.2f}) yaw {math.degrees(yaw):.0f}  wp{k + 1} {d:.2f} m  target '
            f'{math.degrees(tb):+.0f} deg  steer to {math.degrees(h):+.0f} deg (clear {cl:.2f} m)', flush=True)
      last_print = now
finally:
  for _ in range(5): send(0.0, 0.0); time.sleep(0.05)
print(reason, flush=True)
n.destroy_node(); rclpy.shutdown()
sys.exit(0 if reason == 'all waypoints reached' else 1)
