# Car-side node for the sim-trained explore policy (lab task Robotlab-Explore-MentorPiA1): every 50 ms (the sim's 20 Hz
# policy rate) build the 1140-value observation from the real sensors (explore_obs.py, checked against the sim code
# by lab/tests/test_car_explore_obs.py), run the ONNX model, and drive with /controller/cmd_vel.
# Runs inside the MentorPi container, next to the mapping stack (slam_toolbox for /map, depth_scan.py for
# /depth_sectors and /depth_scan, the stock driver, the cmd watchdog):
#   python3 explore_policy.py model.onnx --dry-run       # compute and print actions, publish nothing (first test)
#   python3 explore_policy.py model.onnx --seconds 60    # drive; Ctrl-C or the time limit stops the car
#
# Inputs (sim term <- car source):
#   car_motion     <- /odom twist (EKF, body frame) + the steering angle estimated from our own commands (no servo
#                     readback: ~/pwm_servo/get_state kills the board driver, research/02)
#   lidar_sectors  <- /scan_raw, ray bearings turned into the car frame with TF base_footprint -> lidar frame
#   depth          <- /depth_sectors (depth_scan.py, measured 56.6 deg FOV, same as the sim)
#   map_fine/coarse<- slam_toolbox /map pooled to the sim's 0.10 m cells, at the TF pose map -> base_footprint
# KNOWN DIFFERENCES from the sim (watch for them in the first runs):
#   - Outside the SLAM map counts as unknown here; in the sim, outside the floor patch is occupied. The real map only
#     ends where nothing has been seen yet, which is what "unknown" means to the policy.
#   - slam_toolbox marks free space out to the LiDAR's full range; the sim's built map only out to 3 m.
#   - The sim pose is the car body's origin; here base_footprint (a few cm apart, not measured).
#   - At v = 0 the driver cannot set the steering (it steers by atan(L w / v)), so the steering estimate holds its
#     last value while stopped. How ackermann.py handles v = 0 is not verified.
# Safety (ours, not in the sim): stop if a LiDAR or depth point is inside the car's corridor within STOP_FRONT
# (forward) / STOP_REAR (reverse), and stop if any input is stale. The cmd watchdog stops the motors if this node dies.
import argparse, collections, math, sys, time
import numpy as np
import onnxruntime as ort
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import tf2_ros
from rclpy.signals import SignalHandlerOptions

import explore_obs as O

STEER_LIMIT = math.radians(29)   # lower end of Hiwonder's 29-37 deg until measured (research/01); sim drew 29-37
STEER_RATE = math.radians(60) / 0.16   # rad/s, LD-1501MG servo 0.16 s/60 deg (research/01 sec. 4), as in the sim
HALF_W_REV = 0.10       # m when reversing (= sim SHIELD_HALF_W_REV, goto v6q: wide both ways boxed cars in)
HALF_W = 0.14           # m: half the car's 159 mm width (research/01 sec. 2) + 6 cm (= sim SHIELD_HALF_W since goto
                        # v6p, 2026-11-06; was + 2 cm: the car's corner clipped standing people's feet in sim)
STOP_FRONT = 0.30       # m ahead of base_footprint at standstill: ESTIMATE (bumper vs base_footprint not measured)
STOP_REAR = 0.35        # m behind base_footprint at standstill: ESTIMATE. Was 0.25: no depth camera at the back, the
                        # LiDAR sees legs while toes reach up to 0.19 m closer (sim person.xml); 30% of sim person
                        # contacts (explore v5w) were backing into a person
STOP_TIME = 0.4         # s of travel added to the stop distance (ESTIMATE: 50 ms step + scan age + motor run-down;
                        # not measured at speed): 0.7 m/s -> 0.58 m ahead
MAX_AGE = {'scan': 0.5, 'odom': 0.5, 'depth': 1.0, 'pose': 1.0}   # s before an input counts as stale
DT = 0.05               # s, sim decimation 10 x 5 ms
JUMP_XY, JUMP_YAW, JUMP_HOLD = 0.15, math.radians(10), 2.0   # SLAM correction filter (see pose())
V_MIN = 0.03            # m/s: slowest speed car #1's motors follow (measured 2026-11-03 on the floor, LiDAR distance
                        # to a wall: 0.02 m/s -> 1.1 of 3 cm, 0.03 -> 4.7 of 4.5 cm, 0.05+ within 10%). The sim has no
                        # dead band; commands between V_DEAD and V_MIN are raised to V_MIN.
V_DEAD = 0.005          # m/s: below this the command counts as stop
# Turnaround skill (lab task Robotlab-Turnaround-MentorPiA1, --turn-model): takes over when exploring is blocked and
# hands back once turned. Our switching rules (design choices, tune on the car):
BLOCKED_S = 1.0         # s of continuous obstacle stop -> turn around
STUCK_S, STUCK_M = 3.0, 0.10   # moved < STUCK_M in STUCK_S while commanding motion (dithering) -> turn around
ROUTE_BEHIND = math.radians(120)   # goto: route 1 m ahead further round than this -> the turnaround skill turns the car
                        # to face the route (ours). goto5 (2026-11-05): at the hall U-turn goto v4 kept full speed
                        # forward with the route behind it and drove 5 m away; it never does a 3-point turn there
TURN_DONE_DEG, TURN_DONE_S, TURN_MAX_S, TURN_COOLDOWN_S = 10.0, 0.3, 20.0, 3.0   # sim success: < 10 deg, still 0.3 s
# During a turn the car must pass close to walls (the sim's success needs only 3 cm clearance), so the obstacle stop
# uses shorter reaches. ESTIMATES: base_footprint vs the body (0.179 m ahead of / 0.034 m behind the rear axle,
# turnaround task) is not measured on the car.
TURN_STOP_FRONT, TURN_STOP_REAR, TURN_STOP_TIME = 0.22, 0.20, 0.3
# Rear 0.10 -> 0.20 and a wider band at full lock: in drive 2 on 2026-11-04 car #1 reversed into a chair base during a
# K-turn (the body ends 0.034 m behind base_footprint, so 0.10 m left ~7 cm; the swinging front corners are outside
# HALF_W of the rear-axle arc). ESTIMATES, not tuned in the sim (its turnaround shield used 0.10).
TURN_HALF_W = 0.13
# Multi-point turn skill (2026-11-08, --kturn; = lab teacher.py's multi-point turn, which the imitation students could not
# learn: hidden mode). Starts when the route point 0.75 m on is more than KTURN_ON_DEG off the heading or the stop has
# held the model UNSTICK_AFTER; drives full-lock legs at KTURN_V, forward towards the route, reverse with the opposite lock
# when forward is blocked, the stop using the turn reaches above (sim: manoeuvre mode, explore cfg TURN_*); ends within
# KTURN_OFF_DEG once the model's own move is allowed, then at most EXIT_V for KTURN_COOLDOWN s (teacher v15 values).
KTURN_ON_DEG, KTURN_OFF_DEG, KTURN_V, EXIT_V, KTURN_COOLDOWN = 100.0, 40.0, 0.25, 0.30, 1.0
# Near a turn-back corner of the route (within NEAR_TURN of turn_backs(); = sim hybrid ROBOTLAB_HYBRID_NEAR_TURN) the
# skill drives: pure pursuit on the route preview with the teacher's look-ahead (PP_LOOKAHEAD_T x speed, clamped to
# PP_LOOKAHEAD; Nav2 RPP defaults, teacher.py) and turn speed sqrt(PP_A_LAT / curvature), the multi-point turn when the
# route is behind. Sim 2026-11-08: student v6 hybrid U-turns 85.7% -> 92.0% with it.
NEAR_TURN, PP_LOOKAHEAD_T, PP_LOOKAHEAD, PP_A_LAT = 0.8, 1.5, (0.6, 0.9), 0.8
# Sharp route bend coming (direction of the route preview 0.25 -> 0.5 m vs 0.5 -> 0.75 m differs by more than BEND_DEG;
# sim hybrid ROBOTLAB_HYBRID_BEND_DEG uses 0 -> 0.4 vs 0.4 -> 0.8 m past the progress point) -> pure pursuit as near a U.
# The skill also starts after KTURN_HELD_S held (the scripted back-up keeps UNSTICK_AFTER): door-corner starts were held
# 0.3 s before it took over (sim mean of 3, 2026-11-08: corner hold 1.007 s for a 1.0 s goal).
# Floor goto18 (2026-11-09): the held trigger fired 17 times at the charger desk legs with the route 1-23 deg ahead; each
# turn reversed, then the stall back-up fired, and the car never left the start in 120 s. The held trigger now needs the
# route at least KTURN_HELD_DEG off the heading; nearly straight ahead, the scripted back-up (UNSTICK_AFTER) acts as in goto17.
BEND_DEG, KTURN_HELD_S, KTURN_HELD_DEG = 60.0, 0.1, 45.0
# Steer around (2026-11-09): candidate steering (x steer limit), speed m/s, commit s. Design choices, = lab carskill AV_*.
MEM_REV_REACH = 0.30   # m, = lab MEM_REV_REACH
AV_CAND, AV_V, AV_T = (-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0), 0.25, 0.6   # 0.0: straight, slowly
KTURN_STUCK_S = 1.0     # = teacher v19: both legs blocked on either lock this long -> end the turn, the back-up rule acts
# Obstacle memory: the depth camera looks forward only, and low objects (chair bases) are under the LiDAR plane. Depth
# points seen in the last MEM_S seconds are kept in the map frame and checked as well, so a K-turn that swings the car
# round still sees what was in front of it before.
MEM_S, MEM_RANGE = 15.0, 1.5
MEM_LIDAR_GAP = 0.10    # m: a depth point this close to a LiDAR return is at LiDAR height and is not remembered
# Approaching-object stop, same values and rule as the sim's shield (mentorpi_explore_env_cfg.approaching): an object
# within APPROACH_HALF_ANGLE of the driving direction and APPROACH_HALF_W of the car's axis (its path; explore v7),
# closer than APPROACH_RANGE, that itself approaches faster than APPROACH_SPEED (range rate between the last two scans
# minus the car's own v cos(bearing)) stops the car. Detection must hold in two scans in a row; the car's own motion is
# compensated with the measured /odom vx and yaw rate (previous scan rotated by the yaw change). No hold after it
# (explore v7: the 5 s hold was 85% of the idle time with a clear path in sim and the pauses on the floor, 2026-11-04).
APPROACH_RANGE, APPROACH_SPEED, APPROACH_HALF_ANGLE = 1.5, 0.6, math.radians(60.0)
APPROACH_HALF_W = 0.35
APPROACH_MAX = 2.5      # m/s: faster "approach" is not a person (maximum gait speed of young men ~2.5 m/s, Bohannon
                        # 1997) but a sector minimum jumping between two objects: goto3 (2026-11-05) stopped for 4.3 m/s
APPROACH_SCANS = 3      # scans in a row (sim: 2). goto3: 3 stops on static desk legs / chairs, each a 2-scan blip
# Goto (2026-11-04): the route preview as the sim's navigate.route_preview (mentorpi_goto_env_cfg): route points PREVIEW
# m ahead of the progress point (car frame, x forward, y left), then the remaining length / REMAIN_CLIP (clipped to 1).
GOTO_LAYOUT = O.OBS_LAYOUT_HIST + ';route:17@1284'
GOTO_HELD_LAYOUT = GOTO_LAYOUT + ';stop_held:1@1301'   # goto v6c: + how long our obstacle stop has held the car
# Imitation student v2 (2026-11-08, lab task Robotlab-GotoStudent): + the drive command executed in the last step, as
# mentorpi_goto_env_cfg.last_drive: speed / v_max (forward) or / v_reverse (backward) after the stop and back-up, and the
# rate-limited steering command.
GOTO_LAST_LAYOUT = GOTO_HELD_LAYOUT + ';last_drive:2@1302'
STOP_HELD_FULL = 2.0    # s, as mentorpi_goto_env_cfg.STOP_HELD_FULL
PREVIEW = tuple(0.25 * (i + 1) for i in range(8))
REMAIN_CLIP, GOAL_RADIUS, PROJ_WINDOW = 3.0, 0.35, 6
UNSTICK_AFTER, UNSTICK_BACK_S, UNSTICK_V = 0.3, 0.8, 0.8   # s held, s backing up, x v_reverse (ours, tune on the floor)
REAR_WAIT_S = 1.5  # s: then back up anyway (= mdp.REAR_WAIT_S)
BACK_CLEAR = 0.6   # m: back up only with the LiDAR's rear quarter (sectors 13-22, 135-225 deg) clear this far (= sim v6n)
AWAY_DIST, AWAY_RATE = 0.7, 0.1   # m off the route, m/s moving away: back up too (= goto_env_cfg.moving_away, v6k; the
                                  # sim applies it on turn-back routes, here on any route)
# goto v7 (2026-11-08, = sim mdp.ESCALATE_* / goto_env_cfg.TURN_ZONE): a back-up that starts within ESCALATE_M /
# ESCALATE_DEG of the last one's start backs up 1 + n times as long (n repeats, at most ESCALATE_MAX; only for models
# whose metadata says unstick_escalate). No driving-away back-up within TURN_ZONE of a turn-back corner of the route
# (goto17: back-ups #4-6 fired while the car swung wide through the hall-end U-turn).
ESCALATE_M, ESCALATE_DEG, ESCALATE_MAX = 0.3, 30.0, 2
TURN_ZONE = 0.5   # m (v7c, was 0.8; = goto_env_cfg.TURN_ZONE)
TURN_BACK_DEG, TURN_LOOK_M = 120.0, 1.0   # a route vertex is a turn-back corner if the route 1 m before and 1 m after
                                          # it differ by more than this (the sim knows its U corners; ours)
UNSTICK_WINDOW, UNSTICK_DIST = 2.0, 0.25   # s, m: moved less than this in the window -> back up too (sim v6i: turning in
                                           # place in narrow halls without getting anywhere); = goto_env_cfg.UNSTICK
ALIGN_DEG, ALIGN_M, ALIGN_MIN = 25.0, 1.0, 0.5   # --ref-map search: +-deg, +-m, min share of cells on saved walls (ours)
GOAL_DIST = 1.0         # m from the route's end: reached, as the sim (mentorpi_goto_env_cfg.GOAL_DIST). goto3 (2026-11-05)
                        # used 0.5 m, passed the charger 0.49 m off without the progress used up and drove on into a desk
MAX_ADVANCE = 1.0       # m: only segments starting within this of the progress point count (ours, not in the sim: its
                        # routes are 3 points per hallway). goto3: out and back on one line, the projection jumped to
                        # the way back 3 m early
TURN_LAYOUT = 'car_motion:4@0;lidar_sectors:36@4;depth:12@40;heading_error:2@52;last_action:2@54'


class ExplorePolicy(Node):
  def __init__(self, a):
    super().__init__('explore_policy')
    self.a = a
    so = ort.SessionOptions(); so.intra_op_num_threads = so.inter_op_num_threads = 1
    self.sess = ort.InferenceSession(a.model, so, providers=['CPUExecutionProvider'])
    meta = self.sess.get_modelmeta().custom_metadata_map
    if meta.get('obs_layout') not in (O.OBS_LAYOUT, O.OBS_LAYOUT_HIST, GOTO_LAYOUT, GOTO_HELD_LAYOUT, GOTO_LAST_LAYOUT):
      sys.exit(f'model obs_layout {meta.get("obs_layout")!r}: not an explore / goto model this node knows')
    self.use_hist = meta.get('obs_layout') in (O.OBS_LAYOUT_HIST, GOTO_LAYOUT, GOTO_HELD_LAYOUT, GOTO_LAST_LAYOUT)
    self.goto = meta.get('obs_layout') in (GOTO_LAYOUT, GOTO_HELD_LAYOUT, GOTO_LAST_LAYOUT)   # + route preview, --route
    self.use_held = meta.get('obs_layout') in (GOTO_HELD_LAYOUT, GOTO_LAST_LAYOUT)
    self.use_last = meta.get('obs_layout') == GOTO_LAST_LAYOUT
    self.kturn_skill = a.kturn                                  # multi-point turn skill (KTURN_*)
    self.av_until, self.av_steer = 0.0, 0.0   # steer around (AV_*)
    self.kturn_on, self.kt_dir, self.kt_cool_until, self.kturns = False, 1.0, 0.0, 0
    self.kt_stuck = 0.0
    self.last_xy = (0.0, 0.0)
    self.last_drive = [0.0, 0.0]
    self.held = 0.0                                             # s the stop has held the car (goto v6c input)
    self.back_until, self.back_steer, self.backups, self.back_try = 0.0, 0.0, 0, 0   # back-and-forth when stuck
    self.pos_hist, self.win_start = collections.deque(), -1e9           # positions in the last UNSTICK_WINDOW s
    self.route_d = self.route_d_prev = 0.0                              # m from the route's progress point
    self.rear_wait = 0.0                                                # s a back-up waited for a clear rear
    if self.goto and not a.route:
      sys.exit('goto model: pass --route X1 Y1 [X2 Y2 ...] (map frame)')
    self.route_head, self.route_behind = 0.0, False
    self.route = None                                           # [P, 2] polyline, set at the first step (car first)
    self.scan_hist = collections.deque(maxlen=O.LIDAR_HIST + 1)           # lidar_sectors of the last scans, newest last
    self.scan_dt = 0.1                                               # s between scans (approach_stop)
    self.stop_why = ''
    self.mem = collections.deque(); self.mem_xy, self.mem_yaw = (0.0, 0.0), 0.0   # obstacle memory (remember)
    self.scan_seq = self.appr_seq = 0
    self.appr_hist = []                                         # approach detections, newest scans
    self.last_yaw = 0.0
    if 'v_max' in meta:
      self.v_max, self.v_rev = float(meta['v_max']), float(meta['v_reverse'])
    elif a.v_max is not None and a.v_reverse is not None:   # models exported before 2026-11-03 (explore v1: 0.7 / 0.3)
      self.v_max, self.v_rev = a.v_max, a.v_reverse
    else:
      sys.exit('model has no v_max metadata: pass --v-max and --v-reverse (the speeds it was trained with)')
    self.steer_cmd_rate = float(meta.get('steer_cmd_rate', 0.0))   # models before explore v4e: no limit
    self.steer_ema = float(meta.get('steer_ema', 0.0))   # 2026-11-08: = sim mdp steer_ema (0 = off)
    if a.steer_ema is not None:          # 2026-11-09: override for models exported without it (v6p floor test)
      self.steer_ema = a.steer_ema
    self.v_dead = max(V_DEAD, float(meta.get('v_dead', 0.0)))   # = sim mdp v_dead: |v| below it is a stop
    self.forward_only = meta.get('forward_only', '0') == '1'    # = sim mdp forward_only: reverse only by the back-up
    dm = meta.get('depth_map', '')                     # goto v7: depth hits in the policy's map, as trained
    self.dlayer = O.DepthLayer(*map(float, dm.split(','))) if dm else None
    self.escalate = meta.get('unstick_escalate', '0') == '1'
    self.back_xy, self.back_yaw, self.back_rep = (1e3, 1e3), 0.0, 0
    self.turn_pts = np.zeros((0, 2))
    self.steer_cmd = 0.0
    if self.v_max > 0.7 + 1e-9:
      sys.exit(f'model drives up to {self.v_max} m/s; above the stock joystick limit 0.7 m/s (research/01 sec. 3)')
    self.inp = self.sess.get_inputs()[0].name
    self.turn = None
    if a.turn_model:
      self.turn = ort.InferenceSession(a.turn_model, so, providers=['CPUExecutionProvider'])
      tm = self.turn.get_modelmeta().custom_metadata_map
      if tm.get('obs_layout') != TURN_LAYOUT:
        sys.exit(f'turn model obs_layout {tm.get("obs_layout")!r} != {TURN_LAYOUT!r}')
      self.turn_v = (float(tm['v_max']), float(tm['v_reverse']))
      self.turn_inp = self.turn.get_inputs()[0].name
    self.mode = 'explore'; self.blocked_since = None; self.turn_target = 0.0; self.turn_start = 0.0
    self.turn_last = np.zeros(2, dtype=np.float32); self.done_since = None; self.turn_end = -1e9; self.turns = 0
    self.hist = []               # (time, x, y, |v| commanded) for the stuck check
    self.get_logger().info(f'{a.model}: task {meta.get("task")}, v_max {self.v_max}, v_reverse {self.v_rev}, '
                           f'steer limit {math.degrees(a.steer_limit):.0f} deg, depth map {dm or "off"}, '
                           f'back-up escalation {"on" if self.escalate else "off"}, {"DRY RUN" if a.dry_run else "DRIVING"}')
    self.buf = tf2_ros.Buffer(); tf2_ros.TransformListener(self.buf, self)
    self.pub = self.create_publisher(Twist, '/controller/cmd_vel', 10)
    self.create_subscription(LaserScan, '/scan_raw', self.on_scan, qos_profile_sensor_data)
    self.create_subscription(LaserScan, '/depth_scan', self.on_depth_scan, 10)
    self.create_subscription(Float32MultiArray, '/depth_sectors', self.on_depth, 10)
    self.create_subscription(Odometry, '/odom', self.on_odom, 10)
    map_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
    self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)
    self.t = {}                  # input name -> time of last message
    self.sectors = self.depth = self.motion = self.grid = None
    self.scan_pts = np.zeros((0, 2)); self.depth_pts = np.zeros((0, 2))
    self.scan_rot = None          # (yaw offset, flipped) lidar frame -> base_footprint, from TF once
    self.steer = 0.0             # estimated steering angle (rad)
    self.mo = None; self.jump_since = None; self.jumps = 0; self.slam_yaw = 0.0
    self.start = time.monotonic(); self.last_print = 0.0; self.steps = 0
    self.log_obs, self.log_act, self.log_why = [], [], []
    self.create_timer(DT, self.step)

  def now(self):
    return time.monotonic()

  def on_scan(self, m):
    if self.scan_rot is None:
      try:
        t = self.buf.lookup_transform('base_footprint', m.header.frame_id, rclpy.time.Time())
      except Exception:
        return
      q = t.transform.rotation
      yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
      up_z = 1 - 2 * (q.x * q.x + q.y * q.y)               # lidar z axis . base z axis: < 0 = mounted upside down
      self.scan_rot = (yaw, up_z < 0, (t.transform.translation.x, t.transform.translation.y))
    yaw, flipped, (tx, ty) = self.scan_rot
    r = np.asarray(m.ranges, dtype=np.float32)
    ang = m.angle_min + m.angle_increment * np.arange(len(r))
    ang = yaw + (-ang if flipped else ang)
    valid = np.isfinite(r) & (r >= max(m.range_min, 1e-3)) & (r <= m.range_max)
    self.sectors = O.lidar_sectors(ang, r, valid)
    self.scan_hist.append(self.sectors)
    t = self.now(); self.scan_dt = t - self.t.get('scan', t - 0.1); self.scan_seq += 1
    self.scan_pts = np.stack([tx + r[valid] * np.cos(ang[valid]), ty + r[valid] * np.sin(ang[valid])], -1)
    self.t['scan'] = self.now()

  def on_depth_scan(self, m):           # base_footprint frame (depth_scan.py), for the safety stop only
    r = np.asarray(m.ranges, dtype=np.float32)
    ang = m.angle_min + m.angle_increment * np.arange(len(r))
    ok = np.isfinite(r)
    self.depth_pts = np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])], -1)

  def on_depth(self, m):
    if len(m.data) == 12:
      self.depth = np.asarray(m.data, dtype=np.float32); self.t['depth'] = self.now()

  def on_odom(self, m):
    tw = m.twist.twist
    self.motion = (tw.linear.x, tw.linear.y, tw.angular.z); self.t['odom'] = self.now()

  def on_map(self, m):
    q = m.info.origin.orientation
    if abs(q.z) > 1e-3 or abs(q.x) > 1e-3 or abs(q.y) > 1e-3:
      self.get_logger().error('map origin is rotated: not supported'); return
    self.grid = O.MapGrid.from_occupancy_grid(m.data, m.info.width, m.info.height,
                                              (m.info.origin.position.x, m.info.origin.position.y), m.info.resolution)
    self.map_raw = (np.asarray(m.data, np.int8).reshape(m.info.height, m.info.width),
                    (m.info.origin.position.x, m.info.origin.position.y), m.info.resolution)

  def tf2d(self, parent, child):
    t = self.buf.lookup_transform(parent, child, rclpy.time.Time())
    q = t.transform.rotation
    return (np.array([t.transform.translation.x, t.transform.translation.y]),
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), t.header.stamp)

  def remember(self, now, xy, yaw):
    """Store the current depth points (base_footprint) in the map frame; drop the ones older than MEM_S."""
    p = self.depth_pts[np.hypot(self.depth_pts[:, 0], self.depth_pts[:, 1]) < MEM_RANGE][::2]
    if len(p) and len(self.scan_pts):    # only what the LiDAR does not see (low objects); it sees the rest live, and
      d2 = ((p[:, None, :] - self.scan_pts[None, :, :]) ** 2).sum(-1).min(1)   # remembered legs became ghosts (hall3)
      p = p[d2 > MEM_LIDAR_GAP ** 2]
    c, s_ = math.cos(yaw), math.sin(yaw)
    self.mem.append((now, np.stack([xy[0] + c * p[:, 0] - s_ * p[:, 1], xy[1] + s_ * p[:, 0] + c * p[:, 1]], -1)))
    while self.mem and now - self.mem[0][0] > MEM_S:
      self.mem.popleft()
    self.mem_xy, self.mem_yaw = xy, yaw
    if self.dlayer is not None:          # all depth hits in range, as the sim (no LiDAR filter: the sim has none)
      q = self.depth_pts[np.hypot(self.depth_pts[:, 0], self.depth_pts[:, 1]) <= self.dlayer.max_range]
      self.dlayer.add(now, np.stack([xy[0] + c * q[:, 0] - s_ * q[:, 1], xy[1] + s_ * q[:, 0] + c * q[:, 1]], -1))

  def mem_pts(self):
    """Remembered depth points in base_footprint, at the pose of the last remember()."""
    if not self.mem:
      return np.zeros((0, 2))
    w = np.concatenate([m[1] for m in self.mem]) - np.asarray(self.mem_xy)
    c, s_ = math.cos(self.mem_yaw), math.sin(self.mem_yaw)
    return np.stack([c * w[:, 0] + s_ * w[:, 1], -s_ * w[:, 0] + c * w[:, 1]], -1)

  def pose(self):
    """map -> base_footprint = (map -> odom from slam_toolbox, filtered) x (odom -> base_footprint, smooth EKF).
    slam_toolbox's correction jumped 80-100 deg back and forth in a cluttered corner on car #1 (2026-11-03), which
    rotated the map crops under the policy. A correction that moves more than JUMP_XY / JUMP_YAW from the one in use is
    only accepted once it has held for JUMP_HOLD s (a real loop closure stays; a scan-matching flip does not)."""
    try:
      p_ob, y_ob, stamp = self.tf2d('odom', 'base_footprint')
      p_mo, y_mo, _ = self.tf2d('map', 'odom')
    except Exception:
      return None
    age = (self.get_clock().now() - rclpy.time.Time.from_msg(stamp)).nanoseconds * 1e-9
    if age > MAX_AGE['pose']:
      return None
    now = self.now()
    if self.mo is None:
      self.mo = (p_mo, y_mo)
    else:
      dyaw = abs(math.atan2(math.sin(y_mo - self.mo[1]), math.cos(y_mo - self.mo[1])))
      if np.hypot(*(p_mo - self.mo[0])) < JUMP_XY and dyaw < JUMP_YAW:
        self.mo = (p_mo, y_mo); self.jump_since = None
      else:
        self.jump_since = self.jump_since or now
        if now - self.jump_since > JUMP_HOLD:
          self.get_logger().warn(f'accepting SLAM correction of {math.degrees(dyaw):.0f} deg after {JUMP_HOLD} s')
          self.mo = (p_mo, y_mo); self.jump_since = None
        else:
          self.jumps += 1
    (p0, y0) = self.mo
    c, s_ = math.cos(y0), math.sin(y0)
    self.slam_yaw = y_mo + y_ob
    return ((p0[0] + c * p_ob[0] - s_ * p_ob[1], p0[1] + s_ * p_ob[0] + c * p_ob[1]),
            math.atan2(math.sin(y0 + y_ob), math.cos(y0 + y_ob)))

  def blocked(self, v, steer):
    """A LiDAR / depth point within HALF_W of the arc the car drives at this steering, from the car out to
    STOP_FRONT (+ STOP_TIME * v) ahead, or STOP_REAR behind when reversing. A straight corridor blocked turns that
    would have cleared the obstacle (car #1, 2026-11-03)."""
    self.stop_why = ''
    if v == 0:
      return False
    turn_reach = self.mode == 'turn' or self.kturn_on   # turnaround model or the multi-point turn skill
    if not turn_reach and self.approach_stop(v):       # turnaround was trained without it; turning in place
      return True                                      # also reads as an approaching object (drive 2, 2026-11-04)
    # Remembered depth points only with --memory: the sim's shield has none, and on the floor they stopped the car for
    # obstacles no longer there (goto1, 2026-11-04: "memory 0.21 m" with clear floor ahead in the camera).
    pts = np.concatenate([self.scan_pts, self.depth_pts] + ([self.mem_pts()] if self.a.memory else []))
    x, y = pts[:, 0], pts[:, 1]
    if v < 0:
      x = -x                                       # reverse: the same arc, mirrored (heading turns the other way)
    if turn_reach:
      reach = (TURN_STOP_FRONT if v > 0 else TURN_STOP_REAR) + TURN_STOP_TIME * abs(v)
      half_w = TURN_HALF_W
    else:
      reach = (STOP_FRONT if v > 0 else STOP_REAR) + STOP_TIME * abs(v)
      half_w = HALF_W if v > 0 else HALF_W_REV
    k = math.tan(steer) / O.WHEELBASE              # curvature, + = left
    if abs(k) < 1e-3:
      hit = (x > 0) & (x < reach) & (np.abs(y) < half_w)
    else:
      r = 1.0 / k                                  # turn centre at (0, r)
      dc = np.hypot(x, y - r)
      s = np.abs(r) * np.mod(np.arctan2(x, np.sign(r) * (r - y)), 2 * math.pi)   # arc length along the path to the point
      hit = (np.abs(dc - abs(r)) < half_w) & (s > 0) & (s < reach)
    if hit.any():
      n = len(self.scan_pts)
      i = int(np.argmin(np.where(hit, np.hypot(x, y), 1e9)))
      src = 'lidar' if i < n else 'depth' if i < n + len(self.depth_pts) else 'memory'
      self.stop_why = f'{src} {np.hypot(x[i], y[i]):.2f} m'
    return bool(hit.any())

  def approach_stop(self, v):
    """As the sim's approaching() (v7: in the path, no hold): needs two scans; the lidar_sectors of the newest two
    /scan_raw messages."""
    if len(self.scan_hist) < 2:
      return False
    cur, prev = np.asarray(self.scan_hist[-1]) * O.LIDAR_RANGE, np.asarray(self.scan_hist[-2]) * O.LIDAR_RANGE
    dt = max(self.scan_dt, 1e-3)
    bearing = np.radians(np.arange(O.LIDAR_SECTORS) * 10.0 + 5.0)
    bearing = np.arctan2(np.sin(bearing), np.cos(bearing))
    ahead = (np.pi - np.abs(bearing) if v < 0 else np.abs(bearing)) < APPROACH_HALF_ANGLE
    vx, _, wz = self.motion
    k = int(round(wz * dt / math.radians(10.0)))
    prev = np.roll(prev, -k)                       # prev[(i + k) % 36] -> sector i, as the sim's gather
    rate = (prev - cur) / dt - vx * np.cos(bearing)
    ahead &= np.abs(cur * np.sin(bearing)) < APPROACH_HALF_W     # v7: in the path only
    hit = (cur < APPROACH_RANGE) & (prev < O.LIDAR_RANGE) & (rate > APPROACH_SPEED) & (rate < APPROACH_MAX) & ahead
    if self.scan_seq != self.appr_seq:             # raw detection changes once per new scan
      self.appr_seq = self.scan_seq
      self.appr_hist = (self.appr_hist + [bool(hit.any())])[-APPROACH_SCANS:]
    if len(self.appr_hist) == APPROACH_SCANS and all(self.appr_hist):
      j = int(np.argmin(np.where(hit, cur, 1e9)))
      self.stop_why = f'approach {cur[j]:.2f} m at {math.degrees(bearing[j]):+.0f} deg, {rate[j]:.1f} m/s, wz {wz:+.2f}'
      return True
    return False

  def align_route(self):
    """--ref-map: the route is given in a saved map's frame (a map_dump.py npz). Match this run's first SLAM map to it
    (occupied cells; rotation +-ALIGN_DEG in 1 deg steps, translation by FFT cross-correlation) and move the route into
    this run's map frame. goto3-13 (2026-11-05): a fresh map per run turned with the start heading, so a fixed route
    ended 0.5-3.7 m beside the real hall."""
    ref = np.load(self.a.ref_map)
    rg, (rox, roy), rres = ref['grid'], ref['origin'], float(ref['res'])
    g, (ox, oy), res = self.map_raw
    assert abs(res - rres) < 1e-6
    occ = (rg > 50).astype(np.float32)
    k = np.ones((3, 3), np.float32)               # blur the reference by one cell: SLAM walls shift a little per run
    blur = sum(np.roll(np.roll(occ, i, 0), j, 1) for i in (-1, 0, 1) for j in (-1, 0, 1)) > 0
    H, W = rg.shape
    iy, ix = np.nonzero(g > 50)
    px, py = ox + (ix + 0.5) * res, oy + (iy + 0.5) * res      # fresh occupied cell centres, this run's map frame
    F = np.fft.rfft2(blur.astype(np.float32), s=(2 * H, 2 * W))
    best = (-1.0, 0.0, 0.0, 0.0)
    for deg in np.arange(-ALIGN_DEG, ALIGN_DEG + 0.5, 1.0):
      c, s_ = math.cos(math.radians(deg)), math.sin(math.radians(deg))
      qx, qy = c * px - s_ * py, s_ * px + c * py               # rotated about this run's map origin
      jx, jy = np.floor((qx - rox) / res).astype(int), np.floor((qy - roy) / res).astype(int)
      img = np.zeros((2 * H, 2 * W), np.float32)
      ok = (jx >= -W // 2) & (jx < W + W // 2) & (jy >= -H // 2) & (jy < H + H // 2)
      np.add.at(img, (jy[ok] % (2 * H), jx[ok] % (2 * W)), 1.0)
      corr = np.fft.irfft2(F * np.conj(np.fft.rfft2(img)), s=(2 * H, 2 * W))   # corr[dy, dx]: shift the fresh by +d
      dy, dx = np.unravel_index(np.argmax(corr), corr.shape)
      dy, dx = (dy if dy < H else dy - 2 * H), (dx if dx < W else dx - 2 * W)
      score = float(corr.max()) / max(len(px), 1)            # share of this run's occupied cells on the saved walls
      if abs(dx * res) <= ALIGN_M and abs(dy * res) <= ALIGN_M and score > best[0]:
        best = (score, math.radians(deg), dx * res, dy * res)
    score, th, tx, ty = best
    self.get_logger().info(f'route alignment to {self.a.ref_map}: rotate {math.degrees(th):+.0f} deg, shift '
                           f'({tx:+.2f}, {ty:+.2f}) m, {score:.0%} of {len(px)} occupied cells on the saved walls')
    if score < ALIGN_MIN:
      self.get_logger().error('alignment too weak: stop'); raise SystemExit
    # saved = R(th) fresh + t  ->  fresh = R(-th) (saved - t)
    pts = np.array(list(zip(self.a.route[::2], self.a.route[1::2]))) - (tx, ty)
    c, s_ = math.cos(-th), math.sin(-th)
    return np.stack([c * pts[:, 0] - s_ * pts[:, 1], s_ * pts[:, 0] + c * pts[:, 1]], 1)

  def route_preview(self, xy, yaw):
    """As navigate._project / route_preview: advance the progress point s to the closest route point within the next
    PROJ_WINDOW segments (never back); 8 points PREVIEW m further on in the car frame, + remaining length."""
    if self.route is None:
      pts = self.align_route() if self.a.ref_map else np.array(list(zip(self.a.route[::2], self.a.route[1::2])))
      self.route = np.array([xy] + [tuple(p) for p in pts], dtype=float)
      self.cum = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(self.route, axis=0), axis=1))])
      self.seg, self.s = 0, 0.0
      self.turn_pts = self.turn_backs()
      if len(self.turn_pts):
        self.get_logger().info(f'turn-back corners (no driving-away back-up within {TURN_ZONE} m): '
                               + ', '.join(f'({x:+.2f}, {y:+.2f})' for x, y in self.turn_pts))
    P, p = self.route, np.asarray(xy)
    best = (np.inf, self.seg, self.s)
    for i in range(self.seg, min(self.seg + PROJ_WINDOW, len(P) - 1)):
      if self.cum[i] > self.s + MAX_ADVANCE:
        break
      ab = P[i + 1] - P[i]
      t = float(np.clip(np.dot(p - P[i], ab) / max(np.dot(ab, ab), 1e-9), 0, 1))
      d = np.linalg.norm(P[i] + t * ab - p)
      if d < best[0]:
        best = (d, i, self.cum[i] + t * np.linalg.norm(ab))
    self.seg, self.s = best[1], max(self.s, best[2])
    j = int(np.clip(np.searchsorted(self.cum, self.s, side='right') - 1, 0, len(P) - 2))
    t_ = (self.s - self.cum[j]) / max(self.cum[j + 1] - self.cum[j], 1e-9)
    self.route_d = float(np.linalg.norm(P[j] + t_ * (P[j + 1] - P[j]) - p))   # distance to the progress point
    c, s_ = math.cos(yaw), math.sin(yaw)
    out = []
    for q in np.minimum(self.s + np.asarray(PREVIEW), self.cum[-1]):
      j = int(np.clip(np.searchsorted(self.cum, q, side='right') - 1, 0, len(P) - 2))
      t = (q - self.cum[j]) / max(self.cum[j + 1] - self.cum[j], 1e-9)
      dx, dy = P[j] + t * (P[j + 1] - P[j]) - p
      out += [c * dx + s_ * dy, -s_ * dx + c * dy]
    remain = self.cum[-1] - self.s
    return np.array(out + [min(remain, REMAIN_CLIP) / REMAIN_CLIP], dtype=np.float32), remain

  def at(self, q):
    """Route point at arc length q (clipped to the route)."""
    q = float(np.clip(q, 0.0, self.cum[-1]))
    j = int(np.clip(np.searchsorted(self.cum, q, side='right') - 1, 0, len(self.route) - 2))
    t = (q - self.cum[j]) / max(self.cum[j + 1] - self.cum[j], 1e-9)
    return self.route[j] + t * (self.route[j + 1] - self.route[j])

  def turn_backs(self):
    """Route vertices where the way TURN_LOOK_M before and after differ by more than TURN_BACK_DEG."""
    out = []
    for i in range(1, len(self.route) - 1):
      a = self.route[i] - self.at(self.cum[i] - TURN_LOOK_M)
      b = self.at(self.cum[i] + TURN_LOOK_M) - self.route[i]
      if np.linalg.norm(a) < 1e-6 or np.linalg.norm(b) < 1e-6:
        continue
      cosang = np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))
      if cosang < math.cos(math.radians(TURN_BACK_DEG)):
        out.append(self.route[i])
    return np.array(out).reshape(-1, 2)

  def mem_rev_blocked(self, steer, forward=False):
    """Reverse arc against remembered depth hits (= lab mentorpi_explore_env_cfg.depth_mem_rev_blocked): the depth
    camera looks forward only and chair legs are under the LiDAR plane, so the turn's reverse legs check the depth
    layer. Rear body points: half width 0.08 m (research/01 sec. 2), rear overhang 0.05-0.15 m (ESTIMATE)."""
    if self.dlayer is None or self.last_xy is None:
      return False
    k = math.tan(steer) / O.WHEELBASE
    k = k if abs(k) >= 1e-3 else 1e-3
    px, py = [], []
    for s in np.linspace(0.05, MEM_REV_REACH, 6):
      if forward:     # (2026-11-09) plan ahead: a leg seen earlier, now < 0.2 m (camera minimum) or out of view
        ax, ay, ayaw = math.sin(k * s) / k, (1 - math.cos(k * s)) / k, k * s
        body = ((0.10, -0.08), (0.10, 0.08), (0.18, -0.08), (0.18, 0.0), (0.18, 0.08))   # front, ESTIMATE
      else:
        ax, ay, ayaw = -math.sin(k * s) / k, (1 - math.cos(k * s)) / k, -k * s
        body = ((-0.05, -0.08), (-0.05, 0.08), (-0.15, -0.08), (-0.15, 0.0), (-0.15, 0.08))
      for bx, by in body:
        px.append(ax + math.cos(ayaw) * bx - math.sin(ayaw) * by)
        py.append(ay + math.sin(ayaw) * bx + math.cos(ayaw) * by)
    return bool(self.dlayer.lookup(self.last_xy, self.last_yaw, np.array(px), np.array(py)).any())

  def kturn_step(self, now, rp, v, steer, w):
    """The multi-point turn skill (KTURN_*): returns (v, steer, w), the model's own command when not turning."""
    tx, ty = float(rp[4]), float(rp[5])                 # route point 0.75 m on, car frame
    ang = abs(math.atan2(ty, tx))
    lim = self.a.steer_limit
    if not self.kturn_on and now >= self.kt_cool_until and (ang > math.radians(KTURN_ON_DEG) or
                                                                 (self.held > KTURN_HELD_S and ang > math.radians(KTURN_HELD_DEG))):
      self.kturn_on, self.kt_dir = True, 1.0
      self.kturns += 1
      self.get_logger().warn(f'multi-point turn #{self.kturns}: route at {math.degrees(ang):.0f} deg, '
                             f'held {self.held:.1f} s')
    if not self.kturn_on:
      near = len(self.turn_pts) and float(np.min(np.hypot(*(self.turn_pts - np.asarray(self.last_xy)).T))) < NEAR_TURN
      pts = np.asarray(rp[:16], dtype=float).reshape(8, 2)
      d1, d2 = pts[1] - pts[0], pts[2] - pts[1]
      bend = abs(math.atan2(d1[0] * d2[1] - d1[1] * d2[0], float(np.dot(d1, d2))))
      if near or bend > math.radians(BEND_DEG):         # pure pursuit on the route preview near the U / a sharp bend
        ld = float(np.clip(PP_LOOKAHEAD_T * max(self.motion[0], 0.0), *PP_LOOKAHEAD))
        d = np.hypot(pts[:, 0], pts[:, 1])
        k = int(np.argmax(d >= ld)) if (d >= ld).any() else 7
        px, py = pts[k]
        kappa = 2.0 * py / max(px * px + py * py, 1e-4)
        steer = float(np.clip(math.atan(O.WHEELBASE * kappa), -lim, lim))
        v = float(min(self.v_max, math.sqrt(PP_A_LAT / max(abs(kappa), 1e-3))))
        w = v * math.tan(steer) / O.WHEELBASE
      if now < self.kt_cool_until and v > EXIT_V:       # gentle exit after a turn
        v = EXIT_V; w = v * math.tan(steer) / O.WHEELBASE
      # steer around (2026-11-09, = lab tmp_goto_check.py carskill AV_*): held with the route ahead -> the free
      # steering candidate closest to the model's, slowly, for AV_T s (sim trace: straight back into the obstacle after
      # every back-up; floor goto18/19 at the charger desk legs)
      if self.held > KTURN_HELD_S and ang <= math.radians(KTURN_HELD_DEG) and now >= self.av_until:
        free = [c * lim for c in AV_CAND if not self.blocked(AV_V, c * lim) and
                not self.mem_rev_blocked(c * lim, forward=True)]
        if free:
          self.av_steer, self.av_until = min(free, key=lambda s: abs(s - steer)), now + AV_T
          self.get_logger().info(f'steer around: {math.degrees(self.av_steer):+.0f} deg for {AV_T} s')
        elif now >= self.kt_cool_until:      # no free forward candidate: the multi-point turn (= lab carskill no_av)
          self.kturn_on, self.kt_dir = True, 1.0
          self.kturns += 1
          self.get_logger().warn(f'multi-point turn #{self.kturns}: nothing free ahead, held {self.held:.1f} s')
          return 0.0, steer, 0.0
      if now < self.av_until:
        if self.blocked(AV_V, self.av_steer):
          self.av_until = 0.0
        else:
          v, steer = AV_V, self.av_steer; w = v * math.tan(steer) / O.WHEELBASE
      return v, steer, w
    self.kturn_on = False                               # the model's move with the normal reaches
    model_ok = v > 0 and not self.blocked(v, steer)
    self.kturn_on = True
    if ang < math.radians(KTURN_OFF_DEG) and model_ok:
      self.kturn_on, self.kt_cool_until, self.kt_stuck = False, now + KTURN_COOLDOWN, 0.0   # (audit: reset stuck)
      self.get_logger().info(f'multi-point turn #{self.kturns} done')
      return min(v, EXIT_V), steer, min(v, EXIT_V) * math.tan(steer) / O.WHEELBASE
    side = 1.0 if ty >= 0 else -1.0
    fwd_ok = not self.blocked(KTURN_V, side * lim) and not self.mem_rev_blocked(side * lim, forward=True)
    rev_ok = not self.blocked(-KTURN_V, -side * lim) and not self.mem_rev_blocked(-side * lim)
    if not fwd_ok and not rev_ok:                       # = teacher v19: try the other lock
      f2, r2 = (not self.blocked(KTURN_V, -side * lim) and not self.mem_rev_blocked(-side * lim, forward=True)),                (not self.blocked(-KTURN_V, side * lim) and
                                                      not self.mem_rev_blocked(side * lim))
      if f2 or r2:
        side, fwd_ok, rev_ok = -side, f2, r2
    self.kt_stuck = self.kt_stuck + DT if (not fwd_ok and not rev_ok) else 0.0
    if self.kt_stuck > KTURN_STUCK_S:                   # hand over to the back-up rule
      self.kturn_on, self.kt_cool_until, self.kt_stuck = False, now + KTURN_COOLDOWN, 0.0
      self.get_logger().warn(f'multi-point turn #{self.kturns}: both legs blocked {KTURN_STUCK_S} s, handing over')
      return 0.0, steer, 0.0
    if self.kt_dir > 0 and not fwd_ok and rev_ok:
      self.kt_dir = -1.0
    elif self.kt_dir < 0 and not rev_ok and fwd_ok:
      self.kt_dir = 1.0
    if (self.kt_dir > 0 and fwd_ok) or (self.kt_dir < 0 and rev_ok):
      v, steer = self.kt_dir * KTURN_V, self.kt_dir * side * lim
    else:
      v, steer = 0.0, side * lim                        # both ends blocked: wait (the stop decides)
    return v, steer, v * math.tan(steer) / O.WHEELBASE

  def drive(self, v, w):
    if not self.a.dry_run:
      t = Twist(); t.linear.x = float(v); t.angular.z = float(w); self.pub.publish(t)

  def step(self):
    now = self.now()
    if now - self.start > self.a.seconds:
      self.get_logger().info(f'time limit {self.a.seconds} s reached'); raise SystemExit
    stale = [k for k, age in MAX_AGE.items() if k != 'pose' and now - self.t.get(k, -1e9) > age]
    pose = self.pose()
    if pose is None:
      stale.append('pose (TF map -> base_footprint)')
    if self.grid is None:
      stale.append('map')
    if stale:
      self.drive(0.0, 0.0)
      if now - self.last_print > 2.0:
        self.get_logger().warn(f'waiting / stopped: no fresh {", ".join(stale)}'); self.last_print = now
      return
    xy, yaw = pose
    self.last_yaw = yaw
    self.last_xy = xy
    self.remember(now, xy, yaw)
    motion = O.car_motion(*self.motion, self.steer)
    if self.mode == 'turn':
      err = math.atan2(math.sin(self.turn_target - yaw), math.cos(self.turn_target - yaw))
      obs = np.concatenate([motion, self.sectors, self.depth, [math.sin(err), math.cos(err)], self.turn_last]).astype(np.float32)
      a = self.turn.run(None, {self.turn_inp: obs[None]})[0][0]
      self.turn_last = np.clip(a[:2], -1, 1).astype(np.float32)
      v, steer, w = O.action_to_twist(a, *self.turn_v, self.a.steer_limit)
    else:
      err = 0.0
      hist = None
      if self.use_hist:   # the scans before the current one, newest first; until there are enough, the oldest repeats
        prev = list(self.scan_hist)[:-1][::-1] or [self.sectors]
        hist = (prev + [prev[-1]] * O.LIDAR_HIST)[:O.LIDAR_HIST]
      obs = O.observation(motion, self.sectors, self.depth, self.grid, xy, yaw, hist, self.dlayer)
      if self.goto:
        rp, remain = self.route_preview(xy, yaw)
        if remain < GOAL_RADIUS and math.hypot(*(self.route[-1] - np.asarray(xy))) < GOAL_DIST:
          self.get_logger().info(f'goal reached at ({xy[0]:+.2f}, {xy[1]:+.2f}) after {now - self.start:.1f} s'); raise SystemExit
        obs = np.concatenate([obs, rp] + ([[min(self.held / STOP_HELD_FULL, 1.0)]] if self.use_held else [])
                             + ([self.last_drive] if self.use_last else [])).astype(np.float32)
        b = math.atan2(rp[7], rp[6])                       # route point 1 m on, car frame
        tang = math.atan2(rp[15] - rp[1], rp[14] - rp[0])  # route direction 0.25 -> 2 m on: the turn's target. goto6:
        self.route_head, self.route_behind = yaw + tang, abs(b) > ROUTE_BEHIND   # aiming at the 1 m point (beside
        # the car after a U-turn) gave +33 deg for a hall at +90: the turn hunted 20 s and timed out
      a = self.sess.run(None, {self.inp: obs[None]})[0][0].copy()
      if self.steer_cmd_rate > 0:          # as mdp.AckermannAction.steer_cmd_rate: the limit the model was trained with
        tgt = float(np.clip(a[1], -1, 1))
        if 0.0 < self.steer_ema < 1.0:     # = sim: smoothing before the rate limit
          tgt = self.steer_cmd + self.steer_ema * (tgt - self.steer_cmd)
        self.steer_cmd += float(np.clip(tgt - self.steer_cmd, -self.steer_cmd_rate, self.steer_cmd_rate))
        a[1] = self.steer_cmd
      v, steer, w = O.action_to_twist(a, self.v_max, self.v_rev, self.a.steer_limit)
      if self.forward_only and v < 0:      # = sim forward_only: the model never reverses; the back-up rule does
        v, w = 0.0, 0.0
      if self.kturn_skill and self.goto:
        v, steer, w = self.kturn_step(now, rp, v, steer, w)
      # Back-and-forth when stuck (ours): held by the stop for UNSTICK_AFTER s -> back up for up to UNSTICK_BACK_S with
      # the steering opposite to the model's (the nose swings towards where it wants to go), then the model drives again;
      # repeats until it gets through. goto11-15 (2026-11-05): the model repeated a blocked move 15+ s and never chose
      # reverse (goto15: 880 held steps, speed action never < 0) while 1.6 m behind was free.
      self.pos_hist.append((now, xy[0], xy[1]))
      while self.pos_hist and now - self.pos_hist[0][0] > UNSTICK_WINDOW:
        self.pos_hist.popleft()
      stalled = (now - self.win_start >= UNSTICK_WINDOW and now - self.pos_hist[0][0] >= UNSTICK_WINDOW - 0.2 and
                 math.hypot(xy[0] - self.pos_hist[0][1], xy[1] - self.pos_hist[0][2]) < UNSTICK_DIST)
      away = (self.goto and self.route_d > AWAY_DIST and (self.route_d - self.route_d_prev) / DT > AWAY_RATE and
              (not len(self.turn_pts) or np.min(np.hypot(*(self.turn_pts - np.asarray(xy)).T)) > TURN_ZONE))
      self.route_d_prev = self.route_d
      want = (self.held > UNSTICK_AFTER or stalled or away) and now >= self.back_until and not self.kturn_on
      rear_clear = (float(np.min(self.sectors[13:23])) * O.LIDAR_RANGE > BACK_CLEAR   # = sim v6n: not into someone
                    or self.rear_wait > REAR_WAIT_S)                                 # behind; after the wait: a wall
      if want and not rear_clear:
        self.rear_wait += DT             # reset when a back-up moves or none is wanted (= sim v6q)
      elif not want:
        self.rear_wait = 0.0
      if want and rear_clear:
        self.win_start = now                 # a fresh window after each back-up
        side = (-1.0, 0.0, 1.0)[self.back_try % 3]   # blocked attempts cycle: opposite, straight, same side (= sim v6m)
        dur = UNSTICK_BACK_S
        if self.escalate:                    # = sim mdp ESCALATE_*: the same spot again -> back up longer
          same = (math.hypot(xy[0] - self.back_xy[0], xy[1] - self.back_xy[1]) < ESCALATE_M and
                  abs(math.atan2(math.sin(yaw - self.back_yaw), math.cos(yaw - self.back_yaw))) < math.radians(ESCALATE_DEG))
          self.back_rep = self.back_rep + 1 if same else 0
          self.back_xy, self.back_yaw = (xy[0], xy[1]), yaw
          dur *= 1 + min(self.back_rep, ESCALATE_MAX)
        self.back_until, self.back_steer = now + dur, side * math.copysign(self.a.steer_limit, steer or 1.0)
        self.back_try += 1
        self.backups += 1
        why = f'held {self.held:.1f} s' if self.held > UNSTICK_AFTER else 'stalled' if stalled else f'{self.route_d:.2f} m off the route'
        self.get_logger().warn(f'{why}: '
                               f'back up #{self.backups}, steering {math.degrees(self.back_steer):+.0f} deg, {dur:.1f} s')
        self.held = 0.0                  # the model gets a fresh 0.5 s after each back-up
      if now < self.back_until:
        v, steer = -self.v_rev * UNSTICK_V, self.back_steer
        w = v * math.tan(steer) / O.WHEELBASE
    if abs(v) < self.v_dead:             # = sim v_dead (2026-11-08): near-zero outputs are a stop, not a creep / blip
      v, w = 0.0, 0.0
    elif V_DEAD < abs(v) < self.a.v_min:
      v = math.copysign(self.a.v_min, v); w = v * math.tan(steer) / O.WHEELBASE
    stop = self.blocked(v, steer)
    if stop and now < self.back_until:   # backing up is blocked too: end it, the model drives again
      self.back_until = now
    elif now < self.back_until:          # the back-up moves: the next one starts the steering cycle again
      self.back_try, self.rear_wait = 0, 0.0
    self.held = 0.0 if now < self.back_until else (self.held + DT if stop else 0.0)
    if stop:
      v, w = 0.0, 0.0
      if self.mode == 'turn':
        self.turn_last[0] = 0.0          # executed speed, as the sim's turnaround last_action (v2c)
    self.last_drive = [v / self.v_max if v >= 0 else v / self.v_rev, self.steer_cmd]   # = sim last_drive
    if abs(v) > 1e-3:                    # the driver only steers while moving (see KNOWN DIFFERENCES)
      self.steer += float(np.clip(steer - self.steer, -STEER_RATE * DT, STEER_RATE * DT))
    self.drive(v, w)
    self.steps += 1
    self.switch(now, xy, yaw, v, stop, err)
    if self.a.log:
      self.log_why.append(self.stop_why if stop else '')
      self.log_obs.append(obs); self.log_act.append(np.array([*a[:2], v, steer, float(stop), xy[0], xy[1], yaw, self.slam_yaw, now - self.start, float(self.mode == 'turn')], dtype=np.float32))
    if now - self.last_print > 1.0:
      fine_known = obs[52:452].mean() + obs[452:852].mean() if self.mode == 'explore' else float('nan')
      tag = f'[TURN err {math.degrees(err):+4.0f} deg] ' if self.mode == 'turn' else ''
      self.get_logger().info(
        f'{tag}v {v:+.2f} m/s steer {math.degrees(steer):+5.1f} deg{f" STOP ({self.stop_why})" if stop else ""} | pose '
        f'({xy[0]:+.2f}, {xy[1]:+.2f}, {math.degrees(yaw):+.0f} deg) | lidar min {obs[4:40].min() * 3:.2f} m, '
        f'depth min {obs[40:52].min() * 3:.2f} m, fine crop known {fine_known:.0%}, SLAM jumps held {self.jumps}')
      self.last_print = now

  def switch(self, now, xy, yaw, v, stop, err):
    """Explore <-> turnaround hand-over (rules: BLOCKED_S, STUCK_*, TURN_*)."""
    if self.turn is None:
      return
    if self.mode == 'explore':
      self.blocked_since = (self.blocked_since or now) if stop else None
      self.hist.append((now, xy[0], xy[1], abs(v)))
      while self.hist and now - self.hist[0][0] > STUCK_S:
        self.hist.pop(0)
      h = np.array(self.hist)
      stuck = (now - self.hist[0][0] > STUCK_S - 0.2 and np.hypot(*(h[:, 1:3].max(0) - h[:, 1:3].min(0))) < STUCK_M
               and h[:, 3].mean() > 0.02)
      blocked = self.blocked_since is not None and now - self.blocked_since > BLOCKED_S
      behind = self.goto and self.route_behind
      if (blocked or stuck or behind) and now - self.turn_end > TURN_COOLDOWN_S:
        target = self.route_head if self.goto else yaw + math.pi   # goto: face the route, not just the other way
        self.mode, self.turn_target, self.turn_start = 'turn', target, now
        self.turn_last[:] = 0; self.done_since = None; self.turns += 1
        why = 'blocked' if blocked else 'no progress' if stuck else 'route behind'
        self.get_logger().warn(f'turnaround #{self.turns} ({why}): '
                               f'target heading {math.degrees(math.atan2(math.sin(target), math.cos(target))):+.0f} deg')
      return
    still = abs(v) < 0.05 and abs(self.motion[0]) < 0.05
    still = still or self.goto     # goto: hand back as soon as it faces the route (goto7:
    # waiting for a standstill let the turn model reverse 30 deg past the target and hunt; the route model drives on)
    self.done_since = (self.done_since or now) if (abs(err) < math.radians(TURN_DONE_DEG) and still) else None
    done = self.done_since is not None and now - self.done_since >= TURN_DONE_S
    if done or now - self.turn_start > TURN_MAX_S:
      self.get_logger().warn(f'turnaround #{self.turns} {"done" if done else "TIMED OUT"} after '
                             f'{now - self.turn_start:.1f} s, heading error {math.degrees(err):+.0f} deg')
      self.mode, self.turn_end, self.blocked_since, self.hist = 'explore', now, None, []

  def close(self):
    for _ in range(5):
      self.drive(0.0, 0.0); time.sleep(0.05)
    if self.a.log and self.log_obs:
      w = max(len(o) for o in self.log_obs)        # turnaround steps have a shorter observation: zero-pad
      obs = np.stack([np.pad(o, (0, w - len(o))) for o in self.log_obs])
      np.savez(self.a.log, obs=obs, act=np.stack(self.log_act), why=np.array(self.log_why))
      self.get_logger().info(f'saved {len(self.log_obs)} steps to {self.a.log}')


def main():
  p = argparse.ArgumentParser()
  p.add_argument('model')
  p.add_argument('--turn-model', help='turnaround skill ONNX: take over when exploring is blocked')
  p.add_argument('--dry-run', action='store_true', help='compute and print actions, publish nothing')
  p.add_argument('--seconds', type=float, default=60.0, help='stop after this long')
  p.add_argument('--v-max', type=float, help='only for models without v_max metadata')
  p.add_argument('--v-reverse', type=float, help='only for models without v_reverse metadata')
  p.add_argument('--v-min', type=float, default=V_MIN, help='slowest moving command (m/s); 0 = as the sim')
  p.add_argument('--steer-limit-deg', type=float, default=math.degrees(STEER_LIMIT))
  p.add_argument('--memory', action='store_true', help='safety stop also on remembered depth points (off: as the sim)')
  p.add_argument('--steer-ema', type=float, default=None, help='steering smoothing 0..1 (default: model metadata)')
  p.add_argument('--kturn', action='store_true', help='multi-point turn skill (KTURN_*) for forward-only students')
  p.add_argument('--route', type=float, nargs='+', help='goto model: route points X1 Y1 X2 Y2 ... (map frame, m)')
  p.add_argument('--ref-map', help='route given in this saved map (map_dump.py npz): align to it at the start')
  p.add_argument('--log', default='explore_run.npz', help='save observations and actions here ("" = off)')
  a = p.parse_args()
  a.steer_limit = math.radians(a.steer_limit_deg)
  rclpy.init(signal_handler_options=SignalHandlerOptions.NO)   # keep ROS up on Ctrl-C so close() can stop the car
  n = ExplorePolicy(a)
  try:
    rclpy.spin(n)
  except (KeyboardInterrupt, SystemExit):
    pass
  finally:
    n.close()
    n.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
  main()
