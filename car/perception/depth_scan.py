# Depth camera -> obstacles below the LiDAR plane (chair bases, feet, boxes, cables), for Nav2 and for policies.
# The MS200 LiDAR scans one plane ~0.16 m up (TF base_footprint -> lidar_frame); an office chair's star base is below it,
# which is how car #1 hit a chair on 2026-11-02. Runs inside the MentorPi container:  python3 depth_scan.py
#
# From each depth frame (HP60C, 16UC1 mm, aligned depth0/image_raw + camera_info) it takes every STRIDE-th pixel,
# back-projects it with the camera intrinsics, transforms it to base_footprint with TF, and keeps points with
# FLOOR_Z < z < BAND_TOP and DEPTH_MIN <= range <= DEPTH_MAX. Publishes:
#   /depth_scan     sensor_msgs/LaserScan in base_footprint: nearest such point per 1-deg bearing bin (for Nav2's
#                   costmaps, next to /scan_raw). Bins with nothing are +inf (Nav2 clears along them up to range_max).
#   /depth_sectors  std_msgs/Float32MultiArray, 12 values in [0, 1]: the simulator's depth sectors (lab camera.py
#                   depth_sectors): 12 column bands over the camera's horizontal FOV, nearest horizontal distance from
#                   the camera origin per band, clipped at DEPTH_CLIP and divided by it (1 = nothing). The FOV is the
#                   real image's (from camera_info), so sim and car must use the same value (measured 56.6 deg on car #1).
import math
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, LaserScan
from std_msgs.msg import Float32MultiArray
import tf2_ros

# Same constants as lab/src/robotlab/tasks/camera.py (HP60C range 0.2-4 m, research/03 sec. 1.2).
DEPTH_MIN, DEPTH_MAX, DEPTH_CLIP = 0.2, 4.0, 3.0
FLOOR_Z = 0.02      # m: lower points are floor (fallback when no floor plane is found in the frame)
# Floor plane per frame (2026-11-09 night): on car #1 the floor came out 0.04-0.06 m ABOVE z = 0 at 1-4 m (dsz.py), i.e.
# the camera is pitched ~1 deg vs its TF, and pitch also changes while driving. Floor points then passed FLOOR_Z and
# /depth_scan showed a phantom obstacle ~0.3 m ahead for a whole trial (a_c2_1): RPP stopped, backed up, repeated.
# So: fit z = a x + b y + c to the near-floor points of each frame (2 trimmed least-squares passes) and cut on the
# height ABOVE THAT PLANE. Accept the plane only if it is plausible; otherwise use the fixed FLOOR_Z cut.
FLOOR_BAND = 0.10   # m: candidates |z| < this, 0.3-3.0 m ahead (1 deg pitch error = 5 cm at 3 m)
FLOOR_TRIM = 0.015  # m: inliers within this of the plane after the first pass (HP60C noise, ESTIMATE)
FLOOR_MIN_PTS = 150
FLOOR_MAX_TILT = math.radians(4.0)   # plane tilt accepted (ESTIMATE: mounting tolerance + driving pitch)
FLOOR_MAX_OFFSET = 0.03              # m: |c| accepted (was 0.05; 2026-11-11: office fits through furniture bases had c 0.03-0.07)
# 2026-11-11, car #1 at the charger: the OFFICE laminate gives NO depth return at all (lower half of the depth image empty,
# floordiag.py / snap_depth.png); the only "floor candidates" were chair / table-leg bases at >= 1.15 m, and the fit
# sometimes accepted a plane 3-4 cm up through them, which would cut the bottom ~6 cm of those obstacles. Real floor
# returns are strongest close to the car (steeper incidence), so a plane counts only with enough inliers near the car.
FLOOR_NEAR_R = 1.0                   # m
FLOOR_NEAR_MIN = 50                  # inliers closer than FLOOR_NEAR_R (ESTIMATE; hall test: thousands)
FLOOR_H = 0.03      # m: kept points must be this far above the fitted floor (cables 1-2 cm are lost: accepted)
BAND_TOP = 0.20     # m: above the car's 157 mm height plus margin (research/01 sec. 2)
# Nav2 scan only (not /depth_sectors, which must match the sim): camera range must be >= SCAN_DEPTH_MIN. 2026-11-16 bags
# (near_where.py): in an empty 1.4 m hall (LiDAR min 0.56 m, camera frames empty) ALL /depth_scan returns < 0.45 m were
# at 0.28-0.33 m from base_footprint = camera range ~0.20-0.27 m, just above the HP60C 0.2 m minimum (research/03 sec.
# 1.2); real near returns (charger area, att_c3c) were 0.37-0.43 m. 0.28 m is OUR ESTIMATE from that split. Cost: a NEW
# obstacle below the LiDAR plane appearing within ~0.22 m of the front bumper is not seen (approached ones are marked
# earlier and stay in the costmap: inf_is_valid false, nothing clears them from in front).
SCAN_DEPTH_MIN = 0.28
COLUMNS = 12
STRIDE = 4          # pixel subsampling: 640x480 -> 160x120 points per frame
SCAN_BINS = 120     # 1-deg bins over +-60 deg (covers the 56.6 deg FOV)
TOPIC = '/ascamera/camera_publisher/depth0/image_raw'
INFO = '/ascamera/camera_publisher/depth0/camera_info'


class DepthScan(Node):
  def __init__(self):
    super().__init__('depth_scan')
    self.K = None
    self.buf = tf2_ros.Buffer(); self.tfl = tf2_ros.TransformListener(self.buf, self); self.cam_tf = None
    self.create_subscription(CameraInfo, INFO, self.on_info, qos_profile_sensor_data)
    self.create_subscription(Image, TOPIC, self.on_depth, qos_profile_sensor_data)
    self.scan_pub = self.create_publisher(LaserScan, '/depth_scan', 10)
    self.sec_pub = self.create_publisher(Float32MultiArray, '/depth_sectors', 10)
    self.grid = None
    self.floor = None
    self.prev_ranges, self.n_drop, self.t_droplog = None, 0, 0.0
    self.tstat, self.t_last = [], time.perf_counter()

  def on_info(self, m):
    self.K = (m.k[0], m.k[4], m.k[2], m.k[5], m.width, m.height)

  def on_depth(self, m):
    t_cb = time.perf_counter()
    if self.K is None or m.encoding != '16UC1':
      return
    fx, fy, cx, cy, w, h = self.K
    # 2026-11-16: an Image subscription + a TF listener on one rclpy node busy-loops the executor (~3900 wakeups/s,
    # 70-86 % of a Pi core, measured with probes; either alone 1-4 %). The camera is bolted on: base_footprint ->
    # ascamera_color_0 measured constant (676 lookups over 12 s, zero spread: 0.064, 0, 0.121 m). So look it up once,
    # then drop the listener.
    t = self.cam_tf
    if t is None:
      try:
        t = self.cam_tf = self.buf.lookup_transform('base_footprint', m.header.frame_id, rclpy.time.Time())
      except Exception:
        return
      self.tfl.unregister()
      self.get_logger().info(f'camera TF cached ({m.header.frame_id}), TF listener removed')
    d = np.frombuffer(m.data, dtype=np.uint16).reshape(m.height, m.width)[::STRIDE, ::STRIDE].astype(np.float32) / 1000.0
    if self.grid is None or self.grid[0].shape != d.shape:
      v, u = np.mgrid[0:m.height:STRIDE, 0:m.width:STRIDE].astype(np.float32)
      self.grid = ((u - cx) / fx, (v - cy) / fy)
      self.hfov = 2 * math.atan(w / 2 / fx)
    gx, gy = self.grid
    # Optical frame (z forward, x right, y down) -> base_footprint via TF.
    pc = np.stack([gx * d, gy * d, d], -1).reshape(-1, 3)
    q = t.transform.rotation; tr = t.transform.translation
    R = _quat_to_rot(q.x, q.y, q.z, q.w)
    pb = pc @ R.T + np.array([tr.x, tr.y, tr.z], dtype=np.float32)
    rng = np.linalg.norm(pc, axis=1)
    valid = (d.reshape(-1) > 0) & (rng >= DEPTH_MIN) & (rng <= DEPTH_MAX)
    h, self.floor = height_above_floor(pb, valid)
    keep = valid & (h > (FLOOR_H if self.floor is not None else FLOOR_Z)) & (h < BAND_TOP)
    p = pb[keep]
    # LaserScan for Nav2, from the base_footprint origin.
    s = LaserScan(); s.header.stamp = m.header.stamp; s.header.frame_id = 'base_footprint'
    s.angle_min, s.angle_max = -math.radians(60), math.radians(60)
    s.angle_increment = (s.angle_max - s.angle_min) / SCAN_BINS
    s.range_min, s.range_max = 0.05, DEPTH_MAX
    ranges = np.full(SCAN_BINS, np.inf, dtype=np.float32)
    keep_s = keep & (rng >= SCAN_DEPTH_MIN); ps = pb[keep_s]
    r = b = ok = None
    if len(ps):
      r = np.hypot(ps[:, 0], ps[:, 1]); a = np.arctan2(ps[:, 1], ps[:, 0])
      b = ((a - s.angle_min) / s.angle_increment).astype(int)
      ok = (b >= 0) & (b < SCAN_BINS)
      robust_bin_min(ranges, b[ok], r[ok])
    prev = self.prev_ranges
    drop = confirm_near(ranges, prev)
    self.prev_ranges = ranges.copy()            # raw, so a real obstacle confirms itself on the next frame
    if drop.any():
      self.n_drop += 1
      if time.monotonic() - self.t_droplog > 1.0:   # evidence for the phantom source, at most 1 line/s
        self.t_droplog = time.monotonic()
        idx = np.nonzero(keep_s)[0][ok]; nc = self.grid[0].shape[1]
        sel = np.isin(b[ok], np.nonzero(drop)[0]) & (r[ok] < NEAR_R)
        px = [(int(i // nc * STRIDE), int(i % nc * STRIDE), round(float(d.reshape(-1)[i]), 3), round(float(h[i]), 3))
              for i in idx[sel][:4]]
        self.get_logger().warn(f'near phantom dropped #{self.n_drop}: bins {np.nonzero(drop)[0].tolist()[:6]} at '
                               f'{np.round(ranges[drop][:3], 2).tolist()} m, prev {np.round(prev[drop][:3], 2).tolist() if prev is not None else None}'
                               f' px (row, col, depth m, h m) {px}, floor {"fit" if self.floor else "FIXED cut"}')
      ranges[drop] = np.inf
    s.ranges = ranges.tolist()
    self.scan_pub.publish(s)
    # Simulator depth sectors, from the camera origin, column bands across the image's horizontal FOV.
    cam = np.array([tr.x, tr.y], dtype=np.float32)
    cols = np.full(COLUMNS, DEPTH_CLIP, dtype=np.float32)
    if len(p):
      rel = p[:, :2] - cam
      horiz = np.minimum(np.hypot(rel[:, 0], rel[:, 1]), DEPTH_CLIP)
      # Column index from the pixel's image column: same banding as the sim's evenly spaced azimuths across the FOV
      # (az = atan(-(u - cx) / fx), + = left, matching the sim's +y = left).
      az = np.arctan2(-gx.reshape(-1)[keep], 1.0)
      c = ((az + self.hfov / 2) / self.hfov * COLUMNS).astype(int).clip(0, COLUMNS - 1)
      np.minimum.at(cols, c, horiz)
    # Sim order: column 0 is the rightmost band (az from -HFOV/2 to +HFOV/2), so index by increasing azimuth.
    self.sec_pub.publish(Float32MultiArray(data=(cols / DEPTH_CLIP).tolist()))
    # timing (2026-11-09: this node used 90-96 % of one Pi core at only 7.4 of 12.5 Hz; find out where before fixing):
    # processing time per frame vs time between frames; logged every 100 frames.
    now = time.perf_counter(); self.tstat.append((now - t_cb, now - self.t_last)); self.t_last = now
    if len(self.tstat) >= 100:
      a = np.array(self.tstat); self.tstat = []
      self.get_logger().info(f'depth_scan: process {a[:, 0].mean() * 1e3:.1f} ms/frame, frames every '
                             f'{np.median(a[:, 1]) * 1e3:.0f} ms ({1 / np.median(a[:, 1]):.1f} Hz), floor plane '
                             f'{"yes " + str(tuple(round(v, 3) for v in self.floor)) if self.floor else "NO (fixed cut)"}')


BIN_MIN_PTS = 2     # a bin counts only if >= 2 kept points: its range = the 2nd-nearest (isolated noise pixels dropped;
                    # synthetic test: 5 phantom bins of 145 with the nearest point, see test/test_floor_fit.py)
# Near-range confirmation (2026-11-16): single-frame phantoms at 0.29-0.32 m appeared in EVERY recorded Gate A run (2-18
# per trip, bags att_c1b..g21_c2, phantoms.py): neighbour frames read >= 0.3 m farther, the camera showed an empty hall.
# One landed on a replan in g21_c2 -> "Starting point in lethal space" -> 4 recoveries; att_c3c had 8. Not the floor
# strip: the image bottom row meets the floor at 0.357 m (floor_edge.py) and the office floor returns no depth. Source
# still unknown (logged below). So a bin nearer than NEAR_R is published only if the PREVIOUS frame (~0.1 s, 9-12 Hz)
# had a return within NEAR_SLACK of it in bins +-NEAR_NB; else it is inf (inf_is_valid false in nav2_params_a.yaml:
# no mark, no clear). A real obstacle the car approaches was seen in earlier frames, so it passes; one that appears
# inside NEAR_R from nowhere is delayed by one frame. Values are OUR DESIGN ESTIMATES.
NEAR_R = 0.45       # m (phantoms 0.28-0.34 m + margin)
NEAR_SLACK = 0.15   # m: car at <= 0.5 m/s moves ~5 cm per frame; plus depth noise
NEAR_NB = 2         # bins (deg) either side: turning at the steering limit shifts bearings ~2 deg per frame (ESTIMATE)


def confirm_near(ranges, prev):
  """Boolean mask of bins nearer than NEAR_R with no supporting return in `prev` (previous frame's raw ranges)."""
  near = ranges < NEAR_R
  if not near.any():
    return near
  if prev is None:
    return near
  pad = np.pad(prev, NEAR_NB, constant_values=np.inf)
  pm = np.min(np.stack([pad[k:k + len(prev)] for k in range(2 * NEAR_NB + 1)]), axis=0)
  return near & ~(pm < ranges + NEAR_SLACK)


def robust_bin_min(ranges, b, r):
  """ranges[bin] = the BIN_MIN_PTS-th nearest range of the points in that bin (bins with fewer points stay inf)."""
  if not len(b):
    return
  o = np.lexsort((r, b)); bs, rs = b[o], r[o]
  start = np.r_[0, np.where(np.diff(bs))[0] + 1]
  cnt = np.diff(np.r_[start, len(bs)])
  sel = cnt >= BIN_MIN_PTS
  ranges[bs[start[sel]]] = rs[start[sel] + BIN_MIN_PTS - 1]


def height_above_floor(pb, valid):
  """Height of each point (N x 3, base_footprint) above the floor plane fitted in this frame, and the plane (a, b, c)
  or None when no plausible plane was found (then the height is plain z)."""
  r = np.hypot(pb[:, 0], pb[:, 1])
  cand = valid & (np.abs(pb[:, 2]) < FLOOR_BAND) & (r > 0.3) & (r < 3.0)
  if cand.sum() < FLOOR_MIN_PTS:
    return pb[:, 2], None
  q = pb[cand]
  A = np.c_[q[:, 0], q[:, 1], np.ones(len(q))]
  coef = np.linalg.lstsq(A, q[:, 2], rcond=None)[0]
  for _ in range(2):                                   # trim: obstacle points in the band pull the plane up
    res = q[:, 2] - A @ coef
    inl = np.abs(res) < max(FLOOR_TRIM, 2.5 * np.median(np.abs(res)))
    inl &= res < FLOOR_TRIM                            # never let points above the floor define it
    if inl.sum() < FLOOR_MIN_PTS:
      return pb[:, 2], None
    coef = np.linalg.lstsq(A[inl], q[inl, 2], rcond=None)[0]
  a, b, c = coef
  if (r[cand][inl] < FLOOR_NEAR_R).sum() < FLOOR_NEAR_MIN:
    return pb[:, 2], None
  if math.atan(math.hypot(a, b)) > FLOOR_MAX_TILT or abs(c) > FLOOR_MAX_OFFSET:
    return pb[:, 2], None
  return pb[:, 2] - (a * pb[:, 0] + b * pb[:, 1] + c), (float(a), float(b), float(c))


def _quat_to_rot(x, y, z, w):
  return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                   [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                   [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]], dtype=np.float32)


def main():
  rclpy.init(); rclpy.spin(DepthScan())


if __name__ == '__main__':
  main()
