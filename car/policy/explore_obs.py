# The explore policy's observation and action, built from the real car's sensors exactly as the simulator builds them
# (lab/src/robotlab/tasks/mentorpi_explore_env_cfg.py, mdp.car_motion, mentorpi_navigate_env_cfg.lidar_sectors,
# camera.depth_sectors). Pure numpy, no ROS, so lab/tests/test_car_explore_obs.py can check it against the sim code.
#
# Actor observation, 1140 values in this order (checked against the ONNX file's obs_layout metadata at start-up):
#   car_motion 4 | lidar_sectors 36 | depth 12 | map_fine 800 | map_coarse 288
import math
import numpy as np

# Sim constants (cited where they are defined in the lab).
LIDAR_RANGE = 3.0                  # m, navigate cfg
LIDAR_SECTORS = 36                 # 10 deg each, sector j = rays at j*10 .. j*10+9 deg, CCW from car forward
STEER_NORM = math.radians(35.0)    # mentorpi_a1_constants.STEER_MAX: car_motion divides the steering angle by it
SEEN_RES = 0.10                    # m, the sim's built map
FINE_RES, FINE_X, FINE_Y, FINE_N = 0.15, (-0.9, 2.1), (-1.5, 1.5), 20
COARSE_RES, COARSE_N, COARSE_SUB = 0.6, 12, 3
WHEELBASE = 0.145                  # m, Hiwonder ackermann.py (research/01 sec. 3)
OBS_LAYOUT = 'car_motion:4@0;lidar_sectors:36@4;depth:12@40;map_fine:800@52;map_coarse:288@852'
OBS_DIM = 1140
# Explore v5+: the 4 scans before the current one appended, newest first (mentorpi_explore_env_cfg.lidar_history).
LIDAR_HIST = 4
OBS_LAYOUT_HIST = OBS_LAYOUT + ';lidar_hist:144@1140'
OBS_DIM_HIST = OBS_DIM + 36 * LIDAR_HIST


def car_motion(vx, vy, wz, steer):
  """Forward / lateral speed (m/s, body frame), yaw rate (rad/s), front-left steering angle (rad)."""
  return np.array([vx, vy, wz, steer / STEER_NORM], dtype=np.float32)


def lidar_sectors(angles, ranges, valid):
  """Real scan -> the sim's 36 sectors. angles: each ray's bearing in the car frame (rad, CCW from forward); valid:
  the ray has a return. The sim casts one ray per whole degree, so each real ray is assigned to its nearest degree;
  no return = LIDAR_RANGE, all clipped at LIDAR_RANGE, min per sector, / LIDAR_RANGE."""
  d = np.where(valid, ranges, LIDAR_RANGE).clip(max=LIDAR_RANGE)
  deg = np.round(np.degrees(angles)).astype(int) % 360
  out = np.full(LIDAR_SECTORS, LIDAR_RANGE, dtype=np.float32)
  np.minimum.at(out, deg // 10, d)
  return out / LIDAR_RANGE


class MapGrid:
  """Known-free / known-occupied cells, indexed [ix, iy] like the sim's built map: cell (ix, iy) covers
  origin + [ix, ix+1) x [iy, iy+1) * res."""

  def __init__(self, free, occ, origin, res, outside_occupied):
    self.free, self.occ, self.origin, self.res = free, occ, np.asarray(origin, dtype=np.float64), res
    self.outside_occupied = outside_occupied

  @classmethod
  def from_occupancy_grid(cls, data, width, height, origin, res, outside_occupied=False):
    """slam_toolbox /map (row-major, row = y; -1 unknown, 0 free, 100 occupied) at res (0.05 m) -> the sim's
    SEEN_RES (0.10 m) cells: occupied if any sub-cell is occupied (thin 1-cell walls stay walls), else free if any
    sub-cell is free (the sim also lets occupied win over free)."""
    k = int(round(SEEN_RES / res))
    g = np.asarray(data, dtype=np.int16).reshape(height, width)
    h2, w2 = -(-height // k) * k, -(-width // k) * k
    p = np.full((h2, w2), -1, dtype=np.int16)
    p[:height, :width] = g
    blocks = p.reshape(h2 // k, k, w2 // k, k)
    occ = (blocks >= 50).any((1, 3))
    free = ((blocks >= 0) & (blocks < 50)).any((1, 3)) & ~occ
    return cls(free.T, occ.T, origin, res * k, outside_occupied)

  def lookup(self, xy, yaw, px, py):
    """Cells at car-frame points (px, py) for a car at xy, yaw (map frame): known-free, known-occupied as 0/1."""
    c, s = math.cos(yaw), math.sin(yaw)
    wx, wy = xy[0] + c * px - s * py, xy[1] + s * px + c * py
    ix = np.floor((wx - self.origin[0]) / self.res).astype(int)
    iy = np.floor((wy - self.origin[1]) / self.res).astype(int)
    inside = (ix >= 0) & (iy >= 0) & (ix < self.free.shape[0]) & (iy < self.free.shape[1])
    ix, iy = ix.clip(0, self.free.shape[0] - 1), iy.clip(0, self.free.shape[1] - 1)
    free = self.free[ix, iy] & inside
    occ = self.occ[ix, iy] & inside
    if self.outside_occupied:
      occ = occ | ~inside
    return free.astype(np.float32), occ.astype(np.float32)


class DepthLayer:
  """goto v7: depth-camera hits as occupied built-map cells for hold_s (= sim mentorpi_explore_env_cfg.update_depth_map,
  DEPTH_MAP_S / DEPTH_MAP_RANGE, read from the model's 'depth_map' metadata). Cells are SEEN_RES squares of the map
  frame (anchored at 0, 0; the sim anchors at its patch corner, the offset does not matter). Low objects under the
  LiDAR plane (chair bases) stay in the policy's map for hold_s after they leave the camera's view."""

  def __init__(self, hold_s, max_range):
    self.hold_s, self.max_range = hold_s, max_range
    self.cells = {}                                # (ix, iy) -> time last marked

  def add(self, now, pts_map):
    for ix, iy in set(map(tuple, np.floor(np.asarray(pts_map).reshape(-1, 2) / SEEN_RES).astype(int).tolist())):
      self.cells[(ix, iy)] = now
    old = [k for k, t in self.cells.items() if now - t >= self.hold_s]
    for k in old:
      del self.cells[k]

  def lookup(self, xy, yaw, px, py):
    """1 where the car-frame point (px, py) is in a recently marked cell."""
    if not self.cells:
      return np.zeros(len(px), dtype=bool)
    c, s = math.cos(yaw), math.sin(yaw)
    ix = np.floor((xy[0] + c * px - s * py) / SEEN_RES).astype(np.int64)
    iy = np.floor((xy[1] + s * px + c * py) / SEEN_RES).astype(np.int64)
    keys = np.array(list(self.cells.keys()), dtype=np.int64)
    K = 1 << 20                                    # |index| < 2^19 cells = 52 km
    return np.isin(ix * K + iy, keys[:, 0] * K + keys[:, 1])


def _with_layer(free, occ, layer, xy, yaw, px, py):
  if layer is None:
    return free, occ
  recent = layer.lookup(xy, yaw, px, py)
  return np.where(recent, 0.0, free).astype(np.float32), np.where(recent, 1.0, occ).astype(np.float32)


_gx = FINE_X[0] + FINE_RES * (np.arange(FINE_N) + 0.5)
_gy = FINE_Y[0] + FINE_RES * (np.arange(FINE_N) + 0.5)
_FINE_PX, _FINE_PY = (a.reshape(-1) for a in np.meshgrid(_gx, _gy, indexing='ij'))
_sub = (np.arange(COARSE_SUB) + 0.5) / COARSE_SUB
_g = (np.arange(COARSE_N)[:, None] + _sub[None]).reshape(-1) * COARSE_RES - COARSE_N * COARSE_RES / 2
_COARSE_PX, _COARSE_PY = (a.reshape(-1) for a in np.meshgrid(_g, _g, indexing='ij'))


def map_fine(grid, xy, yaw, layer=None):
  free, occ = _with_layer(*grid.lookup(xy, yaw, _FINE_PX, _FINE_PY), layer, xy, yaw, _FINE_PX, _FINE_PY)
  return np.concatenate([free, occ])


def map_coarse(grid, xy, yaw, layer=None):
  free, occ = _with_layer(*grid.lookup(xy, yaw, _COARSE_PX, _COARSE_PY), layer, xy, yaw, _COARSE_PX, _COARSE_PY)
  pool = lambda t: t.reshape(COARSE_N, COARSE_SUB, COARSE_N, COARSE_SUB).mean((1, 3)).reshape(-1)
  return np.concatenate([pool(free), pool(occ)])


def observation(motion, sectors, depth, grid, xy, yaw, history=None, layer=None):
  """history: None (models before v5) or the LIDAR_HIST previous lidar_sectors vectors, newest first. layer: a
  DepthLayer (goto v7 models) or None."""
  parts = [motion, sectors, np.asarray(depth, dtype=np.float32), map_fine(grid, xy, yaw, layer),
           map_coarse(grid, xy, yaw, layer)]
  if history is not None:
    parts += list(history)
  return np.concatenate(parts).astype(np.float32)


def action_to_twist(a, v_max, v_reverse, steer_limit):
  """Policy output -> (v m/s, steering rad, yaw rate rad/s) as mdp.AckermannAction: a[0] >= 0 -> a[0] * v_max,
  < 0 -> a[0] * v_reverse; steering a[1] * the car's limit. Hiwonder's ackermann.py steers by atan(L * w / v)
  (research/02), so w = v tan(steer) / L gives exactly this steering angle, forward or reverse."""
  a = np.clip(np.asarray(a, dtype=np.float64).reshape(-1)[:2], -1.0, 1.0)
  v = a[0] * (v_max if a[0] >= 0 else v_reverse)
  steer = a[1] * steer_limit
  return v, steer, v * math.tan(steer) / WHEELBASE
