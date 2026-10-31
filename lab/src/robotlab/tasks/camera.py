"""Depth camera (Angstrong HP60C) inputs, modelled on what the car's stock pipeline gives.

Depth sectors: the real car takes the depth image, drops floor points by height, and keeps the nearest point per
image column band (the depthimage_to_laserscan idea, height-filtered; research/03 §2.2). Here a fan of rays from the
camera mount does the same: 12 column bands over the depth FOV, 7 elevations down to the bottom of the FOV; hits on the
floor (z < FLOOR_Z) are ignored, hits up to BAND_TOP count. This sees feet, low boxes and the leader's body, all below
the LiDAR plane (the reason v1 of the navigate policy clipped people's feet).

Person detections: Hiwonder's stock YOLO node labels "person" and body_track.py reads the person's distance from the
depth image (docs/vendor/hiwonder/code: example/.../body_track.py). Modelled as the 2 nearest people inside the RGB FOV,
within detection range, not hidden behind walls, with missed detections, position noise and a per-car frame rate.

Sources: HP60C depth FOV H73.8 x V58.8 deg, range 0.2-4 m, accuracy < 2 mm at 1 m, RGB FOV H80.9 deg (Hiwonder product
page, research/03 §1.2); stock driver at 15 fps (ascamera.launch.py). NO SOURCE (estimates, measure on the car): the
distance-squared growth of depth noise, depth dropouts, YOLO rate / recall / range on a Pi 5 (research/03 G9).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import mujoco
import torch

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv

DEPTH_HFOV = math.radians(73.8)
DEPTH_HALF_VFOV = math.radians(58.8 / 2)
DEPTH_MIN, DEPTH_MAX = 0.2, 4.0          # m, HP60C operating range
DEPTH_CLIP = 3.0                         # m, clipped like the LiDAR
DEPTH_COLUMNS = 12                       # 6.15 deg each
DEPTH_ELEVATIONS = tuple(math.radians(e) for e in (0.0, -3.0, -6.0, -10.0, -15.0, -21.0, -29.4))  # down to FOV bottom
FLOOR_Z = 0.02                           # m: lower hits are floor
BAND_TOP = 0.20                          # m: above the car's 157 mm height plus margin (research/01 §2)
DEPTH_STD_1M = 0.002                     # m at 1 m (spec upper bound); grows with distance^2 (ESTIMATE)
DEPTH_DROPOUT = (0.0, 0.05)              # per ray: ESTIMATE
DEPTH_FRESH_P = 15.0 / 20.0              # 15 fps camera at a 20 Hz policy

RGB_HALF_HFOV = math.radians(80.9 / 2)
DETECT_RANGE = (0.5, 4.0)                # m: closer, too little of the body is in frame (camera ~0.12 m up): ESTIMATE
DETECT_HZ = (5.0, 15.0)                  # YOLO on a Pi 5: unmeasured (research/03 G9): ESTIMATE
DETECT_RECALL = (0.8, 0.98)              # per frame: ESTIMATE
DETECT_POS_STD = 0.05                    # m: bounding-box centre + depth: ESTIMATE
DETECTIONS = 2


@dataclass
class DepthFanPatternCfg:
  """Rays from the camera origin: DEPTH_COLUMNS bands across DEPTH_HFOV, each at every DEPTH_ELEVATIONS angle."""

  def generate_rays(self, mj_model: mujoco.MjModel | None, device: str) -> tuple[torch.Tensor, torch.Tensor]:
    del mj_model
    az = (torch.arange(DEPTH_COLUMNS, device=device, dtype=torch.float32) + 0.5) / DEPTH_COLUMNS * DEPTH_HFOV - DEPTH_HFOV / 2
    el = torch.tensor(DEPTH_ELEVATIONS, device=device, dtype=torch.float32)
    a, e = torch.meshgrid(az, el, indexing="ij")                        # [C, E], column-major for pooling
    dirs = torch.stack([torch.cos(e) * torch.cos(a), torch.cos(e) * torch.sin(a), torch.sin(e)], -1).reshape(-1, 3)
    return torch.zeros_like(dirs), dirs


@dataclass
class CameraState:
  depth_drop: torch.Tensor     # [N]
  depth_held: torch.Tensor     # [N, C]
  det_fresh_p: torch.Tensor    # [N]
  det_recall: torch.Tensor     # [N]
  det_held: torch.Tensor       # [N, DETECTIONS * 3]


def camera_state(env: ManagerBasedRlEnv) -> CameraState:
  if not hasattr(env, "_robotlab_camera"):
    n, dev = env.num_envs, env.device
    z = lambda *s: torch.zeros(n, *s, device=dev)
    env._robotlab_camera = CameraState(z(), torch.full((n, DEPTH_COLUMNS), -1.0, device=dev), z(), z(),
                                       z(DETECTIONS * 3))
  return env._robotlab_camera


def reset_camera(env: ManagerBasedRlEnv, env_ids: torch.Tensor) -> None:
  s, m = camera_state(env), len(env_ids)
  u = lambda r: r[0] + (r[1] - r[0]) * torch.rand(m, device=env.device)
  s.depth_drop[env_ids] = u(DEPTH_DROPOUT)
  s.depth_held[env_ids] = -1.0
  s.det_fresh_p[env_ids] = u(DETECT_HZ) * env.step_dt
  s.det_recall[env_ids] = u(DETECT_RECALL)
  s.det_held[env_ids] = 0.0


def _noisy(env: ManagerBasedRlEnv) -> bool:
  return env.cfg.observations["actor"].enable_corruption


def depth_sectors(env: ManagerBasedRlEnv, noisy: bool = False) -> torch.Tensor:
  """Nearest obstacle per column band (horizontal distance), normalized to [0, 1] (1 = nothing within DEPTH_CLIP)."""
  data = env.scene["depth"].data
  d = data.distances                                                     # [N, C*E], -1 = no hit
  z = data.hit_pos_w[..., 2]
  hit = (d >= DEPTH_MIN) & (d <= DEPTH_MAX) & (z > FLOOR_Z) & (z < BAND_TOP)
  el = torch.tensor(DEPTH_ELEVATIONS, device=env.device).repeat(DEPTH_COLUMNS)
  horiz = d * torch.cos(el)
  if noisy and _noisy(env):
    s = camera_state(env)
    horiz = horiz + DEPTH_STD_1M * d * d * torch.randn_like(horiz)
    hit = hit & (torch.rand_like(horiz) >= s.depth_drop[:, None])
  horiz = torch.where(hit, horiz, DEPTH_CLIP).clamp(0.0, DEPTH_CLIP)
  cols = horiz.view(env.num_envs, DEPTH_COLUMNS, len(DEPTH_ELEVATIONS)).amin(-1)
  if noisy and _noisy(env):
    s = camera_state(env)
    fresh = (torch.rand(env.num_envs, device=env.device) < DEPTH_FRESH_P) | (s.depth_held[:, 0] < 0)
    s.depth_held = torch.where(fresh[:, None], cols, s.depth_held)
    cols = s.depth_held.clone()
  return cols / DEPTH_CLIP


def person_detections(env: ManagerBasedRlEnv, people_xy: torch.Tensor, active: torch.Tensor, car_xy: torch.Tensor,
                      car_yaw: torch.Tensor, lidar_m: torch.Tensor, noisy: bool = False) -> torch.Tensor:
  """The DETECTIONS nearest people the camera sees: (x, y) in the car frame / DETECT_RANGE[1] and a valid flag, each.
  people_xy [N, K, 2] env frame, active [N, K]; lidar_m [N, 36] metres per 10 deg sector, used for wall occlusion."""
  rel = people_xy - car_xy[:, None]
  c, s_ = torch.cos(car_yaw)[:, None], torch.sin(car_yaw)[:, None]
  x, y = c * rel[..., 0] + s_ * rel[..., 1], -s_ * rel[..., 0] + c * rel[..., 1]
  dist, bearing = torch.hypot(x, y), torch.atan2(y, x)
  sector = ((bearing % (2 * math.pi)) / (2 * math.pi / lidar_m.shape[1])).long().clamp(max=lidar_m.shape[1] - 1)
  wall = torch.gather(lidar_m, 1, sector)                                 # nearest LiDAR return toward the person
  visible = active & (bearing.abs() < RGB_HALF_HFOV) & (dist >= DETECT_RANGE[0]) & (dist <= DETECT_RANGE[1]) & (
    wall > dist - 0.35)                                                   # the legs themselves are within ~0.35 m
  if noisy and _noisy(env):
    st = camera_state(env)
    visible = visible & (torch.rand_like(dist) < st.det_recall[:, None])
    x = x + DETECT_POS_STD * torch.randn_like(x)
    y = y + DETECT_POS_STD * torch.randn_like(y)
  key = torch.where(visible, dist, 1e3)
  order = key.argsort(-1)[:, :DETECTIONS]
  g = lambda t: torch.gather(t, 1, order)
  ok = g(key) < 1e3
  out = torch.stack([g(x) / DETECT_RANGE[1] * ok, g(y) / DETECT_RANGE[1] * ok, ok.float()], -1).flatten(1)
  if noisy and _noisy(env):
    st = camera_state(env)
    fresh = torch.rand(env.num_envs, device=env.device) < st.det_fresh_p
    st.det_held = torch.where(fresh[:, None], out, st.det_held)
    out = st.det_held.clone()
  return out
