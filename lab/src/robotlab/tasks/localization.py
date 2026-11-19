"""Map-based localization and map sharing, as the real cars will do it with their stock software.

The plan (open-issues #7, research/03 §3): one slam_toolbox map of the area, copied to both cars; each car localizes
in it with stock AMCL (Nav2 1.1.15 on the cars) and sends its map pose, speed and steering to the other car at 20 Hz.
So both cars' poses are in one shared map frame, and each policy sees:
- a local crop of the saved map around its own *estimated* pose (the map can be out of date: hallway clutter that
  was moved since mapping is missing from it, and people are never in it);
- the other car's estimated map pose, in its own estimated frame, some 0-150 ms old.

AMCL error model, per car (sources: stock `navigation/config/nav2_params.yaml` on car #1, read 2026-11-02):
- AMCL corrects only after the car moves `update_min_d` 0.25 m or turns `update_min_a` 0.2 rad. Between corrections
  the map pose follows odometry and drifts.
- Odometry drift: speed scale error (wheel diameter 65 vs 67 mm, ±3%, research/01 §3) and gyro bias. Measured
  raw gyro bias on car #1 up to 0.0044 rad/s (docs/measurements/car1-2026-11-02.md §2); the EKF heading uses it.
- After a correction: a per-episode converged offset (LOC_BIAS) plus a residual. NO SOURCE for their size until a
  car localizes on a real map (open-issues #7): ESTIMATES around the 0.05 m map cell, same as randomization.LOC_*.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import TYPE_CHECKING

import torch

from robotlab.tasks.mdp import to_frame, wrap
from robotlab.tasks.office_terrain import MAX_CLUTTER, OCC_N, OCC_RES, LayoutTensors

if TYPE_CHECKING:
  from mjlab.envs import ManagerBasedRlEnv

AMCL_UPDATE_MIN_D = 0.25         # m, nav2_params.yaml amcl update_min_d (stock)
AMCL_UPDATE_MIN_A = 0.2          # rad, nav2_params.yaml amcl update_min_a (stock)
ODOM_SCALE_RANGE = (0.97, 1.03)  # wheel diameter 65 vs 67 mm (research/01 §3)
GYRO_BIAS_MAX = 0.005            # rad/s, measured raw bias up to 0.0044 rad/s (car #1, docs/measurements)
LOC_BIAS_M = 0.05                # m, converged AMCL offset per episode: ESTIMATE (randomization.LOC_BIAS_M)
LOC_BIAS_YAW = math.radians(3.0)  # ESTIMATE (randomization.LOC_BIAS_YAW)
LOC_RESID_M = 0.03               # m, std of each correction's residual: ESTIMATE (randomization.LOC_JUMP_M)
LOC_RESID_YAW = math.radians(1.5)  # ESTIMATE (randomization.LOC_JUMP_YAW)

# Map crop seen by the policy: a car-frame grid, more of it ahead than behind. Our design.
MAP_RES = 0.15                   # m per crop cell (the saved map is 0.05 m, slam.yaml resolution)
MAP_X = (-0.9, 2.1)              # m behind / ahead of the car
MAP_Y = (-1.5, 1.5)              # m right / left
MAP_NX = int(round((MAP_X[1] - MAP_X[0]) / MAP_RES))   # 20
MAP_NY = int(round((MAP_Y[1] - MAP_Y[0]) / MAP_RES))   # 20
# Out-of-date map: per episode, each hallway obstacle is in the saved map with probability p, p drawn per episode
# from this range (1.0 = the map is current). Our design; the real rate depends on how often the office changes.
MAP_KEEP_PROB = (0.5, 1.0)


@dataclass
class Localization:
  err: torch.Tensor        # [N, A, 3] estimate minus truth (dx, dy world frame; dyaw)
  anchor: torch.Tensor     # [N, A, 3] true pose at the last AMCL correction
  last: torch.Tensor       # [N, A, 3] true pose at the previous step
  bias: torch.Tensor       # [N, A, 3] converged AMCL offset, per episode
  scale: torch.Tensor      # [N, A] odometry speed scale
  gyro: torch.Tensor       # [N, A] yaw rate bias, rad/s
  keep: torch.Tensor       # [N, MAX_CLUTTER + 1] bool, obstacle id is in the saved map (index 0 unused)


def localization(env: ManagerBasedRlEnv, agents: int = 2) -> Localization:
  if not hasattr(env, "_robotlab_loc"):
    n, dev = env.num_envs, env.device
    z = lambda *s: torch.zeros(n, *s, device=dev)
    env._robotlab_loc = Localization(z(agents, 3), z(agents, 3), z(agents, 3), z(agents, 3), torch.ones(n, agents,
                                     device=dev), z(agents), torch.ones(n, MAX_CLUTTER + 1, dtype=torch.bool, device=dev))
  return env._robotlab_loc


def _u(lo: float, hi: float, *shape, device) -> torch.Tensor:
  return lo + (hi - lo) * torch.rand(*shape, device=device)


def _correction(bias: torch.Tensor) -> torch.Tensor:
  std = torch.tensor([LOC_RESID_M, LOC_RESID_M, LOC_RESID_YAW], device=bias.device)
  return bias + std * torch.randn_like(bias)


def reset_localization(env: ManagerBasedRlEnv, env_ids: torch.Tensor, poses: torch.Tensor, noisy: bool = True) -> None:
  """poses: [n, A, 3] true (x, y, yaw) of each agent of env_ids, after placement."""
  loc, n, dev = localization(env, poses.shape[1]), len(env_ids), env.device
  a = poses.shape[1]
  k = 1.0 if noisy else 0.0
  r = torch.rand(n, a, device=dev)
  phi = 2 * math.pi * torch.rand(n, a, device=dev)
  bias = torch.stack([LOC_BIAS_M * r * torch.cos(phi), LOC_BIAS_M * r * torch.sin(phi),
                      _u(-LOC_BIAS_YAW, LOC_BIAS_YAW, n, a, device=dev)], -1)
  loc.bias[env_ids] = k * bias
  loc.scale[env_ids] = 1.0 + k * (_u(*ODOM_SCALE_RANGE, n, a, device=dev) - 1.0)
  loc.gyro[env_ids] = k * _u(-GYRO_BIAS_MAX, GYRO_BIAS_MAX, n, a, device=dev)
  loc.err[env_ids] = _correction(loc.bias[env_ids]) if noisy else 0.0
  loc.anchor[env_ids] = poses
  loc.last[env_ids] = poses
  p = _u(*MAP_KEEP_PROB, n, 1, device=dev) if noisy else torch.ones(n, 1, device=dev)
  loc.keep[env_ids] = torch.rand(n, MAX_CLUTTER + 1, device=dev) < p


def step_localization(env: ManagerBasedRlEnv, poses: torch.Tensor) -> None:
  """Advance each agent's map-pose error by one policy step. poses: [N, A, 3] true poses now."""
  loc, dt = localization(env, poses.shape[1]), env.step_dt
  d_xy = poses[..., :2] - loc.last[..., :2]
  # Odometry integrates a scaled displacement in its (wrong-by-err_yaw) heading: the error grows with travel.
  e_yaw = loc.err[..., 2]
  c, s = torch.cos(e_yaw), torch.sin(e_yaw)
  est = loc.scale[..., None] * torch.stack([c * d_xy[..., 0] - s * d_xy[..., 1], s * d_xy[..., 0] + c * d_xy[..., 1]], -1)
  loc.err[..., :2] += est - d_xy
  loc.err[..., 2] = wrap(loc.err[..., 2] + loc.gyro * dt)
  loc.last = poses.clone()
  # AMCL correction after enough travel since the last one.
  moved = torch.linalg.norm(poses[..., :2] - loc.anchor[..., :2], dim=-1) >= AMCL_UPDATE_MIN_D
  turned = wrap(poses[..., 2] - loc.anchor[..., 2]).abs() >= AMCL_UPDATE_MIN_A
  fix = moved | turned
  if fix.any():
    loc.err = torch.where(fix[..., None], _correction(loc.bias), loc.err)
    loc.anchor = torch.where(fix[..., None], poses, loc.anchor)


def estimated(env: ManagerBasedRlEnv, agent: int, xy: torch.Tensor, yaw: torch.Tensor) -> tuple[torch.Tensor, torch.Tensor]:
  """An agent's map pose estimate from its true pose."""
  e = localization(env).err[:, agent]
  return xy + e[:, :2], wrap(yaw + e[:, 2])


def map_crop(maps: LayoutTensors, layout: torch.Tensor, keep: torch.Tensor, xy: torch.Tensor, yaw: torch.Tensor,
             current: bool = False) -> torch.Tensor:
  """The saved map around pose (xy, yaw), car frame, [N, MAP_NX * MAP_NY], 1 = occupied or off the map.
  current=True reads the true occupancy (critic); otherwise walls plus the obstacles kept in the saved map."""
  dev = xy.device
  gx = MAP_X[0] + MAP_RES * (torch.arange(MAP_NX, device=dev) + 0.5)
  gy = MAP_Y[0] + MAP_RES * (torch.arange(MAP_NY, device=dev) + 0.5)
  px, py = torch.meshgrid(gx, gy, indexing="ij")
  px, py = px.reshape(-1), py.reshape(-1)                                     # [G]
  c, s = torch.cos(yaw)[:, None], torch.sin(yaw)[:, None]
  wx = xy[:, 0:1] + c * px - s * py
  wy = xy[:, 1:2] + s * px + c * py
  ix, iy = (wx / OCC_RES).floor().long(), (wy / OCC_RES).floor().long()
  inside = (ix >= 0) & (ix < OCC_N) & (iy >= 0) & (iy < OCC_N)
  ix, iy = ix.clamp(0, OCC_N - 1), iy.clamp(0, OCC_N - 1)
  lay = layout[:, None].expand_as(ix)
  if current:
    occ = maps.occ[lay, ix, iy]
  else:
    cid = maps.clutter[lay, ix, iy]
    occ = maps.walls[lay, ix, iy] | ((cid > 0) & keep.gather(1, cid))
  return torch.where(inside, occ, True).float()


def relative_pose(xy: torch.Tensor, yaw: torch.Tensor, o_xy: torch.Tensor, o_yaw: torch.Tensor) -> torch.Tensor:
  """Other pose in the frame (xy, yaw): x, y, sin, cos of the heading difference. [N, 4]"""
  e = wrap(o_yaw - yaw)
  return torch.cat([to_frame(o_xy - xy, yaw), torch.sin(e)[:, None], torch.cos(e)[:, None]], -1)
