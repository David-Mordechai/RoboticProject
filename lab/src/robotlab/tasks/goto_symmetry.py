"""Left/right mirror of the goto observations and actions for rsl_rl's symmetry data augmentation (Mittal et al., ICRA
2024; rsl_rl.extensions.Symmetry). goto v6g (2026-11-05): v6f 7250 completed 76% of left turn-backs but 94% of right
ones in a mirror-symmetric task, so every PPO mini-batch also gets the mirrored samples.

Mirror = reflect the car frame across its x axis (y -> -y):
  car_motion [vx, vy, wz, steer] -> [vx, -vy, -wz, -steer] (mdp.car_motion)
  lidar_sectors: sector i is centred at 5 + 10 i deg, its mirror at -(5 + 10 i) = sector 35 - i (reverse); lidar_hist
    is 4 such scans
  depth: 12 columns across a field of view centred on the axis (camera.DepthFanPatternCfg): reverse
  map_fine: [free, occ], each FINE_NX x FINE_NY with y in FINE_Y = (-1.5, 1.5) (symmetric): flip the y index
  map_coarse: [free, occ], each COARSE_N x COARSE_N centred on the car: flip the y index
  people (critic): 4 x [x, y, vx, vy, active] -> y, vy negated
  route: 8 x (x, y) -> y negated; remaining length unchanged.  stop_held unchanged
Action [speed, steer] -> [speed, -steer].
"""

from __future__ import annotations

import torch

from robotlab.tasks import mentorpi_explore_env_cfg as X

_FINE = X.FINE_NX * X.FINE_NY
_COARSE = X.COARSE_N * X.COARSE_N


def _flip_grid(t: torch.Tensor, nx: int, ny: int) -> torch.Tensor:
  return t.view(-1, nx, ny).flip(-1).reshape(t.shape[0], -1)


def _mirror_obs(o: torch.Tensor, critic: bool) -> torch.Tensor:
  o = o.clone()
  i = 0
  def take(n):
    nonlocal i
    s = slice(i, i + n); i += n
    return s
  cm = take(4)
  o[:, cm.start + 1:cm.start + 4] *= -1                                          # vy, wz, steer
  s = take(36); o[:, s] = o[:, s].flip(-1)                                       # lidar_sectors
  s = take(12); o[:, s] = o[:, s].flip(-1)                                       # depth
  for _ in range(2):                                                             # map_fine free, occ
    s = take(_FINE); o[:, s] = _flip_grid(o[:, s], X.FINE_NX, X.FINE_NY)
  for _ in range(2):                                                             # map_coarse free, occ
    s = take(_COARSE); o[:, s] = _flip_grid(o[:, s], X.COARSE_N, X.COARSE_N)
  if critic:
    s = take(20)                                                                 # people: 4 x [x, y, vx, vy, active]
    p = o[:, s].view(-1, 4, 5).clone(); p[..., 1] *= -1; p[..., 3] *= -1
    o[:, s] = p.reshape(o.shape[0], -1)
  s = take(4 * 36)                                                               # lidar_hist
  o[:, s] = o[:, s].view(-1, 4, 36).flip(-1).reshape(o.shape[0], -1)
  s = take(17)                                                                   # route: 8 (x, y) + remaining
  o[:, s.start + 1:s.start + 16:2] *= -1
  take(1)                                                                        # stop_held
  assert i == o.shape[1], f"goto_symmetry: layout {i} != observation size {o.shape[1]}"
  return o


def mirror(env=None, obs=None, actions=None):
  """rsl_rl data_augmentation_func: returns (obs, actions) with the mirrored samples appended (batch doubled)."""
  out_obs = out_act = None
  if obs is not None:
    m = obs.clone()
    for key in obs.keys():
      m[key] = _mirror_obs(obs[key], critic=obs[key].shape[1] > 1310)
    out_obs = torch.cat([obs, m], 0)
  if actions is not None:
    a = actions.clone(); a[:, 1] *= -1
    out_act = torch.cat([actions, a], 0)
  return out_obs, out_act
