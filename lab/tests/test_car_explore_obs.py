"""The car-side explore observation (car/policy/explore_obs.py, numpy) must match the sim's (torch): same map crops,
same cell indexing and frames, same LiDAR sectors and action mapping. Runs on CPU, no simulation."""

import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import torch

import robotlab.tasks.mentorpi_explore_env_cfg as X
from robotlab.robot.mentorpi_a1_constants import STEER_MAX, WHEELBASE

_spec = importlib.util.spec_from_file_location(
  "explore_obs", Path(__file__).resolve().parents[2] / "car" / "policy" / "explore_obs.py")
O = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(O)


def test_constants_match_sim():
  assert O.SEEN_RES == X.SEEN_RES
  assert (O.FINE_RES, O.FINE_X, O.FINE_Y, O.FINE_N) == (X.FINE_RES, X.FINE_X, X.FINE_Y, X.FINE_NX)
  assert X.FINE_NX == X.FINE_NY
  assert (O.COARSE_RES, O.COARSE_N, O.COARSE_SUB) == (X.COARSE_RES, X.COARSE_N, X.COARSE_SUB)
  assert math.isclose(O.STEER_NORM, STEER_MAX) and O.WHEELBASE == WHEELBASE
  assert O.LIDAR_RANGE == X.LIDAR_RANGE and O.LIDAR_SECTORS * 10 == X.LIDAR_RAYS
  assert O.LIDAR_HIST == X.LIDAR_HIST


def test_map_crops_match_sim(monkeypatch):
  g = torch.Generator().manual_seed(0)
  free = torch.rand(1, X.SEEN_N, X.SEEN_N, generator=g) < 0.4
  occ = (torch.rand(1, X.SEEN_N, X.SEEN_N, generator=g) < 0.2) & ~free
  e = SimpleNamespace(free=free, occ=occ)
  grid = O.MapGrid(free[0].numpy(), occ[0].numpy(), (0.0, 0.0), X.SEEN_RES, outside_occupied=True)
  monkeypatch.setattr(X, "explore", lambda env: e)
  env = SimpleNamespace(device="cpu")
  # Poses inside the patch, near its edges (outside cells) and at awkward headings. Not on the 0.05 m grid: there the
  # coarse sample points sit exactly on cell edges and float32 (sim) vs float64 (car) round to different cells.
  for xy, yaw in [((7.213, 7.187), 0.0), ((3.33, 9.1), 1.1), ((0.41, 0.73), -2.6), ((14.03, 13.12), math.pi),
                  ((6.07, 2.01), -0.3)]:
    pose = (torch.tensor([xy], dtype=torch.float32), torch.tensor([yaw], dtype=torch.float32))
    monkeypatch.setattr(X, "own_pose", lambda env, est, pose=pose: pose)
    np.testing.assert_allclose(O.map_fine(grid, xy, yaw), X.built_map_fine(env)[0].numpy(), atol=1e-6)
    np.testing.assert_allclose(O.map_coarse(grid, xy, yaw), X.built_map_coarse(env)[0].numpy(), atol=1e-6)


def test_slam_map_pooling():
  # 4 x 6 cells of 0.05 m (row = y) -> 2 x 3 cells of 0.10 m, indexed [ix, iy].
  d = np.array([[-1, -1, 0, -1, 100, 0],
                [-1, -1, -1, -1, 0, 0],
                [0, 0, -1, -1, -1, -1],
                [0, 100, -1, -1, -1, 0]])
  m = O.MapGrid.from_occupancy_grid(d.reshape(-1), 6, 4, (-1.0, 2.0), 0.05)
  assert m.res == 0.10 and m.free.shape == (3, 2)
  np.testing.assert_array_equal(m.occ, [[False, True], [False, False], [True, False]])
  np.testing.assert_array_equal(m.free, [[False, False], [True, False], [False, True]])
  f, o = m.lookup((-1.0, 2.0), 0.0, np.array([0.15, 0.05, 1.0]), np.array([0.05, 0.15, 0.0]))
  np.testing.assert_array_equal(f, [1, 0, 0])   # (1, 0) free; (0, 1) occupied; outside the map unknown on the car
  np.testing.assert_array_equal(o, [0, 1, 0])


def test_lidar_sectors_and_action():
  ang = np.radians(np.arange(0.0, 360.0, 0.8))     # MS200-like 0.8 deg steps
  r = np.full_like(ang, 5.0)
  r[np.argmin(np.abs(ang - np.radians(95)))] = 1.5  # one return at 95 deg (left) -> sector 9
  r[np.argmin(np.abs(ang - np.radians(359.2)))] = 0.6  # rounds to 359 deg -> sector 35
  valid = r < 5.0
  s = O.lidar_sectors(ang, r, valid)
  assert s.shape == (36,) and math.isclose(s[9], 0.5, rel_tol=1e-6) and math.isclose(s[35], 0.2, rel_tol=1e-6)
  assert (np.delete(s, [9, 35]) == 1.0).all()
  v, steer, w = O.action_to_twist([0.5, -1.0], 0.2, 0.15, math.radians(29))
  assert math.isclose(v, 0.1) and math.isclose(steer, -math.radians(29))
  assert math.isclose(math.atan(WHEELBASE * w / v), steer)          # Hiwonder: steer = atan(L w / v)
  v, steer, w = O.action_to_twist([-2.0, 0.5], 0.2, 0.15, math.radians(29))
  assert math.isclose(v, -0.15) and math.isclose(math.atan(WHEELBASE * w / v), steer)
