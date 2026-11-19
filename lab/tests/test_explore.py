"""Explore task: the built map grows while driving, never marks space behind walls, the start scan earns no reward,
and the observations have the documented sizes."""

import pytest
import torch

pytestmark = pytest.mark.skipif(not torch.cuda.is_available(), reason="needs the GPU")

TASK = "Robotlab-Explore-MentorPiA1"


@pytest.fixture(scope="module")
def env():
  from mjlab.envs import ManagerBasedRlEnv
  from mjlab.tasks.registry import load_env_cfg

  import robotlab.tasks  # noqa: F401

  cfg = load_env_cfg(TASK)
  cfg.scene.num_envs = 128
  cfg.episode_length_s = 1e6
  cfg.terminations = {}
  e = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
  e.reset()
  yield e
  e.close()


def test_start_scan_is_free_then_map_grows(env):
  from robotlab.tasks.mentorpi_explore_env_cfg import explore

  e = explore(env)
  a = torch.zeros(env.num_envs, 2, device=env.device)
  for _ in range(4):
    env.step(a)                                  # standing still: the start scans
  assert e.free_count.min() > 0                  # every car has mapped its surroundings
  start = e.free_count.clone()
  a[:, 0] = 0.6
  rewarded = torch.zeros(env.num_envs, device=env.device)
  for _ in range(60):                            # 3 s forward
    env.step(a)
    rewarded += e.new_area
  grew = e.free_count > start
  assert grew.float().mean() > 0.8               # most cars see new floor while driving (a few bump and stop)
  assert (rewarded[grew] > 0).all()              # new floor is rewarded


def test_never_sees_through_walls(env):
  from robotlab.tasks.mentorpi_explore_env_cfg import explore

  e = explore(env)
  fl = e.floor[e.layout]
  inside_rooms = (e.free & ~fl).flatten(1).sum(-1).float()
  total = e.free.flatten(1).sum(-1).float().clamp(min=1)
  # Only cells grazed at a wall's edge may be marked: the 0.10 m cell straddles the wall face.
  assert (inside_rooms / total).max() < 0.05
  # Deep inside a wall / room block (cells whose 3 x 3 neighbourhood is all wall) nothing is ever marked free. Low
  # clutter is excluded on purpose: the LiDAR plane (~0.16 m up) passes over it, as on the real map.
  from robotlab.tasks.mentorpi_explore_env_cfg import SEEN_N, SEEN_RES
  from robotlab.tasks.office_terrain import OCC_RES
  k = int(round(SEEN_RES / OCC_RES))
  wall = torch.nn.functional.avg_pool2d(e.maps.walls[:, :SEEN_N * k, :SEEN_N * k].float()[:, None], k)[:, 0] > 0.99
  core = (-torch.nn.functional.max_pool2d(-wall.float()[:, None], 3, stride=1, padding=1)[:, 0]) > 0.5
  assert not (e.free & core[e.layout]).any()


def test_observation_sizes(env):
  from robotlab.tasks.mentorpi_explore_env_cfg import COARSE_N, FINE_NX, FINE_NY, built_map_coarse, built_map_fine, depth

  assert built_map_fine(env, est=True).shape == (env.num_envs, 2 * FINE_NX * FINE_NY)
  assert built_map_coarse(env, est=True).shape == (env.num_envs, 2 * COARSE_N * COARSE_N)
  assert depth(env).shape == (env.num_envs, 12)
  c = built_map_coarse(env)
  assert ((c >= 0) & (c <= 1)).all()
