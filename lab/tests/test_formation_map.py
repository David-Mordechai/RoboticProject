"""Formation task with the shared map: hallway widths vary, the map crop matches the office, AMCL-style localization
error stays bounded and resets on travel, an out-of-date map misses obstacles, and the leader's route stays drivable
in narrow and wide hallways."""

import pytest
import torch

pytestmark = pytest.mark.skipif(not torch.cuda.is_available(), reason="needs the GPU")

TASK = "Robotlab-Formation-MentorPiA1"


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


def test_hall_widths_vary(env):
  from robotlab.tasks.mentorpi_formation_env_cfg import HALL_WIDTH_RANGE
  from robotlab.tasks.mentorpi_office_env_cfg import office_leader

  w = office_leader(env).maps.hall_width
  assert w.min() >= HALL_WIDTH_RANGE[0] and w.max() <= HALL_WIDTH_RANGE[1]
  assert w.max() - w.min() > 0.5


def test_map_crop_matches_office(env):
  from robotlab.tasks import localization as Lz
  from robotlab.tasks.mdp import car_pose
  from robotlab.tasks.mentorpi_office_env_cfg import office_leader

  s, loc = office_leader(env), Lz.localization(env)
  xy, yaw = car_pose(env)
  current = Lz.map_crop(s.maps, s.layout, loc.keep, xy, yaw, current=True)
  assert current.shape == (env.num_envs, Lz.MAP_NX * Lz.MAP_NY)
  # The car stands in a free hallway cell, and a 3 x 3 m crop of an office floor always shows some wall.
  centre = current.view(-1, Lz.MAP_NX, Lz.MAP_NY)[:, Lz.MAP_NX * 6 // 20, Lz.MAP_NY // 2]
  assert (centre == 0).float().mean() > 0.95
  assert (current.sum(-1) > 0).all()
  # Keeping every obstacle gives the current map back.
  all_kept = torch.ones_like(loc.keep)
  assert torch.equal(Lz.map_crop(s.maps, s.layout, all_kept, xy, yaw), current)
  # Dropping every obstacle can only remove occupied cells.
  none_kept = torch.zeros_like(loc.keep)
  walls = Lz.map_crop(s.maps, s.layout, none_kept, xy, yaw)
  assert (walls <= current).all()


def test_localization_bounded_and_corrected(env):
  from robotlab.tasks import localization as Lz

  loc = Lz.localization(env)
  max_err = torch.zeros(env.num_envs, device=env.device)
  a = torch.zeros(env.num_envs, 2, device=env.device)
  a[:, 0] = 0.6                                # drive forward for 20 s
  for _ in range(400):
    env.step(a)
    max_err = torch.maximum(max_err, torch.linalg.norm(loc.err[:, 0, :2], dim=-1))
  # Bias 0.05 + residual 3 x 0.03 + drift over 0.25 m at 3% scale and a few degrees of heading error.
  assert max_err.max() < 0.25
  # Corrections happened: the follower moved, so its anchor is no longer the start pose.
  assert (loc.anchor[:, 0, :2] != 0).any()


def test_actor_sees_estimate_critic_sees_truth(env):
  from robotlab.tasks.mentorpi_formation_env_cfg import leader_map, slot_error

  true_ = slot_error(env)
  est = slot_error(env, est=True)
  assert not torch.allclose(true_, est)
  # Position errors (< 0.25 m each car) plus heading error (a few degrees) times the distance to the target.
  dist = torch.linalg.norm(true_[:, :2], dim=-1)
  assert (torch.linalg.norm((true_ - est)[:, :2], dim=-1) < 0.5 + 0.1 * dist).all()
  assert leader_map(env, est=True).shape == leader_map(env).shape == (env.num_envs, 6)


def test_lidar_cast_every_other_step(env):
  lidar = env.scene["lidar"]
  a = torch.zeros(env.num_envs, 2, device=env.device)
  a[:, 0] = 0.6
  seen = []
  for _ in range(6):
    env.step(a)
    seen.append((env._robotlab_lidar_cast, lidar.data.distances.clone()))
  for (c0, d0), (c1, d1) in zip(seen, seen[1:]):
    assert c0 != c1                              # alternates
    if not c1:
      assert torch.equal(d0, d1)                 # no cast: the previous scan is held
  casts = [d for c, d in seen if c]
  assert not torch.equal(casts[0], casts[1])     # moving cars get new scans when cast
