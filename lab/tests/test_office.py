"""Office task: the scripted leader's routes must be drivable (no wall or obstacle overlap)."""

import pytest
import torch

pytestmark = pytest.mark.skipif(not torch.cuda.is_available(), reason="needs the GPU")


def test_leader_route_is_clear():
  from mjlab.envs import ManagerBasedRlEnv
  from mjlab.tasks.registry import load_env_cfg

  import robotlab.tasks  # noqa: F401
  from robotlab.tasks.mdp import place_car
  from robotlab.tasks.mentorpi_office_env_cfg import bumped, office_leader

  cfg = load_env_cfg("Robotlab-Office-MentorPiA1")
  cfg.scene.num_envs = 128
  cfg.episode_length_s = 1e6
  cfg.terminations = {}
  env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
  env.reset()
  ids = torch.arange(env.num_envs, device=env.device)
  s = office_leader(env)
  hits = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
  segments = torch.zeros(env.num_envs, device=env.device)
  for _ in range(1200):            # 60 s of driving per env
    prev = s.prev.clone()
    env.step(torch.zeros(env.num_envs, 2, device=env.device))
    segments += (s.prev != prev).float()
    place_car(env, ids, s.xy.clone(), s.yaw.clone())   # a car body exactly on the leader's pose
    env.sim.forward()
    env.scene.update(env.physics_dt)
    hits |= bumped(env)
  env.close()
  assert segments.min() >= 3, "leader got stuck"
  assert not hits.any(), f"{int(hits.sum())} envs: leader route touches a wall or obstacle"
