"""Smoke test for a registered task: build it, step random actions, report throughput and episode ends.

  docker compose run --rm lab python lab/scripts/smoke_task.py Robotlab-Office-MentorPiA1 --num-envs 1024
"""

import argparse
import time

import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg

import robotlab.tasks  # noqa: F401  (registers the tasks)

p = argparse.ArgumentParser()
p.add_argument("task")
p.add_argument("--num-envs", type=int, default=256)
p.add_argument("--steps", type=int, default=200)
args = p.parse_args()

cfg = load_env_cfg(args.task)
cfg.scene.num_envs = args.num_envs
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
obs, _ = env.reset()
print({k: tuple(v.shape) for k, v in obs.items()})
act_dim = env.action_manager.total_action_dim

ends: dict[str, int] = {}
for i in range(args.steps):
  if i == 20:
    torch.cuda.synchronize()
    t0 = time.perf_counter()
  # Mostly forward, mild random steering: exercises walls, obstacles and the leader.
  a = torch.zeros(args.num_envs, act_dim, device=env.device)
  a[:, 0] = -0.2
  a[:, 1] = torch.randn(args.num_envs, device=env.device) * 0.3
  obs, rew, terminated, truncated, extras = env.step(a)
  for name in env.termination_manager.active_terms:
    ends[name] = ends.get(name, 0) + int(env.termination_manager.get_term(name).sum())
  assert torch.isfinite(rew).all() and all(torch.isfinite(v).all() for v in obs.values())
torch.cuda.synchronize()
steps = (args.steps - 20) * args.num_envs
print(f"{steps / (time.perf_counter() - t0):,.0f} env-steps/s at {args.num_envs} envs")
print("episode ends:", ends)
env.close()
