"""Is a turnaround possible with the car's turn-mode stop (turnaround v2 shield) in the loop? Scripted K-turn as
test_turnaround._k_turn, but it also switches direction whenever the stop holds the car. Per corridor width.
  python lab/scripts/tmp_kturn_shield.py"""
import math
import sys

import torch

sys.path.insert(0, "lab/tests")
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg

import robotlab.tasks  # noqa: F401
from robotlab.tasks import mentorpi_turnaround_env_cfg as T
from robotlab.tasks.mdp import car_pose
import test_turnaround as TT

cfg = load_env_cfg(TT.TASK, play=True)
cfg.scene.num_envs = TT.N
cfg.episode_length_s = 1e6
cfg.terminations = {}
for k in ("tyre_friction", "mass_inertia", "drive_motor"):
  cfg.events.pop(k, None)
d0 = cfg.actions["drive"]
d0.latency_range_s = d0.steer_deadtime_range_s = (0.0, 0.0)
d0.steer_rate_scale_range = (1.0, 1.0)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
env.reset()
n, dev = env.num_envs, env.device
i = torch.arange(n, device=dev)
for W in (0.6, 0.7, 0.8, 0.9, 1.0):
  steer_deg = torch.where(i % 2 == 0, 29.0, 37.0)
  x_gap = 0.3 + 1.2 * ((i // 2) % 4).float() / 3
  width = torch.full((n,), W, device=dev)
  drive = TT._setup(env, width, torch.stack([-x_gap - T.BODY_HALF_L, torch.zeros(n, device=dev)], -1),
                    torch.zeros(n, device=dev), steer_deg, torch.ones(n, dtype=torch.bool, device=dev))
  d = torch.ones(n, device=dev)
  hit = torch.zeros(n, dtype=torch.bool, device=dev)
  done = torch.zeros(n, dtype=torch.bool, device=dev)
  held = torch.zeros(n, device=dev)
  sw = torch.zeros(n, device=dev)
  for k in range(int(30.0 / env.step_dt)):
    xy, yaw = car_pose(env)
    done |= T.heading_error(env).abs() < math.radians(5.0)
    steer = d * drive.steer_max
    blocked = TT._predicted_clearance(env, xy, yaw, d, steer, 0.06) < 0.015
    held = torch.where(drive.shielded, held + 1, 0)
    flip = (blocked | (held >= 3)) & ~done
    d = torch.where(flip, -d, d)
    held = torch.where(flip, 0, held)
    sw += flip.float()
    env.step(TT._act(env, torch.where(done | hit, 0.0, d * 0.15), d))
    hit |= T.bumped(env)
  for deg in (29.0, 37.0):
    m = steer_deg == deg
    print(f"W {W:.1f} m, {deg:.0f} deg: turned {(done & ~hit)[m].float().mean().item():.0%}, contact "
          f"{hit[m].float().mean().item():.0%}, switches median {sw[m].median().item():.0f}")
