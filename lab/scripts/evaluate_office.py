"""Evaluate an office-follow checkpoint: run one 30 s episode per env and classify how each ended.

  docker compose run --rm lab python lab/scripts/evaluate_office.py --checkpoint logs/rsl_rl/office/<run>/model_599.pt

Outcome per episode: completed (30 s without failing), crash (chassis touched a wall/obstacle),
lost (more than 1.5 m off the trail). Also: gap tracking error while driving, time-to-failure.
Deterministic policy (mean action), fresh random offices/routes per env. Physics/actuation randomization is
on (randomization.py); sensor noise and delay only with --obs-noise.
"""

import argparse
from dataclasses import asdict

import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg

import robotlab.tasks  # noqa: F401
from robotlab.tasks.mentorpi_office_env_cfg import office_leader, too_close, trail_distance

TASK = "Robotlab-Office-MentorPiA1"
p = argparse.ArgumentParser()
p.add_argument("--checkpoint", required=True)
p.add_argument("--num-envs", type=int, default=512)
p.add_argument("--seconds", type=float, default=30.0)
p.add_argument("--seed", type=int, default=123)
p.add_argument("--obs-noise", action="store_true", help="keep the actor's sensor noise/delay on (randomization.py)")
args = p.parse_args()

torch.manual_seed(args.seed)
cfg = load_env_cfg(TASK, play=True)
cfg.scene.num_envs = args.num_envs
cfg.seed = args.seed
cfg.episode_length_s = 1e6                 # we stop each env at its first episode end ourselves
cfg.observations["actor"].enable_corruption = args.obs_noise
for sensor in cfg.scene.sensors:
  if hasattr(sensor, "debug_vis"):
    sensor.debug_vis = False
agent_cfg = load_rl_cfg(TASK)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
venv = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
runner = MjlabOnPolicyRunner(venv, asdict(agent_cfg), device="cuda:0")
runner.load(args.checkpoint, load_cfg={"actor": True}, strict=True, map_location="cuda:0")
policy = runner.get_inference_policy(device="cuda:0")

n, dev = args.num_envs, env.device
alive = torch.ones(n, dtype=torch.bool, device=dev)
outcome = torch.zeros(n, dtype=torch.long, device=dev)       # 0 completed, 1 crash, 2 lost
t_end = torch.full((n,), args.seconds, device=dev)
err_sum = torch.zeros(n, device=dev)
err_max = torch.zeros(n, device=dev)
close_steps = torch.zeros(n, device=dev)
steps_alive = torch.zeros(n, device=dev)
dist = torch.zeros(n, device=dev)
s = office_leader(env)

obs = venv.get_observations()
steps = int(args.seconds / env.step_dt)
with torch.inference_mode():
  for k in range(steps):
    prev_xy = s.xy.clone()
    obs, _, dones, _ = venv.step(policy(obs))
    tm = env.termination_manager
    crash, lost = tm.get_term("bump"), tm.get_term("lost_trail")
    ended = alive & dones.bool()
    outcome[ended & crash] = 1
    outcome[ended & ~crash & lost] = 2
    t_end[ended] = (k + 1) * env.step_dt
    alive &= ~ended
    e = trail_distance(env)
    err_sum += torch.where(alive, e, 0.0)
    err_max = torch.where(alive, torch.maximum(err_max, e), err_max)
    close_steps += (alive & (too_close(env, 0.3) > 0)).float()
    steps_alive += alive.float()
    dist += torch.where(alive, torch.linalg.norm(s.xy - prev_xy, dim=-1), 0.0)

done_ok = (outcome == 0).float().mean().item()
crash_r = (outcome == 1).float().mean().item()
lost_r = (outcome == 2).float().mean().item()
fail = outcome > 0
print(f"checkpoint: {args.checkpoint}")
print(f"episodes: {n} x {args.seconds:.0f} s")
print(f"  completed:  {done_ok:6.1%}")
print(f"  crashed:    {crash_r:6.1%}")
print(f"  lost:       {lost_r:6.1%}")
if fail.any():
  print(f"  failures happen after {t_end[fail].median().item():.1f} s (median)")
print(f"  mean time driven: {t_end.mean().item():.1f} s of {args.seconds:.0f}, leader distance covered "
      f"{dist.mean().item():.1f} m on average")
print(f"  trail error while driving: mean {(err_sum.sum() / steps_alive.sum()).item() * 100:.0f} cm, "
      f"per-episode max (median) {err_max.median().item() * 100:.0f} cm")
print(f"  time closer than 0.3 m to the leader: {(close_steps.sum() / steps_alive.sum()).item():.1%}")
env.close()
