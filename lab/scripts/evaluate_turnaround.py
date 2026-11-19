"""Evaluate a turnaround checkpoint: one run (the first episode) per env, randomized corridors as in training.

  docker compose run --rm lab python lab/scripts/evaluate_turnaround.py --checkpoint logs/rsl_rl/turnaround/<run>/model_X.pt

Reports, overall, per corridor-width bucket and with / without a low obstacle: success rate (task definition: heading
within 10 deg, still, clear of the walls), time to success (median / p90), forward <-> reverse switches of successful
runs (median / p90), contact rate and time-outs. Physics / actuation randomization on; sensor noise with --obs-noise.
"""

import argparse
from dataclasses import asdict

import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg

import robotlab.tasks  # noqa: F401
from robotlab.tasks import mentorpi_turnaround_env_cfg as T

TASK = "Robotlab-Turnaround-MentorPiA1"
BUCKETS = ((0.0, 0.6), (0.6, 0.9), (0.9, 1.2), (1.2, 9.0))
p = argparse.ArgumentParser()
p.add_argument("--checkpoint", required=True)
p.add_argument("--num-envs", type=int, default=2000)
p.add_argument("--seed", type=int, default=123)
p.add_argument("--obs-noise", action="store_true")
args = p.parse_args()

torch.manual_seed(args.seed)
cfg = load_env_cfg(TASK, play=True)
cfg.scene.num_envs, cfg.seed = args.num_envs, args.seed
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
env.reset()
obs = venv.get_observations()
t = T.turn(env)
width, kind = t.width.clone(), t.kind.clone()           # the first episode's corridor
rear = t.rear_blocked.clone() if hasattr(t, "rear_blocked") else torch.zeros_like(kind, dtype=torch.bool)   # v2
open_room = t.open_room.clone() if hasattr(t, "open_room") else torch.zeros_like(rear)                      # v3
drive = env.action_manager.get_term("drive")
steer_deg = torch.rad2deg(drive.steer_max).clone()
alive = torch.ones(n, dtype=torch.bool, device=dev)
success = torch.zeros(n, dtype=torch.bool, device=dev)
contact = torch.zeros(n, dtype=torch.bool, device=dev)
t_end = torch.zeros(n, device=dev)
switches = torch.zeros(n, device=dev)
steps = int(round(cfg.episode_length_s / env.step_dt)) + 2
with torch.inference_mode():
  for k in range(steps):
    obs, _, dones, _ = venv.step(policy(obs))
    switches += (t.switched & alive).float()            # set before physics this step; not cleared by the reset
    tm = env.termination_manager
    ended = alive & dones.bool()
    success |= ended & tm.get_term("success").bool()
    contact |= ended & tm.get_term("bump").bool()
    t_end = torch.where(ended, (k + 1) * env.step_dt, t_end)
    alive &= ~ended
    if not alive.any():
      break

timeout = ~success & ~contact


def q(x: torch.Tensor, p_: float) -> str:
  return f"{x.quantile(p_).item():5.1f}" if len(x) else "  -  "


def row(name: str, m: torch.Tensor) -> str:
  c = int(m.sum())
  if c == 0:
    return f"{name:<22} {0:>5}"
  s = m & success
  return (f"{name:<22} {c:>5}  {s.sum().item() / c:7.1%}  {(m & contact).sum().item() / c:7.1%}  "
          f"{(m & timeout).sum().item() / c:7.1%}   {q(t_end[s], 0.5)} {q(t_end[s], 0.9)}   "
          f"{q(switches[s], 0.5)} {q(switches[s], 0.9)}")


print(f"checkpoint: {args.checkpoint}  ({'with' if args.obs_noise else 'no'} sensor noise)")
print(f"{n} runs, first episode each (20 s limit); widths {T.W_MIN}-{T.W_MAX} m (split {T.W_SPLIT:.3f}), "
      f"low obstacle in {(kind != T.NONE).float().mean().item():.0%}, steering 29-37 deg")
print(f"{'':<22} {'runs':>5}  {'success':>7}  {'contact':>7}  {'timeout':>7}   t_med t_p90   sw_med sw_p90  (successes)")
print(row("all", torch.ones(n, dtype=torch.bool, device=dev)))
for lo, hi in BUCKETS:
  print(row(f"width {lo:.1f}-{hi if hi < 9 else 1.4:.1f} m", (width >= lo) & (width < hi)))
print(row("no low obstacle", kind == T.NONE))
print(row("chair base", kind == T.CHAIR))
print(row("low box", kind == T.BOX))
print(row("steer < 33 deg", steer_deg < 33))
print(row("steer >= 33 deg", steer_deg >= 33))
print(row("width < 0.45 m", width < 0.45))
print(row("rear blocked (v2)", rear))
print(row("rear open (corridor)", ~rear & ~open_room))
print(row("open room + post (v3)", open_room))
