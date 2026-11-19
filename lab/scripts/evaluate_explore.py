"""Evaluate an explore checkpoint: one run per env, office floors with walking people.

  docker compose run --rm lab python lab/scripts/evaluate_explore.py --checkpoint logs/rsl_rl/explore/<run>/model_1499.pt

Reports floor coverage (free cells seen / floor cells of the 14.4 m office patch) at 30, 60 and 90 s, how runs ended
(wall / person contact; a run that ends keeps the coverage it had), time with a person inside 0.45 m, and mean speed.
Physics/actuation randomization on; sensor and localization noise with --obs-noise.
"""

import argparse
from dataclasses import asdict

import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg

import robotlab.tasks  # noqa: F401
from robotlab.tasks import mentorpi_explore_env_cfg as T

TASK = "Robotlab-Explore-MentorPiA1"
p = argparse.ArgumentParser()
p.add_argument("--checkpoint", required=True)
p.add_argument("--num-envs", type=int, default=1000)
p.add_argument("--seconds", type=float, default=90.0)
p.add_argument("--seed", type=int, default=123)
p.add_argument("--obs-noise", action="store_true")
args = p.parse_args()

torch.manual_seed(args.seed)
cfg = load_env_cfg(TASK, play=True)
cfg.scene.num_envs, cfg.seed, cfg.episode_length_s = args.num_envs, args.seed, 1e6
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
marks = {30.0: None, 60.0: None, 90.0: None}
alive = torch.ones(n, dtype=torch.bool, device=dev)
final_cov = torch.zeros(n, device=dev)
wall = torch.zeros(n, dtype=torch.bool, device=dev)
person = torch.zeros(n, dtype=torch.bool, device=dev)
close = torch.zeros(n, device=dev)
speed = torch.zeros(n, device=dev)
steps = torch.zeros(n, device=dev)

obs = venv.get_observations()
with torch.inference_mode():
  for k in range(int(args.seconds / env.step_dt)):
    cov_before = T.coverage(env)
    final_cov = torch.where(alive, cov_before, final_cov)
    obs, _, dones, _ = venv.step(policy(obs))
    tm = env.termination_manager
    ended = alive & dones.bool()
    wall |= ended & tm.get_term("bump").bool()
    person |= ended & ~wall & tm.get_term("hit_person").bool()
    alive &= ~ended
    a = alive.float()
    close += (T.person_clearance(env) < 0.45).float() * a
    speed += env.scene["car"].data.root_link_lin_vel_b[:, 0] * a
    steps += a
    t = (k + 1) * env.step_dt
    final_cov = torch.where(alive, T.coverage(env), final_cov)
    for m in marks:
      if marks[m] is None and t >= m - 1e-6:
        marks[m] = final_cov.clone()

print(f"checkpoint: {args.checkpoint}  ({'with' if args.obs_noise else 'no'} sensor noise)")
print(f"{n} runs x {args.seconds:.0f} s, office floors (hallways 1.2-2.4 m) with up to 4 walking people")
for m, c in marks.items():
  if c is not None:
    print(f"coverage at {m:3.0f} s: mean {c.mean().item():6.1%}  median {c.median().item():6.1%}  "
          f"p10 {c.quantile(0.1).item():6.1%}  p90 {c.quantile(0.9).item():6.1%}")
print(f"ended by wall contact {wall.float().mean().item():.1%}  person contact {person.float().mean().item():.1%}  "
      f"ran the full time {alive.float().mean().item():.1%}")
print(f"time with a person inside 0.45 m: {(close.sum() / steps.sum()).item():.2%}  "
      f"mean forward speed {(speed.sum() / steps.sum()).item():.2f} m/s")
