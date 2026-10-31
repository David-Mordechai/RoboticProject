"""Evaluate a navigate checkpoint: one long run per env, errand after errand, among walking people.

  docker compose run --rm lab python lab/scripts/evaluate_navigate.py --checkpoint logs/rsl_rl/navigate/<run>/model_999.pt

Per env, until its first failure or the end of the run:
- goals reached (errands completed) and goals per minute;
- how it ended: completed the run, crash (wall/obstacle), hit a person, lost (> 1.5 m off the route);
- stuck: no route progress for 10 s while not ended (counted, the episode continues);
- people: share of time with a person inside the 0.45 m personal space (Hall's intimate zone), closest approach.
Deterministic policy, fresh offices, routes and people. Physics/actuation randomization on; sensor noise with --obs-noise.
"""

import argparse
from dataclasses import asdict

import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg

import robotlab.tasks  # noqa: F401
from robotlab.tasks import mentorpi_navigate_env_cfg as T

TASK = "Robotlab-Navigate-MentorPiA1"
p = argparse.ArgumentParser()
p.add_argument("--checkpoint", required=True)
p.add_argument("--num-envs", type=int, default=512)
p.add_argument("--seconds", type=float, default=120.0)
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
nv = T.nav(env)
alive = torch.ones(n, dtype=torch.bool, device=dev)
outcome = torch.zeros(n, dtype=torch.long, device=dev)        # 0 completed, 1 crash, 2 hit person, 3 lost
goals = torch.zeros(n, device=dev)
t_alive = torch.zeros(n, device=dev)
close_steps = torch.zeros(n, device=dev)
min_clear = torch.full((n,), 1e3, device=dev)
stuck_events = torch.zeros(n, device=dev)
no_progress = torch.zeros(n, device=dev)
STUCK_STEPS = int(10.0 / env.step_dt)

obs = venv.get_observations()
with torch.inference_mode():
  for k in range(int(args.seconds / env.step_dt)):
    obs, _, dones, _ = venv.step(policy(obs))
    tm = env.termination_manager
    crash, person, lost = tm.get_term("bump"), tm.get_term("hit_person"), tm.get_term("lost_route")
    ended = alive & dones.bool()
    outcome[ended & crash] = 1
    outcome[ended & ~crash & person] = 2
    outcome[ended & ~crash & ~person & lost] = 3
    alive &= ~ended
    goals += (alive & nv.reached).float()
    t_alive += alive.float()
    c = T.person_clearance(env)
    close_steps += (alive & (c < T.PERSONAL_SPACE)).float()
    min_clear = torch.where(alive, torch.minimum(min_clear, c), min_clear)
    moving = (nv.s - nv.s_prev) > 0.002
    no_progress = torch.where(moving | ~alive, 0.0, no_progress + 1)
    stuck_events += (no_progress == STUCK_STEPS).float()

minutes = t_alive.sum().item() * env.step_dt / 60
r = lambda i: (outcome == i).float().mean().item()
met = min_clear < 1e2
print(f"checkpoint: {args.checkpoint}  ({'with' if args.obs_noise else 'no'} sensor noise)")
print(f"envs: {n} x {args.seconds:.0f} s, office floors with up to 4 walking people")
print(f"  completed the run:  {r(0):6.1%}")
print(f"  crashed (wall):     {r(1):6.1%}")
print(f"  hit a person:       {r(2):6.1%}")
print(f"  lost the route:     {r(3):6.1%}")
print(f"  errands (goals) reached: {goals.sum().item() / minutes:.2f} per minute of driving, "
      f"{goals.mean().item():.1f} per env")
print(f"  stuck 10 s without progress: {stuck_events.sum().item() / minutes:.2f} times per minute")
print(f"  time with a person inside 0.45 m: {close_steps.sum().item() / t_alive.sum().item():.2%}; "
      f"closest approach (median over envs that met people): {min_clear[met].median().item():.2f} m")
env.close()
