"""Evaluate a follow checkpoint: one run per env behind the scripted leader on open floor (no walls, no people).

  docker compose run --rm lab python lab/scripts/evaluate_follow.py --checkpoint logs/rsl_rl/follow/<run>/model_299.pt

Reports how runs ended (completed / lost the slot), mean distance to the slot 0.6 m behind the leader, time within
0.15 m and 0.25 m of it, and time closer than the task's too_close gap. Physics/actuation randomization as configured;
sensor noise with --obs-noise.
"""

import argparse
from dataclasses import asdict

import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg

import robotlab.tasks  # noqa: F401
from robotlab.tasks import mentorpi_follow_env_cfg as T
from robotlab.tasks.mdp import car_pose

TASK = "Robotlab-Follow-MentorPiA1"
p = argparse.ArgumentParser()
p.add_argument("--checkpoint", required=True)
p.add_argument("--num-envs", type=int, default=1000)
p.add_argument("--seconds", type=float, default=60.0)
p.add_argument("--seed", type=int, default=123)
p.add_argument("--obs-noise", action="store_true")
args = p.parse_args()

torch.manual_seed(args.seed)
cfg = load_env_cfg(TASK, play=True)
cfg.scene.num_envs, cfg.seed, cfg.episode_length_s = args.num_envs, args.seed, 1e6
cfg.observations["actor"].enable_corruption = args.obs_noise
agent_cfg = load_rl_cfg(TASK)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
venv = RslRlVecEnvWrapper(env, clip_actions=agent_cfg.clip_actions)
runner = MjlabOnPolicyRunner(venv, asdict(agent_cfg), device="cuda:0")
runner.load(args.checkpoint, load_cfg={"actor": True}, strict=True, map_location="cuda:0")
policy = runner.get_inference_policy(device="cuda:0")

n, dev = args.num_envs, env.device
min_gap = cfg.rewards["too_close"].params["min_gap"]
alive = torch.ones(n, dtype=torch.bool, device=dev)
lost = torch.zeros(n, dtype=torch.bool, device=dev)
err_sum, near15, near25, close, steps = (torch.zeros(n, device=dev) for _ in range(5))
obs = venv.get_observations()
with torch.inference_mode():
  for _ in range(int(args.seconds / env.step_dt)):
    obs, _, dones, _ = venv.step(policy(obs))
    e = T.slot_distance(env)
    gap = torch.linalg.norm(T.leader_state(env).xy - car_pose(env)[0], dim=-1)
    a = alive.float()
    err_sum += e * a; near15 += (e < 0.15).float() * a; near25 += (e < 0.25).float() * a
    close += (gap < min_gap).float() * a; steps += a
    ended = dones.bool() & alive
    lost |= ended & env.termination_manager.get_term("lost_slot").bool()
    alive &= ~ended

done = ~lost
tot = steps.sum()
print(f"checkpoint: {args.checkpoint}  obs noise: {args.obs_noise}  runs: {n} x {args.seconds:.0f} s")
print(f"completed {done.float().mean().item():.1%}  lost slot {lost.float().mean().item():.1%}")
print(f"slot error {(err_sum.sum() / tot).item() * 100:.1f} cm  within 0.15 m {(near15.sum() / tot).item():.1%}"
      f"  within 0.25 m {(near25.sum() / tot).item():.1%}  closer than {min_gap} m {(close.sum() / tot).item():.2%}")
