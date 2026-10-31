"""Evaluate a formation (T1) checkpoint: one run per env, per formation slot, among walking people.

  docker compose run --rm lab python lab/scripts/evaluate_formation.py --checkpoint logs/rsl_rl/formation/<run>/model_599.pt

Per slot: how runs ended (completed / wall / person / lost), mean error to the adapted slot while driving, time within
0.25 m of it, and time the side slots were fallen back behind the leader for lack of room. People: contacts split by whether the person was standing (the v1 failure: clipping standing
people's feet), time with a person inside 0.45 m. Physics/actuation randomization on; sensor noise with --obs-noise.
"""

import argparse
from dataclasses import asdict

import torch

from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg

import robotlab.tasks  # noqa: F401
from robotlab.tasks import mentorpi_formation_env_cfg as T
from robotlab.tasks.mentorpi_office_env_cfg import office_leader
from robotlab.tasks.people import walkers

TASK = "Robotlab-Formation-MentorPiA1"
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
f, s = T.formation(env), office_leader(env)
w = walkers(env, s.maps, s.layout)
names = list(T.SLOTS)
slot = torch.stack([(f.slot == torch.tensor(v, device=dev)).all(-1) for v in T.SLOTS.values()], -1).float().argmax(-1)
alive = torch.ones(n, dtype=torch.bool, device=dev)
outcome = torch.zeros(n, dtype=torch.long, device=dev)       # 0 completed, 1 wall, 2 person, 3 lost
err_sum, in_slot, steps = torch.zeros(n, device=dev), torch.zeros(n, device=dev), torch.zeros(n, device=dev)
close = torch.zeros(n, device=dev)
fallback = torch.zeros(n, device=dev)
person_standing = torch.zeros(n, dtype=torch.bool, device=dev)

obs = venv.get_observations()
with torch.inference_mode():
  for k in range(int(args.seconds / env.step_dt)):
    xy, _ = T.car_pose(env)
    near = torch.where(w.active, torch.linalg.norm(w.xy - xy[:, None], dim=-1), 1e3).argmin(-1)
    standing = torch.linalg.norm(w.vel[torch.arange(n, device=dev), near], dim=-1) < 0.05
    obs, _, dones, _ = venv.step(policy(obs))
    tm = env.termination_manager
    wall, person, lost = tm.get_term("bump"), tm.get_term("hit_person"), tm.get_term("lost_slot")
    ended = alive & dones.bool()
    outcome[ended & wall] = 1
    outcome[ended & ~wall & person] = 2
    person_standing |= ended & ~wall & person & standing
    outcome[ended & ~wall & ~person & lost] = 3
    alive &= ~ended
    e = T.slot_distance(env)
    err_sum += torch.where(alive, e, 0.0)
    in_slot += (alive & (e < 0.25)).float()
    steps += alive.float()
    close += (alive & (T.person_clearance(env) < T.PERSONAL_SPACE)).float()
    fallback += (alive & (f.lat_eff.abs() < 0.9 * f.slot[:, 1].abs())).float()

print(f"checkpoint: {args.checkpoint}  ({'with' if args.obs_noise else 'no'} sensor noise)")
print(f"{n} runs x {args.seconds:.0f} s, office floors with up to 4 walking people, Nav2-like leader")
print(f"{'slot':16s} {'runs':>5s} {'completed':>10s} {'wall':>6s} {'person':>7s} {'lost':>6s} {'slot err':>9s} {'in slot':>8s} {'fell back':>10s}")
for i, name in enumerate(names):
  m = slot == i
  r = lambda j: (outcome[m] == j).float().mean().item()
  print(f"{name:16s} {int(m.sum()):5d} {r(0):10.1%} {r(1):6.1%} {r(2):7.1%} {r(3):6.1%} "
        f"{(err_sum[m].sum() / steps[m].sum()).item() * 100:7.1f}cm {(in_slot[m].sum() / steps[m].sum()).item():8.1%} "
        f"{(fallback[m].sum() / steps[m].sum()).item():10.1%}")
r = lambda j: (outcome == j).float().mean().item()
print(f"{'ALL':16s} {n:5d} {r(0):10.1%} {r(1):6.1%} {r(2):7.1%} {r(3):6.1%} "
      f"{(err_sum.sum() / steps.sum()).item() * 100:7.1f}cm {(in_slot.sum() / steps.sum()).item():8.1%}")
print(f"person contacts: {int((outcome == 2).sum())}, of which with a STANDING person: {int(person_standing.sum())}")
print(f"time with a person inside 0.45 m: {(close.sum() / steps.sum()).item():.2%}")
env.close()
