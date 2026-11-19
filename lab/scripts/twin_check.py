"""Office-twin exam (2026-11-09): N cars drive the floor route from the charger in the real office (env ROBOTLAB_TWIN,
robotlab/tasks/office_twin.py). The floor measure is the trip: finished within the floor run's 120 s, its time, the
back-ups, and where the others got stuck. Calibration rule: it must rank models as the floor did before it is used.

  ROBOTLAB_TWIN=lab/src/robotlab/tasks/twin_office.npz python lab/scripts/twin_check.py <ckpt.pt | carskill:<ckpt.pt> |
      carskill:teacher | teacher> [N]
"""
import math
import sys
from dataclasses import asdict

import numpy as np
import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg

import robotlab.tasks  # noqa: F401
from robotlab.tasks import mentorpi_navigate_env_cfg as N
from robotlab.tasks.mdp import car_pose

TASK = "Robotlab-Goto-MentorPiA1"
ARG = sys.argv[1]
NCARS = int(sys.argv[2]) if len(sys.argv) > 2 else 256
TRIP_S = 120.0                               # the floor runs' --seconds
cfg = load_env_cfg(TASK, play=True)
cfg.scene.num_envs = NCARS
cfg.episode_length_s = TRIP_S + 5.0
cfg.observations["actor"].enable_corruption = True
ag = load_rl_cfg(TASK)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
venv = RslRlVecEnvWrapper(env, clip_actions=ag.clip_actions)
dev, n = env.device, env.num_envs


def load(path):
  r = MjlabOnPolicyRunner(venv, asdict(ag), device="cuda:0")
  r.load(path, load_cfg={"actor": True}, strict=True, map_location="cuda:0")
  return r.get_inference_policy(device="cuda:0")


KS = None
if ARG == "teacher":
  from robotlab.tasks import teacher as TCH
  pol = lambda _o: TCH.act(env)[0]
elif ARG.startswith("carskill:"):
  from robotlab.tasks import carskill as CS
  if ARG[9:] == "teacher":
    from robotlab.tasks import teacher as TCH
    def stu(_o):
      at = TCH.act(env)[0].clone(); at[:, 0] = at[:, 0].clamp(min=0.0); return at
  else:
    stu = load(ARG[9:])
  pol, KS = CS.make_policy(env, stu)
else:
  pol = load(ARG)

drv = env.action_manager.get_term("drive")
nv = N.nav(env)
total = float(nv.cum[0].max())               # route length (same for every car)
obs = venv.get_observations()
alive = torch.ones(n, dtype=torch.bool, device=dev)
finish_t = torch.full((n,), math.nan, device=dev)
backups = torch.zeros(n, device=dev); was_backing = torch.zeros(n, dtype=torch.bool, device=dev)
turns = torch.zeros(n, device=dev); was_turn = torch.zeros(n, dtype=torch.bool, device=dev)
max_s = torch.zeros(n, device=dev); end_kind = ["running"] * n
held_t = torch.zeros(n, device=dev)
steps = int(TRIP_S / env.step_dt)
with torch.inference_mode():
  for it in range(steps):
    obs, _, dones, _ = venv.step(pol(obs))
    t = (it + 1) * env.step_dt
    live = alive & torch.isnan(finish_t)
    backups += (live & drv.backing & ~was_backing).float(); was_backing = drv.backing.clone()
    if KS is not None:
      backups_k = KS["on"]
      turns += (live & backups_k & ~was_turn).float(); was_turn = backups_k.clone()
    held_t += (live & drv.shielded).float() * env.step_dt
    max_s = torch.where(live, torch.maximum(max_s, nv.s), max_s)
    xy, _ = car_pose(env)
    fin_now = live & (nv.s > total - 0.4) & (torch.linalg.norm(xy - nv.path[:, -1], dim=-1) < 0.4)
    finish_t = torch.where(fin_now, torch.full_like(finish_t, t), finish_t)
    if dones.any():
      tm = env.termination_manager
      for i in (dones.bool() & live & ~fin_now).nonzero()[:, 0].tolist():
        kind = [k for k in ("bump", "hit_person", "lost") if k in tm.active_terms and bool(tm.get_term(k)[i])]
        end_kind[i] = kind[0] if kind else "timeout"
      alive &= ~(dones.bool() & ~fin_now)
fin = ~torch.isnan(finish_t)
ft = finish_t[fin].cpu().numpy()
print(f"== office twin: {ARG}, {n} cars, route {total:.1f} m, {TRIP_S:.0f} s")
print(f"finished the trip: {fin.float().mean():.1%}   time median {np.median(ft) if len(ft) else float('nan'):.1f} s "
      f"(p10 {np.percentile(ft, 10) if len(ft) else float('nan'):.1f}, p90 {np.percentile(ft, 90) if len(ft) else float('nan'):.1f})")
print(f"back-ups per car: median {backups.median():.0f}, mean {backups.mean():.1f}" +
      (f";  multi-point turns per car: mean {turns.mean():.1f}" if KS is not None else "") +
      f";  time held by the stop: mean {held_t.mean():.1f} s")
kinds = {}
for i in range(n):
  if not bool(fin[i]):
    kinds[end_kind[i]] = kinds.get(end_kind[i], 0) + 1
print("not finished, by end:", {k: f"{v / n:.1%}" for k, v in kinds.items()})
# where the unfinished got stuck: route progress -> named stretch of the floor route
# (arc length of the floor route: first waypoint 1.24 m (on the desk leg cell), door 2.76-3.56 m, hall end 8.16-10.36 m,
# door back 13.9-15.8 m, charger 18.6 m)
edges = [(0.0, "charger -> first waypoint"), (1.24, "-> door"), (2.76, "door"), (4.0, "hall out"),
         (8.16, "U-turn at hall end"), (10.36, "hall back"), (13.9, "door back"), (15.8, "back to charger")]
cum = nv.cum[0].cpu().numpy()
where = {}
for i in (~fin).nonzero()[:, 0].tolist():
  s = float(max_s[i])
  name = [nm for e0, nm in edges if s >= e0][-1] if s >= 0 else edges[0][1]
  where[name] = where.get(name, 0) + 1
print("not finished, furthest stretch reached:", {k: f"{v / n:.1%}" for k, v in sorted(where.items(), key=lambda kv: -kv[1])})
