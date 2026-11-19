"""Where the teacher's steering std on straight stretches comes from (2026-11-08): the checker's straight-stretch mask,
split into steps with the path offset at the route centre and no person within 1.5 m ("free straight") and the rest
(sidestepping clutter / people)."""
import math
import numpy as np
import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import mentorpi_explore_env_cfg as T
from robotlab.tasks import mentorpi_navigate_env_cfg as N
from robotlab.tasks import teacher as TCH
from robotlab.tasks.mdp import car_pose
from robotlab.tasks.people import walkers

cfg = load_env_cfg("Robotlab-Goto-MentorPiA1", play=True); cfg.scene.num_envs = 512
cfg.observations["actor"].enable_corruption = True
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
env.reset()
dev = env.device
S_free, S_other = [], []
TR = []
with torch.inference_mode():
  for it in range(20 * 40):
    a, _ = TCH.act(env)
    env.step(a)
    drv = env.action_manager.get_term("drive"); nv = N.nav(env)
    rp = N._points_at(nv, nv.s[:, None] + torch.tensor([0.0, 0.75, 1.5], device=dev))
    a1, a2 = rp[:, 1] - rp[:, 0], rp[:, 2] - rp[:, 1]
    bend = torch.atan2(a1[:, 0] * a2[:, 1] - a1[:, 1] * a2[:, 0], (a1 * a2).sum(-1)).abs()
    vx = env.scene["car"].data.root_link_lin_vel_b[:, 0]
    st = (bend < math.radians(10)) & (vx > 0.3) & ~drv.shielded & ~drv.backing & (env.episode_length_buf > 1)
    xy, _ = car_pose(env); e = T.explore(env); w = walkers(env, e.maps, e.layout)
    pnear = ((torch.linalg.norm(w.xy - xy[:, None], dim=-1) < 1.5) & w.active).any(-1)
    free = st & (env._teacher_off == 0) & ~pnear
    deg = torch.rad2deg(drv._steer_cmd * drv.steer_max)
    S_free.append(torch.where(free, deg, torch.nan).cpu().numpy())
    if it >= 200:
      TR.append((free.cpu().numpy(), deg.cpu().numpy(), (drv.cfg.steer_cmd_rate), env._teacher_kturn["on"].cpu().numpy(),
                 N.route_distance(env).cpu().numpy()))
    S_other.append(torch.where(st & ~free, deg, torch.nan).cpu().numpy())
for name, S in (("free straight", S_free), ("sidestepping clutter / people", S_other)):
  A = np.array(S)
  stds = [np.nanstd(A[:, j]) for j in range(A.shape[1]) if np.sum(~np.isnan(A[:, j])) >= 20]
  print(f"{name}: steps {int(np.sum(~np.isnan(A)))}, steering std per car mean {np.mean(stds):.2f} deg over {len(stds)} cars")

# one car with the longest free-straight run: its steering every step (deg) and distance to the route (cm)
F = np.array([t[0] for t in TR]); Dg = np.array([t[1] for t in TR]); Rd = np.array([t[4] for t in TR])
best, bj = 0, 0
for j in range(F.shape[1]):
  run = 0
  for k in range(F.shape[0]):
    run = run + 1 if F[k, j] else 0
    if run > best: best, bj, bend_k = run, j, k
print(f"car {bj}: {best} free-straight steps")
k0 = bend_k - best + 1
print("steer deg:", " ".join(f"{x:+.0f}" for x in Dg[k0:k0 + min(best, 80), bj]))
print("route dist cm:", " ".join(f"{100 * x:.0f}" for x in Rd[k0:k0 + min(best, 80), bj]))
