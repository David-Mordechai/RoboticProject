"""Trace the teacher on mid-hallway turn-back routes (2026-11-08): for runs that end lost, print the last 3 s per step
(pose, route distance, K-turn state, action, shield, back-up) so the failure can be read, not guessed."""
import math
import torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.tasks.registry import load_env_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import mentorpi_navigate_env_cfg as N
from robotlab.tasks import teacher as TCH
from robotlab.tasks.mdp import car_pose

cfg = load_env_cfg("Robotlab-Goto-MentorPiA1", play=True); cfg.scene.num_envs = 512
cfg.observations["actor"].enable_corruption = True
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
env.reset()
n = env.num_envs
H = 120
hist = []
shown = 0
mid0 = env._robotlab_midturn.clone()
with torch.inference_mode():
  for it in range(20 * 40):
    a, rel = TCH.act(env)
    xy, yaw = car_pose(env)
    drv = env.action_manager.get_term("drive")
    st = env._teacher_kturn
    rec = torch.stack([xy[:, 0], xy[:, 1], torch.rad2deg(yaw), N.route_distance(env), N.nav(env).s, st["on"].float(),
                       st["dir"], a[:, 0], a[:, 1], drv.shielded.float(), drv.backing.float(), rel[:, 5, 0], rel[:, 5, 1]], -1)
    hist = (hist + [rec.cpu()])[-H:]
    mo = env._robotlab_midturn_open.clone()
    _, _, term, trunc, _ = env.step(a)
    # v2: turn-back routes still open after 25 s (not lost, not done): what holds them up
    lost = (mo & (env.episode_length_buf == 500)) if it >= 499 else torch.zeros_like(mo)
    for j in torch.nonzero(lost)[:, 0].tolist():
      if shown >= 6:
        break
      shown += 1
      print(f"=== car {j} still on its turn-back route at step {it}, hall width {N.nav(env).maps.hall_width[N.nav(env).layout[j]]:.2f} m")
      print(" x     y     yaw   dist  s     kt dir  a_v   a_st  shld back  path0.6x path0.6y")
      for r in [h[j] for h in hist][::6]:
        print(" ".join(f"{v:5.2f}" for v in r.tolist()))
    if shown >= 6:
      break
