"""Step trace of a few goto mid-hallway turn-back routes near the turn point (goto v5, 2026-11-05)."""
import sys
from dataclasses import asdict
import numpy as np, torch
from mjlab.envs import ManagerBasedRlEnv
from mjlab.rl import MjlabOnPolicyRunner, RslRlVecEnvWrapper
from mjlab.tasks.registry import load_env_cfg, load_rl_cfg
import robotlab.tasks  # noqa
from robotlab.tasks import mentorpi_navigate_env_cfg as N
from robotlab.tasks import mentorpi_explore_env_cfg as X
from robotlab.tasks.mdp import car_pose
TASK = "Robotlab-Goto-MentorPiA1"
cfg = load_env_cfg(TASK, play=True); cfg.scene.num_envs = 256; cfg.episode_length_s = 20.0
cfg.observations["actor"].enable_corruption = True
ag = load_rl_cfg(TASK)
env = ManagerBasedRlEnv(cfg=cfg, device="cuda:0")
venv = RslRlVecEnvWrapper(env, clip_actions=ag.clip_actions)
r = MjlabOnPolicyRunner(venv, asdict(ag), device="cuda:0")
r.load(sys.argv[1], load_cfg={"actor": True}, strict=True, map_location="cuda:0")
pol = r.get_inference_policy(device="cuda:0")
MID = env._robotlab_midturn.nonzero()[:, 0][:24]
nv = N.nav(env); T = nv.path[MID, 1].clone(); X0 = nv.path[MID, 0].clone()
ax = (T - X0) / torch.linalg.norm(T - X0, dim=-1, keepdim=True)
e = X.explore(env)
obs = venv.get_observations(); rows = {int(i): [] for i in MID}
with torch.inference_mode():
  for it in range(20 * 12):
    xy, yaw = car_pose(env)
    a = pol(obs)
    p = xy[MID] - T
    along, lat = (p * ax).sum(-1), p[:, 0] * ax[:, 1] - p[:, 1] * ax[:, 0]
    v = env.scene["car"].data.root_link_lin_vel_b[MID, 0]
    hd = torch.rad2deg(torch.atan2(torch.sin(yaw[MID] - torch.atan2(ax[:, 1], ax[:, 0])), torch.cos(yaw[MID] - torch.atan2(ax[:, 1], ax[:, 0]))))
    lid = obs["actor"][MID, 4:40] * 3
    for j, i in enumerate(MID.tolist()):
      rows[i].append((it * 0.05, along[j].item(), lat[j].item(), hd[j].item(), v[j].item(), a[i, 0].item(), a[i, 1].item(),
                      nv.s[i].item(), lid[j, :2].min().item(), lid[j, 8:10].min().item(), lid[j, 26:28].min().item()))
    obs, _, dones, _ = venv.step(a)
fails = [i for i, rr in rows.items() if max(x[1] for x in rr) > 0.9][:3]
for i, rr in ((i, rows[i]) for i in fails):
  print(f"--- env {i}: hall width ~{rr[0][9] + rr[0][10]:.2f} m (lidar left+right at start)")
  for row in rr[::4]:
    if -1.2 < row[1] and row[0] < 9:
      print("t%5.2f along %+.2f lat %+.2f head %+5.0f v %+.2f act(%+.2f,%+.2f) s %.2f | front %.2f left %.2f right %.2f" % row)
